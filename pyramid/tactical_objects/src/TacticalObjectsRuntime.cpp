#include <TacticalObjectsRuntime.h>

namespace tactical_objects {

TacticalObjectsRuntime::TacticalObjectsRuntime()
  : store_(std::make_shared<ObjectStore>()),
    spatial_(std::make_shared<SpatialIndex>(1.0)),
    milclass_(std::make_shared<MilClassEngine>(store_)),
    correlation_(std::make_shared<CorrelationEngine>(store_, spatial_, milclass_)),
    zone_(std::make_shared<ZoneEngine>(store_, spatial_)),
    query_(std::make_shared<QueryEngine>(store_, spatial_)) {}

UUIDKey TacticalObjectsRuntime::createObject(const ObjectDefinition& def) {
  auto id = store_->createObject(def.type);

  KinematicsComponent kc;
  kc.position = def.position;
  kc.velocity = def.velocity;
  store_->kinematics().set(id, kc);

  spatial_->insert(id, def.position);

  if (!def.identity_name.empty() || !def.source_refs.empty()) {
    IdentityComponent ic;
    ic.name = def.identity_name;
    ic.source_refs = def.source_refs;
    store_->identities().set(id, ic);
  }

  MilClassProfile profile = def.mil_class;
  if (def.affiliation != Affiliation::Unknown &&
      profile.affiliation == Affiliation::Unknown) {
    profile.affiliation = def.affiliation;
  }
  milclass_->setProfile(id, profile);

  // Mark all fields dirty on creation (new entity, full snapshot needed)
  dirty_entities_.insert(id);
  store_->setDirtyBits(id, FieldMaskBit::ALL);

  return id;
}

bool TacticalObjectsRuntime::updateObject(const UUIDKey& id, const ObjectUpdate& update) {
  if (!store_->getRecord(id)) return false;

  uint16_t dirty_bits = 0;

  if (update.position || update.velocity) {
    auto* kc = store_->kinematics().getMutable(id);
    if (kc) {
      Position old_pos = kc->position;
      if (update.position) { kc->position = *update.position; dirty_bits |= FieldMaskBit::POSITION; }
      if (update.velocity) { kc->velocity = *update.velocity; dirty_bits |= FieldMaskBit::VELOCITY; }
      if (update.position) spatial_->update(id, old_pos, kc->position);
    } else {
      KinematicsComponent new_kc;
      if (update.position) { new_kc.position = *update.position; dirty_bits |= FieldMaskBit::POSITION; }
      if (update.velocity) { new_kc.velocity = *update.velocity; dirty_bits |= FieldMaskBit::VELOCITY; }
      store_->kinematics().set(id, new_kc);
      if (update.position) spatial_->insert(id, new_kc.position);
    }
  }

  if (update.mil_class) {
    milclass_->setProfile(id, *update.mil_class);
    dirty_bits |= FieldMaskBit::MIL_CLASS | FieldMaskBit::AFFILIATION;
  }

  if (update.affiliation) {
    auto profile = milclass_->getProfile(id);
    MilClassProfile p = profile ? *profile : MilClassProfile{};
    p.affiliation = *update.affiliation;
    milclass_->setProfile(id, p);
    dirty_bits |= FieldMaskBit::AFFILIATION | FieldMaskBit::MIL_CLASS;
  }

  if (update.identity_name) {
    auto* ic = store_->identities().getMutable(id);
    if (ic) {
      ic->name = *update.identity_name;
    } else {
      IdentityComponent new_ic;
      new_ic.name = *update.identity_name;
      store_->identities().set(id, new_ic);
    }
    dirty_bits |= FieldMaskBit::IDENTITY_NAME;
  }

  if (update.behavior_pattern || update.operational_state) {
    auto* bc = store_->behaviors().getMutable(id);
    if (bc) {
      if (update.behavior_pattern) bc->behavior_pattern = *update.behavior_pattern;
      if (update.operational_state) bc->operational_state = *update.operational_state;
    } else {
      BehaviorComponent new_bc;
      if (update.behavior_pattern) new_bc.behavior_pattern = *update.behavior_pattern;
      if (update.operational_state) new_bc.operational_state = *update.operational_state;
      store_->behaviors().set(id, new_bc);
    }
    dirty_bits |= FieldMaskBit::BEHAVIOR;
  }

  store_->bumpVersion(id);
  if (dirty_bits) {
    dirty_entities_.insert(id);
    store_->setDirtyBits(id, dirty_bits);
  }
  return true;
}

bool TacticalObjectsRuntime::deleteObject(const UUIDKey& id) {
  const auto* kc = store_->kinematics().get(id);
  if (kc) {
    spatial_->remove(id, kc->position);
  }
  bool ok = store_->deleteObject(id);
  if (ok) {
    dirty_entities_.erase(id);
    deleted_entities_.insert(id);
  }
  return ok;
}

const EntityRecord* TacticalObjectsRuntime::getRecord(const UUIDKey& id) const {
  return store_->getRecord(id);
}

void TacticalObjectsRuntime::setMilClass(const UUIDKey& id, const MilClassProfile& profile) {
  milclass_->setProfile(id, profile);
}

tl::optional<MilClassProfile> TacticalObjectsRuntime::getMilClass(const UUIDKey& id) const {
  return milclass_->getProfile(id);
}

std::string TacticalObjectsRuntime::deriveSymbolKey(const UUIDKey& id) const {
  return milclass_->deriveSymbolKey(id);
}

UUIDKey TacticalObjectsRuntime::createZone(const ZoneDefinition& def) {
  return zone_->upsertZone(def);
}

bool TacticalObjectsRuntime::removeZone(const UUIDKey& id) {
  return zone_->removeZone(id);
}

tl::optional<ZoneDefinition> TacticalObjectsRuntime::getZone(const UUIDKey& id) const {
  return zone_->getZone(id);
}

bool TacticalObjectsRuntime::isInsideZone(const Position& pos, const UUIDKey& zone_id) const {
  return zone_->isInside(pos, zone_id);
}

ZoneRelationship TacticalObjectsRuntime::evaluateZoneTransition(
    const UUIDKey& obj, const UUIDKey& zone, const Position& pos, double time) {
  return zone_->evaluateTransition(obj, zone, pos, time);
}

CorrelationResult TacticalObjectsRuntime::processObservation(const Observation& obs) {
  auto result = correlation_->processObservation(obs);
  // Mark the correlated entity dirty
  if (!result.object_id.isNull()) {
    dirty_entities_.insert(result.object_id);
    store_->setDirtyBits(result.object_id, FieldMaskBit::ALL);
  }
  return result;
}

CorrelationResult TacticalObjectsRuntime::processObservationBatch(const ObservationBatch& batch) {
  CorrelationResult last{CorrelationOutcome::Created, UUIDKey{}, UUIDKey{}};
  for (auto& obs : batch.observations) {
    last = processObservation(obs);
  }
  return last;
}

QueryResponse TacticalObjectsRuntime::query(const QueryRequest& req) const {
  return query_->query(req);
}

void TacticalObjectsRuntime::setBehavior(const UUIDKey& id,
                                          const std::string& behavior_pattern,
                                          const std::string& operational_state) {
  BehaviorComponent bc;
  bc.behavior_pattern = behavior_pattern;
  bc.operational_state = operational_state;
  store_->behaviors().set(id, bc);
  store_->bumpVersion(id);
  dirty_entities_.insert(id);
  store_->setDirtyBits(id, FieldMaskBit::BEHAVIOR);
}

tl::optional<BehaviorComponent> TacticalObjectsRuntime::getBehavior(const UUIDKey& id) const {
  const auto* bc = store_->behaviors().get(id);
  if (!bc) return tl::nullopt;
  return *bc;
}

void TacticalObjectsRuntime::logMissingInfo(const UUIDKey& id, const std::string& field) {
  std::string msg = "Missing information for entity " +
                    pyramid::core::uuid::UUIDHelper::toString(id.uuid) +
                    ": " + field;
  logger_.log(pyramid::core::logging::LogLevel::Warning, msg);
}

// ---------------------------------------------------------------------------
// Streaming subscriptions
// ---------------------------------------------------------------------------

SubscriptionHandle TacticalObjectsRuntime::registerStreamSubscriber(
    const UUIDKey& interest_id, EntityUpdateCallback cb) {
  SubscriptionHandle h = next_handle_++;
  subscribers_[interest_id].emplace_back(h, std::move(cb));
  handle_to_interest_[h] = interest_id;
  // New subscriber: mark all currently-existing matching entities for full snapshot
  // by setting version 0 (they will be sent with full field mask on first tick).
  // subscriber_versions_[h] starts empty — absence == "never seen" == send full snapshot.
  return h;
}

void TacticalObjectsRuntime::removeStreamSubscriber(SubscriptionHandle handle) {
  auto it = handle_to_interest_.find(handle);
  if (it == handle_to_interest_.end()) return;
  UUIDKey interest_id = it->second;
  handle_to_interest_.erase(it);

  auto& vec = subscribers_[interest_id];
  for (auto vit = vec.begin(); vit != vec.end(); ++vit) {
    if (vit->first == handle) {
      vec.erase(vit);
      break;
    }
  }
  if (vec.empty()) subscribers_.erase(interest_id);
  subscriber_versions_.erase(handle);
}

// ---------------------------------------------------------------------------
// Stream frame assembly (Step 5)
// ---------------------------------------------------------------------------

StreamFrame TacticalObjectsRuntime::assembleStreamFrame(
    const UUIDKey& interest_id, SubscriptionHandle subscriber, double timestamp) {
  StreamFrame frame;
  frame.timestamp = timestamp;

  const InterestRecord* interest_rec = interest_manager_.get(interest_id);
  if (!interest_rec || interest_rec->status != InterestStatus::Active) return frame;

  auto& sub_versions = subscriber_versions_[subscriber];

  // If this subscriber has no version records at all, perform a full snapshot
  // scan across all entities in the store (initial connect case).
  bool is_new_subscriber = sub_versions.empty();
  if (is_new_subscriber) {
    store_->forEachObject([&](const EntityRecord& rec) {
      if (!interest_manager_.matchesInterest(interest_rec->criteria, rec, *store_)) return;

      EntityUpdateFrame upd;
      upd.message_type = STREAM_MSG_ENTITY_UPDATE;
      upd.entity_id    = rec.id;
      upd.version      = rec.version;
      upd.timestamp    = timestamp;

      uint16_t mask = 0;
      const auto* kc = store_->kinematics().get(rec.id);
      if (kc) {
        upd.position = kc->position;  mask |= FieldMaskBit::POSITION;
        upd.velocity = kc->velocity;  mask |= FieldMaskBit::VELOCITY;
      }
      const auto* mc = store_->milclass().get(rec.id);
      if (mc) {
        upd.affiliation = mc->profile.affiliation; mask |= FieldMaskBit::AFFILIATION;
        upd.mil_class = mc->profile;               mask |= FieldMaskBit::MIL_CLASS;
      }
      upd.object_type = rec.type;  mask |= FieldMaskBit::OBJECT_TYPE;
      const auto* qc = store_->quality().get(rec.id);
      if (qc) { upd.confidence = qc->confidence; mask |= FieldMaskBit::CONFIDENCE; }
      const auto* lc = store_->lifecycle().get(rec.id);
      if (lc) { upd.lifecycle_status = lc->status; mask |= FieldMaskBit::LIFECYCLE_STATUS; }
      const auto* bc = store_->behaviors().get(rec.id);
      if (bc) { upd.behavior = *bc; mask |= FieldMaskBit::BEHAVIOR; }
      const auto* ic = store_->identities().get(rec.id);
      if (ic) { upd.identity_name = ic->name; mask |= FieldMaskBit::IDENTITY_NAME; }
      upd.field_mask = mask;

      frame.updates.push_back(upd);
      sub_versions[rec.id] = rec.version;
    });
    return frame;
  }

  // Deleted entities matching this interest
  for (const auto& del_id : deleted_entities_) {
    // We can't check interest match on a deleted entity — just send delete to all
    // subscribers for any entity they had previously seen.
    auto vit = sub_versions.find(del_id);
    if (vit != sub_versions.end()) {
      frame.deletes.push_back(del_id);
      sub_versions.erase(vit);
    }
  }

  // Updated entities
  for (const auto& eid : dirty_entities_) {
    const auto* rec = store_->getRecord(eid);
    if (!rec) continue;
    if (!interest_manager_.matchesInterest(interest_rec->criteria, *rec, *store_)) continue;

    uint64_t entity_version = rec->version;
    auto vit = sub_versions.find(eid);
    bool first_time = (vit == sub_versions.end());

    if (!first_time && vit->second == entity_version) {
      continue;  // version unchanged — skip
    }

    uint16_t mask;
    if (first_time) {
      // Full snapshot for new subscriber or newly-matching entity
      mask = FieldMaskBit::ALL;
    } else {
      // Delta: only fields that changed since last publish
      mask = store_->getDirtyMask(eid);
      if (mask == 0) mask = FieldMaskBit::ALL;
    }

    EntityUpdateFrame upd;
    upd.message_type = STREAM_MSG_ENTITY_UPDATE;
    upd.entity_id    = eid;
    upd.version      = entity_version;
    upd.timestamp    = timestamp;

    // Build actual_mask: only include bits for components that exist
    uint16_t actual_mask = 0;
    if (mask & FieldMaskBit::POSITION) {
      const auto* kc = store_->kinematics().get(eid);
      if (kc) { upd.position = kc->position; actual_mask |= FieldMaskBit::POSITION; }
    }
    if (mask & FieldMaskBit::VELOCITY) {
      const auto* kc = store_->kinematics().get(eid);
      if (kc) { upd.velocity = kc->velocity; actual_mask |= FieldMaskBit::VELOCITY; }
    }
    if (mask & FieldMaskBit::AFFILIATION) {
      const auto* mc = store_->milclass().get(eid);
      if (mc) { upd.affiliation = mc->profile.affiliation; actual_mask |= FieldMaskBit::AFFILIATION; }
    }
    if (mask & FieldMaskBit::OBJECT_TYPE) {
      upd.object_type = rec->type; actual_mask |= FieldMaskBit::OBJECT_TYPE;
    }
    if (mask & FieldMaskBit::CONFIDENCE) {
      const auto* qc = store_->quality().get(eid);
      if (qc) { upd.confidence = qc->confidence; actual_mask |= FieldMaskBit::CONFIDENCE; }
    }
    if (mask & FieldMaskBit::LIFECYCLE_STATUS) {
      const auto* lc = store_->lifecycle().get(eid);
      if (lc) { upd.lifecycle_status = lc->status; actual_mask |= FieldMaskBit::LIFECYCLE_STATUS; }
    }
    if (mask & FieldMaskBit::MIL_CLASS) {
      const auto* mc = store_->milclass().get(eid);
      if (mc) { upd.mil_class = mc->profile; actual_mask |= FieldMaskBit::MIL_CLASS; }
    }
    if (mask & FieldMaskBit::BEHAVIOR) {
      const auto* bc = store_->behaviors().get(eid);
      if (bc) { upd.behavior = *bc; actual_mask |= FieldMaskBit::BEHAVIOR; }
    }
    if (mask & FieldMaskBit::IDENTITY_NAME) {
      const auto* ic = store_->identities().get(eid);
      if (ic) { upd.identity_name = ic->name; actual_mask |= FieldMaskBit::IDENTITY_NAME; }
    }
    upd.field_mask = actual_mask;

    frame.updates.push_back(upd);
    sub_versions[eid] = entity_version;
  }

  return frame;
}

// ---------------------------------------------------------------------------
// Flush dirty entities — called by on_tick()
// ---------------------------------------------------------------------------

void TacticalObjectsRuntime::flushDirtyEntities(double timestamp) {
  // Step 1: fire simple callbacks
  for (const auto& eid : dirty_entities_) {
    const auto* rec = store_->getRecord(eid);
    if (!rec) continue;
    auto matching = interest_manager_.matchingInterests(*rec, *store_);
    for (const auto& interest_id : matching) {
      auto sit = subscribers_.find(interest_id);
      if (sit == subscribers_.end()) continue;
      for (auto& sub_pair : sit->second) {
        sub_pair.second(interest_id, eid);
      }
    }
  }

  // Clean up deleted entity version entries
  for (const auto& del_id : deleted_entities_) {
    for (auto& sub_pair : subscriber_versions_) {
      sub_pair.second.erase(del_id);
    }
  }

  // Clear state for next tick
  dirty_entities_.clear();
  deleted_entities_.clear();
  store_->clearAllDirtyMasks();
}

} // namespace tactical_objects
