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

  milclass_->setProfile(id, def.mil_class);

  return id;
}

bool TacticalObjectsRuntime::updateObject(const UUIDKey& id, const ObjectUpdate& update) {
  if (!store_->getRecord(id)) return false;

  if (update.position || update.velocity) {
    auto* kc = store_->kinematics().getMutable(id);
    if (kc) {
      Position old_pos = kc->position;
      if (update.position) kc->position = *update.position;
      if (update.velocity) kc->velocity = *update.velocity;
      if (update.position) spatial_->update(id, old_pos, kc->position);
    } else {
      KinematicsComponent new_kc;
      if (update.position) new_kc.position = *update.position;
      if (update.velocity) new_kc.velocity = *update.velocity;
      store_->kinematics().set(id, new_kc);
      if (update.position) spatial_->insert(id, new_kc.position);
    }
  }

  if (update.mil_class) {
    milclass_->setProfile(id, *update.mil_class);
  }

  if (update.affiliation) {
    auto profile = milclass_->getProfile(id);
    MilClassProfile p = profile ? *profile : MilClassProfile{};
    p.affiliation = *update.affiliation;
    milclass_->setProfile(id, p);
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
  }

  store_->bumpVersion(id);
  return true;
}

bool TacticalObjectsRuntime::deleteObject(const UUIDKey& id) {
  const auto* kc = store_->kinematics().get(id);
  if (kc) {
    spatial_->remove(id, kc->position);
  }
  return store_->deleteObject(id);
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
  return correlation_->processObservation(obs);
}

CorrelationResult TacticalObjectsRuntime::processObservationBatch(const ObservationBatch& batch) {
  CorrelationResult last{CorrelationOutcome::Created, UUIDKey{}, UUIDKey{}};
  for (auto& obs : batch.observations) {
    last = correlation_->processObservation(obs);
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

} // namespace tactical_objects
