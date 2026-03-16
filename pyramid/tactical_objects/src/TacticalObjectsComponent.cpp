#include <TacticalObjectsComponent.h>
#include <uuid/UUIDHelper.h>

#include <nlohmann/json.hpp>
#include <pcl/pcl_container.h>
#include <iostream>

namespace tactical_objects {

using json = nlohmann::json;

namespace {
const char* TOPIC_OBSERVATION_INGRESS      = "observation_ingress";
const char* TOPIC_ENTITY_UPDATES           = "entity_updates";
const char* TOPIC_EVIDENCE_REQUIREMENTS    = "evidence_requirements";
const char* SVC_CREATE_OBJECT              = "create_object";
const char* SVC_UPDATE_OBJECT              = "update_object";
const char* SVC_DELETE_OBJECT              = "delete_object";
const char* SVC_QUERY                      = "query";
const char* SVC_GET_OBJECT                 = "get_object";
const char* SVC_UPSERT_ZONE               = "upsert_zone";
const char* SVC_REMOVE_ZONE               = "remove_zone";
const char* SVC_SUBSCRIBE_INTEREST         = "subscribe_interest";
const char* SVC_RESYNC                     = "tactical_objects.resync";
const char* TYPE_JSON                      = "application/json";
const char* TYPE_BINARY                    = "application/octet-stream";
} // namespace

TacticalObjectsComponent::TacticalObjectsComponent()
  : pcl::Component("tactical_objects"),
    runtime_(std::make_shared<TacticalObjectsRuntime>()) {}

pcl_status_t TacticalObjectsComponent::on_configure() {
  entity_updates_port_ = addPublisher(TOPIC_ENTITY_UPDATES, TYPE_BINARY);
  evidence_requirements_port_ = addPublisher(TOPIC_EVIDENCE_REQUIREMENTS, TYPE_JSON);

  addSubscriber(TOPIC_OBSERVATION_INGRESS, TYPE_JSON, onObservationIngress, this);

  if (!addService(SVC_CREATE_OBJECT,      TYPE_JSON, handleCreateObject,      this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_UPDATE_OBJECT,      TYPE_JSON, handleUpdateObject,      this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_DELETE_OBJECT,      TYPE_JSON, handleDeleteObject,      this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_QUERY,              TYPE_JSON, handleQuery,              this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_GET_OBJECT,         TYPE_JSON, handleGetObject,          this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_UPSERT_ZONE,        TYPE_JSON, handleUpsertZone,         this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_REMOVE_ZONE,        TYPE_JSON, handleRemoveZone,         this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_SUBSCRIBE_INTEREST, TYPE_JSON, handleSubscribeInterest,  this)) return PCL_ERR_CALLBACK;
  if (!addService(SVC_RESYNC,             TYPE_JSON, handleResync,             this)) return PCL_ERR_CALLBACK;

  runtime_->logger().subscribe([](const pyramid::core::logging::LogEntry& entry) {
    std::cout << entry.message << std::endl;
  });

  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::on_activate() {
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::on_deactivate() {
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::on_cleanup() {
  runtime_ = std::make_shared<TacticalObjectsRuntime>();
  tick_count_ = 0;
  sequence_ids_.clear();
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::on_shutdown() {
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::on_tick(double dt) {
  ++tick_count_;

  // Only publish batched streaming frames every streaming_tick_divisor_ ticks
  if (streaming_tick_divisor_ > 1 &&
      (static_cast<int>(tick_count_) % streaming_tick_divisor_) != 0) {
    return PCL_OK;
  }

  // For each active interest that has registered subscribers, assemble and publish a batch frame
  auto active_interests = runtime_->interestManager().activeInterests();
  for (const auto& interest_rec : active_interests) {
    const UUIDKey& interest_id = interest_rec.interest_id;

    // Collect all subscribers for this interest by asking the runtime
    // We assemble one frame per interest (shared across all subscribers on this interest)
    // For simplicity, we publish one aggregated frame per interest to the entity_updates topic.

    // Build the sequence ID for this interest
    auto& seq_id = sequence_ids_[interest_id];

    // Assemble using the first subscriber handle for version-tracking purposes.
    // In a more complete implementation each subscriber would have its own frame.
    // Here we use handle=0 as a "broadcast" subscriber for the publish path.
    StreamFrame sf = runtime_->assembleStreamFrame(interest_id, 0, dt);

    if (sf.updates.empty() && sf.deletes.empty()) continue;

    sf.tick_id = seq_id++;

    // Split into chunks if max_entities_per_frame_ is set
    int chunk_start = 0;
    int total = static_cast<int>(sf.updates.size());
    do {
      int chunk_end = std::min(chunk_start + max_entities_per_frame_, total);
      std::vector<EntityUpdateFrame> chunk(
          sf.updates.begin() + chunk_start,
          sf.updates.begin() + chunk_end);

      // Add delete frames as EntityUpdateFrames with delete type
      if (chunk_start == 0) {
        for (const auto& del_id : sf.deletes) {
          EntityUpdateFrame del_frame;
          del_frame.message_type = STREAM_MSG_ENTITY_DELETE;
          del_frame.entity_id    = del_id;
          del_frame.version      = 0;
          del_frame.timestamp    = dt;
          chunk.push_back(del_frame);
        }
      }

      encode_buf_ = StreamingCodec::encodeBatchFrame(chunk, dt);

      if (entity_updates_port_) {
        pcl_msg_t pub_msg;
        pub_msg.data      = encode_buf_.data();
        pub_msg.size      = static_cast<uint32_t>(encode_buf_.size());
        pub_msg.type_name = TYPE_BINARY;
        pcl_port_publish(entity_updates_port_, &pub_msg);
      }

      chunk_start = chunk_end;
    } while (chunk_start < total);
  }

  runtime_->flushDirtyEntities(dt);
  return PCL_OK;
}

void TacticalObjectsComponent::onObservationIngress(pcl_container_t*, const pcl_msg_t* msg,
                                                   void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!msg->data || msg->size == 0) return;

  comp->runtime_->logger().log(pyramid::core::logging::LogLevel::Debug, 
      "[DEBUG] onObservationIngress received: " + std::to_string(msg->size) + " bytes");
  std::string str(static_cast<const char*>(msg->data), msg->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    comp->runtime_->logger().log(pyramid::core::logging::LogLevel::Debug, 
        "[DEBUG] onObservationIngress JSON parse failed");
    return;
  }

  ObservationBatch batch = TacticalObjectsCodec::decodeObservationBatch(j);
  comp->runtime_->logger().log(pyramid::core::logging::LogLevel::Debug, 
      "[DEBUG] onObservationIngress decoded batch of " + std::to_string(batch.observations.size()) + " observations");
  comp->runtime_->processObservationBatch(batch);
}

static std::string uuidToJsonString(const UUIDKey& k) {
  return TacticalObjectsCodec::encodeUUID(k).get<std::string>();
}

pcl_status_t TacticalObjectsComponent::handleCreateObject(pcl_container_t*,
                                                          const pcl_msg_t* request,
                                                          pcl_msg_t* response,
                                                          void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  ObjectDefinition def = TacticalObjectsCodec::decodeObjectDefinition(j);
  auto id = comp->runtime_->createObject(def);

  comp->response_buffer_ = json{{"object_id", uuidToJsonString(id)}}.dump();
  response->data = comp->response_buffer_.data();
  response->size = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleUpdateObject(pcl_container_t*,
                                                          const pcl_msg_t* request,
                                                          pcl_msg_t* response,
                                                          void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  std::string id_str = j.value("object_id", "");
  if (id_str.empty()) return PCL_ERR_INVALID;

  auto parsed = pyramid::core::uuid::UUIDHelper::fromString(id_str);
  if (!parsed.second) return PCL_ERR_INVALID;

  UUIDKey obj_id(parsed.first);
  ObjectUpdate upd = TacticalObjectsCodec::decodeObjectUpdate(j);
  bool ok = comp->runtime_->updateObject(obj_id, upd);
  comp->response_buffer_ = ok ? "{\"success\":true}" : "{\"success\":false}";
  response->data = comp->response_buffer_.data();
  response->size = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleDeleteObject(pcl_container_t*,
                                                          const pcl_msg_t* request,
                                                          pcl_msg_t* response,
                                                          void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  std::string id_str = j.value("object_id", "");
  if (id_str.empty()) return PCL_ERR_INVALID;

  auto parsed = pyramid::core::uuid::UUIDHelper::fromString(id_str);
  if (!parsed.second) return PCL_ERR_INVALID;

  UUIDKey obj_id(parsed.first);
  bool ok = comp->runtime_->deleteObject(obj_id);
  comp->response_buffer_ = ok ? "{\"success\":true}" : "{\"success\":false}";
  response->data = comp->response_buffer_.data();
  response->size = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleQuery(pcl_container_t*,
                                                   const pcl_msg_t* request,
                                                   pcl_msg_t* response,
                                                   void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  QueryRequest req;
  if (request->data && request->size > 0) {
    std::string str(static_cast<const char*>(request->data), request->size);
    try {
      json j = json::parse(str);
      req = TacticalObjectsCodec::decodeQueryRequest(j);
    } catch (...) {
      req = QueryRequest();
    }
  }

  auto resp = comp->runtime_->query(req);
  comp->response_buffer_ = TacticalObjectsCodec::encodeQueryResponse(resp).dump();
  response->data = comp->response_buffer_.data();
  response->size = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleGetObject(pcl_container_t*,
                                                       const pcl_msg_t* request,
                                                       pcl_msg_t* response,
                                                       void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  std::string id_str = j.value("object_id", "");
  if (id_str.empty()) return PCL_ERR_INVALID;

  auto parsed = pyramid::core::uuid::UUIDHelper::fromString(id_str);
  if (!parsed.second) return PCL_ERR_INVALID;

  UUIDKey obj_id(parsed.first);
  const auto* rec = comp->runtime_->getRecord(obj_id);
  if (!rec) {
    comp->response_buffer_ = "{\"found\":false}";
  } else {
    json out;
    out["found"] = true;
    out["id"] = TacticalObjectsCodec::encodeUUID(obj_id).get<std::string>();
    out["type"] = TacticalObjectsCodec::objectTypeToString(rec->type);
    out["version"] = rec->version;

    out["has_identity"] = comp->runtime_->store()->identities().has(obj_id);

    auto mil = comp->runtime_->getMilClass(obj_id);
    if (mil.has_value()) {
      out["mil_class"] = TacticalObjectsCodec::encodeMilClassProfile(*mil);
    }

    auto beh = comp->runtime_->getBehavior(obj_id);
    out["has_behavior"] = beh.has_value();
    if (beh.has_value()) {
      out["behavior"] = json{
        {"behavior_pattern", beh->behavior_pattern},
        {"operational_state", beh->operational_state}
      };
    }

    const auto* cc = comp->runtime_->store()->correlation().get(obj_id);
    out["has_correlation"] = (cc != nullptr);
    if (cc) {
      out["source_refs_count"] = cc->source_refs.size();
      out["contributing_observations_count"] = cc->contributing_observations.size();
    }

    const auto* qc = comp->runtime_->store()->quality().get(obj_id);
    if (qc) {
      out["confidence"] = qc->confidence;
    }

    comp->response_buffer_ = out.dump();
  }
  response->data = comp->response_buffer_.data();
  response->size = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleUpsertZone(pcl_container_t*,
                                                        const pcl_msg_t* request,
                                                        pcl_msg_t* response,
                                                        void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  ZoneDefinition def = TacticalObjectsCodec::decodeZoneDefinition(j);
  auto zid = comp->runtime_->createZone(def);

  comp->response_buffer_ = json{{"zone_id", uuidToJsonString(zid)}}.dump();
  response->data = comp->response_buffer_.data();
  response->size = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleRemoveZone(pcl_container_t*,
                                                        const pcl_msg_t* request,
                                                        pcl_msg_t* response,
                                                        void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  std::string id_str = j.value("zone_id", "");
  if (id_str.empty()) return PCL_ERR_INVALID;

  auto parsed = pyramid::core::uuid::UUIDHelper::fromString(id_str);
  if (!parsed.second) return PCL_ERR_INVALID;

  UUIDKey zid(parsed.first);
  bool ok = comp->runtime_->removeZone(zid);
  comp->response_buffer_ = ok ? "{\"success\":true}" : "{\"success\":false}";
  response->data = comp->response_buffer_.data();
  response->size = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleSubscribeInterest(pcl_container_t*,
                                                                const pcl_msg_t* request,
                                                                pcl_msg_t* response,
                                                                void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  // Decode interest criteria
  InterestCriteria criteria;

  // Query mode: "read_current" (default) or "active_find"
  if (j.contains("query_mode") && !j["query_mode"].is_null()) {
    std::string qm = j["query_mode"].get<std::string>();
    if (qm == "active_find") {
      criteria.query_mode = QueryMode::ActiveFind;
    } else {
      criteria.query_mode = QueryMode::ReadCurrent;
    }
  }
  if (j.contains("object_type") && !j["object_type"].is_null()) {
    try {
      criteria.object_type = TacticalObjectsCodec::objectTypeFromString(j["object_type"].get<std::string>());
    } catch (...) {}
  }
  if (j.contains("affiliation") && !j["affiliation"].is_null()) {
    try {
      criteria.affiliation = TacticalObjectsCodec::affiliationFromString(j["affiliation"].get<std::string>());
    } catch (...) {}
  }
  if (j.contains("battle_dimension") && !j["battle_dimension"].is_null()) {
    try {
      criteria.battle_dimension = TacticalObjectsCodec::battleDimensionFromString(
          j["battle_dimension"].get<std::string>());
    } catch (...) {}
  }
  if (j.contains("area") && j["area"].is_object()) {
    BoundingBox bb;
    bb.min_lat = j["area"].value("min_lat", 0.0);
    bb.max_lat = j["area"].value("max_lat", 0.0);
    bb.min_lon = j["area"].value("min_lon", 0.0);
    bb.max_lon = j["area"].value("max_lon", 0.0);
    criteria.area = bb;
  }
  if (j.contains("behavior_pattern") && !j["behavior_pattern"].is_null()) {
    criteria.behavior_pattern = j["behavior_pattern"].get<std::string>();
  }
  criteria.minimum_confidence = j.value("minimum_confidence", 0.0);
  criteria.time_window_start  = j.value("time_window_start", 0.0);
  criteria.time_window_end    = j.value("time_window_end", 0.0);

  double expires_at = j.value("expires_at", 0.0);
  auto interest_id = comp->runtime_->interestManager().registerInterest(criteria, 0.0, expires_at);

  json resp_json;
  resp_json["interest_id"] = uuidToJsonString(interest_id);

  // For ActiveFind interests, determine a solution and publish evidence requirements
  bool is_active_find = criteria.query_mode.has_value() &&
                        *criteria.query_mode == QueryMode::ActiveFind;
  if (is_active_find) {
    auto solution = comp->runtime_->determineSolution(interest_id);
    resp_json["solution_id"] = uuidToJsonString(solution.solution_id);
    resp_json["predicted_quality"]      = solution.predicted_quality;
    resp_json["predicted_completeness"] = solution.predicted_completeness;

    // Publish each derived evidence requirement on the evidence_requirements topic
    json ev_reqs = json::array();
    for (const auto& der : solution.evidence_requirements) {
      json ev;
      ev["requirement_id"]     = uuidToJsonString(der.requirement_id);
      ev["source_interest_id"] = uuidToJsonString(der.source_interest_id);
      ev["evidence_description"] = der.evidence_description;
      ev_reqs.push_back(ev);

      // Publish to evidence_requirements topic (Object_Solution_Evidence service)
      if (comp->evidence_requirements_port_) {
        std::string ev_str = ev.dump();
        pcl_msg_t ev_msg = {};
        ev_msg.data      = ev_str.data();
        ev_msg.size      = static_cast<uint32_t>(ev_str.size());
        ev_msg.type_name = TYPE_JSON;
        pcl_port_publish(comp->evidence_requirements_port_, &ev_msg);
      }
    }
    resp_json["evidence_requirements"] = ev_reqs;
  }

  comp->response_buffer_ = resp_json.dump();
  response->data      = comp->response_buffer_.data();
  response->size      = static_cast<uint32_t>(comp->response_buffer_.size());
  response->type_name = TYPE_JSON;
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::handleResync(pcl_container_t*,
                                                     const pcl_msg_t* request,
                                                     pcl_msg_t* response,
                                                     void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!request->data || request->size == 0) return PCL_ERR_INVALID;

  std::string str(static_cast<const char*>(request->data), request->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return PCL_ERR_INVALID;
  }

  std::string id_str = j.value("interest_id", "");
  if (id_str.empty()) return PCL_ERR_INVALID;

  auto parsed = pyramid::core::uuid::UUIDHelper::fromString(id_str);
  if (!parsed.second) return PCL_ERR_INVALID;

  UUIDKey interest_id(parsed.first);
  const InterestRecord* interest_rec = comp->runtime_->interestManager().get(interest_id);
  if (!interest_rec) {
    comp->response_buffer_ = "{\"error\":\"interest not found\"}";
    response->data      = comp->response_buffer_.data();
    response->size      = static_cast<uint32_t>(comp->response_buffer_.size());
    response->type_name = TYPE_JSON;
    return PCL_ERR_INVALID;
  }

  // Build a full snapshot of all entities matching this interest
  std::vector<EntityUpdateFrame> snapshot;
  auto& store = *comp->runtime_->store();
  store.forEachObject([&](const EntityRecord& rec) {
    if (!comp->runtime_->interestManager().matchesInterest(
            interest_rec->criteria, rec, store)) return;

    EntityUpdateFrame upd;
    upd.message_type = STREAM_MSG_ENTITY_UPDATE;
    upd.entity_id    = rec.id;
    upd.version      = rec.version;
    upd.timestamp    = 0.0;

    uint16_t mask = 0;
    const auto* kc = store.kinematics().get(rec.id);
    if (kc) {
      upd.position = kc->position;  mask |= FieldMaskBit::POSITION;
      upd.velocity = kc->velocity;  mask |= FieldMaskBit::VELOCITY;
    }

    const auto* mc = store.milclass().get(rec.id);
    if (mc) {
      upd.affiliation = mc->profile.affiliation; mask |= FieldMaskBit::AFFILIATION;
      upd.mil_class = mc->profile;               mask |= FieldMaskBit::MIL_CLASS;
    }

    upd.object_type = rec.type;  mask |= FieldMaskBit::OBJECT_TYPE;

    const auto* qc = store.quality().get(rec.id);
    if (qc) { upd.confidence = qc->confidence; mask |= FieldMaskBit::CONFIDENCE; }

    const auto* lc = store.lifecycle().get(rec.id);
    if (lc) { upd.lifecycle_status = lc->status; mask |= FieldMaskBit::LIFECYCLE_STATUS; }

    const auto* bc = store.behaviors().get(rec.id);
    if (bc) { upd.behavior = *bc; mask |= FieldMaskBit::BEHAVIOR; }

    const auto* ic = store.identities().get(rec.id);
    if (ic) { upd.identity_name = ic->name; mask |= FieldMaskBit::IDENTITY_NAME; }
    upd.field_mask = mask;

    snapshot.push_back(upd);
  });

  comp->encode_buf_ = StreamingCodec::encodeBatchFrame(snapshot, 0.0);
  response->data      = comp->encode_buf_.data();
  response->size      = static_cast<uint32_t>(comp->encode_buf_.size());
  response->type_name = TYPE_BINARY;
  return PCL_OK;
}

} // namespace tactical_objects
