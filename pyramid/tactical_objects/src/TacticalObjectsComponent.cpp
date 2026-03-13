#include <TacticalObjectsComponent.h>
#include <uuid/UUIDHelper.h>

#include <nlohmann/json.hpp>
#include <pcl/pcl_container.h>

namespace tactical_objects {

using json = nlohmann::json;

namespace {
const char* TOPIC_OBSERVATION_INGRESS = "observation_ingress";
const char* SVC_CREATE_OBJECT = "create_object";
const char* SVC_UPDATE_OBJECT = "update_object";
const char* SVC_DELETE_OBJECT = "delete_object";
const char* SVC_QUERY = "query";
const char* SVC_GET_OBJECT = "get_object";
const char* SVC_UPSERT_ZONE = "upsert_zone";
const char* SVC_REMOVE_ZONE = "remove_zone";
const char* TYPE_JSON = "application/json";
} // namespace

TacticalObjectsComponent::TacticalObjectsComponent()
  : pcl::Component("tactical_objects"),
    runtime_(std::make_shared<TacticalObjectsRuntime>()) {}

pcl_status_t TacticalObjectsComponent::on_configure() {
  addSubscriber(TOPIC_OBSERVATION_INGRESS, TYPE_JSON, onObservationIngress, this);

  if (!addService(SVC_CREATE_OBJECT, TYPE_JSON, handleCreateObject, this))
    return PCL_ERR_CALLBACK;
  if (!addService(SVC_UPDATE_OBJECT, TYPE_JSON, handleUpdateObject, this))
    return PCL_ERR_CALLBACK;
  if (!addService(SVC_DELETE_OBJECT, TYPE_JSON, handleDeleteObject, this))
    return PCL_ERR_CALLBACK;
  if (!addService(SVC_QUERY, TYPE_JSON, handleQuery, this))
    return PCL_ERR_CALLBACK;
  if (!addService(SVC_GET_OBJECT, TYPE_JSON, handleGetObject, this))
    return PCL_ERR_CALLBACK;
  if (!addService(SVC_UPSERT_ZONE, TYPE_JSON, handleUpsertZone, this))
    return PCL_ERR_CALLBACK;
  if (!addService(SVC_REMOVE_ZONE, TYPE_JSON, handleRemoveZone, this))
    return PCL_ERR_CALLBACK;

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
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::on_shutdown() {
  return PCL_OK;
}

pcl_status_t TacticalObjectsComponent::on_tick(double dt) {
  (void)dt;
  return PCL_OK;
}

void TacticalObjectsComponent::onObservationIngress(pcl_container_t*, const pcl_msg_t* msg,
                                                   void* user_data) {
  auto* comp = static_cast<TacticalObjectsComponent*>(user_data);
  if (!msg->data || msg->size == 0) return;

  std::string str(static_cast<const char*>(msg->data), msg->size);
  json j;
  try {
    j = json::parse(str);
  } catch (...) {
    return;
  }

  ObservationBatch batch = TacticalObjectsCodec::decodeObservationBatch(j);
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

} // namespace tactical_objects
