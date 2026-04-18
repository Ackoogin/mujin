#include <StandardBridge.h>

#include "pyramid_data_model_tactical_codec.hpp"
#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_log.h>
#include <uuid/UUIDHelper.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace tactical_objects {

namespace prov = pyramid::services::tactical_objects::provided;
namespace cons = pyramid::services::tactical_objects::consumed;
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
namespace protobuf_codec = pyramid::services::tactical_objects::protobuf_codec;
namespace tactical_codec = pyramid::data_model::tactical;
namespace data_model = pyramid::data_model;
using pyramid::core::uuid::UUIDHelper;

namespace {

constexpr const char* kJsonContentType = "application/json";
constexpr const char* kFlatBuffersContentType = "application/flatbuffers";
constexpr const char* kProtobufContentType = "application/protobuf";

bool is_flatbuffers_content_type(const char* content_type) {
  return content_type && std::strcmp(content_type, kFlatBuffersContentType) == 0;
}

bool is_protobuf_content_type(const char* content_type) {
  return content_type && std::strcmp(content_type, kProtobufContentType) == 0;
}

bool is_json_content_type(const char* content_type) {
  return !content_type || std::strcmp(content_type, kJsonContentType) == 0;
}

data_model::PolyArea make_poly_area(const BoundingBox& bbox) {
  data_model::PolyArea area;
  area.points.push_back({bbox.min_lat, bbox.min_lon});
  area.points.push_back({bbox.min_lat, bbox.max_lon});
  area.points.push_back({bbox.max_lat, bbox.max_lon});
  area.points.push_back({bbox.max_lat, bbox.min_lon});
  return area;
}

data_model::BattleDimension standard_battle_dimension(BattleDimension dim) {
  const int ordinal = static_cast<int>(dim);
  if (ordinal >= static_cast<int>(data_model::BattleDimension::Unspecified) &&
      ordinal <= static_cast<int>(data_model::BattleDimension::Unknown)) {
    return static_cast<data_model::BattleDimension>(ordinal);
  }
  return data_model::BattleDimension::Unknown;
}

BattleDimension internal_battle_dimension(data_model::BattleDimension dim) {
  const int ordinal = static_cast<int>(dim);
  if (ordinal >= static_cast<int>(BattleDimension::Unspecified) &&
      ordinal <= static_cast<int>(BattleDimension::Unknown)) {
    return static_cast<BattleDimension>(ordinal);
  }
  return BattleDimension::Unknown;
}

data_model::StandardIdentity standard_identity(Affiliation affiliation) {
  return static_cast<data_model::StandardIdentity>(static_cast<int>(affiliation));
}

Affiliation internal_affiliation(data_model::StandardIdentity identity) {
  return static_cast<Affiliation>(static_cast<int>(identity));
}

data_model::ObjectSource object_source_from_system(const std::string& source_system) {
  std::string lowered = source_system;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered.find("radar") != std::string::npos) {
    return data_model::ObjectSource::Radar;
  }
  if (!lowered.empty()) {
    return data_model::ObjectSource::Local;
  }
  return data_model::ObjectSource::Unspecified;
}

double velocity_speed(const Velocity& velocity) {
  return std::sqrt(velocity.north * velocity.north +
                   velocity.east * velocity.east +
                   velocity.down * velocity.down);
}

double velocity_course_rad(const Velocity& velocity) {
  double course = std::atan2(velocity.east, velocity.north);
  if (course < 0.0) {
    course += 2.0 * 3.14159265358979323846;
  }
  return course;
}

double earliest_first_seen(const std::vector<SourceRef>& refs, double fallback) {
  double value = std::numeric_limits<double>::max();
  for (const auto& ref : refs) {
    if (ref.first_seen > 0.0) {
      value = std::min(value, ref.first_seen);
    }
  }
  return (value == std::numeric_limits<double>::max()) ? fallback : value;
}

std::vector<SourceRef> collect_source_refs(const TacticalObjectsRuntime& runtime,
                                           const UUIDKey& id) {
  std::vector<SourceRef> refs;
  if (const auto* cc = runtime.store()->correlation().get(id)) {
    refs.insert(refs.end(), cc->source_refs.begin(), cc->source_refs.end());
  }
  if (const auto* ic = runtime.store()->identities().get(id)) {
    refs.insert(refs.end(), ic->source_refs.begin(), ic->source_refs.end());
  }
  return refs;
}

std::vector<data_model::ObjectSource> standard_sources(const TacticalObjectsRuntime& runtime,
                                                       const UUIDKey& id) {
  std::vector<data_model::ObjectSource> sources;
  for (const auto& ref : collect_source_refs(runtime, id)) {
    const auto mapped = object_source_from_system(ref.source_system);
    if (mapped == data_model::ObjectSource::Unspecified) continue;
    if (std::find(sources.begin(), sources.end(), mapped) == sources.end()) {
      sources.push_back(mapped);
    }
  }
  if (sources.empty()) {
    sources.push_back(data_model::ObjectSource::Local);
  }
  return sources;
}

InterestCriteria criteria_from_standard(const data_model::ObjectInterestRequirement& req,
                                        double* expires_at = nullptr) {
  InterestCriteria criteria;
  criteria.query_mode =
      (req.policy == data_model::DataPolicy::Obtain)
          ? tl::optional<QueryMode>(QueryMode::ActiveFind)
          : tl::optional<QueryMode>(QueryMode::ReadCurrent);

  if (!req.dimension.empty() &&
      req.dimension.front() != data_model::BattleDimension::Unspecified) {
    criteria.battle_dimension = internal_battle_dimension(req.dimension.front());
  }

  auto set_bbox = [&](double min_lat, double max_lat, double min_lon, double max_lon) {
    BoundingBox bbox;
    bbox.min_lat = min_lat;
    bbox.max_lat = max_lat;
    bbox.min_lon = min_lon;
    bbox.max_lon = max_lon;
    criteria.area = bbox;
  };

  if (req.point.has_value()) {
    const auto& p = req.point->position;
    set_bbox(p.latitude, p.latitude, p.longitude, p.longitude);
  } else if (req.circle_area.has_value()) {
    const auto& c = req.circle_area.value();
    set_bbox(c.position.latitude - c.radius,
             c.position.latitude + c.radius,
             c.position.longitude - c.radius,
             c.position.longitude + c.radius);
  } else if (req.poly_area.has_value() && !req.poly_area->points.empty()) {
    double min_lat = req.poly_area->points.front().latitude;
    double max_lat = min_lat;
    double min_lon = req.poly_area->points.front().longitude;
    double max_lon = min_lon;
    for (const auto& point : req.poly_area->points) {
      min_lat = std::min(min_lat, point.latitude);
      max_lat = std::max(max_lat, point.latitude);
      min_lon = std::min(min_lon, point.longitude);
      max_lon = std::max(max_lon, point.longitude);
    }
    set_bbox(min_lat, max_lat, min_lon, max_lon);
  }

  if (expires_at) {
    *expires_at = 9999.0;
  }
  return criteria;
}

data_model::ObjectInterestRequirement standard_requirement_from_interest(const InterestRecord& rec) {
  data_model::ObjectInterestRequirement req;
  req.base.id = UUIDHelper::toString(rec.interest_id.uuid);
  req.policy =
      (rec.criteria.query_mode.has_value() &&
       *rec.criteria.query_mode == QueryMode::ActiveFind)
          ? data_model::DataPolicy::Obtain
          : data_model::DataPolicy::Query;

  if (rec.criteria.battle_dimension.has_value()) {
    req.dimension.push_back(standard_battle_dimension(*rec.criteria.battle_dimension));
  }
  if (rec.criteria.area.has_value()) {
    req.poly_area = make_poly_area(*rec.criteria.area);
  }
  return req;
}

data_model::ObjectEvidenceRequirement standard_evidence_requirement(
    const DerivedEvidenceRequirement& requirement) {
  data_model::ObjectEvidenceRequirement req;
  req.base.id = UUIDHelper::toString(requirement.requirement_id.uuid);
  req.base.source = UUIDHelper::toString(requirement.source_interest_id.uuid);
  req.policy = data_model::DataPolicy::Query;
  if (requirement.criteria.battle_dimension.has_value()) {
    req.dimension.push_back(
        standard_battle_dimension(*requirement.criteria.battle_dimension));
  }
  if (requirement.criteria.area.has_value()) {
    req.poly_area = make_poly_area(*requirement.criteria.area);
  }
  return req;
}

data_model::ObjectDetail standard_detail_from_object(const TacticalObjectsRuntime& runtime,
                                                     const UUIDKey& id) {
  data_model::ObjectDetail detail;
  detail.id = UUIDHelper::toString(id.uuid);

  if (const auto* lifecycle = runtime.store()->lifecycle().get(id)) {
    detail.update_time = lifecycle->last_update_timestamp;
  }

  const auto refs = collect_source_refs(runtime, id);
  if (!refs.empty()) {
    detail.entity_source = refs.front().source_system;
    detail.creation_time = earliest_first_seen(refs, detail.update_time.value_or(0.0));
  } else {
    detail.creation_time = detail.update_time.value_or(0.0);
  }
  detail.source = standard_sources(runtime, id);

  if (const auto* kc = runtime.store()->kinematics().get(id)) {
    detail.position.latitude = kc->position.lat;
    detail.position.longitude = kc->position.lon;
    detail.course = velocity_course_rad(kc->velocity);
    detail.speed = velocity_speed(kc->velocity);
  }

  if (const auto* qc = runtime.store()->quality().get(id)) {
    detail.quality = qc->confidence;
  }

  if (const auto* mc = runtime.store()->milclass().get(id)) {
    detail.identity = standard_identity(mc->profile.affiliation);
    detail.dimension = standard_battle_dimension(mc->profile.battle_dim);
  }

  return detail;
}

data_model::ObjectMatch standard_match_from_object(const TacticalObjectsRuntime& runtime,
                                                   const UUIDKey& id,
                                                   const char* default_source) {
  data_model::ObjectMatch match;
  match.id = UUIDHelper::toString(id.uuid);
  match.matching_object_id = match.id;
  if (const auto* lifecycle = runtime.store()->lifecycle().get(id)) {
    match.update_time = lifecycle->last_update_timestamp;
  }

  const auto refs = collect_source_refs(runtime, id);
  match.source = refs.empty() ? std::string(default_source) : refs.front().source_system;
  return match;
}

std::string encode_match_array(const std::vector<data_model::ObjectMatch>& matches,
                               const char* content_type) {
  if (is_flatbuffers_content_type(content_type)) {
    return flatbuffers_codec::toBinary(matches);
  }
  if (is_protobuf_content_type(content_type)) {
    return protobuf_codec::toBinary(matches);
  }
  std::string payload = "[";
  bool first = true;
  for (const auto& match : matches) {
    if (!first) payload += ",";
    first = false;
    payload += tactical_codec::toJson(match);
  }
  payload += "]";
  return payload;
}

std::string encode_evidence_requirement(const data_model::ObjectEvidenceRequirement& req,
                                        const char* content_type) {
  if (is_flatbuffers_content_type(content_type)) {
    return flatbuffers_codec::toBinary(req);
  }
  if (is_protobuf_content_type(content_type)) {
    return protobuf_codec::toBinary(req);
  }
  return tactical_codec::toJson(req);
}

bool decode_object_evidence(const pcl_msg_t* msg, data_model::ObjectDetail& detail) {
  if (!msg || !msg->data || msg->size == 0) {
    return false;
  }
  try {
    if (is_flatbuffers_content_type(msg->type_name)) {
      detail = flatbuffers_codec::fromBinaryObjectDetail(msg->data, msg->size);
    } else if (is_protobuf_content_type(msg->type_name)) {
      detail = protobuf_codec::fromBinaryObjectDetail(msg->data, msg->size);
    } else {
      detail = tactical_codec::fromJson(
          std::string(static_cast<const char*>(msg->data), msg->size),
          static_cast<data_model::ObjectDetail*>(nullptr));
    }
    return true;
  } catch (...) {
    return false;
  }
}

} // namespace

class BridgeServiceHandler : public prov::ServiceHandler {
public:
  explicit BridgeServiceHandler(StandardBridge& bridge) : bridge_(bridge) {}

  data_model::Identifier
  handleCreateRequirement(const data_model::ObjectInterestRequirement& request) override;

  std::vector<data_model::ObjectInterestRequirement>
  handleReadRequirement(const data_model::Query& request) override;

  data_model::Ack
  handleUpdateRequirement(const data_model::ObjectInterestRequirement& request) override;

  data_model::Ack
  handleDeleteRequirement(const data_model::Identifier& request) override;

  std::vector<data_model::ObjectMatch>
  handleReadMatch(const data_model::Query& request) override;

  std::vector<data_model::ObjectDetail>
  handleReadDetail(const data_model::Query& request) override;

private:
  StandardBridge& bridge_;
};

StandardBridge::StandardBridge(TacticalObjectsRuntime& runtime, pcl_executor_t* exec,
                               std::string frontend_content_type,
                               bool expose_consumed_interface)
    : pcl::Component("standard_bridge"),
      runtime_(runtime),
      exec_(exec),
      frontend_content_type_(std::move(frontend_content_type)),
      expose_consumed_interface_(expose_consumed_interface) {}

pcl_status_t StandardBridge::on_configure() {
  if (!addService(prov::kSvcCreateRequirement, frontend_content_type_.c_str(),
                  handleCreateRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcReadRequirement, frontend_content_type_.c_str(),
                  handleReadRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcUpdateRequirement, frontend_content_type_.c_str(),
                  handleUpdateRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcDeleteRequirement, frontend_content_type_.c_str(),
                  handleDeleteRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcReadMatch, frontend_content_type_.c_str(),
                  handleReadMatch, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcReadDetail, frontend_content_type_.c_str(),
                  handleReadDetail, this)) {
    return PCL_ERR_CALLBACK;
  }

  pub_entity_matches_ =
      addPublisher(prov::kTopicEntityMatches, frontend_content_type_.c_str());

  if (expose_consumed_interface_) {
    addSubscriber(cons::kTopicObjectEvidence, frontend_content_type_.c_str(),
                  onStandardObjectEvidence, this);
    pub_evidence_reqs_ =
        addPublisher(prov::kTopicEvidenceRequirements, frontend_content_type_.c_str());
  }

  return PCL_OK;
}

pcl_status_t StandardBridge::on_activate() {
  return PCL_OK;
}

bool StandardBridge::supportsContentType(const char* content_type) const {
  if (is_json_content_type(content_type)) {
    return frontend_content_type_ == kJsonContentType;
  }
  if (is_flatbuffers_content_type(content_type)) {
    return frontend_content_type_ == kFlatBuffersContentType;
  }
  if (is_protobuf_content_type(content_type)) {
    return frontend_content_type_ == kProtobufContentType;
  }
  return false;
}

pcl_status_t StandardBridge::dispatchProvidedService(int channel, const pcl_msg_t* request,
                                                     pcl_msg_t* response) {
  if (!supportsContentType(request ? request->type_name : nullptr)) {
    return PCL_ERR_INVALID;
  }

  BridgeServiceHandler handler(*this);
  void* response_buf = nullptr;
  size_t response_size = 0;
  prov::dispatch(handler, static_cast<prov::ServiceChannel>(channel),
                 request ? request->data : nullptr,
                 request ? request->size : 0u,
                 frontend_content_type_.c_str(),
                 &response_buf, &response_size);

  response_buffer_.clear();
  if (response_buf && response_size > 0) {
    response_buffer_.assign(static_cast<const char*>(response_buf), response_size);
    std::free(response_buf);
  }

  response->data = response_buffer_.empty()
                       ? nullptr
                       : const_cast<char*>(response_buffer_.data());
  response->size = static_cast<uint32_t>(response_buffer_.size());
  response->type_name = frontend_content_type_.c_str();
  return PCL_OK;
}

pcl_status_t StandardBridge::handleCreateRequirement(pcl_container_t*,
                                                     const pcl_msg_t* request,
                                                     pcl_msg_t* response,
                                                     pcl_svc_context_t*,
                                                     void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::CreateRequirement), request, response);
}

pcl_status_t StandardBridge::handleReadRequirement(pcl_container_t*,
                                                   const pcl_msg_t* request,
                                                   pcl_msg_t* response,
                                                   pcl_svc_context_t*,
                                                   void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::ReadRequirement), request, response);
}

pcl_status_t StandardBridge::handleUpdateRequirement(pcl_container_t*,
                                                     const pcl_msg_t* request,
                                                     pcl_msg_t* response,
                                                     pcl_svc_context_t*,
                                                     void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::UpdateRequirement), request, response);
}

pcl_status_t StandardBridge::handleDeleteRequirement(pcl_container_t*,
                                                     const pcl_msg_t* request,
                                                     pcl_msg_t* response,
                                                     pcl_svc_context_t*,
                                                     void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::DeleteRequirement), request, response);
}

pcl_status_t StandardBridge::handleReadMatch(pcl_container_t*,
                                             const pcl_msg_t* request,
                                             pcl_msg_t* response,
                                             pcl_svc_context_t*,
                                             void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::ReadMatch), request, response);
}

pcl_status_t StandardBridge::handleReadDetail(pcl_container_t*,
                                              const pcl_msg_t* request,
                                              pcl_msg_t* response,
                                              pcl_svc_context_t*,
                                              void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::ReadDetail), request, response);
}

void StandardBridge::publishEntityMatches(const std::vector<std::string>& entity_ids) {
  if (!pub_entity_matches_ || entity_ids.empty()) {
    return;
  }

  std::vector<data_model::ObjectMatch> matches;
  matches.reserve(entity_ids.size());
  for (const auto& id_str : entity_ids) {
    const auto parsed = UUIDHelper::fromString(id_str);
    if (!parsed.second) continue;
    matches.push_back(
        standard_match_from_object(runtime_, UUIDKey(parsed.first), "tactical_objects"));
  }
  if (matches.empty()) {
    return;
  }

  publish_buffer_ = encode_match_array(matches, frontend_content_type_.c_str());
  pcl_msg_t out{};
  out.data = publish_buffer_.data();
  out.size = static_cast<uint32_t>(publish_buffer_.size());
  out.type_name = frontend_content_type_.c_str();
  pcl_port_publish(pub_entity_matches_, &out);
}

void StandardBridge::publishEvidenceRequirement(const std::string& payload) {
  if (!pub_evidence_reqs_ || payload.empty()) {
    return;
  }
  publish_buffer_ = payload;
  pcl_msg_t out{};
  out.data = publish_buffer_.data();
  out.size = static_cast<uint32_t>(publish_buffer_.size());
  out.type_name = frontend_content_type_.c_str();
  pcl_port_publish(pub_evidence_reqs_, &out);
}

pcl_status_t StandardBridge::on_tick(double dt) {
  if (!pub_entity_matches_ || interests_.empty()) {
    return PCL_OK;
  }

  for (const auto& interest : interests_) {
    const auto stream = runtime_.assembleStreamFrame(interest.first, interest.second, dt);
    if (stream.updates.empty()) continue;

    std::vector<std::string> updated_ids;
    updated_ids.reserve(stream.updates.size());
    for (const auto& update : stream.updates) {
      if (update.message_type == STREAM_MSG_ENTITY_DELETE) continue;
      updated_ids.push_back(UUIDHelper::toString(update.entity_id.uuid));
    }
    publishEntityMatches(updated_ids);
  }

  return PCL_OK;
}

void StandardBridge::onStandardObjectEvidence(pcl_container_t*, const pcl_msg_t* msg,
                                              void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  data_model::ObjectDetail detail;
  if (!decode_object_evidence(msg, detail)) {
    return;
  }

  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.received_at = detail.update_time.value_or(0.0);
  obs.observed_at = detail.creation_time;
  obs.object_hint_type = ObjectType::Platform;
  obs.affiliation_hint = internal_affiliation(detail.identity);
  obs.position.lat = detail.position.latitude;
  obs.position.lon = detail.position.longitude;
  obs.position.alt = 0.0;
  obs.confidence = detail.quality.value_or(0.0);

  const auto dim = internal_battle_dimension(detail.dimension);
  switch (dim) {
    case BattleDimension::SeaSurface: obs.source_sidc = "SHSP------*****"; break;
    case BattleDimension::Air: obs.source_sidc = "SHAP------*****"; break;
    case BattleDimension::Subsurface: obs.source_sidc = "SHUP------*****"; break;
    case BattleDimension::Ground: obs.source_sidc = "SHGP------*****"; break;
    default: break;
  }

  if (!detail.entity_source.empty()) {
    obs.source_ref.source_system = detail.entity_source;
  }

  ObservationBatch batch;
  batch.observations.push_back(obs);
  self->runtime_.processObservationBatch(batch);
}

data_model::Identifier BridgeServiceHandler::handleCreateRequirement(
    const data_model::ObjectInterestRequirement& request) {
  double expires_at = 0.0;
  const auto criteria = criteria_from_standard(request, &expires_at);
  const auto interest_id =
      bridge_.runtime_.interestManager().registerInterest(criteria, 0.0, expires_at);

  const auto existing = std::find_if(
      bridge_.interests_.begin(), bridge_.interests_.end(),
      [&](const std::pair<UUIDKey, SubscriptionHandle>& item) {
        return item.first == interest_id;
      });
  if (existing == bridge_.interests_.end()) {
    const auto handle = bridge_.runtime_.registerStreamSubscriber(
        interest_id, [](const UUIDKey&, const UUIDKey&) {});
    bridge_.interests_.push_back({interest_id, handle});
  }

  if (bridge_.expose_consumed_interface_ &&
      criteria.query_mode.has_value() &&
      *criteria.query_mode == QueryMode::ActiveFind) {
    const auto solution = bridge_.runtime_.determineSolution(interest_id);
    for (const auto& requirement : solution.evidence_requirements) {
      const auto standard_req = standard_evidence_requirement(requirement);
      const auto payload =
          encode_evidence_requirement(standard_req, bridge_.frontend_content_type_.c_str());
      bridge_.publishEvidenceRequirement(payload);

      pcl_msg_t req{};
      req.data = const_cast<char*>(payload.data());
      req.size = static_cast<uint32_t>(payload.size());
      req.type_name = bridge_.frontend_content_type_.c_str();

      pcl_msg_t resp{};
      char resp_buf[1024] = {};
      resp.data = resp_buf;
      resp.size = sizeof(resp_buf);
      resp.type_name = bridge_.frontend_content_type_.c_str();
      pcl_executor_invoke_service(bridge_.exec_, cons::kSvcCreateRequirement, &req, &resp);
    }
  }

  return UUIDHelper::toString(interest_id.uuid);
}

std::vector<data_model::ObjectInterestRequirement>
BridgeServiceHandler::handleReadRequirement(const data_model::Query& request) {
  std::vector<data_model::ObjectInterestRequirement> requirements;

  auto append_interest = [&](const UUIDKey& interest_id) {
    const auto* rec = bridge_.runtime_.interestManager().get(interest_id);
    if (!rec) return;
    requirements.push_back(standard_requirement_from_interest(*rec));
  };

  if (request.id.empty()) {
    for (const auto& record : bridge_.runtime_.interestManager().activeInterests()) {
      requirements.push_back(standard_requirement_from_interest(record));
    }
    return requirements;
  }

  for (const auto& id : request.id) {
    const auto parsed = UUIDHelper::fromString(id);
    if (!parsed.second) continue;
    append_interest(UUIDKey(parsed.first));
  }
  return requirements;
}

data_model::Ack BridgeServiceHandler::handleUpdateRequirement(
    const data_model::ObjectInterestRequirement& request) {
  const auto parsed = UUIDHelper::fromString(request.base.id);
  if (!parsed.second) {
    return data_model::kAckFail;
  }

  double expires_at = 0.0;
  const auto criteria = criteria_from_standard(request, &expires_at);
  const bool ok = bridge_.runtime_.interestManager().updateInterest(
      UUIDKey(parsed.first), criteria, expires_at);
  if (!ok) {
    return data_model::kAckFail;
  }

  const auto existing = std::find_if(
      bridge_.interests_.begin(), bridge_.interests_.end(),
      [&](const std::pair<UUIDKey, SubscriptionHandle>& item) {
        return item.first == UUIDKey(parsed.first);
      });
  if (existing == bridge_.interests_.end()) {
    const auto handle = bridge_.runtime_.registerStreamSubscriber(
        UUIDKey(parsed.first), [](const UUIDKey&, const UUIDKey&) {});
    bridge_.interests_.push_back({UUIDKey(parsed.first), handle});
  }

  if (bridge_.expose_consumed_interface_ &&
      criteria.query_mode.has_value() &&
      *criteria.query_mode == QueryMode::ActiveFind) {
    const auto solution = bridge_.runtime_.determineSolution(UUIDKey(parsed.first));
    for (const auto& requirement : solution.evidence_requirements) {
      const auto standard_req = standard_evidence_requirement(requirement);
      bridge_.publishEvidenceRequirement(
          encode_evidence_requirement(standard_req, bridge_.frontend_content_type_.c_str()));
    }
  }

  return data_model::kAckOk;
}

data_model::Ack BridgeServiceHandler::handleDeleteRequirement(
    const data_model::Identifier& request) {
  const auto parsed = UUIDHelper::fromString(request);
  if (!parsed.second) {
    return data_model::kAckFail;
  }

  const UUIDKey interest_id(parsed.first);
  const bool ok = bridge_.runtime_.interestManager().cancelInterest(interest_id);
  if (!ok) {
    return data_model::kAckFail;
  }

  auto it = std::find_if(
      bridge_.interests_.begin(), bridge_.interests_.end(),
      [&](const std::pair<UUIDKey, SubscriptionHandle>& item) {
        return item.first == interest_id;
      });
  if (it != bridge_.interests_.end()) {
    bridge_.runtime_.removeStreamSubscriber(it->second);
    bridge_.interests_.erase(it);
  }

  return data_model::kAckOk;
}

std::vector<data_model::ObjectMatch>
BridgeServiceHandler::handleReadMatch(const data_model::Query& request) {
  std::vector<data_model::ObjectMatch> matches;

  auto append_query = [&](const QueryRequest& qreq) {
    const auto response = bridge_.runtime_.query(qreq);
    for (const auto& entry : response.entries) {
      matches.push_back(
          standard_match_from_object(bridge_.runtime_, entry.id, "tactical_objects"));
    }
  };

  if (request.id.empty()) {
    append_query(QueryRequest{});
    return matches;
  }

  for (const auto& id : request.id) {
    const auto parsed = UUIDHelper::fromString(id);
    if (!parsed.second) continue;
    QueryRequest qreq;
    qreq.by_uuid = UUIDKey(parsed.first);
    append_query(qreq);
  }
  return matches;
}

std::vector<data_model::ObjectDetail>
BridgeServiceHandler::handleReadDetail(const data_model::Query& request) {
  std::vector<data_model::ObjectDetail> details;

  auto append_detail = [&](const UUIDKey& id) {
    if (!bridge_.runtime_.getRecord(id)) return;
    details.push_back(standard_detail_from_object(bridge_.runtime_, id));
  };

  if (request.id.empty()) {
    bridge_.runtime_.store()->forEachObject([&](const EntityRecord& rec) {
      append_detail(rec.id);
    });
    return details;
  }

  for (const auto& id : request.id) {
    const auto parsed = UUIDHelper::fromString(id);
    if (!parsed.second) continue;
    append_detail(UUIDKey(parsed.first));
  }
  return details;
}

} // namespace tactical_objects
