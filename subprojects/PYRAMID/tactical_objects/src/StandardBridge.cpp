#include <StandardBridge.h>

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_log.h>
#include <uuid/UUIDHelper.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace tactical_objects {

namespace prov = pyramid::components::tactical_objects::services::provided;
namespace cons = pyramid::components::tactical_objects::services::consumed;
namespace data_model = pyramid::domain_model;
using pyramid::core::uuid::UUIDHelper;

namespace {

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

} // namespace

class BridgeServiceHandler : public prov::ServiceHandler {
public:
  explicit BridgeServiceHandler(StandardBridge& bridge) : bridge_(bridge) {}

  data_model::Identifier
  handleObjectOfInterestCreateRequirement(
      const data_model::ObjectInterestRequirement& request) override;

  std::vector<data_model::ObjectInterestRequirement>
  handleObjectOfInterestReadRequirement(
      const data_model::Query& request) override;

  data_model::Ack
  handleObjectOfInterestUpdateRequirement(
      const data_model::ObjectInterestRequirement& request) override;

  data_model::Ack
  handleObjectOfInterestDeleteRequirement(
      const data_model::Identifier& request) override;

  std::vector<data_model::ObjectMatch>
  handleMatchingObjectsReadMatch(const data_model::Query& request) override;

  std::vector<data_model::ObjectDetail>
  handleSpecificObjectDetailReadDetail(
      const data_model::Query& request) override;

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

StandardBridge::StandardBridge(TacticalObjectsRuntime& runtime, pcl_executor_t* exec,
                               bool expose_consumed_interface)
    : StandardBridge(runtime, exec, std::string("application/json"),
                     expose_consumed_interface) {}

pcl_status_t StandardBridge::on_configure() {
  if (!supportsContentType(frontend_content_type_.c_str())) {
    return PCL_ERR_INVALID;
  }

  if (!addService(prov::kSvcObjectOfInterestCreateRequirement, frontend_content_type_.c_str(),
                  handleCreateRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcObjectOfInterestReadRequirement, frontend_content_type_.c_str(),
                  handleReadRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcObjectOfInterestUpdateRequirement, frontend_content_type_.c_str(),
                  handleUpdateRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcObjectOfInterestDeleteRequirement, frontend_content_type_.c_str(),
                  handleDeleteRequirement, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcMatchingObjectsReadMatch, frontend_content_type_.c_str(),
                  handleReadMatch, this)) {
    return PCL_ERR_CALLBACK;
  }
  if (!addService(prov::kSvcSpecificObjectDetailReadDetail, frontend_content_type_.c_str(),
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
  const char* requested = content_type ? content_type : prov::kJsonContentType;
  if (frontend_content_type_ != requested) {
    return false;
  }
  return prov::supportsContentType(requested) &&
         (!expose_consumed_interface_ || cons::supportsContentType(requested));
}

pcl_status_t StandardBridge::dispatchProvidedService(int channel, const pcl_msg_t* request,
                                                     pcl_msg_t* response) {
  if (!supportsContentType(request ? request->type_name : nullptr)) {
    return PCL_ERR_INVALID;
  }

  const auto service_channel = static_cast<prov::ServiceChannel>(channel);
  const bool query_service =
      service_channel == prov::ServiceChannel::MatchingObjectsReadMatch ||
      service_channel == prov::ServiceChannel::ObjectOfInterestReadRequirement ||
      service_channel == prov::ServiceChannel::SpecificObjectDetailReadDetail;
  const bool empty_json_query =
      query_service &&
      frontend_content_type_ == prov::kJsonContentType &&
      (!request || request->size == 0u);
  const std::string empty_query_payload = "{}";
  const void* request_data =
      empty_json_query ? empty_query_payload.data() : (request ? request->data : nullptr);
  const size_t request_size =
      empty_json_query ? empty_query_payload.size() : (request ? request->size : 0u);

  BridgeServiceHandler handler(*this);
  void* response_buf = nullptr;
  size_t response_size = 0;
  prov::dispatch(handler, service_channel, request_data, request_size,
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
      static_cast<int>(prov::ServiceChannel::ObjectOfInterestCreateRequirement),
      request, response);
}

pcl_status_t StandardBridge::handleReadRequirement(pcl_container_t*,
                                                   const pcl_msg_t* request,
                                                   pcl_msg_t* response,
                                                   pcl_svc_context_t*,
                                                   void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::ObjectOfInterestReadRequirement),
      request, response);
}

pcl_status_t StandardBridge::handleUpdateRequirement(pcl_container_t*,
                                                     const pcl_msg_t* request,
                                                     pcl_msg_t* response,
                                                     pcl_svc_context_t*,
                                                     void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::ObjectOfInterestUpdateRequirement),
      request, response);
}

pcl_status_t StandardBridge::handleDeleteRequirement(pcl_container_t*,
                                                     const pcl_msg_t* request,
                                                     pcl_msg_t* response,
                                                     pcl_svc_context_t*,
                                                     void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::ObjectOfInterestDeleteRequirement),
      request, response);
}

pcl_status_t StandardBridge::handleReadMatch(pcl_container_t*,
                                             const pcl_msg_t* request,
                                             pcl_msg_t* response,
                                             pcl_svc_context_t*,
                                             void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::MatchingObjectsReadMatch),
      request, response);
}

pcl_status_t StandardBridge::handleReadDetail(pcl_container_t*,
                                              const pcl_msg_t* request,
                                              pcl_msg_t* response,
                                              pcl_svc_context_t*,
                                              void* user_data) {
  auto* self = static_cast<StandardBridge*>(user_data);
  return self->dispatchProvidedService(
      static_cast<int>(prov::ServiceChannel::SpecificObjectDetailReadDetail),
      request, response);
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

  if (!prov::encodeEntityMatches(matches, frontend_content_type_.c_str(), &publish_buffer_)) {
    return;
  }
  prov::publishEntityMatches(pub_entity_matches_, publish_buffer_, frontend_content_type_.c_str());
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
  if (!cons::decodeObjectEvidence(msg, &detail)) {
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

data_model::Identifier BridgeServiceHandler::handleObjectOfInterestCreateRequirement(
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
      if (bridge_.pub_evidence_reqs_) {
        prov::publishEvidenceRequirements(
            bridge_.pub_evidence_reqs_, standard_req,
            bridge_.frontend_content_type_.c_str());
      }

      cons::invokeObjectSolutionEvidenceCreateRequirement(
          bridge_.exec_, standard_req, bridge_.frontend_content_type_.c_str());
    }
  }

  return UUIDHelper::toString(interest_id.uuid);
}

std::vector<data_model::ObjectInterestRequirement>
BridgeServiceHandler::handleObjectOfInterestReadRequirement(
    const data_model::Query& request) {
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

data_model::Ack BridgeServiceHandler::handleObjectOfInterestUpdateRequirement(
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
      if (bridge_.pub_evidence_reqs_) {
        if (!prov::encodeEvidenceRequirements(
                standard_req, bridge_.frontend_content_type_.c_str(),
                &bridge_.publish_buffer_)) {
          continue;
        }
        prov::publishEvidenceRequirements(
            bridge_.pub_evidence_reqs_, bridge_.publish_buffer_,
            bridge_.frontend_content_type_.c_str());
      }
    }
  }

  return data_model::kAckOk;
}

data_model::Ack BridgeServiceHandler::handleObjectOfInterestDeleteRequirement(
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
BridgeServiceHandler::handleMatchingObjectsReadMatch(
    const data_model::Query& request) {
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
BridgeServiceHandler::handleSpecificObjectDetailReadDetail(
    const data_model::Query& request) {
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
