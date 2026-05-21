#include "tobj_evidence_provider.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace tobj_example {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

void log(const char* msg) {
  std::fprintf(stderr, "[evidence_provider] %s\n", msg);
  std::fflush(stderr);
}

}  // namespace

void TobjEvidenceProvider::onEvidenceRequirement(const pcl_msg_t* msg) {
  if (!msg || !msg->data || msg->size == 0) return;

  evidence_req_received_ = true;

  ObjectEvidenceRequirement req;
  if (!Provided::decodeEvidenceRequirements(msg, &req)) {
    return;
  }
  (void)req;

  if (observation_sent_ || !publisher_) return;

  // Position: 51 degN 0 degE; HOSTILE; SEA_SURFACE.
  ObjectDetail obs;
  obs.id                 = "obj-1";
  obs.identity           = StandardIdentity::Hostile;
  obs.dimension          = BattleDimension::SeaSurface;
  obs.position.latitude  = 51.0 * kDegToRad;
  obs.position.longitude = 0.0;
  obs.quality            = 0.9;
  obs.creation_time      = 0.5;

  log(("Publishing standard observation on "
       + std::string(Consumed::kTopicObjectEvidence)).c_str());

  Consumed::publishObjectEvidence(publisher_, obs, content_type_.c_str());
  observation_sent_ = true;
  log("Standard observation published");
}

}  // namespace tobj_example
