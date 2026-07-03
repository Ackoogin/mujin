/// \file sdk_tactical_objects_fail_closed_smoke.cpp
/// \brief Offline SDK smoke test for generated facade fail-closed behavior.

#include <pcl/pcl_types.h>

#include <cstdint>
#include <iostream>
#include <string>

#include "pyramid_services_tactical_objects_consumed.hpp"

namespace cons = pyramid::components::tactical_objects::services::consumed;
namespace types = pyramid::domain_model;

namespace {

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "[sdk_tactical_objects_fail_closed_smoke] FAIL: " << message
              << '\n';
  }
  return condition;
}

}  // namespace

int main() {
  constexpr const char* kMissingContentType = "application/no-pyramid-codec";

  types::ObjectDetail evidence;
  evidence.id = "sdk-smoke-missing-codec";

  std::string encoded;
  bool ok = expect(!cons::encodeObjectEvidence(evidence, kMissingContentType,
                                               &encoded),
                   "encodeObjectEvidence unexpectedly succeeded");
  ok = expect(encoded.empty(), "failed encode produced a payload") && ok;

  const char payload[] = "{}";
  pcl_msg_t msg{};
  msg.data = payload;
  msg.size = static_cast<uint32_t>(sizeof(payload) - 1u);
  msg.type_name = kMissingContentType;

  types::ObjectDetail decoded;
  ok = expect(!cons::decodeObjectEvidence(&msg, &decoded),
              "decodeObjectEvidence unexpectedly succeeded") &&
       ok;

  if (ok) {
    std::cout << "[sdk_tactical_objects_fail_closed_smoke] PASS\n";
    return 0;
  }
  return 1;
}
