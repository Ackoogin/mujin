/// \file test_plugin_only_fail_closed.cpp
/// \brief Verifies generated facades fail closed without a registered codec.

#include <gtest/gtest.h>

#include "pyramid_services_tactical_objects_consumed.hpp"

namespace cons = pyramid::components::tactical_objects::services::consumed;
namespace types = pyramid::domain_model;

TEST(PluginOnly, FacadeFailsClosedWithoutCodec) {
  constexpr const char* kMissingContentType = "application/no-pyramid-codec";

  types::ObjectDetail evidence;
  evidence.id = "missing-codec-object";

  std::string encoded;
  EXPECT_FALSE(cons::encodeObjectEvidence(
      evidence, kMissingContentType, &encoded));
  EXPECT_TRUE(encoded.empty());

  const char payload[] = "{}";
  pcl_msg_t msg{};
  msg.data = payload;
  msg.size = static_cast<uint32_t>(sizeof(payload) - 1u);
  msg.type_name = kMissingContentType;

  types::ObjectDetail decoded;
  EXPECT_FALSE(cons::decodeObjectEvidence(&msg, &decoded));
}
