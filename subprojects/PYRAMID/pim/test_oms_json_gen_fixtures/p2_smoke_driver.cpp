// Test driver for the generated P2 (A-GRA 5.0a) OMS-JSON codec plugin
// (pim/test_oms_json_gen.py).  Two modes:
//
//   (no args)             self-test: build an MA_MissionPlanCommandStatusMT
//                         natively, encode -> decode -> re-encode, assert
//                         byte-stable, print the wire JSON + RT_OK.  The
//                         printed document is the parity golden that
//                         p2_ada_parity_driver.adb must reproduce byte for
//                         byte.  Exercises the A-GRA hexBinary UUID form,
//                         dual-id dispatch, sidecar wire names, enum
//                         literals, oneof choice, and omit-empty arrays.
//
//   --roundtrip <RootId>  Read one OMS JSON document (as produced by the
//                         la-cal-harness XSD-derived generator, with UUIDs
//                         normalized to the A-GRA hexBinary form) on stdin,
//                         decode it by root id through the plugin, re-encode,
//                         and print the wire JSON.  The Python side asserts
//                         semantic equality with the schema-derived input.
//
// This mirrors p1_smoke_driver.cpp over the 20 roots of the
// p2_agra_planning_core profile.
#include "pyramid_data_model_agra_types.hpp"
#include "pyramid_data_model_agra_cabi_marshal.hpp"
#include <pcl/pcl_codec.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <string>

extern "C" const pcl_codec_t* pcl_codec_plugin_entry(const char*);
extern "C" void* pcl_alloc(size_t n) { return std::malloc(n); }
extern "C" void pcl_free(void* p) { std::free(p); }

namespace agra = pyramid::domain_model::agra;
namespace cabi = pyramid::cabi;

// The P2 profile roots (element name -> C ABI struct type), in the order
// pim/uci_profiles/p2_agra_planning_core.json lists them.
#define P2_ROOTS(X)                                                          \
  X(MA_Action, pyramid_data_model_agra_MA_ActionMT_c)                        \
  X(MA_ActionStatus, pyramid_data_model_agra_MA_ActionStatusMT_c)            \
  X(MA_ApprovalPolicy, pyramid_data_model_agra_MA_ApprovalPolicyMT_c)        \
  X(MA_ApprovalRequest, pyramid_data_model_agra_MA_ApprovalRequestMT_c)      \
  X(MA_ApprovalRequestStatus,                                                \
    pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c)                    \
  X(MA_MissionPlan, pyramid_data_model_agra_MA_MissionPlanMT_c)              \
  X(MA_MissionPlanActivationCommand,                                         \
    pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c)             \
  X(MA_MissionPlanActivationCommandStatus,                                   \
    pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c)       \
  X(MA_MissionPlanActivationStatus,                                          \
    pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c)              \
  X(MA_MissionPlanCommand,                                                   \
    pyramid_data_model_agra_MA_MissionPlanCommandMT_c)                       \
  X(MA_MissionPlanCommandStatus,                                             \
    pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c)                 \
  X(MA_MissionPlanExecutionStatus,                                           \
    pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c)               \
  X(MA_PlanningFunction, pyramid_data_model_agra_MA_PlanningFunctionMT_c)    \
  X(MA_PlanningFunctionSettingsCommand,                                      \
    pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c)          \
  X(MA_PlanningFunctionSettingsCommandStatus,                                \
    pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c)    \
  X(MA_PlanningFunctionStatus,                                               \
    pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c)                   \
  X(MA_Response, pyramid_data_model_agra_MA_ResponseMT_c)                    \
  X(MA_Task, pyramid_data_model_agra_MA_TaskMT_c)                            \
  X(MA_TaskStatus, pyramid_data_model_agra_MA_TaskStatusMT_c)                \
  X(MissionContingencyAlert,                                                 \
    pyramid_data_model_agra_MissionContingencyAlertMT_c)

static int roundtrip(const char* root_id) {
  std::string input((std::istreambuf_iterator<char>(std::cin)),
                    std::istreambuf_iterator<char>());
  const pcl_codec_t* codec = pcl_codec_plugin_entry(nullptr);
  if (!codec) { std::fputs("FAIL: no codec\n", stderr); return 1; }
  pcl_msg_t in{};
  in.data = input.data();
  in.size = static_cast<uint32_t>(input.size());
  in.type_name = codec->content_type;

#define TRY_ROOT(element, ctype)                                             \
  if (std::strcmp(root_id, #element) == 0) {                                 \
    ctype c{};                                                               \
    if (codec->decode(nullptr, #element, &in, &c) != PCL_OK) {               \
      std::fputs("FAIL: decode " #element "\n", stderr);                     \
      return 1;                                                              \
    }                                                                        \
    pcl_msg_t out{};                                                         \
    if (codec->encode(nullptr, #element, &c, &out) != PCL_OK) {              \
      std::fputs("FAIL: encode " #element "\n", stderr);                     \
      return 1;                                                              \
    }                                                                        \
    std::fwrite(out.data, 1, out.size, stdout);                              \
    std::fputc('\n', stdout);                                                \
    return 0;                                                                \
  }
  P2_ROOTS(TRY_ROOT)
#undef TRY_ROOT
  std::fputs("FAIL: unknown root id\n", stderr);
  return 1;
}

// The value the cross-language parity golden pins.  Keep this in step with
// p2_ada_parity_driver.adb: the two must build the same value, and the
// Python side compares both against test_oms_json_gen_fixtures/
// p2_parity_golden.json.  Every UUID here is in the A-GRA 5.0a wire form
// (xs:hexBinary length 16 -- 32 hex digits, no hyphens), which is what
// distinguishes this golden from the P1/UCI 2.5 one.
static int selftest() {
  agra::MA_MissionPlanCommandStatusMT m{};
  m.security_information.classification = agra::ClassificationEnum::U;
  agra::OwnerProducerChoiceType op{};
  op.government_identifier = agra::OwnerProducerEnum::Usa;
  m.security_information.owner_producer.push_back(op);
  m.message_header.system_id.uuid = "123E4567E89B42D3A456426614174000";
  m.message_header.timestamp = "2026-01-01T00:00:00Z";
  m.message_header.schema_version = "005.0a.ASK";
  m.message_header.mode = agra::MessageModeEnum::Simulation;
  m.message_data.command_id.uuid = "123E4567E89B42D3A456426614174001";
  m.message_data.planning_status.command_processing_state =
      agra::CommandProcessingStateEnum::Received;
  m.message_data.planning_status.command_status =
      agra::ProcessingStatusEnum::Queued;

  pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c c{};
  cabi::to_c(m, &c);

  const pcl_codec_t* codec = pcl_codec_plugin_entry(nullptr);
  if (!codec) { std::puts("FAIL: no codec"); return 1; }

  pcl_msg_t wire{};
  // Root dual-id: both the element name and the message name must resolve.
  if (codec->encode(nullptr, "MA_MissionPlanCommandStatus", &c, &wire)
      != PCL_OK) {
    std::puts("FAIL: encode by element id"); return 1;
  }
  std::string json1(static_cast<const char*>(wire.data), wire.size);
  pcl_msg_t wire2{};
  if (codec->encode(nullptr, "MA_MissionPlanCommandStatusMT", &c, &wire2)
      != PCL_OK) {
    std::puts("FAIL: encode by message id"); return 1;
  }
  std::string json_msgid(static_cast<const char*>(wire2.data), wire2.size);
  if (json1 != json_msgid) {
    std::puts("FAIL: dual-id outputs differ"); return 1;
  }

  pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c back{};
  if (codec->decode(nullptr, "MA_MissionPlanCommandStatus", &wire, &back)
      != PCL_OK) {
    std::puts("FAIL: decode"); return 1;
  }
  pcl_msg_t wire3{};
  if (codec->encode(nullptr, "MA_MissionPlanCommandStatus", &back, &wire3)
      != PCL_OK) {
    std::puts("FAIL: re-encode"); return 1;
  }
  std::string json2(static_cast<const char*>(wire3.data), wire3.size);
  if (json1 != json2) {
    std::printf("FAIL: round-trip drift\n%s\n%s\n",
                json1.c_str(), json2.c_str());
    return 1;
  }
  std::printf("%s\n", json1.c_str());
  std::puts("RT_OK");
  return 0;
}

// --identity <config_json>: ask the plugin entry point to admit this loader
// configuration.  Prints ADMITTED or REFUSED.  Exit status is 0 either way:
// a refusal is a correct answer, and the Python side asserts which answer it
// expects.  The literal "null" stands for a NULL config pointer, which the
// PCL entry contract defines as "no configuration".
static int identity(const char* config_json) {
  const bool null_config = std::strcmp(config_json, "null") == 0;
  const pcl_codec_t* codec =
      pcl_codec_plugin_entry(null_config ? nullptr : config_json);
  std::puts(codec ? "ADMITTED" : "REFUSED");
  return 0;
}

int main(int argc, char** argv) {
  if (argc == 3 && std::strcmp(argv[1], "--roundtrip") == 0) {
    return roundtrip(argv[2]);
  }
  if (argc == 3 && std::strcmp(argv[1], "--identity") == 0) {
    return identity(argv[2]);
  }
  return selftest();
}
