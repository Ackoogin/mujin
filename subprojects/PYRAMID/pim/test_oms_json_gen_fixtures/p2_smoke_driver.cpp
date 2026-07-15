// Test driver for the generated P2 (A-GRA 5.0a) OMS-JSON codec plugin
// (pim/test_oms_json_gen.py).  One mode:
//
//   --roundtrip <RootId>  Read one OMS JSON document (as produced by the
//                         la-cal-harness XSD-derived generator, with UUIDs
//                         normalized to the A-GRA hexBinary form) on stdin,
//                         decode it by root id through the plugin, re-encode,
//                         and print the wire JSON.  The Python side asserts
//                         semantic equality with the schema-derived input.
//
// This mirrors p1_smoke_driver.cpp's --roundtrip mode over the 20 roots of
// the p2_agra_planning_core profile.  There is no native self-test here: the
// P1 driver already pins the hand-built encode path, and the shapes this
// profile adds are covered by the fixture tests in pim/test_oms_json_gen.py.
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

int main(int argc, char** argv) {
  if (argc == 3 && std::strcmp(argv[1], "--roundtrip") == 0) {
    return roundtrip(argv[2]);
  }
  std::fputs("usage: p2_smoke_driver --roundtrip <RootId>\n", stderr);
  return 2;
}
