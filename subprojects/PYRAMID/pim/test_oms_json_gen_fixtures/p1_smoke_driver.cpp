// Round-trip smoke driver for the generated P1 OMS-JSON codec plugin.
// Exercises: inlined envelope (enums), sidecar wire names, nested-base
// flattening (ActionCommandStatusMDT.base), oneof choice (OwnerProducer),
// optionals, arrays.  encode -> decode -> re-encode must be byte-stable.
#include "pyramid_data_model_uci_types.hpp"
#include "pyramid_data_model_uci_cabi_marshal.hpp"
#include <pcl/pcl_codec.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

extern "C" const pcl_codec_t* pcl_codec_plugin_entry(const char*);
extern "C" void* pcl_alloc(size_t n) { return std::malloc(n); }
extern "C" void pcl_free(void* p) { std::free(p); }

namespace uci = pyramid::domain_model::uci;
namespace cabi = pyramid::cabi;

int main() {
  uci::ActionCommandStatusMT m{};
  m.security_information.classification = uci::ClassificationEnum::U;
  uci::OwnerProducerChoiceType op{};
  op.government_identifier = uci::OwnerProducerEnum::Usa;
  m.security_information.owner_producer.push_back(op);
  m.message_header.system_id.uuid = "123e4567-e89b-42d3-a456-426614174000";
  m.message_header.timestamp = "2026-01-01T00:00:00Z";
  m.message_header.schema_version = "002.5.0";
  m.message_header.mode = uci::MessageModeEnum::Simulation;
  m.message_data.base.command_id.uuid = "123e4567-e89b-42d3-a456-426614174001";
  m.message_data.base.command_processing_state =
      uci::CommandProcessingStateEnum::Received;

  pyramid_data_model_uci_ActionCommandStatusMT_c c{};
  cabi::to_c(m, &c);

  const pcl_codec_t* codec = pcl_codec_plugin_entry(nullptr);
  if (!codec) { std::puts("FAIL: no codec"); return 1; }

  pcl_msg_t wire{};
  // Root dual-id: both the element name and the message name must resolve.
  if (codec->encode(nullptr, "ActionCommandStatus", &c, &wire) != PCL_OK) {
    std::puts("FAIL: encode by element id"); return 1;
  }
  std::string json1(static_cast<const char*>(wire.data), wire.size);
  pcl_msg_t wire2{};
  if (codec->encode(nullptr, "ActionCommandStatusMT", &c, &wire2) != PCL_OK) {
    std::puts("FAIL: encode by message id"); return 1;
  }
  std::string json_msgid(static_cast<const char*>(wire2.data), wire2.size);
  if (json1 != json_msgid) { std::puts("FAIL: dual-id outputs differ"); return 1; }

  pyramid_data_model_uci_ActionCommandStatusMT_c back{};
  if (codec->decode(nullptr, "ActionCommandStatus", &wire, &back) != PCL_OK) {
    std::puts("FAIL: decode"); return 1;
  }
  pcl_msg_t wire3{};
  if (codec->encode(nullptr, "ActionCommandStatus", &back, &wire3) != PCL_OK) {
    std::puts("FAIL: re-encode"); return 1;
  }
  std::string json2(static_cast<const char*>(wire3.data), wire3.size);
  if (json1 != json2) {
    std::printf("FAIL: round-trip drift\n%s\n%s\n", json1.c_str(), json2.c_str());
    return 1;
  }
  std::printf("%s\n", json1.c_str());
  std::puts("RT_OK");
  return 0;
}
