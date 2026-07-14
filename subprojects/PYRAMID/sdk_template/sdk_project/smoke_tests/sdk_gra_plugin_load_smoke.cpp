/// \file sdk_gra_plugin_load_smoke.cpp
/// \brief Offline SDK smoke test for the prebuilt OMS/CAL runtime profile.

#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin.h>
#include <pcl/pcl_plugin_loader.h>

#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "[sdk_gra_plugin_load_smoke] FAIL: " << message << '\n';
  }
  return condition;
}

}  // namespace

int main() {
  bool ok = true;

  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  if (!expect(registry != nullptr, "failed to create codec registry")) {
    return 1;
  }

  pcl_plugin_handle_t* codec_handle = nullptr;
  const pcl_status_t codec_rc = pcl_plugin_load_codec(
      PYRAMID_SDK_OMS_CODEC_PATH, nullptr, registry, &codec_handle);
  ok = expect(codec_rc == PCL_OK && codec_handle != nullptr,
              "OMS JSON codec did not load") &&
       ok;
  ok = expect(pcl_codec_registry_get(registry, "application/oms-json") !=
                  nullptr,
              "OMS JSON codec did not register application/oms-json") &&
       ok;

  pcl_codec_registry_clear(registry);
  pcl_plugin_unload(codec_handle);
  pcl_codec_registry_destroy(registry);

  pcl_plugin_handle_t* transport_handle =
      pcl_plugin_open(PYRAMID_SDK_LACAL_TRANSPORT_PATH);
  ok = expect(transport_handle != nullptr,
              "LA-CAL WebSocket transport did not load") &&
       ok;
  if (transport_handle != nullptr) {
    const auto abi = reinterpret_cast<pcl_transport_abi_version_fn>(
        pcl_plugin_symbol(transport_handle,
                          PCL_TRANSPORT_ABI_VERSION_SYMBOL));
    const auto entry = reinterpret_cast<pcl_transport_plugin_entry_fn>(
        pcl_plugin_symbol(transport_handle,
                          PCL_TRANSPORT_PLUGIN_ENTRY_SYMBOL));
    const auto caps = reinterpret_cast<pcl_transport_plugin_caps_fn>(
        pcl_plugin_symbol(transport_handle,
                          PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL));

    ok = expect(abi != nullptr && abi() == PCL_TRANSPORT_ABI_VERSION,
                "LA-CAL transport ABI is missing or incompatible") &&
         ok;
    ok = expect(entry != nullptr, "LA-CAL transport entry point is missing") &&
         ok;
    ok = expect(caps != nullptr &&
                    (caps(nullptr) & PCL_CAP_PUBSUB) == PCL_CAP_PUBSUB,
                "LA-CAL transport does not advertise pub/sub") &&
         ok;
    pcl_plugin_unload(transport_handle);
  }

  if (ok) {
    std::cout << "[sdk_gra_plugin_load_smoke] PASS: OMS JSON codec and "
                 "LA-CAL WebSocket transport loaded\n";
    return 0;
  }
  return 1;
}
