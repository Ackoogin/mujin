/// \file sdk_codec_plugin_load_smoke.cpp
/// \brief Offline SDK smoke test for loading generated codec plugins.

#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_types.h>

#include <iostream>
#include <vector>

#include "pyramid_sdk_smoke_plugin_paths.hpp"

namespace {

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "[sdk_codec_plugin_load_smoke] FAIL: " << message << '\n';
  }
  return condition;
}

}  // namespace

int main() {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  if (!expect(registry != nullptr, "failed to create codec registry")) {
    return 1;
  }

  std::vector<pcl_plugin_handle_t*> handles;
  handles.reserve(kPyramidSdkCodecPluginPaths.size());

  bool ok = true;
  for (const char* path : kPyramidSdkCodecPluginPaths) {
    pcl_plugin_handle_t* handle = nullptr;
    const pcl_status_t rc =
        pcl_plugin_load_codec(path, nullptr, registry, &handle);
    if (rc != PCL_OK || handle == nullptr) {
      std::cerr << "[sdk_codec_plugin_load_smoke] FAIL: cannot load " << path
                << " (status " << static_cast<int>(rc) << ")\n";
      ok = false;
      continue;
    }
    handles.push_back(handle);
  }

  ok = expect(pcl_codec_registry_count(registry) ==
                  kPyramidSdkCodecPluginPaths.size(),
              "not every generated codec plugin registered") &&
       ok;
  ok = expect(pcl_codec_registry_get(registry, "application/json") != nullptr,
              "JSON codec was not registered") &&
       ok;

  if (kPyramidSdkExpectFlatbuffers) {
    ok = expect(pcl_codec_registry_get(registry, "application/flatbuffers") !=
                    nullptr,
                "FlatBuffers codec was not registered") &&
         ok;
  }

  pcl_codec_registry_clear(registry);
  for (pcl_plugin_handle_t* handle : handles) {
    pcl_plugin_unload(handle);
  }
  pcl_codec_registry_destroy(registry);

  if (ok) {
    std::cout << "[sdk_codec_plugin_load_smoke] PASS: loaded "
              << kPyramidSdkCodecPluginPaths.size() << " codec plugins\n";
    return 0;
  }
  return 1;
}
