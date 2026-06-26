/// \file pyramid_cpp_codec_plugin_environment.cpp
/// \brief GTest environment that loads generated codec plugins for C++ tests.

#include "pyramid_codec_plugin_test_paths.hpp"

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
}

namespace {

class PyramidCodecPluginEnvironment final : public ::testing::Environment {
public:
  void SetUp() override {
    pcl_codec_registry_load_plugins_from_paths(
        pcl_codec_registry_default(),
        kPyramidCodecPluginPaths.data(),
        kPyramidCodecPluginPaths.size());
  }
};

[[maybe_unused]] ::testing::Environment* const kPyramidCodecPluginEnvironment =
    ::testing::AddGlobalTestEnvironment(new PyramidCodecPluginEnvironment);

}  // namespace
