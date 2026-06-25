/// \file test_codec_registry_bridge.cpp
/// \brief Bridge scenario: one process loads several per-component codec
///        plugins under the same content_type and dispatches by schema_id.
///
/// A PYRAMID bridge consumes one component and provides toward another, so it
/// must load multiple per-component codec plugins at once (all
/// "application/json").  This exercises the multi-codec-per-content_type
/// registry and the try-each-by-schema_id dispatch the generated facades use.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_types.h>
}

#include <cstring>
#include <string>

#include "pyramid_codec_plugin_test_paths.hpp"
#include "pyramid_data_model_autonomy_cabi_marshal.hpp"
#include "pyramid_data_model_tactical_cabi_marshal.hpp"
#include "pyramid_data_model_types.hpp"

namespace types = pyramid::domain_model;

namespace {

/// Encode \p value by trying each codec registered for \p content_type until one
/// accepts \p schema_id -- mirrors the generated facade's registry encode path.
template <typename CStruct, typename Native, typename ToC, typename FreeC>
std::string encodeViaBridge(pcl_codec_registry_t* registry,
                            const char* content_type, const char* schema_id,
                            const Native& value, ToC to_c, FreeC free_c) {
  for (uint32_t i = 0;; ++i) {
    const pcl_codec_t* codec =
        pcl_codec_registry_get_at(registry, content_type, i);
    if (!codec) {
      break;
    }
    if (!codec->encode) {
      continue;
    }
    CStruct cs;
    to_c(value, &cs);
    pcl_msg_t msg{};
    const pcl_status_t rc =
        codec->encode(codec->codec_ctx, schema_id, &cs, &msg);
    free_c(&cs);
    if (rc == PCL_OK) {
      std::string bytes(static_cast<const char*>(msg.data), msg.size);
      if (codec->free_msg) {
        codec->free_msg(codec->codec_ctx, &msg);
      }
      return bytes;
    }
  }
  return {};
}

/// Decode \p bytes into \p out by trying each codec for \p content_type.
template <typename CStruct, typename Native, typename FromC, typename FreeC>
bool decodeViaBridge(pcl_codec_registry_t* registry, const char* content_type,
                     const char* schema_id, const std::string& bytes,
                     Native* out, FromC from_c, FreeC free_c) {
  pcl_msg_t msg{};
  msg.data = bytes.data();
  msg.size = static_cast<uint32_t>(bytes.size());
  msg.type_name = content_type;
  for (uint32_t i = 0;; ++i) {
    const pcl_codec_t* codec =
        pcl_codec_registry_get_at(registry, content_type, i);
    if (!codec) {
      break;
    }
    if (!codec->decode) {
      continue;
    }
    CStruct cs;
    std::memset(&cs, 0, sizeof(cs));
    const pcl_status_t rc =
        codec->decode(codec->codec_ctx, schema_id, &msg, &cs);
    if (rc == PCL_OK) {
      from_c(&cs, *out);
      free_c(&cs);
      return true;
    }
    free_c(&cs);
  }
  return false;
}

}  // namespace

TEST(CodecRegistryBridge, TwoComponentCodecPluginsCoexistAndDispatchBySchema) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  // A bridge loads codec plugins for two distinct components.
  pcl_plugin_handle_t* tactical_handle = nullptr;
  pcl_plugin_handle_t* autonomy_handle = nullptr;
  ASSERT_EQ(pcl_plugin_load_codec(kPyramidJsonCodecPluginTactical, registry,
                                  &tactical_handle),
            PCL_OK);
  ASSERT_EQ(pcl_plugin_load_codec(kPyramidJsonCodecPluginAutonomy, registry,
                                  &autonomy_handle),
            PCL_OK);

  // Both registered under application/json and both are reachable.
  EXPECT_EQ(pcl_codec_registry_count(registry), 2u);
  EXPECT_NE(pcl_codec_registry_get_at(registry, "application/json", 0u),
            nullptr);
  EXPECT_NE(pcl_codec_registry_get_at(registry, "application/json", 1u),
            nullptr);
  EXPECT_EQ(pcl_codec_registry_get_at(registry, "application/json", 2u),
            nullptr);

  // A tactical_objects type round-trips (only the tactical plugin handles it).
  types::ObjectDetail od;
  od.id = "obj-7";
  od.identity = types::StandardIdentity::Hostile;
  od.dimension = types::BattleDimension::Air;
  od.position.latitude = 12.5;
  od.creation_time = 3.0;

  const std::string od_bytes =
      encodeViaBridge<pyramid_ObjectDetail_c>(
          registry, "application/json", "ObjectDetail", od,
          [](const types::ObjectDetail& v, pyramid_ObjectDetail_c* cs) {
            pyramid::cabi::to_c(v, cs);
          },
          [](pyramid_ObjectDetail_c* cs) { pyramid_ObjectDetail_c_free(cs); });
  ASSERT_FALSE(od_bytes.empty());

  types::ObjectDetail od_rt{};
  ASSERT_TRUE(decodeViaBridge<pyramid_ObjectDetail_c>(
      registry, "application/json", "ObjectDetail", od_bytes, &od_rt,
      [](const pyramid_ObjectDetail_c* cs, types::ObjectDetail& v) {
        pyramid::cabi::from_c(cs, v);
      },
      [](pyramid_ObjectDetail_c* cs) { pyramid_ObjectDetail_c_free(cs); }));
  EXPECT_EQ(od_rt.id, "obj-7");
  EXPECT_EQ(od_rt.identity, types::StandardIdentity::Hostile);
  EXPECT_EQ(od_rt.dimension, types::BattleDimension::Air);

  // An autonomy_backend type round-trips (only the autonomy plugin handles it).
  types::Capabilities cap;
  cap.backend_id = "ame-1";
  cap.supports_planning_requirements = true;
  cap.supports_replanning = true;

  const std::string cap_bytes =
      encodeViaBridge<pyramid_Capabilities_c>(
          registry, "application/json", "Capabilities", cap,
          [](const types::Capabilities& v, pyramid_Capabilities_c* cs) {
            pyramid::cabi::to_c(v, cs);
          },
          [](pyramid_Capabilities_c* cs) { pyramid_Capabilities_c_free(cs); });
  ASSERT_FALSE(cap_bytes.empty());

  types::Capabilities cap_rt{};
  ASSERT_TRUE(decodeViaBridge<pyramid_Capabilities_c>(
      registry, "application/json", "Capabilities", cap_bytes, &cap_rt,
      [](const pyramid_Capabilities_c* cs, types::Capabilities& v) {
        pyramid::cabi::from_c(cs, v);
      },
      [](pyramid_Capabilities_c* cs) { pyramid_Capabilities_c_free(cs); }));
  EXPECT_EQ(cap_rt.backend_id, "ame-1");
  EXPECT_TRUE(cap_rt.supports_planning_requirements);
  EXPECT_TRUE(cap_rt.supports_replanning);
  EXPECT_FALSE(cap_rt.supports_execution_requirements);

  pcl_plugin_unload(tactical_handle);
  pcl_plugin_unload(autonomy_handle);
  pcl_codec_registry_destroy(registry);
}
