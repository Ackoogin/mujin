/// \file test_codec_plugin_swap.cpp
/// \brief Runtime codec plugin swap tests.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_types.h>
}

#include <cstdint>
#include <cstring>
#include <string>

#include "pyramid_data_model_tactical_codec.hpp"
#include "pyramid_data_model_tactical_cabi_marshal.hpp"
#include "pyramid_data_model_types.hpp"
#include "pyramid_codec_plugin_test_paths.hpp"

namespace tactical_codec = pyramid::domain_model::tactical;
namespace types = pyramid::domain_model;

namespace {

types::ObjectDetail makeTestEvidence() {
  types::ObjectDetail ev;
  ev.id = "obj-1";
  ev.identity = types::StandardIdentity::Hostile;
  ev.dimension = types::BattleDimension::SeaSurface;
  ev.position.latitude = 0.8901;
  ev.position.longitude = 0.0012;
  ev.creation_time = 1.5;
  ev.quality = 0.95;
  return ev;
}

void expectEvidenceEqual(const types::ObjectDetail& a,
                         const types::ObjectDetail& b) {
  EXPECT_EQ(a.id, b.id);
  EXPECT_EQ(a.identity, b.identity);
  EXPECT_EQ(a.dimension, b.dimension);
  EXPECT_DOUBLE_EQ(a.position.latitude, b.position.latitude);
  EXPECT_DOUBLE_EQ(a.position.longitude, b.position.longitude);
  ASSERT_TRUE(a.quality.has_value());
  ASSERT_TRUE(b.quality.has_value());
  EXPECT_DOUBLE_EQ(a.quality.value(), b.quality.value());
  EXPECT_DOUBLE_EQ(a.creation_time, b.creation_time);
}

struct LoadedCodec {
  pcl_codec_registry_t* registry = nullptr;
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = nullptr;

  ~LoadedCodec() {
    if (handle) {
      pcl_plugin_unload(handle);
    }
    if (registry) {
      pcl_codec_registry_destroy(registry);
    }
  }

  LoadedCodec(const LoadedCodec&) = delete;
  LoadedCodec& operator=(const LoadedCodec&) = delete;

  LoadedCodec() = default;

  LoadedCodec(LoadedCodec&& other) noexcept
      : registry(other.registry), handle(other.handle), codec(other.codec) {
    other.registry = nullptr;
    other.handle = nullptr;
    other.codec = nullptr;
  }

  LoadedCodec& operator=(LoadedCodec&& other) noexcept {
    if (this != &other) {
      if (handle) {
        pcl_plugin_unload(handle);
      }
      if (registry) {
        pcl_codec_registry_destroy(registry);
      }
      registry = other.registry;
      handle = other.handle;
      codec = other.codec;
      other.registry = nullptr;
      other.handle = nullptr;
      other.codec = nullptr;
    }
    return *this;
  }
};

LoadedCodec loadCodec(const char* path, const char* content_type) {
  LoadedCodec loaded;
  loaded.registry = pcl_codec_registry_create();
  EXPECT_NE(loaded.registry, nullptr);
  if (!loaded.registry) {
    return loaded;
  }
  EXPECT_EQ(pcl_plugin_load_codec(path, nullptr, loaded.registry,
                                  &loaded.handle),
            PCL_OK);
  EXPECT_NE(loaded.handle, nullptr);
  loaded.codec = pcl_codec_registry_get(loaded.registry, content_type);
  EXPECT_NE(loaded.codec, nullptr);
  return loaded;
}

std::string roundTripObjectDetail(const char* plugin_path,
                                  const char* content_type,
                                  const types::ObjectDetail& ev,
                                  types::ObjectDetail* decoded) {
  LoadedCodec loaded = loadCodec(plugin_path, content_type);
  EXPECT_NE(loaded.codec, nullptr);
  if (!loaded.codec) {
    return {};
  }

  // Encode crosses the C-ABI typed-value boundary: marshal the native value
  // into its frozen C struct, hand that to the plugin, then free it.
  pcl_msg_t msg{};
  pyramid_ObjectDetail_c enc_cs;
  pyramid::cabi::to_c(ev, &enc_cs);
  EXPECT_EQ(loaded.codec->encode(
                loaded.codec->codec_ctx, "ObjectDetail", &enc_cs, &msg),
            PCL_OK);
  pyramid_ObjectDetail_c_free(&enc_cs);
  EXPECT_NE(msg.data, nullptr);
  const std::string bytes(static_cast<const char*>(msg.data), msg.size);

  // Decode is symmetric: the plugin fills a C struct (plugin-allocated), which
  // we marshal back into the native value and then free.
  types::ObjectDetail roundtripped{};
  pyramid_ObjectDetail_c dec_cs;
  std::memset(&dec_cs, 0, sizeof(dec_cs));
  EXPECT_EQ(loaded.codec->decode(
                loaded.codec->codec_ctx, "ObjectDetail", &msg, &dec_cs),
            PCL_OK);
  pyramid::cabi::from_c(&dec_cs, roundtripped);
  pyramid_ObjectDetail_c_free(&dec_cs);
  if (decoded) {
    *decoded = roundtripped;
  }

  EXPECT_NE(loaded.codec->free_msg, nullptr);
  if (!loaded.codec->free_msg) {
    return bytes;
  }
  loaded.codec->free_msg(loaded.codec->codec_ctx, &msg);
  EXPECT_EQ(msg.data, nullptr);
  EXPECT_EQ(msg.size, 0u);

  return bytes;
}

}  // namespace

TEST(CodecPluginSwap, SwapJsonThenFlatbuffersNoRebuild) {
  const auto ev = makeTestEvidence();

  types::ObjectDetail json_decoded{};
  const std::string json_bytes = roundTripObjectDetail(
      kPyramidJsonCodecPluginTactical, "application/json", ev, &json_decoded);
  ASSERT_FALSE(json_bytes.empty());
  expectEvidenceEqual(json_decoded, ev);

  if (!kPyramidFlatbuffersCodecPluginTactical) {
    GTEST_SKIP() << "FlatBuffers codec plugin target is disabled";
  }
  types::ObjectDetail flatbuffers_decoded{};
  const std::string flatbuffers_bytes = roundTripObjectDetail(
      kPyramidFlatbuffersCodecPluginTactical, "application/flatbuffers", ev,
      &flatbuffers_decoded);
  ASSERT_FALSE(flatbuffers_bytes.empty());
  expectEvidenceEqual(flatbuffers_decoded, ev);

  EXPECT_NE(flatbuffers_bytes, json_bytes);
  EXPECT_NE(flatbuffers_bytes.size(), json_bytes.size());
}

TEST(CodecPluginSwap, LoadedJsonCodecMatchesStaticToJson) {
  const auto ev = makeTestEvidence();
  LoadedCodec loaded = loadCodec(
      kPyramidJsonCodecPluginTactical, "application/json");
  ASSERT_NE(loaded.codec, nullptr);

  pcl_msg_t msg{};
  pyramid_ObjectDetail_c enc_cs;
  pyramid::cabi::to_c(ev, &enc_cs);
  ASSERT_EQ(loaded.codec->encode(
                loaded.codec->codec_ctx, "ObjectDetail", &enc_cs, &msg),
            PCL_OK);
  pyramid_ObjectDetail_c_free(&enc_cs);
  ASSERT_NE(msg.data, nullptr);

  const std::string plugin_bytes(static_cast<const char*>(msg.data), msg.size);
  EXPECT_EQ(plugin_bytes, tactical_codec::toJson(ev));

  ASSERT_NE(loaded.codec->free_msg, nullptr);
  loaded.codec->free_msg(loaded.codec->codec_ctx, &msg);
}

TEST(CodecPluginSwap, UnknownSchemaIdFailsClosed) {
  const auto ev = makeTestEvidence();
  LoadedCodec loaded = loadCodec(
      kPyramidJsonCodecPluginTactical, "application/json");
  ASSERT_NE(loaded.codec, nullptr);

  pcl_msg_t msg{};
  EXPECT_EQ(loaded.codec->encode(
                loaded.codec->codec_ctx, "NoSuchType", &ev, &msg),
            PCL_ERR_NOT_FOUND);
  EXPECT_EQ(msg.data, nullptr);
  EXPECT_EQ(msg.size, 0u);
}
