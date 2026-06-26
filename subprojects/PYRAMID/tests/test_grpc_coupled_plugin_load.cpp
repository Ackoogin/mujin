/// \file test_grpc_coupled_plugin_load.cpp
/// \brief Loader checks for the coupled gRPC target plugin (application/grpc).
///
/// The gRPC target is the one content type where transport and codec are
/// coupled: a single \c .so exposes both a transport vtable and a codec vtable
/// under \c application/grpc. These tests verify the plugin loads through the
/// generic loader and presents both halves with the expected ABI — the
/// structural contract the runtime relies on when composing the coupled target
/// at run time.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_codec.h>
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
}

#include "pyramid_codec_plugin_test_paths.hpp"

namespace {

constexpr const char* kGrpcContentType = "application/grpc";

}  // namespace

TEST(GrpcCoupledPluginLoad, TransportFailsClosedWithoutExecutor) {
  ASSERT_NE(kPyramidGrpcCoupledPlugin, nullptr);

  // The transport half needs a live executor and starts a real gRPC server, so
  // its end-to-end behaviour is covered by test_grpc_coupled_plugin_e2e. With
  // an empty config the plugin cannot start a server: the entry point returns
  // NULL and the loader reports PCL_ERR_STATE rather than handing back a
  // half-initialised transport.
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(kPyramidGrpcCoupledPlugin,
                                      "{}",
                                      &handle,
                                      &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(transport, nullptr);
}

TEST(GrpcCoupledPluginLoad, RegistersCodecUnderGrpcContentType) {
  ASSERT_NE(kPyramidGrpcCoupledPlugin, nullptr);

  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_plugin_handle_t* handle = nullptr;
  ASSERT_EQ(pcl_plugin_load_codec(kPyramidGrpcCoupledPlugin,
                                  "{}",
                                  registry,
                                  &handle),
            PCL_OK);
  ASSERT_NE(handle, nullptr);

  // The single coupled .so registers its codec under application/grpc — the
  // same content type its transport serves.
  const pcl_codec_t* codec = pcl_codec_registry_get(registry, kGrpcContentType);
  ASSERT_NE(codec, nullptr);
  EXPECT_EQ(codec->abi_version, PCL_CODEC_ABI_VERSION);
  ASSERT_NE(codec->content_type, nullptr);
  EXPECT_STREQ(codec->content_type, kGrpcContentType);
  EXPECT_NE(codec->encode, nullptr);
  EXPECT_NE(codec->decode, nullptr);

  pcl_plugin_unload(handle);
  pcl_codec_registry_destroy(registry);
}
