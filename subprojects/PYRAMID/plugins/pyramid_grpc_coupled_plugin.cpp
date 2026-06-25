/// \file pyramid_grpc_coupled_plugin.cpp
/// \brief Structural coupled gRPC target plugin for application/grpc.

#include <pcl/pcl_codec.h>
#include <pcl/pcl_plugin.h>

#if defined(_WIN32)
#  define PYRAMID_GRPC_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PYRAMID_GRPC_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PYRAMID_GRPC_PLUGIN_EXPORT
#endif

namespace {

constexpr const char* kGrpcContentType = "application/grpc";

pcl_status_t grpcCodecEncode(void*,
                             const char*,
                             const void*,
                             pcl_msg_t*) {
  return PCL_ERR_NOT_FOUND;
}

pcl_status_t grpcCodecDecode(void*,
                             const char*,
                             const pcl_msg_t*,
                             void*) {
  return PCL_ERR_NOT_FOUND;
}

void grpcCodecFreeMsg(void*, pcl_msg_t*) {}

pcl_status_t grpcPublish(void*,
                         const char*,
                         const pcl_msg_t*) {
  return PCL_ERR_NOT_FOUND;
}

pcl_status_t grpcServe(void*,
                       const char*,
                       const pcl_msg_t*,
                       pcl_msg_t*) {
  return PCL_ERR_NOT_FOUND;
}

pcl_status_t grpcSubscribe(void*,
                           const char*,
                           const char*) {
  return PCL_OK;
}

pcl_status_t grpcInvokeAsync(void*,
                             const char*,
                             const pcl_msg_t*,
                             pcl_resp_cb_fn_t,
                             void*) {
  return PCL_ERR_NOT_FOUND;
}

const pcl_codec_t grpc_codec = {
    PCL_CODEC_ABI_VERSION,
    kGrpcContentType,
    grpcCodecEncode,
    grpcCodecDecode,
    grpcCodecFreeMsg,
    nullptr,
};

const pcl_transport_t grpc_transport = {
    grpcPublish,
    grpcServe,
    grpcSubscribe,
    grpcInvokeAsync,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};

}  // namespace

extern "C" {

PYRAMID_GRPC_PLUGIN_EXPORT uint32_t pcl_transport_abi_version() {
  return PCL_TRANSPORT_ABI_VERSION;
}

PYRAMID_GRPC_PLUGIN_EXPORT const pcl_transport_t* pcl_transport_plugin_entry(
    const char*) {
  return &grpc_transport;
}

PYRAMID_GRPC_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry() {
  return &grpc_codec;
}

}
