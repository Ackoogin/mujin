#pragma once

#include <grpcpp/grpcpp.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#if defined(_WIN32)
#define PYRAMID_GRPC_C_SHIM_EXPORT __declspec(dllexport)
#else
#define PYRAMID_GRPC_C_SHIM_EXPORT
#endif

namespace pyramid::services::tactical_objects::grpc_transport::c_api {

using EncodeJsonFn = void* (*)(const char*, size_t*);
using DecodeJsonFn = char* (*)(const void*, size_t);

inline char* copyCString(const std::string& value) {
  char* out = static_cast<char*>(std::malloc(value.size() + 1));
  if (out == nullptr) {
    return nullptr;
  }
  std::memcpy(out, value.c_str(), value.size() + 1);
  return out;
}

inline char* errorCString(const std::string& message) {
  return copyCString("ERROR:" + message);
}

template <typename RequestPb>
bool parseRequest(const char* request_json, EncodeJsonFn encode_json,
                  RequestPb* request, std::string* error_message) {
  if (!request_json || !request || !error_message) {
    return false;
  }

  size_t request_size = 0;
  std::unique_ptr<void, decltype(&std::free)> request_bytes(
      encode_json(request_json, &request_size), &std::free);
  if (!request_bytes || request_size == 0) {
    *error_message = "encode-request";
    return false;
  }
  if (!request->ParseFromArray(request_bytes.get(),
                               static_cast<int>(request_size))) {
    *error_message = "parse-request";
    return false;
  }
  return true;
}

template <typename ResponsePb>
char* renderUnaryResponse(const ResponsePb& response, DecodeJsonFn decode_json) {
  std::string response_bytes;
  if (!response.SerializeToString(&response_bytes)) {
    return errorCString("serialize-response");
  }
  char* response_json = decode_json(response_bytes.data(), response_bytes.size());
  if (!response_json) {
    return errorCString("decode-response");
  }
  return response_json;
}

template <typename RequestPb, typename ResponsePb, typename StubFactory,
          typename RpcInvoker>
char* invokeUnaryJsonRpc(const char* endpoint, const char* request_json,
                         EncodeJsonFn encode_json, DecodeJsonFn decode_json,
                         StubFactory stub_factory, RpcInvoker rpc_invoker) {
  if (!endpoint || !request_json) {
    return errorCString("null-argument");
  }

  RequestPb request;
  std::string request_error;
  if (!parseRequest(request_json, encode_json, &request, &request_error)) {
    return errorCString(request_error);
  }

  auto channel =
      grpc::CreateChannel(endpoint, grpc::InsecureChannelCredentials());
  auto stub = stub_factory(channel);
  if (!stub) {
    return errorCString("create-stub");
  }

  ResponsePb response;
  grpc::ClientContext context;
  const grpc::Status status =
      rpc_invoker(stub.get(), &context, request, &response);
  if (!status.ok()) {
    return errorCString(status.error_message());
  }

  return renderUnaryResponse(response, decode_json);
}

template <typename RequestPb, typename ItemPb, typename StubFactory,
          typename RpcInvoker>
char* invokeServerStreamJsonRpc(const char* endpoint, const char* request_json,
                                EncodeJsonFn encode_json,
                                DecodeJsonFn decode_json,
                                StubFactory stub_factory,
                                RpcInvoker rpc_invoker) {
  if (!endpoint || !request_json) {
    return errorCString("null-argument");
  }

  RequestPb request;
  std::string request_error;
  if (!parseRequest(request_json, encode_json, &request, &request_error)) {
    return errorCString(request_error);
  }

  auto channel =
      grpc::CreateChannel(endpoint, grpc::InsecureChannelCredentials());
  auto stub = stub_factory(channel);
  if (!stub) {
    return errorCString("create-stub");
  }

  grpc::ClientContext context;
  auto reader = rpc_invoker(stub.get(), &context, request);
  if (!reader) {
    return errorCString("create-reader");
  }

  nlohmann::json items = nlohmann::json::array();
  ItemPb item;
  while (reader->Read(&item)) {
    std::string item_bytes;
    if (!item.SerializeToString(&item_bytes)) {
      return errorCString("serialize-stream-item");
    }

    std::unique_ptr<char, decltype(&std::free)> item_json(
        decode_json(item_bytes.data(), item_bytes.size()), &std::free);
    if (!item_json) {
      return errorCString("decode-stream-item");
    }

    auto parsed =
        nlohmann::json::parse(item_json.get(), nullptr, false);
    if (parsed.is_discarded()) {
      return errorCString("parse-stream-item-json");
    }
    items.push_back(std::move(parsed));
  }

  const grpc::Status status = reader->Finish();
  if (!status.ok()) {
    return errorCString(status.error_message());
  }

  return copyCString(items.dump());
}

}  // namespace pyramid::services::tactical_objects::grpc_transport::c_api

extern "C" {
PYRAMID_GRPC_C_SHIM_EXPORT void pyramid_services_tactical_objects_grpc_free_string(
    void* value);
}
