#pragma once

#include <grpcpp/grpcpp.h>

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pyramid::services::tactical_objects::grpc_transport::detail {

inline constexpr const char* kProtobufContentType = "application/protobuf";

struct FreeDeleter {
  void operator()(void* ptr) const {
    std::free(ptr);
  }
};

using UniqueResponseBuffer = std::unique_ptr<void, FreeDeleter>;

inline bool readVarint32(const char*& cursor, const char* end, uint32_t& value) {
  value = 0;
  uint32_t shift = 0;
  while (cursor < end && shift <= 28U) {
    const uint8_t byte = static_cast<uint8_t>(*cursor++);
    value |= static_cast<uint32_t>(byte & 0x7FU) << shift;
    if ((byte & 0x80U) == 0U) {
      return true;
    }
    shift += 7U;
  }
  return false;
}

template <typename MessageT>
std::string serializeMessage(const MessageT& message) {
  std::string out;
  if (!message.SerializeToString(&out)) {
    throw std::runtime_error("failed to serialize protobuf request");
  }
  return out;
}

template <typename MessageT>
MessageT parseMessage(const void* data, size_t size, const char* rpc_name) {
  MessageT message;
  if (size == 0) {
    return message;
  }
  if (!message.ParseFromArray(data, static_cast<int>(size))) {
    throw std::runtime_error(std::string("failed to parse protobuf response for ") +
                             rpc_name);
  }
  return message;
}

template <typename DispatchFn>
std::pair<UniqueResponseBuffer, size_t> invokeDispatch(const std::string& request_bytes,
                                                       DispatchFn dispatch_fn) {
  void* response_buf = nullptr;
  size_t response_size = 0;
  dispatch_fn(request_bytes.data(), request_bytes.size(), &response_buf, &response_size);
  return {UniqueResponseBuffer(response_buf), response_size};
}

template <typename RequestPb, typename ResponsePb, typename DispatchFn>
grpc::Status dispatchUnary(const RequestPb* request, ResponsePb* response,
                           DispatchFn dispatch_fn, const char* rpc_name) {
  try {
    const auto request_bytes = serializeMessage(*request);
    auto [response_buf, response_size] = invokeDispatch(request_bytes, dispatch_fn);
    *response = parseMessage<ResponsePb>(response_buf.get(), response_size, rpc_name);
    return grpc::Status::OK;
  } catch (const std::exception& ex) {
    return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
  } catch (...) {
    return grpc::Status(grpc::StatusCode::INTERNAL, "unknown dispatch failure");
  }
}

template <typename RequestPb, typename ResponsePb, typename WriterT, typename DispatchFn>
grpc::Status dispatchServerStream(const RequestPb* request, WriterT* writer,
                                  DispatchFn dispatch_fn, const char* rpc_name) {
  try {
    const auto request_bytes = serializeMessage(*request);
    auto [response_buf, response_size] = invokeDispatch(request_bytes, dispatch_fn);
    const char* cursor = static_cast<const char*>(response_buf.get());
    const char* end = cursor + response_size;
    while (cursor < end) {
      uint32_t frame_size = 0;
      if (!readVarint32(cursor, end, frame_size) ||
          static_cast<size_t>(end - cursor) < frame_size) {
        throw std::runtime_error(std::string("invalid protobuf stream frame for ") +
                                 rpc_name);
      }
      ResponsePb item;
      if (!item.ParseFromArray(cursor, static_cast<int>(frame_size))) {
        throw std::runtime_error(std::string("failed to parse protobuf stream item for ") +
                                 rpc_name);
      }
      cursor += frame_size;
      if (!writer->Write(item)) {
        return grpc::Status(grpc::StatusCode::CANCELLED,
                            "client cancelled gRPC stream");
      }
    }
    return grpc::Status::OK;
  } catch (const std::exception& ex) {
    return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
  } catch (...) {
    return grpc::Status(grpc::StatusCode::INTERNAL, "unknown dispatch failure");
  }
}

}  // namespace pyramid::services::tactical_objects::grpc_transport::detail
