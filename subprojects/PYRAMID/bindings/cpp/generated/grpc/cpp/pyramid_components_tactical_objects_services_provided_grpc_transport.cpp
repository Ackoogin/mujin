// Auto-generated gRPC transport -- do not edit

#include "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"

#include <google/protobuf/message_lite.h>
#include <pcl/pcl_types.h>

#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char* kProtobufContentType = "application/protobuf";

struct ExecutorResponse {
    pcl_status_t status = PCL_OK;
    std::string content_type;
    std::string payload;
};

struct UnaryState {
    std::promise<ExecutorResponse> promise;
};

void executor_response_callback(const pcl_msg_t* response, void* user_data)
{
    auto* state = static_cast<UnaryState*>(user_data);
    if (!state) {
        return;
    }
    ExecutorResponse out;
    if (response && response->type_name) {
        out.content_type = response->type_name;
    }
    if (response && response->data && response->size > 0U) {
        const auto* bytes = static_cast<const char*>(response->data);
        out.payload.assign(bytes, bytes + response->size);
    }
    state->promise.set_value(std::move(out));
}

ExecutorResponse invoke_executor(pcl_executor_t* executor,
                                 const char* service_name,
                                 const std::string& request_payload)
{
    ExecutorResponse error;
    if (!executor || !service_name || !service_name[0]) {
        error.status = PCL_ERR_INVALID;
        return error;
    }
    UnaryState state;
    pcl_msg_t request{};
    request.data = request_payload.empty() ? nullptr : request_payload.data();
    request.size = static_cast<uint32_t>(request_payload.size());
    request.type_name = kProtobufContentType;
    const auto rc = pcl_executor_post_service_request(
        executor, service_name, &request, executor_response_callback, &state);
    if (rc != PCL_OK) {
        error.status = rc;
        return error;
    }
    return state.promise.get_future().get();
}

std::string serialize_grpc_request(const google::protobuf::MessageLite& request)
{
    std::string payload;
    if (!request.SerializeToString(&payload)) {
        throw std::runtime_error("could not serialize gRPC request");
    }
    return payload;
}

template <typename ResponsePb>
grpc::Status parse_unary_response(const ExecutorResponse& response,
                                  ResponsePb* out,
                                  const char* rpc_name)
{
    if (response.status != PCL_OK) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "executor service dispatch failed");
    }
    if (!out || !out->ParseFromArray(response.payload.data(), static_cast<int>(response.payload.size()))) {
        return grpc::Status(grpc::StatusCode::INTERNAL, std::string("could not parse response for ") + rpc_name);
    }
    return grpc::Status::OK;
}

bool read_varint32(const char*& cursor, const char* end, uint32_t& value)
{
    value = 0U;
    int shift = 0;
    while (cursor < end && shift <= 28) {
        const auto byte = static_cast<unsigned char>(*cursor++);
        value |= static_cast<uint32_t>(byte & 0x7FU) << shift;
        if ((byte & 0x80U) == 0U) {
            return true;
        }
        shift += 7;
    }
    return false;
}

template <typename ResponsePb, typename WriterT>
grpc::Status write_stream_response(const ExecutorResponse& response,
                                  WriterT* writer,
                                  const char* rpc_name)
{
    if (response.status != PCL_OK) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "executor stream dispatch failed");
    }
    if (!writer) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "missing gRPC stream writer");
    }
    const char* cursor = response.payload.data();
    const char* end = cursor + response.payload.size();
    while (cursor < end) {
        uint32_t frame_size = 0U;
        if (!read_varint32(cursor, end, frame_size) || static_cast<size_t>(end - cursor) < frame_size) {
            return grpc::Status(grpc::StatusCode::INTERNAL, std::string("invalid stream frame for ") + rpc_name);
        }
        ResponsePb item;
        if (!item.ParseFromArray(cursor, static_cast<int>(frame_size))) {
            return grpc::Status(grpc::StatusCode::INTERNAL, std::string("could not parse stream frame for ") + rpc_name);
        }
        cursor += frame_size;
        if (!writer->Write(item)) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "client cancelled gRPC stream");
        }
    }
    return grpc::Status::OK;
}

} // namespace

namespace pyramid::services::tactical_objects::provided::grpc_transport {

grpc::Status Matching_Objects_ServiceImpl::ReadMatch(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectMatch>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "matching_objects.read_match", payload);
        return write_stream_response<::pyramid::data_model::tactical::ObjectMatch>(
            executor_response, writer, "ReadMatch");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Object_Of_Interest_ServiceImpl::CreateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
    ::pyramid::data_model::base::Identifier* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "object_of_interest.create_requirement", payload);
        return parse_unary_response(
            executor_response, response, "CreateRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Object_Of_Interest_ServiceImpl::ReadRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectInterestRequirement>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "object_of_interest.read_requirement", payload);
        return write_stream_response<::pyramid::data_model::tactical::ObjectInterestRequirement>(
            executor_response, writer, "ReadRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Object_Of_Interest_ServiceImpl::UpdateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "object_of_interest.update_requirement", payload);
        return parse_unary_response(
            executor_response, response, "UpdateRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Object_Of_Interest_ServiceImpl::DeleteRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "object_of_interest.delete_requirement", payload);
        return parse_unary_response(
            executor_response, response, "DeleteRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Specific_Object_Detail_ServiceImpl::ReadDetail(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "specific_object_detail.read_detail", payload);
        return write_stream_response<::pyramid::data_model::tactical::ObjectDetail>(
            executor_response, writer, "ReadDetail");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

} // namespace pyramid::services::tactical_objects::provided::grpc_transport

namespace pyramid::services::tactical_objects::provided {

class GrpcServer {
public:
    GrpcServer();
    GrpcServer(GrpcServer&&) noexcept;
    GrpcServer& operator=(GrpcServer&&) noexcept;
    GrpcServer(const GrpcServer&) = delete;
    GrpcServer& operator=(const GrpcServer&) = delete;
    ~GrpcServer();

    bool started() const;
    explicit operator bool() const { return started(); }
    void wait();
    void shutdown();

private:
    struct Impl;
    explicit GrpcServer(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
    friend GrpcServer buildGrpcServer(const std::string& listen_address,
                                      pcl_executor_t* executor);
};

struct GrpcServer::Impl {
    std::unique_ptr<grpc_transport::Matching_Objects_ServiceImpl> matching_objects_service;
    std::unique_ptr<grpc_transport::Object_Of_Interest_ServiceImpl> object_of_interest_service;
    std::unique_ptr<grpc_transport::Specific_Object_Detail_ServiceImpl> specific_object_detail_service;
    std::unique_ptr<grpc::Server> server;
};

GrpcServer::GrpcServer() = default;

GrpcServer::GrpcServer(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl))
{}

GrpcServer::GrpcServer(GrpcServer&&) noexcept = default;

GrpcServer& GrpcServer::operator=(GrpcServer&&) noexcept = default;

GrpcServer::~GrpcServer() = default;

bool GrpcServer::started() const
{
    return impl_ && impl_->server != nullptr;
}

void GrpcServer::wait()
{
    if (started()) {
        impl_->server->Wait();
    }
}

void GrpcServer::shutdown()
{
    if (started()) {
        impl_->server->Shutdown();
    }
}

GrpcServer buildGrpcServer(const std::string& listen_address,
                           pcl_executor_t* executor)
{
    if (!executor) {
        return {};
    }

    auto impl = std::make_unique<GrpcServer::Impl>();
    grpc::ServerBuilder builder;
    builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());

    impl->matching_objects_service = std::make_unique<grpc_transport::Matching_Objects_ServiceImpl>(executor);
    builder.RegisterService(impl->matching_objects_service.get());

    impl->object_of_interest_service = std::make_unique<grpc_transport::Object_Of_Interest_ServiceImpl>(executor);
    builder.RegisterService(impl->object_of_interest_service.get());

    impl->specific_object_detail_service = std::make_unique<grpc_transport::Specific_Object_Detail_ServiceImpl>(executor);
    builder.RegisterService(impl->specific_object_detail_service.get());

    impl->server = builder.BuildAndStart();
    if (!impl->server) {
        return {};
    }
    return GrpcServer(std::move(impl));
}

} // namespace pyramid::services::tactical_objects::provided
