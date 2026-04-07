// Auto-generated gRPC transport — do not edit
// Backend: grpc | Namespace: pyramid::components::tactical_objects::services::provided::grpc_transport
//
// Exposes proto services as gRPC services, delegating to
// the same ServiceHandler interface used by PCL bindings.
//
// Requires: protoc + grpc_cpp_plugin generated code
// Link with: grpc++ protobuf
#pragma once

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <memory>
#include <string>

namespace pyramid::components::tactical_objects::services::provided::grpc_transport {

// ---------------------------------------------------------------------------
// ServiceHandler — transport-agnostic business logic interface
//
// Implement this interface once; it works with both PCL and gRPC transport.
// ---------------------------------------------------------------------------

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Matching_Objects_Service
    virtual std::vector<ObjectMatch> handleReadMatch(const Query& request);

    // Object_Of_Interest_Service
    virtual Identifier handleCreateRequirement(const ObjectInterestRequirement& request);
    virtual std::vector<ObjectInterestRequirement> handleReadRequirement(const Query& request);
    virtual Ack handleUpdateRequirement(const ObjectInterestRequirement& request);
    virtual Ack handleDeleteRequirement(const Identifier& request);

    // Specific_Object_Detail_Service
    virtual std::vector<ObjectDetail> handleReadDetail(const Query& request);
};

// ---------------------------------------------------------------------------
// Matching_Objects_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Matching_Objects_ServiceImpl final : public pyramid::components::tactical_objects::services::provided::Matching_Objects_Service::Service {
public:
    explicit Matching_Objects_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadMatch(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectMatch>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Object_Of_Interest_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Of_Interest_ServiceImpl final : public pyramid::components::tactical_objects::services::provided::Object_Of_Interest_Service::Service {
public:
    explicit Object_Of_Interest_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status CreateRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
        ::pyramid::data_model::base::Identifier* response) override;

    grpc::Status ReadRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectInterestRequirement>* writer) override;

    grpc::Status UpdateRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
        ::pyramid::data_model::common::Ack* response) override;

    grpc::Status DeleteRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::base::Identifier* request,
        ::pyramid::data_model::common::Ack* response) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Specific_Object_Detail_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Specific_Object_Detail_ServiceImpl final : public pyramid::components::tactical_objects::services::provided::Specific_Object_Detail_Service::Service {
public:
    explicit Specific_Object_Detail_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadDetail(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Server builder — registers all services on one gRPC server
// ---------------------------------------------------------------------------

std::unique_ptr<grpc::Server> buildServer(
    const std::string& listen_address,
    ServiceHandler& handler);

} // namespace pyramid::components::tactical_objects::services::provided::grpc_transport
