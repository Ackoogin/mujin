// Auto-generated gRPC transport — do not edit
// Backend: grpc | Namespace: pyramid::components::tactical_objects::services::consumed::grpc_transport
//
// Exposes proto services as gRPC services, delegating to
// the same ServiceHandler interface used by PCL bindings.
//
// Requires: protoc + grpc_cpp_plugin generated code
// Link with: grpc++ protobuf
#pragma once

#include "pyramid/components/tactical_objects/services/consumed.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <memory>
#include <string>

namespace pyramid::components::tactical_objects::services::consumed::grpc_transport {

// ---------------------------------------------------------------------------
// ServiceHandler — transport-agnostic business logic interface
//
// Implement this interface once; it works with both PCL and gRPC transport.
// ---------------------------------------------------------------------------

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Object_Evidence_Service
    virtual std::vector<ObjectDetail> handleReadDetail(const Query& request);

    // Object_Solution_Evidence_Service
    virtual Identifier handleCreateRequirement(const ObjectEvidenceRequirement& request);
    virtual std::vector<ObjectEvidenceRequirement> handleReadRequirement(const Query& request);
    virtual Ack handleUpdateRequirement(const ObjectEvidenceRequirement& request);
    virtual Ack handleDeleteRequirement(const Identifier& request);

    // Object_Source_Capability_Service
    virtual std::vector<Capability> handleReadCapability(const Query& request);
};

// ---------------------------------------------------------------------------
// Object_Evidence_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Evidence_ServiceImpl final : public pyramid::components::tactical_objects::services::consumed::Object_Evidence_Service::Service {
public:
    explicit Object_Evidence_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadDetail(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Object_Solution_Evidence_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Solution_Evidence_ServiceImpl final : public pyramid::components::tactical_objects::services::consumed::Object_Solution_Evidence_Service::Service {
public:
    explicit Object_Solution_Evidence_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status CreateRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
        ::pyramid::data_model::base::Identifier* response) override;

    grpc::Status ReadRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectEvidenceRequirement>* writer) override;

    grpc::Status UpdateRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
        ::pyramid::data_model::common::Ack* response) override;

    grpc::Status DeleteRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::base::Identifier* request,
        ::pyramid::data_model::common::Ack* response) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Object_Source_Capability_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Source_Capability_ServiceImpl final : public pyramid::components::tactical_objects::services::consumed::Object_Source_Capability_Service::Service {
public:
    explicit Object_Source_Capability_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadCapability(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::common::Capability>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Server builder — registers all services on one gRPC server
// ---------------------------------------------------------------------------

std::unique_ptr<grpc::Server> buildServer(
    const std::string& listen_address,
    ServiceHandler& handler);

} // namespace pyramid::components::tactical_objects::services::consumed::grpc_transport
