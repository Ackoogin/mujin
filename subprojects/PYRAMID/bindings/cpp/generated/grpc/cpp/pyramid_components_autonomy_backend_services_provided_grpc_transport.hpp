// Auto-generated gRPC transport — do not edit
// Backend: grpc | Namespace: pyramid::components::autonomy_backend::services::provided::grpc_transport
//
// Exposes proto services as gRPC services, delegating to
// the same ServiceHandler interface used by PCL bindings.
//
// Requires: protoc + grpc_cpp_plugin generated code
// Link with: grpc++ protobuf
#pragma once

#include "pyramid/components/autonomy_backend/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <memory>
#include <string>

namespace pyramid::components::autonomy_backend::services::provided::grpc_transport {

// ---------------------------------------------------------------------------
// ServiceHandler — transport-agnostic business logic interface
//
// Implement this interface once; it works with both PCL and gRPC transport.
// ---------------------------------------------------------------------------

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Capabilities_Service
    virtual std::vector<Capabilities> handleReadCapabilities(const Query& request);

    // Planning_Execution_Service
    virtual Identifier handleCreateRequirement(const PlanningExecutionRequirement& request);
    virtual std::vector<PlanningExecutionRequirement> handleReadRequirement(const Query& request);
    virtual Ack handleUpdateRequirement(const PlanningExecutionRequirement& request);
    virtual Ack handleDeleteRequirement(const Identifier& request);

    // State_Service
    virtual Identifier handleCreateState(const StateUpdate& request);
    virtual Ack handleUpdateState(const StateUpdate& request);
    virtual Ack handleDeleteState(const Identifier& request);

    // Plan_Service
    virtual std::vector<Plan> handleReadPlan(const Query& request);

    // Execution_Run_Service
    virtual std::vector<ExecutionRun> handleReadRun(const Query& request);

    // Requirement_Placement_Service
    virtual std::vector<RequirementPlacement> handleReadPlacement(const Query& request);
};

// ---------------------------------------------------------------------------
// Capabilities_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Capabilities_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Capabilities_Service::Service {
public:
    explicit Capabilities_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadCapabilities(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::Capabilities>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Planning_Execution_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Planning_Execution_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Planning_Execution_Service::Service {
public:
    explicit Planning_Execution_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status CreateRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::PlanningExecutionRequirement* request,
        ::pyramid::data_model::base::Identifier* response) override;

    grpc::Status ReadRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::PlanningExecutionRequirement>* writer) override;

    grpc::Status UpdateRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::PlanningExecutionRequirement* request,
        ::pyramid::data_model::common::Ack* response) override;

    grpc::Status DeleteRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::base::Identifier* request,
        ::pyramid::data_model::common::Ack* response) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// State_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class State_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::State_Service::Service {
public:
    explicit State_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status CreateState(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::StateUpdate* request,
        ::pyramid::data_model::base::Identifier* response) override;

    grpc::Status UpdateState(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::StateUpdate* request,
        ::pyramid::data_model::common::Ack* response) override;

    grpc::Status DeleteState(
        grpc::ServerContext* context,
        const ::pyramid::data_model::base::Identifier* request,
        ::pyramid::data_model::common::Ack* response) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Plan_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Plan_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Plan_Service::Service {
public:
    explicit Plan_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadPlan(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::Plan>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Execution_Run_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Execution_Run_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Execution_Run_Service::Service {
public:
    explicit Execution_Run_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadRun(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::ExecutionRun>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Requirement_Placement_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Requirement_Placement_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Requirement_Placement_Service::Service {
public:
    explicit Requirement_Placement_ServiceImpl(ServiceHandler& handler)
        : handler_(handler) {}

    grpc::Status ReadPlacement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::RequirementPlacement>* writer) override;

private:
    ServiceHandler& handler_;
};

// ---------------------------------------------------------------------------
// Server builder — registers all services on one gRPC server
// ---------------------------------------------------------------------------

std::unique_ptr<grpc::Server> buildServer(
    const std::string& listen_address,
    ServiceHandler& handler);

} // namespace pyramid::components::autonomy_backend::services::provided::grpc_transport
