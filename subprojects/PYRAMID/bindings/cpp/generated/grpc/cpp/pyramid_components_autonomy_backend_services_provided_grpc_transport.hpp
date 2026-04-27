// Auto-generated gRPC transport -- do not edit
// Backend: grpc | Namespace: pyramid::services::autonomy_backend::provided::grpc_transport
//
// Exposes proto services as gRPC services, delegating to
// the standard generated service facade and PCL executor.
//
// Requires: protoc + grpc_cpp_plugin generated code
// Link with: grpc++ protobuf
#pragma once

#include "pyramid/components/autonomy_backend/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <pcl/pcl_executor.h>
#include <memory>
#include <string>

namespace pyramid::services::autonomy_backend::provided::grpc_transport {

// ---------------------------------------------------------------------------
// Capabilities_Service -- gRPC service implementation
// ---------------------------------------------------------------------------

class Capabilities_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Capabilities_Service::Service {
public:
    explicit Capabilities_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status ReadCapabilities(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::Capabilities>* writer) override;

private:
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Planning_Requirement_Service -- gRPC service implementation
// ---------------------------------------------------------------------------

class Planning_Requirement_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Planning_Requirement_Service::Service {
public:
    explicit Planning_Requirement_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status CreatePlanningRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::PlanningRequirement* request,
        ::pyramid::data_model::base::Identifier* response) override;

    grpc::Status ReadPlanningRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::PlanningRequirement>* writer) override;

    grpc::Status UpdatePlanningRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::PlanningRequirement* request,
        ::pyramid::data_model::common::Ack* response) override;

    grpc::Status DeletePlanningRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::base::Identifier* request,
        ::pyramid::data_model::common::Ack* response) override;

private:
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Execution_Requirement_Service -- gRPC service implementation
// ---------------------------------------------------------------------------

class Execution_Requirement_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Execution_Requirement_Service::Service {
public:
    explicit Execution_Requirement_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status CreateExecutionRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::ExecutionRequirement* request,
        ::pyramid::data_model::base::Identifier* response) override;

    grpc::Status ReadExecutionRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::ExecutionRequirement>* writer) override;

    grpc::Status UpdateExecutionRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::ExecutionRequirement* request,
        ::pyramid::data_model::common::Ack* response) override;

    grpc::Status DeleteExecutionRequirement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::base::Identifier* request,
        ::pyramid::data_model::common::Ack* response) override;

private:
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// State_Service -- gRPC service implementation
// ---------------------------------------------------------------------------

class State_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::State_Service::Service {
public:
    explicit State_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

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
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Plan_Service -- gRPC service implementation
// ---------------------------------------------------------------------------

class Plan_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Plan_Service::Service {
public:
    explicit Plan_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status CreatePlan(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::Plan* request,
        ::pyramid::data_model::base::Identifier* response) override;

    grpc::Status ReadPlan(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::Plan>* writer) override;

    grpc::Status UpdatePlan(
        grpc::ServerContext* context,
        const ::pyramid::data_model::autonomy::Plan* request,
        ::pyramid::data_model::common::Ack* response) override;

    grpc::Status DeletePlan(
        grpc::ServerContext* context,
        const ::pyramid::data_model::base::Identifier* request,
        ::pyramid::data_model::common::Ack* response) override;

private:
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Execution_Run_Service -- gRPC service implementation
// ---------------------------------------------------------------------------

class Execution_Run_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Execution_Run_Service::Service {
public:
    explicit Execution_Run_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status ReadRun(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::ExecutionRun>* writer) override;

private:
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Requirement_Placement_Service -- gRPC service implementation
// ---------------------------------------------------------------------------

class Requirement_Placement_ServiceImpl final : public pyramid::components::autonomy_backend::services::provided::Requirement_Placement_Service::Service {
public:
    explicit Requirement_Placement_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status ReadPlacement(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::autonomy::RequirementPlacement>* writer) override;

private:
    pcl_executor_t* executor_;
};

} // namespace pyramid::services::autonomy_backend::provided::grpc_transport
