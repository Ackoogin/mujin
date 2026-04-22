// Auto-generated gRPC transport — do not edit
// Backend: grpc | Namespace: pyramid::services::tactical_objects::consumed::grpc_transport
//
// Exposes proto services as gRPC services, delegating to
// the standard generated service facade and PCL executor.
//
// Requires: protoc + grpc_cpp_plugin generated code
// Link with: grpc++ protobuf
#pragma once

#include "pyramid/components/tactical_objects/services/consumed.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <pcl/pcl_executor.h>
#include <memory>
#include <string>

namespace pyramid::services::tactical_objects::consumed::grpc_transport {

// ---------------------------------------------------------------------------
// Object_Evidence_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Evidence_ServiceImpl final : public pyramid::components::tactical_objects::services::consumed::Object_Evidence_Service::Service {
public:
    explicit Object_Evidence_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status ReadDetail(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer) override;

private:
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Object_Solution_Evidence_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Solution_Evidence_ServiceImpl final : public pyramid::components::tactical_objects::services::consumed::Object_Solution_Evidence_Service::Service {
public:
    explicit Object_Solution_Evidence_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

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
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Object_Source_Capability_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Source_Capability_ServiceImpl final : public pyramid::components::tactical_objects::services::consumed::Object_Source_Capability_Service::Service {
public:
    explicit Object_Source_Capability_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status ReadCapability(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::common::Capability>* writer) override;

private:
    pcl_executor_t* executor_;
};

} // namespace pyramid::services::tactical_objects::consumed::grpc_transport
