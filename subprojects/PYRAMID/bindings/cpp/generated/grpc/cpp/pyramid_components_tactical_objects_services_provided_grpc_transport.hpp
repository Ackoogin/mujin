// Auto-generated gRPC transport — do not edit
// Backend: grpc | Namespace: pyramid::services::tactical_objects::provided::grpc_transport
//
// Exposes proto services as gRPC services, delegating to
// the standard generated service facade and PCL executor.
//
// Requires: protoc + grpc_cpp_plugin generated code
// Link with: grpc++ protobuf
#pragma once

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <pcl/pcl_executor.h>
#include <memory>
#include <string>

namespace pyramid::services::tactical_objects::provided::grpc_transport {

// ---------------------------------------------------------------------------
// Matching_Objects_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Matching_Objects_ServiceImpl final : public pyramid::components::tactical_objects::services::provided::Matching_Objects_Service::Service {
public:
    explicit Matching_Objects_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status ReadMatch(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectMatch>* writer) override;

private:
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Object_Of_Interest_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Object_Of_Interest_ServiceImpl final : public pyramid::components::tactical_objects::services::provided::Object_Of_Interest_Service::Service {
public:
    explicit Object_Of_Interest_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

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
    pcl_executor_t* executor_;
};

// ---------------------------------------------------------------------------
// Specific_Object_Detail_Service — gRPC service implementation
// ---------------------------------------------------------------------------

class Specific_Object_Detail_ServiceImpl final : public pyramid::components::tactical_objects::services::provided::Specific_Object_Detail_Service::Service {
public:
    explicit Specific_Object_Detail_ServiceImpl(pcl_executor_t* executor)
        : executor_(executor) {}

    grpc::Status ReadDetail(
        grpc::ServerContext* context,
        const ::pyramid::data_model::common::Query* request,
        grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer) override;

private:
    pcl_executor_t* executor_;
};

} // namespace pyramid::services::tactical_objects::provided::grpc_transport
