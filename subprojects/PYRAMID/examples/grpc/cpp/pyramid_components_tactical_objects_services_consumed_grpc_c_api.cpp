#include "pyramid_components_tactical_objects_services_consumed_grpc_c_api.hpp"

#include "pyramid_services_tactical_objects_protobuf_shim.h"

#include "pyramid/components/tactical_objects/services/consumed.grpc.pb.h"

#include "pyramid/data_model/base.pb.h"
#include "pyramid/data_model/common.pb.h"
#include "pyramid/data_model/tactical.pb.h"

namespace grpc_c_api = pyramid::services::tactical_objects::grpc_transport::c_api;
namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_services = pyramid::components::tactical_objects::services::consumed;
namespace proto_tactical = pyramid::data_model::tactical;

extern "C" {

char*
pyramid_components_tactical_objects_services_consumed_object_evidence_service_read_detail_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeServerStreamJsonRpc<
      proto_common::Query, proto_tactical::ObjectDetail>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Query_to_protobuf_json,
      pyramid_services_tactical_objects_ObjectDetail_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Evidence_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_common::Query& request) {
        return stub->ReadDetail(context, request);
      });
}

char* grpc_consumed_object_evidence_service_read_detail_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_consumed_object_evidence_service_read_detail_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_create_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeUnaryJsonRpc<
      proto_tactical::ObjectEvidenceRequirement, proto_base::Identifier>(
      endpoint, request_json,
      pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_protobuf_json,
      pyramid_services_tactical_objects_Identifier_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Solution_Evidence_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_tactical::ObjectEvidenceRequirement& request,
         proto_base::Identifier* response) {
        return stub->CreateRequirement(context, request, response);
      });
}

char* grpc_consumed_object_solution_evidence_service_create_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_create_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_read_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeServerStreamJsonRpc<
      proto_common::Query, proto_tactical::ObjectEvidenceRequirement>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Query_to_protobuf_json,
      pyramid_services_tactical_objects_ObjectEvidenceRequirement_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Solution_Evidence_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_common::Query& request) {
        return stub->ReadRequirement(context, request);
      });
}

char* grpc_consumed_object_solution_evidence_service_read_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_read_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_update_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeUnaryJsonRpc<
      proto_tactical::ObjectEvidenceRequirement, proto_common::Ack>(
      endpoint, request_json,
      pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_protobuf_json,
      pyramid_services_tactical_objects_Ack_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Solution_Evidence_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_tactical::ObjectEvidenceRequirement& request,
         proto_common::Ack* response) {
        return stub->UpdateRequirement(context, request, response);
      });
}

char* grpc_consumed_object_solution_evidence_service_update_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_update_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_delete_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeUnaryJsonRpc<proto_base::Identifier, proto_common::Ack>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Identifier_to_protobuf_json,
      pyramid_services_tactical_objects_Ack_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Solution_Evidence_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_base::Identifier& request, proto_common::Ack* response) {
        return stub->DeleteRequirement(context, request, response);
      });
}

char* grpc_consumed_object_solution_evidence_service_delete_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_delete_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_consumed_object_source_capability_service_read_capability_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeServerStreamJsonRpc<
      proto_common::Query, proto_common::Capability>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Query_to_protobuf_json,
      pyramid_services_tactical_objects_Capability_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Source_Capability_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_common::Query& request) {
        return stub->ReadCapability(context, request);
      });
}

char* grpc_consumed_object_source_capability_service_read_capability_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_consumed_object_source_capability_service_read_capability_json(
          endpoint, request_json);
}

}  // extern "C"
