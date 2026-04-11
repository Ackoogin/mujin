#include "pyramid_components_tactical_objects_services_provided_grpc_c_api.hpp"

#include "pyramid_services_tactical_objects_protobuf_shim.h"

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include "pyramid/data_model/base.pb.h"
#include "pyramid/data_model/common.pb.h"
#include "pyramid/data_model/tactical.pb.h"

namespace grpc_c_api = pyramid::services::tactical_objects::grpc_transport::c_api;
namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_tactical = pyramid::data_model::tactical;

extern "C" {

char*
pyramid_components_tactical_objects_services_provided_matching_objects_service_read_match_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeServerStreamJsonRpc<
      proto_common::Query, proto_tactical::ObjectMatch>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Query_to_protobuf_json,
      pyramid_services_tactical_objects_ObjectMatch_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Matching_Objects_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_common::Query& request) {
        return stub->ReadMatch(context, request);
      });
}

char* grpc_provided_matching_objects_service_read_match_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_provided_matching_objects_service_read_match_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_create_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeUnaryJsonRpc<
      proto_tactical::ObjectInterestRequirement, proto_base::Identifier>(
      endpoint, request_json,
      pyramid_services_tactical_objects_ObjectInterestRequirement_to_protobuf_json,
      pyramid_services_tactical_objects_Identifier_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Of_Interest_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_tactical::ObjectInterestRequirement& request,
         proto_base::Identifier* response) {
        return stub->CreateRequirement(context, request, response);
      });
}

char* grpc_provided_object_of_interest_service_create_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_provided_object_of_interest_service_create_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_read_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeServerStreamJsonRpc<
      proto_common::Query, proto_tactical::ObjectInterestRequirement>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Query_to_protobuf_json,
      pyramid_services_tactical_objects_ObjectInterestRequirement_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Of_Interest_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_common::Query& request) {
        return stub->ReadRequirement(context, request);
      });
}

char* grpc_provided_object_of_interest_service_read_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_provided_object_of_interest_service_read_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_update_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeUnaryJsonRpc<
      proto_tactical::ObjectInterestRequirement, proto_common::Ack>(
      endpoint, request_json,
      pyramid_services_tactical_objects_ObjectInterestRequirement_to_protobuf_json,
      pyramid_services_tactical_objects_Ack_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Of_Interest_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_tactical::ObjectInterestRequirement& request,
         proto_common::Ack* response) {
        return stub->UpdateRequirement(context, request, response);
      });
}

char* grpc_provided_object_of_interest_service_update_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_provided_object_of_interest_service_update_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_delete_requirement_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeUnaryJsonRpc<proto_base::Identifier, proto_common::Ack>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Identifier_to_protobuf_json,
      pyramid_services_tactical_objects_Ack_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Object_Of_Interest_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_base::Identifier& request, proto_common::Ack* response) {
        return stub->DeleteRequirement(context, request, response);
      });
}

char* grpc_provided_object_of_interest_service_delete_requirement_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_provided_object_of_interest_service_delete_requirement_json(
          endpoint, request_json);
}

char*
pyramid_components_tactical_objects_services_provided_specific_object_detail_service_read_detail_json(
    const char* endpoint, const char* request_json) {
  return grpc_c_api::invokeServerStreamJsonRpc<
      proto_common::Query, proto_tactical::ObjectDetail>(
      endpoint, request_json,
      pyramid_services_tactical_objects_Query_to_protobuf_json,
      pyramid_services_tactical_objects_ObjectDetail_from_protobuf_json,
      [](const std::shared_ptr<grpc::Channel>& channel) {
        return proto_services::Specific_Object_Detail_Service::NewStub(channel);
      },
      [](auto* stub, grpc::ClientContext* context,
         const proto_common::Query& request) {
        return stub->ReadDetail(context, request);
      });
}

char* grpc_provided_specific_object_detail_service_read_detail_json(
    const char* endpoint, const char* request_json) {
  return
      pyramid_components_tactical_objects_services_provided_specific_object_detail_service_read_detail_json(
          endpoint, request_json);
}

}  // extern "C"
