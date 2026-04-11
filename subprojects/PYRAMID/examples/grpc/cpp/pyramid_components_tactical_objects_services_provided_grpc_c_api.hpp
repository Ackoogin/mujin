#pragma once

#include "pyramid_services_tactical_objects_grpc_c_api_support.hpp"

extern "C" {

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_provided_matching_objects_service_read_match_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char* grpc_provided_matching_objects_service_read_match_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_create_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_provided_object_of_interest_service_create_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_read_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_provided_object_of_interest_service_read_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_update_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_provided_object_of_interest_service_update_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_provided_object_of_interest_service_delete_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_provided_object_of_interest_service_delete_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_provided_specific_object_detail_service_read_detail_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_provided_specific_object_detail_service_read_detail_json(
    const char* endpoint, const char* request_json);

}
