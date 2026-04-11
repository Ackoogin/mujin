#pragma once

#include "pyramid_services_tactical_objects_grpc_c_api_support.hpp"

extern "C" {

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_consumed_object_evidence_service_read_detail_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_consumed_object_evidence_service_read_detail_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_create_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_consumed_object_solution_evidence_service_create_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_read_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_consumed_object_solution_evidence_service_read_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_update_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_consumed_object_solution_evidence_service_update_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_consumed_object_solution_evidence_service_delete_requirement_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_consumed_object_solution_evidence_service_delete_requirement_json(
    const char* endpoint, const char* request_json);

PYRAMID_GRPC_C_SHIM_EXPORT char*
pyramid_components_tactical_objects_services_consumed_object_source_capability_service_read_capability_json(
    const char* endpoint, const char* request_json);
PYRAMID_GRPC_C_SHIM_EXPORT char*
grpc_consumed_object_source_capability_service_read_capability_json(
    const char* endpoint, const char* request_json);

}
