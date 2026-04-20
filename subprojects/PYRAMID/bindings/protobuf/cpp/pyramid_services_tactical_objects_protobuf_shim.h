#pragma once

#include <cstddef>

extern "C" {

void* pyramid_services_tactical_objects_GeodeticPosition_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_GeodeticPosition_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_PolyArea_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_PolyArea_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Achievement_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Achievement_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Entity_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Entity_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_CircleArea_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_CircleArea_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Point_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Point_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Contraint_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Contraint_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Ack_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Ack_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Query_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Query_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectDetail_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectDetail_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectEvidenceRequirement_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectInterestRequirement_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectInterestRequirement_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectMatch_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectMatch_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Capability_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Capability_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_Identifier_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_Identifier_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectDetailArray_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectDetailArray_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_CapabilityArray_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_CapabilityArray_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectMatchArray_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectMatchArray_from_protobuf_json(
    const void* data, size_t size);

void* pyramid_services_tactical_objects_ObjectInterestRequirementArray_to_protobuf_json(
    const char* json, size_t* size_out);
char* pyramid_services_tactical_objects_ObjectInterestRequirementArray_from_protobuf_json(
    const void* data, size_t size);

}
