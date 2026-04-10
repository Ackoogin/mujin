// Auto-generated service binding header
// Generated from: consumed.proto by generate_bindings.py
// Namespace: pyramid::services::tactical_objects::consumed
//
// Architecture: component logic > service binding (this) > PCL
//
// This header provides:
//   1. Wire-name constants and topic constants
//   2. EntityActions handler base class (ServiceHandler — override Handle*)
//   3. PCL binding functions (subscribe*, publish*)
//   4. msgToString utility for PCL message payloads
#pragma once

#include "pyramid_data_model_types.hpp"

#include "pyramid_services_tactical_objects_wire_types.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_types.h>

#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::consumed {

// ---------------------------------------------------------------------------
// Service wire-name constants (generated from proto)
// ---------------------------------------------------------------------------

constexpr const char* kSvcReadDetail         = "object_evidence.read_detail";
constexpr const char* kSvcCreateRequirement  = "object_solution_evidence.create_requirement";
constexpr const char* kSvcReadRequirement    = "object_solution_evidence.read_requirement";
constexpr const char* kSvcUpdateRequirement  = "object_solution_evidence.update_requirement";
constexpr const char* kSvcDeleteRequirement  = "object_solution_evidence.delete_requirement";
constexpr const char* kSvcReadCapability     = "object_source_capability.read_capability";

// ---------------------------------------------------------------------------
// Standard topic name constants
// ---------------------------------------------------------------------------

constexpr const char* kTopicObjectEvidence = "standard.object_evidence";

// ---------------------------------------------------------------------------
// Service channel discriminant
// ---------------------------------------------------------------------------

enum class ServiceChannel {
    ReadDetail,
    CreateRequirement,
    ReadRequirement,
    UpdateRequirement,
    DeleteRequirement,
    ReadCapability,
};

// ---------------------------------------------------------------------------
// PCL message utility
// ---------------------------------------------------------------------------

/// \brief Convert a raw PCL message buffer to a std::string.
std::string msgToString(const void* data, unsigned size);

// ---------------------------------------------------------------------------
// EntityActions handler base class
//
// Subclass and override the handle* methods to implement business logic.
// Default implementations return empty / null values (stub behaviour).
// ---------------------------------------------------------------------------

using pyramid::data_model::Ack;
using pyramid::data_model::Capability;
using pyramid::data_model::Identifier;
using pyramid::data_model::ObjectDetail;
using pyramid::data_model::ObjectEvidenceRequirement;
using pyramid::data_model::Query;
namespace wire_types = pyramid::services::tactical_objects::wire_types;

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;

    // Object_Evidence_Service
    virtual std::vector<ObjectDetail>
    handleReadDetail(const Query& request);

    // Object_Solution_Evidence_Service
    virtual Identifier
    handleCreateRequirement(const ObjectEvidenceRequirement& request);

    virtual std::vector<ObjectEvidenceRequirement>
    handleReadRequirement(const Query& request);

    virtual Ack
    handleUpdateRequirement(const ObjectEvidenceRequirement& request);

    virtual Ack
    handleDeleteRequirement(const Identifier& request);

    // Object_Source_Capability_Service
    virtual std::vector<Capability>
    handleReadCapability(const Query& request);
};

// ---------------------------------------------------------------------------
// PCL binding functions — Subscribe / Publish (typed)
// ---------------------------------------------------------------------------

/// \brief Subscribe to object-evidence publications on kTopicObjectEvidence.
pcl_port_t* subscribeObjectEvidence(pcl_container_t*  container,
                                    pcl_sub_callback_t callback,
                                    void*             user_data = nullptr,
                                    const char*       content_type = "application/json");

/// \brief Publish a typed message on kTopicObjectEvidence.
///
/// \p publisher must be the pcl_port_t* returned by addPublisher for
/// kTopicObjectEvidence, obtained during on_configure.
pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const wire_types::ObjectEvidence& payload,
                                   const char*        content_type = "application/json");

pcl_status_t publishObjectEvidence(pcl_port_t*        publisher,
                                   const std::string& payload,
                                   const char*        content_type = "application/json");

// ---------------------------------------------------------------------------
// Dispatch — deserialises request, calls handler, serialises response.
//
// Response buffer is heap-allocated via std::malloc; caller frees with std::free.
// ---------------------------------------------------------------------------

void dispatch(ServiceHandler& handler,
              ServiceChannel  channel,
              const void*     request_buf,
              size_t          request_size,
              const char*     content_type,
              void**          response_buf,
              size_t*         response_size);

inline void dispatch(ServiceHandler& handler,
                     ServiceChannel  channel,
                     const void*     request_buf,
                     size_t          request_size,
                     void**          response_buf,
                     size_t*         response_size)
{
    dispatch(handler, channel, request_buf, request_size, "application/json", response_buf, response_size);
}

} // namespace pyramid::services::tactical_objects::consumed
