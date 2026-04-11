/// \file pcl_transport_shared_memory.h
/// \brief Shared-memory-style central bus transport adapter for PCL.
///
/// Each transport instance joins a process-local named bus as a participant.
/// Topic publish fans out to the other participants on the same bus, while
/// async service requests are routed through an internal gateway topic and
/// correlated back to the originating participant.
#ifndef PCL_TRANSPORT_SHARED_MEMORY_H
#define PCL_TRANSPORT_SHARED_MEMORY_H

#include "pcl/pcl_transport.h"
#include "pcl/pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pcl_shared_memory_transport_t pcl_shared_memory_transport_t;

/// \brief Create a shared-memory bus participant bound to an executor.
///
/// The caller should:
/// 1. call pcl_executor_set_transport(exec, pcl_shared_memory_transport_get_transport(t))
/// 2. configure, activate, and add pcl_shared_memory_transport_gateway_container(t)
///    when remote services are required
///
/// \param bus_name        Logical bus name shared by all participants.
/// \param participant_id  Unique participant identifier within the bus.
/// \param executor        Owning executor for ingress and response delivery.
/// \return Handle on success, NULL on failure.
pcl_shared_memory_transport_t* pcl_shared_memory_transport_create(
    const char*      bus_name,
    const char*      participant_id,
    pcl_executor_t*  executor);

/// \brief Get the transport vtable for pcl_executor_set_transport().
const pcl_transport_t* pcl_shared_memory_transport_get_transport(
    pcl_shared_memory_transport_t* ctx);

/// \brief Get the internal gateway container used for remote services.
///
/// The gateway subscribes to an internal bus topic and invokes matching remote
/// services on the owning executor thread.
pcl_container_t* pcl_shared_memory_transport_gateway_container(
    pcl_shared_memory_transport_t* ctx);

/// \brief Destroy a transport and detach it from the named bus.
void pcl_shared_memory_transport_destroy(pcl_shared_memory_transport_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_SHARED_MEMORY_H
