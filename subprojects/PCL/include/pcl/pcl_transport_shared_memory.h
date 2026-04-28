/// \file pcl_transport_shared_memory.h
/// \brief Shared-memory-style central bus transport adapter for PCL.
///
/// Each transport instance joins a process-local named bus as a participant
/// holding one mailbox slot inside an OS-named shared-memory region.  The
/// adapter is symmetric: every participant can publish, subscribe, host
/// provided services, and call remote services on other participants of the
/// same bus.
///
/// ## Bus Layout
///
/// On first attach, the adapter creates the OS shared-memory object and a
/// header containing a magic number, a version, a participant table, and a
/// per-participant ring buffer for inbound frames.  Subsequent participants
/// reuse the existing region.  When the last participant detaches, the
/// region is unlinked.
///
/// ## Frame Kinds
///
/// All on-bus traffic is carried as discrete frames in the per-participant
/// ring buffers:
///
///   - PUBLISH   - topic name + content type + payload, fanned out from the
///                 source participant to every other participant
///   - SVC_REQ   - service name + serialised request, addressed to the
///                 unique advertised provider on the bus
///   - SVC_RESP  - serialised response, addressed back to the requester via
///                 a sequence id correlating with the original SVC_REQ
///
/// ## Gateway Container
///
/// Service handlers must run on the executor thread (single-threaded
/// runtime), but the bus poll thread is the one that drains SVC_REQ frames.
/// The "gateway container" bridges the two: when a SVC_REQ arrives, the
/// poll thread reformats it and posts it as remote ingress on an internal
/// topic the gateway subscribes to with PCL_ROUTE_REMOTE.  On the next
/// executor spin the gateway's subscriber callback wakes up, looks up the
/// matching provided service via remote-aware port lookup (so per-peer
/// routes and allow-lists are honoured), invokes it, and writes the
/// response back to the originating participant's mailbox as a SVC_RESP
/// frame.
///
/// Pure publishers and pure clients can omit the gateway entirely.  Any
/// participant that exposes a service to the bus must configure, activate,
/// and add the gateway container to its executor.
#ifndef PCL_TRANSPORT_SHARED_MEMORY_H
#define PCL_TRANSPORT_SHARED_MEMORY_H

#include "pcl/pcl_transport.h"
#include "pcl/pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pcl_shared_memory_transport_t pcl_shared_memory_transport_t;

/// \brief Join (or create) the shared-memory bus and bind it to an executor.
///
/// The caller should:
/// 1. call pcl_executor_set_transport(exec, pcl_shared_memory_transport_get_transport(t))
///    or register it per-peer with pcl_executor_register_transport().
/// 2. configure, activate, and add pcl_shared_memory_transport_gateway_container(t)
///    when this participant exposes any provided service to the bus.
///
/// Participant IDs must be unique within a bus; collisions cause this call
/// to return NULL without attaching.
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

/// \brief Get the gateway container used to dispatch inbound service requests.
///
/// The returned container subscribes to an internal bus topic populated by
/// the transport's poll thread and invokes the matching provided service on
/// the executor thread (see file-level "Gateway Container" comment for the
/// full flow).  Activate it and add it to the executor on any participant
/// that wants to be reachable as a service provider; clients and pure
/// publishers may skip it.  Returns NULL when ``ctx`` is NULL.
pcl_container_t* pcl_shared_memory_transport_gateway_container(
    pcl_shared_memory_transport_t* ctx);

/// \brief Detach from the bus and release all transport resources.
///
/// The mailbox slot is freed, the gateway container (if any) is removed
/// from the executor, and the underlying OS shared-memory object is
/// unlinked when the last participant leaves.
void pcl_shared_memory_transport_destroy(pcl_shared_memory_transport_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_SHARED_MEMORY_H
