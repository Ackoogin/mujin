/// \file pcl_transport_routing.h
/// \brief Manifest-driven per-endpoint transport routing.
///
/// A deployment manifest enumerates the transport plugins a component uses and
/// maps each endpoint to one (or more) of them, so a single component can run
/// heterogeneous middleware -- e.g. services over gRPC and topics over UDP. The
/// loader stands up each transport plugin, records its declared capabilities and
/// QoS, registers it on the executor as a named peer, installs the endpoint
/// routes, and validates every route at compose time (failing closed). The
/// returned handle owns the loaded plugin libraries; destroying it tears each
/// transport down (via pcl_plugin_unload_transport) before unloading.
///
/// Manifest format (line-based; '#' comments and blank lines ignored):
/// \code
///   # transport <peer_id> <plugin_path> [config_json...]
///   transport svc_grpc   /opt/plugins/libpyramid_grpc_coupled_plugin.so {"role":"consumed","address":"127.0.0.1:50051"}
///   transport topic_udp  /opt/plugins/libpcl_transport_udp_plugin.so    {"bind":"239.0.0.1:9000"}
///
///   # exclusive <group_name> <side_a_endpoint>[,<side_a_endpoint>...] <side_b_endpoint>[,<side_b_endpoint>...]
///   #   Declares two mutually-exclusive realizations of one logical leg (e.g.
///   #   an rpc realization vs. a pub/sub realization of the same request/
///   #   response transaction). Any number of same-side endpoints may be
///   #   routed together freely; routing at least one endpoint from EACH side
///   #   is a compose-time error (PCL_ERR_STATE), reported by whichever
///   #   `route` line completes the second side. An endpoint named in no
///   #   `exclusive` group is entirely unaffected by this check. Must be
///   #   declared before any `route` line for one of its member endpoints
///   #   (declare-before-use, same convention as `transport` before `route`).
///   exclusive ma_action.request_leg ma_action.create,ma_action.update,ma_action.cancel agra.ma_action.request
///
///   # route <endpoint_name> <kind> <peer_id>[,<peer_id>...] [reliability]
///   #   kind        : publisher | subscriber | provided | consumed | stream_provided
///   #   reliability : best_effort | reliable   (optional QoS floor)
///   route create_requirement consumed  svc_grpc  reliable
///   route object_evidence    publisher topic_udp best_effort
///   route ma_action.create   consumed  svc_grpc          # side A of ma_action.request_leg
///   route ma_action.update   consumed  svc_grpc          # side A of ma_action.request_leg
///   route ma_action.cancel   consumed  svc_grpc          # side A of ma_action.request_leg -- OK together
///   # route agra.ma_action.request publisher topic_udp   # would fail closed: side B of the same group
/// \endcode
#ifndef PCL_TRANSPORT_ROUTING_H
#define PCL_TRANSPORT_ROUTING_H

#include "pcl_executor.h"
#include "pcl_types.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Opaque owner of the transport plugins loaded for a routing manifest.
typedef struct pcl_transport_routing_t pcl_transport_routing_t;

/// \brief Load a routing manifest: stand up its transports, register them on
///        \p e, install + validate every endpoint route.
///
/// On success \p *out_routing receives a handle owning the loaded plugin
/// libraries (free it with \ref pcl_transport_routing_destroy, typically at
/// executor teardown). On failure nothing is left registered for that manifest:
/// any transports already loaded are torn down, and a precise reason is written
/// to \p diag (when non-NULL and \p diag_size > 0).
///
/// Fails closed:
/// - PCL_ERR_NOT_FOUND  : manifest missing, or a route targets an unknown peer;
/// - PCL_ERR_STATE      : a routed transport lacks the endpoint's required
///                        capability or QoS floor, or a route completes both
///                        sides of a declared `exclusive` group;
/// - PCL_ERR_INVALID    : malformed manifest line / bad arguments.
///
/// An empty or comment-only manifest is PCL_OK with an empty routing handle.
pcl_status_t pcl_transport_routing_load(pcl_executor_t*           e,
                                        const char*               manifest_path,
                                        pcl_transport_routing_t** out_routing,
                                        char*                     diag,
                                        size_t                    diag_size);

/// \brief Tear down all transports loaded for \p routing and free it.
///
/// Each transport is released via \ref pcl_plugin_unload_transport (teardown
/// before dlclose). Safe to call with NULL.
void pcl_transport_routing_destroy(pcl_transport_routing_t* routing);

/// \brief Number of transport plugins \p routing loaded (0 if NULL).
size_t pcl_transport_routing_transport_count(const pcl_transport_routing_t* routing);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_ROUTING_H
