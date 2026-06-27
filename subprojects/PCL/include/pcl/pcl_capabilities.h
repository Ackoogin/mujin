/// \file pcl_capabilities.h
/// \brief Transport interaction-capability flags and vtable derivation.
///
/// A transport plugin advertises which interaction patterns it supports so the
/// framework can validate, at compose time, that an endpoint's required pattern
/// is actually served by the transport it is routed to (fail closed otherwise).
///
/// Capabilities may be declared explicitly by a plugin (via the optional
/// pcl_transport_plugin_caps symbol, see pcl_plugin.h) or, when a plugin does
/// not declare them, conservatively derived from which vtable slots are non-NULL
/// (see \ref pcl_transport_caps_from_vtable).
#ifndef PCL_CAPABILITIES_H
#define PCL_CAPABILITIES_H

#include "pcl_transport.h"
#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// pcl_transport_caps_t and the PCL_CAP_* flags are defined in pcl_types.h so the
// transport/executor API can reference them without a circular include.
//
// PCL_CAP_RPC_ACTION has no vtable slot, so it can only be advertised by an
// explicit pcl_transport_plugin_caps symbol; it is never derived from a vtable.

/// \brief Conservatively derive capabilities from non-NULL vtable slots.
///
/// Used when a plugin does not export an explicit capability symbol. The mapping
/// is intentionally loose -- a non-NULL slot means "may support", so the derived
/// mask is an upper bound on what the vtable can be asked to do:
///
/// - \ref PCL_CAP_PUBSUB     if \c publish or \c subscribe is set.
/// - \ref PCL_CAP_RPC_UNARY  if \c serve or \c invoke_async is set.
/// - \ref PCL_CAP_RPC_STREAM if \c invoke_stream or \c stream_send is set.
///
/// \ref PCL_CAP_RPC_ACTION is never derived (no corresponding slot).
///
/// Returns \ref PCL_CAP_NONE when \p transport is NULL.
pcl_transport_caps_t pcl_transport_caps_from_vtable(
    const pcl_transport_t* transport);

/// \brief The transport capability an endpoint of \p kind requires.
///
/// - PUBLISHER / SUBSCRIBER        -> \ref PCL_CAP_PUBSUB
/// - PROVIDED / CONSUMED           -> \ref PCL_CAP_RPC_UNARY
/// - STREAM_PROVIDED               -> \ref PCL_CAP_RPC_STREAM
///
/// Returns \ref PCL_CAP_NONE for an unrecognised kind (no requirement).
pcl_transport_caps_t pcl_endpoint_required_caps(pcl_endpoint_kind_t kind);

/// \brief Whether \p have includes every capability in \p required.
///
/// Returns 1 if `(have & required) == required` (vacuously true when
/// \p required is \ref PCL_CAP_NONE), else 0.
int pcl_transport_caps_supports(pcl_transport_caps_t have,
                                pcl_transport_caps_t required);

#ifdef __cplusplus
}
#endif

#endif // PCL_CAPABILITIES_H
