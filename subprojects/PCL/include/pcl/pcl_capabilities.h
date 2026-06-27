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

/// \brief Bitmask of transport interaction capabilities.
typedef uint32_t pcl_transport_caps_t;

/// \brief No interaction capability.
#define PCL_CAP_NONE        0x0u

/// \brief Publish/subscribe (topic) interaction.
#define PCL_CAP_PUBSUB      0x1u

/// \brief Unary request/response RPC (provided or consumed).
#define PCL_CAP_RPC_UNARY   0x2u

/// \brief Server-streaming (or richer streaming) RPC.
#define PCL_CAP_RPC_STREAM  0x4u

/// \brief Action interaction (goal / feedback / result).
///
/// There is no vtable slot for actions, so this capability can only be
/// advertised by an explicit pcl_transport_plugin_caps symbol; it is never
/// derived from the vtable.
#define PCL_CAP_RPC_ACTION  0x8u

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

#ifdef __cplusplus
}
#endif

#endif // PCL_CAPABILITIES_H
