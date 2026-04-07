/// \file pcl_bridge.h
/// \brief PCL bridge: semantic and multiplicity adapter between components.
///
/// A bridge is a managed container that subscribes to one topic, applies a
/// user-supplied transform function, and immediately dispatches the result to
/// a different topic — potentially with a different type name and payload
/// layout.  This is the standard mechanism for:
///
///   - Unit conversion       e.g. float m/s → int32 km/h
///   - Encoding change       e.g. int32 state enum → const-char* label
///   - Multiplicity          one input topic fan-out to many output topics
///   - Protocol translation  between component layers
///
/// Usage
/// -----
/// \code
///   pcl_bridge_t* b = pcl_bridge_create(exec,
///                                       "speed_conv",
///                                       "sensors/speed_mps", "SpeedMps",
///                                       "hmi/speed_kmph",   "SpeedKmph",
///                                       mps_to_kmph, NULL);
///
///   pcl_executor_add(exec, pcl_bridge_container(b));
///   pcl_container_configure(pcl_bridge_container(b));
///   pcl_container_activate(pcl_bridge_container(b));
///   // ... spin ...
///   pcl_bridge_destroy(b);   // also destroys the internal container
/// \endcode
///
/// Threading model
/// ---------------
/// The transform function and the subsequent dispatch are both called on the
/// executor thread (same thread as every other container callback).  No
/// additional locking is required inside the transform function.
///
/// The bridge uses pcl_executor_dispatch_incoming() for immediate, same-tick
/// delivery to downstream subscribers.  This is safe because PCL's intra-
/// process dispatch path is re-entrant (no locks) and single-threaded.
#ifndef PCL_BRIDGE_H
#define PCL_BRIDGE_H

#include "pcl_types.h"
#include "pcl_container.h"
#include "pcl_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

// -- Transform function --------------------------------------------------

/// \brief Bridge transform callback.
///
/// Called on the executor thread each time a message arrives on the bridge's
/// input topic.  The function must populate \p out and return PCL_OK to
/// forward the result; any other return code suppresses forwarding silently.
///
/// \param in        Incoming message (borrowed; valid for the duration of the
///                  call only — do not store the pointer).
/// \param out       Output message to populate.  Set out->data, out->size.
///                  out->type_name is pre-populated with the out_type passed
///                  to pcl_bridge_create(); the function may override it.
/// \param user_data Opaque pointer supplied at bridge creation time.
/// \return PCL_OK   → transformed message is dispatched to out_topic.
///         anything else → message is dropped silently.
typedef pcl_status_t (*pcl_bridge_fn_t)(const pcl_msg_t* in,
                                        pcl_msg_t*       out,
                                        void*            user_data);

// -- Opaque bridge handle ------------------------------------------------

typedef struct pcl_bridge_t pcl_bridge_t;

// -- Lifecycle -----------------------------------------------------------

/// \brief Create a bridge between two topics.
///
/// The bridge allocates an internal container (owned by the bridge).
/// Callers must:
///   1. Call pcl_executor_add(exec, pcl_bridge_container(b)).
///   2. Configure and activate the container through the normal lifecycle.
///
/// The executor pointer is captured at creation time; the bridge dispatches
/// transformed messages via pcl_executor_dispatch_incoming() on that executor.
///
/// \param executor   Executor that will drive this bridge.
/// \param name       Human-readable name for the internal container (copied).
/// \param in_topic   Topic to subscribe to (copied).
/// \param in_type    Expected type name on the input side (copied).
/// \param out_topic  Topic to dispatch transformed messages to (copied).
/// \param out_type   Type name for outbound messages (copied; pre-filled into
///                   out->type_name before the transform function is called).
/// \param fn         Transform function — must not be NULL.
/// \param user_data  Opaque pointer forwarded verbatim to every \p fn call.
/// \return           New bridge handle, or NULL if any argument is NULL or on
///                   allocation failure.
pcl_bridge_t* pcl_bridge_create(pcl_executor_t*  executor,
                                 const char*      name,
                                 const char*      in_topic,
                                 const char*      in_type,
                                 const char*      out_topic,
                                 const char*      out_type,
                                 pcl_bridge_fn_t  fn,
                                 void*            user_data);

/// \brief Get the bridge's internal container.
///
/// Add this to the executor and drive it through configure/activate.
/// The bridge owns the container — do not call pcl_container_destroy() on it
/// directly; use pcl_bridge_destroy() instead.
pcl_container_t* pcl_bridge_container(pcl_bridge_t* b);

/// \brief Destroy the bridge and its internal container.
///
/// Must only be called after the executor has been stopped (or the container
/// removed from the executor).  Frees all bridge-owned resources.
/// Passing NULL is a no-op.
void pcl_bridge_destroy(pcl_bridge_t* b);

#ifdef __cplusplus
}
#endif

#endif // PCL_BRIDGE_H
