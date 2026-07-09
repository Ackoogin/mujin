/// \file pcl_types.h
/// \brief PYRAMID Composition Library core types.
///
/// All public types used across the PCL API.  Pure C, no external dependencies.
#ifndef PCL_TYPES_H
#define PCL_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// -- Opaque handles ------------------------------------------------------

typedef struct pcl_executor_t      pcl_executor_t;
typedef struct pcl_container_t     pcl_container_t;
typedef struct pcl_port_t          pcl_port_t;
typedef struct pcl_svc_context_t   pcl_svc_context_t;
typedef struct pcl_stream_context_t pcl_stream_context_t;

#define PCL_MAX_ENDPOINT_PEERS 8u

// -- Return codes --------------------------------------------------------

typedef enum {
  PCL_OK              =  0,
  PCL_PENDING         =  1,   // operation deferred (e.g., service will respond later)
  PCL_STREAMING       =  2,   // stream opened, more data coming
  PCL_ERR_INVALID     = -1,   // NULL handle or bad argument
  PCL_ERR_STATE       = -2,   // wrong lifecycle state for operation
  PCL_ERR_TIMEOUT     = -3,   // deadline exceeded
  PCL_ERR_CALLBACK    = -4,   // user callback returned error
  PCL_ERR_NOMEM       = -5,   // allocation failure
  PCL_ERR_NOT_FOUND   = -6,   // parameter key / port name not found
  PCL_ERR_PORT_CLOSED = -7,   // port not available (container inactive)
  PCL_ERR_CANCELLED   = -8,   // stream cancelled by peer
} pcl_status_t;

// -- Lifecycle states ----------------------------------------------------

typedef enum {
  PCL_STATE_UNCONFIGURED = 0,
  PCL_STATE_CONFIGURED   = 1,
  PCL_STATE_ACTIVE       = 2,
  PCL_STATE_FINALIZED    = 3,
} pcl_state_t;

// -- Port types ----------------------------------------------------------

typedef enum {
  PCL_PORT_PUBLISHER       = 0,
  PCL_PORT_SUBSCRIBER      = 1,
  PCL_PORT_SERVICE         = 2,  // request-reply server
  PCL_PORT_CLIENT          = 3,  // request-reply client
  PCL_PORT_STREAM_SERVICE  = 4,  // streaming server
} pcl_port_type_t;

// -- Transport QoS -------------------------------------------------------
//
// A minimal, ordered QoS profile carried alongside capabilities. An endpoint
// may declare a QoS *floor* (the weakest profile it tolerates); a transport
// advertises the QoS it *offers*. At compose time the framework checks the
// offered profile meets the floor, failing closed otherwise (see
// pcl_qos_satisfies in pcl_capabilities.h). Helpers live in pcl_capabilities.h;
// the types are here so the transport/executor/routing API can use them without
// a circular include.
//
// Reliability is the first (and currently only) dimension, deliberately ordered
// so a numeric >= comparison expresses "meets the floor":
//   UNSPECIFIED (0) < BEST_EFFORT (1) < RELIABLE (2).
// A transport that does not declare its reliability is UNSPECIFIED and therefore
// satisfies only an UNSPECIFIED floor -- asking for reliability requires the
// transport to prove it offers it (fail closed).

typedef enum {
  PCL_QOS_RELIABILITY_UNSPECIFIED = 0,  ///< Not declared; no guarantee.
  PCL_QOS_RELIABILITY_BEST_EFFORT = 1,  ///< Delivery may be dropped.
  PCL_QOS_RELIABILITY_RELIABLE    = 2,  ///< Delivery is retried/guaranteed.
} pcl_qos_reliability_t;

typedef struct {
  pcl_qos_reliability_t reliability;
} pcl_qos_t;

// -- Endpoint routing ----------------------------------------------------

typedef enum {
  PCL_ROUTE_NONE   = 0,
  PCL_ROUTE_LOCAL  = 1 << 0,
  PCL_ROUTE_REMOTE = 1 << 1,
} pcl_route_mode_t;

typedef enum {
  PCL_ENDPOINT_PUBLISHER      = 0,
  PCL_ENDPOINT_SUBSCRIBER     = 1,
  PCL_ENDPOINT_PROVIDED       = 2,
  PCL_ENDPOINT_CONSUMED       = 3,
  PCL_ENDPOINT_STREAM_PROVIDED = 4,
  /// \brief Client side of a server-streaming rpc, symmetric with
  ///        PCL_ENDPOINT_STREAM_PROVIDED. Distinct from PCL_ENDPOINT_CONSUMED
  ///        (unary) so pcl_endpoint_required_caps() can require
  ///        PCL_CAP_RPC_STREAM specifically -- a manifest route for a
  ///        streaming client invoke must not compose against a unary-only
  ///        transport just because "consumed" is satisfied by RPC_UNARY.
  PCL_ENDPOINT_STREAM_CONSUMED = 5,
} pcl_endpoint_kind_t;

typedef struct {
  const char*               endpoint_name;
  pcl_endpoint_kind_t       endpoint_kind;
  uint32_t                  route_mode;
  const char* const*        peer_ids;
  uint32_t                  peer_count;
  /// Weakest QoS the endpoint tolerates on its remote leg. Zero-initialised
  /// (PCL_QOS_RELIABILITY_UNSPECIFIED) means "no floor" -- back-compatible with
  /// callers that predate QoS. Validated at compose time against the offered
  /// QoS of the transport(s) the endpoint routes to.
  pcl_qos_t                 qos_floor;
} pcl_endpoint_route_t;

// -- Transport interaction capabilities ----------------------------------
//
// Bitmask of the interaction patterns a transport can carry. Helpers live in
// pcl_capabilities.h; the type is here so the transport/executor API can use it
// without a circular include.

typedef uint32_t pcl_transport_caps_t;

#define PCL_CAP_NONE        0x0u  ///< No interaction capability.
#define PCL_CAP_PUBSUB      0x1u  ///< Publish/subscribe (topic).
#define PCL_CAP_RPC_UNARY   0x2u  ///< Unary request/response RPC.
#define PCL_CAP_RPC_STREAM  0x4u  ///< Server-streaming (or richer) RPC.
#define PCL_CAP_RPC_ACTION  0x8u  ///< Action (goal/feedback/result).

// -- Log levels ----------------------------------------------------------

typedef enum {
  PCL_LOG_DEBUG = 0,
  PCL_LOG_INFO  = 1,
  PCL_LOG_WARN  = 2,
  PCL_LOG_ERROR = 3,
  PCL_LOG_FATAL = 4,
} pcl_log_level_t;

// -- Message buffer ------------------------------------------------------

/// \brief Generic message exchanged through ports.
///
/// The container borrows pointers -- caller owns the underlying memory
/// and must keep it alive for the duration of the call.
///
/// When crossing threads, prefer \ref pcl_executor_post_incoming(), which
/// deep-copies the message into the executor's ingress queue.
typedef struct {
  const void* data;           // serialized (or raw struct) payload
  uint32_t    size;           // payload size in bytes
  const char* type_name;     // e.g. "WorldState", "GetFact_Request"
} pcl_msg_t;

/// \brief Callback fired on the executor thread when an async service response arrives.
/// \param resp       Response message (borrow -- do not retain pointers after return).
/// \param user_data  Caller-supplied context passed to invoke_remote_async.
typedef void (*pcl_resp_cb_fn_t)(const pcl_msg_t* resp, void* user_data);

/// \brief Callback fired for each message in a streaming service response.
/// \param msg        Stream message (borrow -- do not retain pointers after return).
/// \param end        True if this is the final message (stream complete).
/// \param status     PCL_OK for normal messages, error code on abort/cancel.
/// \param user_data  Caller-supplied context passed to invoke_stream.
typedef void (*pcl_stream_msg_fn_t)(const pcl_msg_t* msg,
                                    bool             end,
                                    pcl_status_t     status,
                                    void*            user_data);

#ifdef __cplusplus
}
#endif

#endif // PCL_TYPES_H
