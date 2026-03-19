/// \file pcl_types.h
/// \brief PYRAMID Container Library core types.
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

typedef struct pcl_executor_t  pcl_executor_t;
typedef struct pcl_container_t pcl_container_t;
typedef struct pcl_port_t      pcl_port_t;

// -- Return codes --------------------------------------------------------

typedef enum {
  PCL_OK              =  0,
  PCL_ERR_INVALID     = -1,   // NULL handle or bad argument
  PCL_ERR_STATE       = -2,   // wrong lifecycle state for operation
  PCL_ERR_TIMEOUT     = -3,   // deadline exceeded
  PCL_ERR_CALLBACK    = -4,   // user callback returned error
  PCL_ERR_NOMEM       = -5,   // allocation failure
  PCL_ERR_NOT_FOUND   = -6,   // parameter key / port name not found
  PCL_ERR_PORT_CLOSED = -7,   // port not available (container inactive)
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
  PCL_PORT_PUBLISHER  = 0,
  PCL_PORT_SUBSCRIBER = 1,
  PCL_PORT_SERVICE    = 2,    // request-reply server
  PCL_PORT_CLIENT     = 3,    // request-reply client
} pcl_port_type_t;

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
/// The container borrows pointers — caller owns the underlying memory
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
/// \param resp       Response message (borrow — do not retain pointers after return).
/// \param user_data  Caller-supplied context passed to invoke_remote_async.
typedef void (*pcl_resp_cb_fn_t)(const pcl_msg_t* resp, void* user_data);

#ifdef __cplusplus
}
#endif

#endif // PCL_TYPES_H
