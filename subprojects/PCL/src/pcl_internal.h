/// \file pcl_internal.h
/// \brief PCL internal shared types between pcl_container.c and pcl_executor.c.
///
/// NOT part of the public API.
#ifndef PCL_INTERNAL_H
#define PCL_INTERNAL_H

#include "pcl/pcl_container.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCL_MAX_PORTS  64
#define PCL_MAX_PARAMS 128
#define PCL_MAX_TRANSPORTS 16
#define PCL_MAX_ENDPOINT_ROUTES 128

// -- Parameter value tagged union ----------------------------------------

typedef enum {
  PCL_PARAM_STR,
  PCL_PARAM_F64,
  PCL_PARAM_I64,
  PCL_PARAM_BOOL,
} pcl_param_kind_t;

typedef struct {
  char              key[128];
  pcl_param_kind_t  kind;
  union {
    char    str_val[256];
    double  f64_val;
    int64_t i64_val;
    bool    bool_val;
  } u;
} pcl_param_t;

// -- Internal port representation ----------------------------------------

struct pcl_port_t {
  pcl_port_type_t       type;
  char                  name[128];
  char                  type_name[128];
  uint32_t              route_mode;
  int                   route_configured;
  char                  peer_ids[PCL_MAX_ENDPOINT_PEERS][64];
  uint32_t              peer_count;

  pcl_sub_callback_t    sub_cb;
  void*                 sub_user_data;

  pcl_service_handler_t svc_handler;
  void*                 svc_user_data;

  pcl_stream_handler_t  stream_handler;
  void*                 stream_user_data;

  pcl_container_t*      owner;
};

// -- Internal container representation -----------------------------------

// -- Internal executor representation ------------------------------------

#include "pcl/pcl_transport.h"

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
typedef CRITICAL_SECTION pcl_mutex_t;
#else
#  include <pthread.h>
typedef pthread_mutex_t pcl_mutex_t;
#endif

#define PCL_MAX_CONTAINERS 64

typedef struct pcl_pending_msg_t {
  char*                     topic;
  char*                     type_name;
  char*                     source_peer_id;
  uint32_t                  source_route_mode;
  void*                     data;
  uint32_t                  size;
  struct pcl_pending_msg_t* next;
} pcl_pending_msg_t;

typedef struct pcl_resp_cb_node_t {
  pcl_resp_cb_fn_t            cb;
  void*                       user_data;
  void*                       data;
  uint32_t                    size;
  char*                       type_name;
  struct pcl_resp_cb_node_t*  next;
} pcl_resp_cb_node_t;

/// Pending intra-process service request queued from an external thread.
/// Drained and dispatched to the service handler on the executor thread.
typedef struct pcl_pending_svc_req_t {
  char*                          service_name; ///< deep copy
  char*                          type_name;    ///< deep copy, may be NULL
  void*                          data;         ///< deep copy, may be NULL
  uint32_t                       size;
  pcl_resp_cb_fn_t               callback;     ///< fired on executor thread
  void*                          user_data;
  struct pcl_pending_svc_req_t*  next;
} pcl_pending_svc_req_t;

typedef struct {
  char            peer_id[64];
  pcl_transport_t transport;
  int             in_use;
} pcl_registered_transport_t;

typedef struct {
  char                endpoint_name[128];
  pcl_endpoint_kind_t endpoint_kind;
  uint32_t            route_mode;
  char                peer_ids[PCL_MAX_ENDPOINT_PEERS][64];
  uint32_t            peer_count;
  int                 in_use;
} pcl_endpoint_route_entry_t;

struct pcl_executor_t {
  pcl_container_t* containers[PCL_MAX_CONTAINERS];
  uint32_t         container_count;

  pcl_transport_t  transport;
  int              has_transport;
  pcl_registered_transport_t transports[PCL_MAX_TRANSPORTS];
  uint32_t                 transport_count;
  pcl_endpoint_route_entry_t endpoint_routes[PCL_MAX_ENDPOINT_ROUTES];
  uint32_t                   endpoint_route_count;

  volatile int     shutdown_requested;

  double           prev_time;

  pcl_pending_msg_t* incoming_head;
  pcl_pending_msg_t* incoming_tail;
  pcl_mutex_t        incoming_lock;

  pcl_resp_cb_node_t* resp_cb_head;
  pcl_resp_cb_node_t* resp_cb_tail;
  pcl_mutex_t         resp_cb_lock;

  /// Queue of intra-process service requests posted from external threads.
  /// Guarded by svc_req_lock; drained exclusively on the executor thread.
  pcl_pending_svc_req_t* svc_req_head;
  pcl_pending_svc_req_t* svc_req_tail;
  pcl_mutex_t            svc_req_lock;
};

// -- Service context for deferred responses -------------------------------

struct pcl_svc_context_t {
  struct pcl_executor_t* executor;
  pcl_resp_cb_fn_t       callback;      // for intra-process: callback to fire
  void*                  user_data;     // for intra-process: user data
  const pcl_transport_t* transport;     // transport to use for remote response
  char                   peer_id[64];   // remote peer source, if any
  void*                  transport_ctx; // for transport: opaque context (e.g., seq_id)
};

// -- Stream context for streaming services --------------------------------

struct pcl_stream_context_t {
  struct pcl_executor_t* executor;
  pcl_stream_msg_fn_t    callback;      // client callback for stream messages
  void*                  user_data;     // client user data
  const pcl_transport_t* transport;     // transport to use for remote stream ops
  void*                  transport_ctx; // transport-specific handle
  volatile int           cancelled;     // set by client cancel
  volatile int           ended;         // set by server end/abort
};

struct pcl_container_t {
  char              name[128];
  pcl_state_t       state;
  pcl_callbacks_t   callbacks;
  void*             user_data;

  double            tick_rate_hz;
  double            tick_accumulator;

  bool              configuring;

  struct pcl_port_t ports[PCL_MAX_PORTS];
  uint32_t          port_count;

  pcl_param_t       params[PCL_MAX_PARAMS];
  uint32_t          param_count;

  struct pcl_executor_t* executor;
};

pcl_status_t pcl_executor_publish_port(pcl_executor_t*     e,
                                       const pcl_port_t*   port,
                                       const pcl_msg_t*    msg);

#ifdef __cplusplus
}
#endif

#endif /* PCL_INTERNAL_H */
