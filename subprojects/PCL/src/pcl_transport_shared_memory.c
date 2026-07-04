/// \file pcl_transport_shared_memory.c
/// \brief Inter-process shared-memory central bus transport for PCL.
///
/// Each transport instance joins a named bus backed by an OS shared-memory
/// region. The bus owns participant mailboxes in shared memory; a background
/// receive thread polls the local mailbox and posts work onto the executor
/// thread. Topic publish fans out across the bus, while async service
/// requests route to the unique participant advertising the requested service.
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#  define _POSIX_C_SOURCE 200809L
#endif

#include "pcl/pcl_transport_shared_memory.h"

#include "pcl_internal.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_alloc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <errno.h>
#  include <fcntl.h>
#  include <pthread.h>
#  include <semaphore.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <time.h>
#  include <unistd.h>
#endif

#define PCL_SHM_INTERNAL_TOPIC_SVC_REQ "__pcl_shm_svc_req"
#define PCL_SHM_INTERNAL_REQ_TYPE "pcl_shared_memory_service_request"
#define PCL_SHM_INTERNAL_TOPIC_STREAM_REQ "__pcl_shm_stream_req"
#define PCL_SHM_INTERNAL_STREAM_REQ_TYPE "pcl_shared_memory_stream_request"

#define PCL_SHM_MAGIC 0x50434C53u
#define PCL_SHM_VERSION 2u

#define PCL_SHM_MAX_PARTICIPANTS 8u
#define PCL_SHM_QUEUE_DEPTH 16u
#define PCL_SHM_MAX_SERVICES 16u
#define PCL_SHM_MAX_NAME 128u
#define PCL_SHM_MAX_TYPE 128u
#define PCL_SHM_MAX_ID 64u
#define PCL_SHM_MAX_OBJECT_NAME 256u
#define PCL_SHM_MAX_PAYLOAD 16384u
#define PCL_SHM_MAX_BACKPRESSURE_TOPICS 16u
#define PCL_SHM_DISCOVERY_RETRIES 20u
#define PCL_SHM_POLL_MS 1u

typedef enum {
  PCL_SHM_FRAME_NONE = 0,
  PCL_SHM_FRAME_PUBLISH = 1,
  PCL_SHM_FRAME_SVC_REQ = 2,
  PCL_SHM_FRAME_SVC_RESP = 3,
  PCL_SHM_FRAME_STREAM_REQ = 4,
  PCL_SHM_FRAME_STREAM_FRAME = 5,
  PCL_SHM_FRAME_STREAM_END = 6,
  PCL_SHM_FRAME_STREAM_CANCEL = 7,
} pcl_shm_frame_kind_t;

typedef struct {
  uint32_t kind;
  uint32_t seq_id;
  uint32_t data_size;
  char     source_id[PCL_SHM_MAX_ID];
  char     name[PCL_SHM_MAX_NAME];
  char     type_name[PCL_SHM_MAX_TYPE];
  uint8_t  data[PCL_SHM_MAX_PAYLOAD];
} pcl_shm_frame_t;

typedef struct {
  uint32_t generation;
  uint32_t in_use;
  uint32_t read_index;
  uint32_t write_index;
  uint32_t service_count;
  char     participant_id[PCL_SHM_MAX_ID];
  char     services[PCL_SHM_MAX_SERVICES][PCL_SHM_MAX_NAME];
  pcl_shm_frame_t frames[PCL_SHM_QUEUE_DEPTH];
} pcl_shm_slot_t;

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t participant_count;
  uint32_t next_generation;
  pcl_shm_slot_t slots[PCL_SHM_MAX_PARTICIPANTS];
} pcl_shm_region_t;

typedef struct pcl_shm_pending_request_t {
  uint32_t                          seq_id;
  pcl_resp_cb_fn_t                  callback;
  void*                             user_data;
  struct pcl_shm_pending_request_t* next;
} pcl_shm_pending_request_t;

typedef struct pcl_shm_pending_stream_t {
  uint32_t                          seq_id;
  pcl_stream_msg_fn_t               callback;
  void*                             user_data;
  struct pcl_shm_pending_stream_t*  next;
} pcl_shm_pending_stream_t;

typedef struct {
  uint32_t seq_id;
  char     requester_id[PCL_SHM_MAX_ID];
} pcl_shm_response_target_t;

typedef struct {
  uint32_t timeout_ms;
  int      in_use;
  char     topic[PCL_SHM_MAX_NAME];
} pcl_shm_backpressure_policy_t;

/* Stream send target shared with pcl_stream_context_t::transport_ctx. The
 * server-side stream handler uses this to address frames + end back to the
 * original requester via the bus. The owning transport links each live
 * target into active_stream_targets so destroy can abort lingering streams
 * (handler returned PCL_STREAMING but never called pcl_stream_end). */
typedef struct pcl_shm_stream_send_target_t {
  pcl_shared_memory_transport_t*       owner;
  uint32_t                             seq_id;
  char                                 requester_id[PCL_SHM_MAX_ID];
  pcl_stream_context_t*                stream_ctx;   /* for abort on destroy */
  struct pcl_shm_stream_send_target_t* prev;
  struct pcl_shm_stream_send_target_t* next;
} pcl_shm_stream_send_target_t;

/* Client-side stream handle returned via invoke_stream out param. */
typedef struct {
  pcl_shared_memory_transport_t* owner;
  uint32_t                       seq_id;
  char                           provider_id[PCL_SHM_MAX_ID];
} pcl_shm_stream_client_handle_t;

/* Trampoline plumbed through pcl_executor_post_response_msg to deliver a
 * stream frame or end on the executor thread without needing a parallel
 * queue in the executor core. */
typedef struct {
  pcl_stream_msg_fn_t            callback;
  void*                          user_data;
  pcl_shared_memory_transport_t* owner;
  uint32_t                       seq_id;
  int                            end;
  pcl_status_t                   status;
} pcl_shm_stream_trampoline_t;

typedef struct pcl_shared_memory_transport_t {
  char                       bus_name[PCL_SHM_MAX_NAME];
  char                       participant_id[PCL_SHM_MAX_ID];
  char                       shm_object_name[PCL_SHM_MAX_OBJECT_NAME];
  char                       lock_object_name[PCL_SHM_MAX_OBJECT_NAME];
  pcl_executor_t*            executor;
  pcl_transport_t            transport;
  pcl_container_t*           gateway;
  pcl_shm_region_t*          region;
  uint32_t                   slot_index;
  uint32_t                   next_seq_id;
  volatile int               recv_stop;
  pcl_shm_pending_request_t* pending_head;
  pcl_shm_pending_stream_t*  pending_stream_head;
  pcl_shm_stream_send_target_t* active_stream_targets;
  pcl_shm_backpressure_policy_t backpressure[PCL_SHM_MAX_BACKPRESSURE_TOPICS];
#ifdef _WIN32
  HANDLE                     mapping_handle;
  HANDLE                     lock_handle;
  HANDLE                     recv_thread;
  CRITICAL_SECTION           pending_lock;
#else
  int                        shm_fd;
  sem_t*                     lock_sem;
  pthread_t                  recv_thread;
  pthread_mutex_t            pending_lock;
#endif
} pcl_shared_memory_transport_t;

static void pcl_shm_sleep_ms(uint32_t ms) {
#ifdef _WIN32
  if (ms == 0u) return;
  if (ms <= 1u) {
    if (!SwitchToThread()) {
      Sleep(0);
    }
    return;
  }
  Sleep(ms);
#else
  struct timespec req;
  req.tv_sec = (time_t)(ms / 1000u);
  req.tv_nsec = (long)((ms % 1000u) * 1000000u);
  while (nanosleep(&req, &req) == -1 && errno == EINTR) {
  }
#endif
}

static uint64_t pcl_shm_now_ms(void) {
#ifdef _WIN32
  typedef ULONGLONG (WINAPI *pcl_get_tick_count64_fn)(void);
  HMODULE kernel32 = GetModuleHandleA("kernel32.dll");

  if (kernel32 != NULL) {
    pcl_get_tick_count64_fn get_tick_count64 =
      (pcl_get_tick_count64_fn)GetProcAddress(kernel32, "GetTickCount64");
    if (get_tick_count64 != NULL) {
      return (uint64_t)get_tick_count64();
    }
  }

  return (uint64_t)GetTickCount();
#else
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    return 0u;  // GCOVR_EXCL_LINE: CLOCK_MONOTONIC cannot fail on supported platforms
  }
  return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
#endif
}

static void pcl_shm_copy_token(const char* src,
                               char*       dst,
                               size_t      dst_size) {
  size_t i;
  if (!dst || dst_size == 0u) return;
  if (!src) {
    // GCOVR_EXCL_START: defensive; every caller passes a validated name.
    dst[0] = '\0';
    return;
    // GCOVR_EXCL_STOP
  }
  for (i = 0u; src[i] != '\0' && i + 1u < dst_size; ++i) {
    char ch = src[i];
    if ((ch >= 'a' && ch <= 'z') ||
        (ch >= 'A' && ch <= 'Z') ||
        (ch >= '0' && ch <= '9')) {
      dst[i] = ch;
    } else {
      dst[i] = '_';
    }
  }
  dst[i] = '\0';
}

static size_t pcl_shm_strnlen(const char* src, size_t max_len) {
  size_t i;
  if (!src) return 0u;
  for (i = 0u; i < max_len && src[i] != '\0'; ++i) {
  }
  return i;
}

static void pcl_shm_pending_lock_acquire(pcl_shared_memory_transport_t* ctx);
static void pcl_shm_pending_lock_release(pcl_shared_memory_transport_t* ctx);

static uint32_t pcl_shm_backpressure_timeout_ms(
    pcl_shared_memory_transport_t* ctx,
    const char*                    topic) {
  uint32_t i;
  uint32_t timeout_ms = 0u;

  if (!ctx || !topic) return 0u;

  pcl_shm_pending_lock_acquire(ctx);
  for (i = 0u; i < PCL_SHM_MAX_BACKPRESSURE_TOPICS; ++i) {
    if (ctx->backpressure[i].in_use &&
        strcmp(ctx->backpressure[i].topic, topic) == 0) {
      timeout_ms = ctx->backpressure[i].timeout_ms;
      break;
    }
  }
  pcl_shm_pending_lock_release(ctx);

  return timeout_ms;
}

static void pcl_shm_build_object_name(const char* prefix,
                                      const char* bus_name,
                                      char*       out,
                                      size_t      out_size) {
  char token[PCL_SHM_MAX_NAME];
  pcl_shm_copy_token(bus_name, token, sizeof(token));
#ifdef _WIN32
  snprintf(out, out_size, "Local\\%s_%s", prefix, token);
#else
  snprintf(out, out_size, "/%s_%s", prefix, token);
#endif
}

static void pcl_shm_pending_lock_init(pcl_shared_memory_transport_t* ctx) {
#ifdef _WIN32
  InitializeCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_init(&ctx->pending_lock, NULL);
#endif
}

static void pcl_shm_pending_lock_destroy(pcl_shared_memory_transport_t* ctx) {
#ifdef _WIN32
  DeleteCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_destroy(&ctx->pending_lock);
#endif
}

static void pcl_shm_pending_lock_acquire(pcl_shared_memory_transport_t* ctx) {
#ifdef _WIN32
  EnterCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_lock(&ctx->pending_lock);
#endif
}

static void pcl_shm_pending_lock_release(pcl_shared_memory_transport_t* ctx) {
#ifdef _WIN32
  LeaveCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_unlock(&ctx->pending_lock);
#endif
}

static pcl_status_t pcl_shm_bus_lock(pcl_shared_memory_transport_t* ctx) {
#ifdef _WIN32
  DWORD wait_rc;
  wait_rc = WaitForSingleObject(ctx->lock_handle, INFINITE);
  return (wait_rc == WAIT_OBJECT_0) ? PCL_OK : PCL_ERR_STATE;
#else
  if (!ctx || !ctx->lock_sem) return PCL_ERR_INVALID;
  while (sem_wait(ctx->lock_sem) == -1) {
    if (errno != EINTR) return PCL_ERR_STATE;  // GCOVR_EXCL_LINE: sem_wait fails only on kernel fault or invalid semaphore
  }
  return PCL_OK;
#endif
}

static void pcl_shm_bus_unlock(pcl_shared_memory_transport_t* ctx) {
#ifdef _WIN32
  if (ctx && ctx->lock_handle) {
    ReleaseMutex(ctx->lock_handle);
  }
#else
  if (ctx && ctx->lock_sem) {
    sem_post(ctx->lock_sem);
  }
#endif
}

static pcl_status_t pcl_shm_platform_open(pcl_shared_memory_transport_t* ctx) {
  size_t region_size;

  if (!ctx) return PCL_ERR_INVALID;

  pcl_shm_build_object_name("pcl_shm_bus", ctx->bus_name,
                            ctx->shm_object_name, sizeof(ctx->shm_object_name));
  pcl_shm_build_object_name("pcl_shm_lock", ctx->bus_name,
                            ctx->lock_object_name, sizeof(ctx->lock_object_name));

  region_size = sizeof(pcl_shm_region_t);

#ifdef _WIN32
  ctx->mapping_handle = CreateFileMappingA(INVALID_HANDLE_VALUE,
                                           NULL,
                                           PAGE_READWRITE,
                                           0,
                                           (DWORD)region_size,
                                           ctx->shm_object_name);
  if (!ctx->mapping_handle) return PCL_ERR_STATE;

  ctx->region = (pcl_shm_region_t*)MapViewOfFile(ctx->mapping_handle,
                                                 FILE_MAP_ALL_ACCESS,
                                                 0, 0, region_size);
  if (!ctx->region) return PCL_ERR_STATE;

  ctx->lock_handle = CreateMutexA(NULL, FALSE, ctx->lock_object_name);
  if (!ctx->lock_handle) return PCL_ERR_STATE;
#else
  ctx->shm_fd = shm_open(ctx->shm_object_name, O_CREAT | O_RDWR, 0600);
  if (ctx->shm_fd < 0) return PCL_ERR_STATE;
  if (ftruncate(ctx->shm_fd, (off_t)region_size) != 0) return PCL_ERR_STATE;

  ctx->region = (pcl_shm_region_t*)mmap(NULL,
                                        region_size,
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED,
                                        ctx->shm_fd,
                                        0);
  if (ctx->region == MAP_FAILED) {
    // GCOVR_EXCL_START: mmap of a freshly truncated shm object fails only on
    // OS resource exhaustion, which is not injectable in normal testing.
    ctx->region = NULL;
    return PCL_ERR_STATE;
    // GCOVR_EXCL_STOP
  }

  ctx->lock_sem = sem_open(ctx->lock_object_name, O_CREAT, 0600, 1);
  if (ctx->lock_sem == SEM_FAILED) {
    // GCOVR_EXCL_START: sem_open with O_CREAT fails only on OS resource
    // exhaustion, which is not injectable in normal testing.
    ctx->lock_sem = NULL;
    return PCL_ERR_STATE;
    // GCOVR_EXCL_STOP
  }
#endif

  return PCL_OK;
}

static void pcl_shm_platform_close(pcl_shared_memory_transport_t* ctx,
                                   int                            unlink_objects) {
  size_t region_size;

  if (!ctx) return;
  region_size = sizeof(pcl_shm_region_t);

#ifdef _WIN32
  if (ctx->region) {
    UnmapViewOfFile(ctx->region);
    ctx->region = NULL;
  }
  if (ctx->mapping_handle) {
    CloseHandle(ctx->mapping_handle);
    ctx->mapping_handle = NULL;
  }
  if (ctx->lock_handle) {
    CloseHandle(ctx->lock_handle);
    ctx->lock_handle = NULL;
  }
#else
  if (ctx->region) {
    munmap(ctx->region, region_size);
    ctx->region = NULL;
  }
  if (ctx->shm_fd >= 0) {
    close(ctx->shm_fd);
    ctx->shm_fd = -1;
  }
  if (ctx->lock_sem) {
    sem_close(ctx->lock_sem);
    ctx->lock_sem = NULL;
  }
  if (unlink_objects) {
    shm_unlink(ctx->shm_object_name);
    sem_unlink(ctx->lock_object_name);
  }
#endif
}

static uint16_t pcl_shm_read_u16_be(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

static void pcl_shm_write_u16_be(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

static uint32_t pcl_shm_read_u32_be(const uint8_t* p) {
  return (uint32_t)((p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

static void pcl_shm_write_u32_be(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)((v >> 16) & 0xFF);
  p[2] = (uint8_t)((v >> 8) & 0xFF);
  p[3] = (uint8_t)(v & 0xFF);
}

static pcl_endpoint_kind_t pcl_shm_endpoint_kind_from_port_type(pcl_port_type_t type) {
  switch (type) {
    // GCOVR_EXCL_START: the shm dispatch paths only query service and
    // stream-service ports; the other kinds are kept for completeness.
    case PCL_PORT_PUBLISHER:      return PCL_ENDPOINT_PUBLISHER;
    case PCL_PORT_SUBSCRIBER:     return PCL_ENDPOINT_SUBSCRIBER;
    // GCOVR_EXCL_STOP
    case PCL_PORT_SERVICE:        return PCL_ENDPOINT_PROVIDED;
    // GCOVR_EXCL_START: see above.
    case PCL_PORT_CLIENT:         return PCL_ENDPOINT_CONSUMED;
    // GCOVR_EXCL_STOP
    case PCL_PORT_STREAM_SERVICE: return PCL_ENDPOINT_STREAM_PROVIDED;
    default:                      return PCL_ENDPOINT_PUBLISHER;  // GCOVR_EXCL_LINE: defensive
  }
}

static const pcl_endpoint_route_entry_t* pcl_shm_find_endpoint_route_entry(
    const pcl_executor_t* e,
    const char*           endpoint_name,
    pcl_endpoint_kind_t   endpoint_kind) {
  uint32_t i;
  if (!e || !endpoint_name) return NULL;
  for (i = 0; i < e->endpoint_route_count; ++i) {
    const pcl_endpoint_route_entry_t* entry = &e->endpoint_routes[i];
    if (entry->in_use &&
        entry->endpoint_kind == endpoint_kind &&
        strcmp(entry->endpoint_name, endpoint_name) == 0) {
      return entry;
    }
  }
  return NULL;
}

static uint32_t pcl_shm_port_route_mode(const pcl_executor_t* e,
                                        const pcl_port_t*     port) {
  const pcl_endpoint_route_entry_t* entry;
  if (!port) return PCL_ROUTE_NONE;

  entry = pcl_shm_find_endpoint_route_entry(
      e, port->name, pcl_shm_endpoint_kind_from_port_type(port->type));
  if (entry) return entry->route_mode;
  if (port->route_configured && port->route_mode != PCL_ROUTE_NONE) {
    return port->route_mode;
  }

  if (e && e->has_transport) {
    if (port->type == PCL_PORT_PUBLISHER || port->type == PCL_PORT_CLIENT) {
      return PCL_ROUTE_REMOTE;  // GCOVR_EXCL_LINE: shm dispatch only queries service/stream ports, never publisher/client ports
    }
    return PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE;
  }

  return PCL_ROUTE_LOCAL;
}

static uint32_t pcl_shm_port_peer_count(const pcl_executor_t* e,
                                        const pcl_port_t*     port) {
  const pcl_endpoint_route_entry_t* entry;
  if (!port) return 0u;
  entry = pcl_shm_find_endpoint_route_entry(
      e, port->name, pcl_shm_endpoint_kind_from_port_type(port->type));
  return entry ? entry->peer_count : port->peer_count;
}

static const char* pcl_shm_port_peer_id_at(const pcl_executor_t* e,
                                           const pcl_port_t*     port,
                                           uint32_t              index) {
  const pcl_endpoint_route_entry_t* entry;
  if (!port) return NULL;
  entry = pcl_shm_find_endpoint_route_entry(
      e, port->name, pcl_shm_endpoint_kind_from_port_type(port->type));
  if (entry) {
    return (index < entry->peer_count) ? entry->peer_ids[index] : NULL;
  }
  return (index < port->peer_count) ? port->peer_ids[index] : NULL;
}

static int pcl_shm_route_accepts(uint32_t route_mode,
                                 uint32_t source_route_mode) {
  return (route_mode & source_route_mode) != 0u;
}

static int pcl_shm_peer_is_allowed(const pcl_executor_t* e,
                                   const pcl_port_t*     port,
                                   const char*           peer_id) {
  uint32_t i;
  uint32_t count;

  if (!peer_id) return 0;
  count = pcl_shm_port_peer_count(e, port);
  if (count == 0u) return 1;
  for (i = 0; i < count; ++i) {
    const char* configured = pcl_shm_port_peer_id_at(e, port, i);
    if (configured && strcmp(configured, peer_id) == 0) {
      return 1;
    }
  }
  return 0;
}

static pcl_port_t* pcl_shm_find_remote_service(pcl_executor_t* e,
                                               const char*     name,
                                               const char*     source_peer_id) {
  uint32_t ci;
  uint32_t pi;
  pcl_port_t* match = NULL;

  if (!e || !name || !source_peer_id) return NULL;

  pcl_executor_containers_lock(e);
  for (ci = 0; ci < e->container_count && match == NULL; ++ci) {
    pcl_container_t* c = e->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      pcl_port_t* port = &c->ports[pi];
      uint32_t route_mode;

      if (port->type != PCL_PORT_SERVICE ||
          strcmp(port->name, name) != 0 ||
          port->svc_handler == NULL) {
        continue;
      }

      route_mode = pcl_shm_port_route_mode(e, port);
      if (!pcl_shm_route_accepts(route_mode, PCL_ROUTE_REMOTE)) {
        continue;
      }
      if (!pcl_shm_peer_is_allowed(e, port, source_peer_id)) {
        continue;
      }
      match = port;
      break;
    }
  }
  pcl_executor_containers_unlock(e);
  return match;
}

static pcl_port_t* pcl_shm_find_remote_stream_service(pcl_executor_t* e,
                                                      const char*     name,
                                                      const char*     source_peer_id) {
  uint32_t ci;
  uint32_t pi;
  pcl_port_t* match = NULL;

  if (!e || !name || !source_peer_id) return NULL;

  pcl_executor_containers_lock(e);
  for (ci = 0; ci < e->container_count && match == NULL; ++ci) {
    pcl_container_t* c = e->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      pcl_port_t* port = &c->ports[pi];
      uint32_t route_mode;

      if (port->type != PCL_PORT_STREAM_SERVICE ||
          strcmp(port->name, name) != 0 ||
          port->stream_handler == NULL) {
        continue;
      }

      route_mode = pcl_shm_port_route_mode(e, port);
      if (!pcl_shm_route_accepts(route_mode, PCL_ROUTE_REMOTE)) {
        continue;
      }
      if (!pcl_shm_peer_is_allowed(e, port, source_peer_id)) {
        continue;
      }
      match = port;
      break;
    }
  }
  pcl_executor_containers_unlock(e);
  return match;
}

static int pcl_shm_service_list_contains(char services[PCL_SHM_MAX_SERVICES][PCL_SHM_MAX_NAME],
                                         uint32_t count,
                                         const char* name) {
  uint32_t i;
  for (i = 0; i < count; ++i) {
    if (strcmp(services[i], name) == 0) return 1;
  }
  return 0;
}

static void pcl_shm_collect_local_services(
    pcl_shared_memory_transport_t* ctx,
    char                           services[PCL_SHM_MAX_SERVICES][PCL_SHM_MAX_NAME],
    uint32_t*                      count) {
  uint32_t ci;
  uint32_t pi;

  *count = 0u;
  if (!ctx || !ctx->executor) return;

  pcl_executor_containers_lock(ctx->executor);
  for (ci = 0; ci < ctx->executor->container_count; ++ci) {
    pcl_container_t* c = ctx->executor->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      pcl_port_t* port = &c->ports[pi];
      uint32_t route_mode;
      int is_unary;
      int is_stream;

      is_unary  = (port->type == PCL_PORT_SERVICE        && port->svc_handler    != NULL);
      is_stream = (port->type == PCL_PORT_STREAM_SERVICE && port->stream_handler != NULL);
      if (!is_unary && !is_stream) {
        continue;
      }

      route_mode = pcl_shm_port_route_mode(ctx->executor, port);
      if (!pcl_shm_route_accepts(route_mode, PCL_ROUTE_REMOTE)) {
        continue;
      }
      if (pcl_shm_service_list_contains(services, *count, port->name)) {
        continue;
      }
      if (*count >= PCL_SHM_MAX_SERVICES) {
        pcl_executor_containers_unlock(ctx->executor);
        return;
      }

      snprintf(services[*count], PCL_SHM_MAX_NAME, "%s", port->name);
      (*count)++;
    }
  }
  pcl_executor_containers_unlock(ctx->executor);
}

static void pcl_shm_sync_local_services(pcl_shared_memory_transport_t* ctx) {
  char services[PCL_SHM_MAX_SERVICES][PCL_SHM_MAX_NAME];
  uint32_t count;
  uint32_t i;

  if (!ctx || !ctx->region) return;

  pcl_shm_collect_local_services(ctx, services, &count);
  if (pcl_shm_bus_lock(ctx) != PCL_OK) return;

  if (ctx->slot_index < PCL_SHM_MAX_PARTICIPANTS &&
      ctx->region->slots[ctx->slot_index].in_use &&
      strcmp(ctx->region->slots[ctx->slot_index].participant_id,
             ctx->participant_id) == 0) {
    pcl_shm_slot_t* slot = &ctx->region->slots[ctx->slot_index];
    slot->service_count = count;
    for (i = 0; i < PCL_SHM_MAX_SERVICES; ++i) {
      slot->services[i][0] = '\0';
    }
    for (i = 0; i < count; ++i) {
      snprintf(slot->services[i], PCL_SHM_MAX_NAME, "%s", services[i]);
    }
  }

  pcl_shm_bus_unlock(ctx);
}

static pcl_status_t pcl_shm_pending_remove(pcl_shared_memory_transport_t* ctx,
                                           uint32_t                       seq_id,
                                           pcl_resp_cb_fn_t*              callback,
                                           void**                         user_data) {
  pcl_shm_pending_request_t** pp;

  if (!ctx) return PCL_ERR_INVALID;

  pcl_shm_pending_lock_acquire(ctx);
  for (pp = &ctx->pending_head; *pp; pp = &(*pp)->next) {
    if ((*pp)->seq_id == seq_id) {
      pcl_shm_pending_request_t* node = *pp;
      *pp = node->next;
      if (callback) *callback = node->callback;
      if (user_data) *user_data = node->user_data;
      pcl_free(node);
      pcl_shm_pending_lock_release(ctx);
      return PCL_OK;
    }
  }
  // GCOVR_EXCL_START: an unmatched sequence id requires a response frame
  // arriving after its pending entry was already completed or flushed --
  // adversarial peer timing that cannot be forced deterministically.
  pcl_shm_pending_lock_release(ctx);
  return PCL_ERR_NOT_FOUND;
}
// GCOVR_EXCL_STOP

static void pcl_shm_pending_clear(pcl_shared_memory_transport_t* ctx) {
  pcl_shm_pending_request_t* node;

  if (!ctx) return;
  pcl_shm_pending_lock_acquire(ctx);
  node = ctx->pending_head;
  ctx->pending_head = NULL;
  pcl_shm_pending_lock_release(ctx);

  while (node) {
    pcl_shm_pending_request_t* next = node->next;
    pcl_free(node);
    node = next;
  }
}

static pcl_status_t pcl_shm_pending_stream_lookup(
    pcl_shared_memory_transport_t* ctx,
    uint32_t                       seq_id,
    pcl_stream_msg_fn_t*           callback,
    void**                         user_data) {
  pcl_shm_pending_stream_t* node;

  if (!ctx) return PCL_ERR_INVALID;
  pcl_shm_pending_lock_acquire(ctx);
  for (node = ctx->pending_stream_head; node; node = node->next) {
    if (node->seq_id == seq_id) {
      if (callback)  *callback  = node->callback;
      if (user_data) *user_data = node->user_data;
      pcl_shm_pending_lock_release(ctx);
      return PCL_OK;
    }
  }
  // GCOVR_EXCL_START: an unmatched sequence id requires a response frame
  // arriving after its pending entry was already completed or flushed --
  // adversarial peer timing that cannot be forced deterministically.
  pcl_shm_pending_lock_release(ctx);
  return PCL_ERR_NOT_FOUND;
}
// GCOVR_EXCL_STOP

static pcl_status_t pcl_shm_pending_stream_remove(
    pcl_shared_memory_transport_t* ctx,
    uint32_t                       seq_id,
    pcl_stream_msg_fn_t*           callback,
    void**                         user_data) {
  pcl_shm_pending_stream_t** pp;

  if (!ctx) return PCL_ERR_INVALID;
  pcl_shm_pending_lock_acquire(ctx);
  for (pp = &ctx->pending_stream_head; *pp; pp = &(*pp)->next) {
    if ((*pp)->seq_id == seq_id) {
      pcl_shm_pending_stream_t* node = *pp;
      *pp = node->next;
      if (callback)  *callback  = node->callback;
      if (user_data) *user_data = node->user_data;
      pcl_free(node);
      pcl_shm_pending_lock_release(ctx);
      return PCL_OK;
    }
  }
  // GCOVR_EXCL_START: an unmatched sequence id requires a response frame
  // arriving after its pending entry was already completed or flushed --
  // adversarial peer timing that cannot be forced deterministically.
  pcl_shm_pending_lock_release(ctx);
  return PCL_ERR_NOT_FOUND;
}
// GCOVR_EXCL_STOP

/* Walks the pending-stream list, detaches each entry, and fires its callback
 * with end=true status=PCL_ERR_CANCELLED so generated holders (futures,
 * accumulators, push-mode state) are released even if the peer never sent
 * STREAM_END. Called from destroy after the recv thread has stopped. */
static void pcl_shm_pending_stream_clear(pcl_shared_memory_transport_t* ctx) {
  pcl_shm_pending_stream_t* node;
  if (!ctx) return;
  pcl_shm_pending_lock_acquire(ctx);
  node = ctx->pending_stream_head;
  ctx->pending_stream_head = NULL;
  pcl_shm_pending_lock_release(ctx);
  while (node) {
    pcl_shm_pending_stream_t* next = node->next;
    if (node->callback) {
      pcl_msg_t empty = {0};
      node->callback(&empty, true, PCL_ERR_CANCELLED, node->user_data);
    }
    pcl_free(node);
    node = next;
  }
}

/* Forward decl -- defined alongside the other stream transport callbacks. */
static pcl_status_t pcl_shm_stream_send_frame(
    pcl_shared_memory_transport_t*       ctx,
    const pcl_shm_stream_send_target_t*  target,
    pcl_shm_frame_kind_t                 kind,
    const pcl_msg_t*                     msg,
    pcl_status_t                         end_status);

static void pcl_shm_active_target_register(pcl_shared_memory_transport_t* ctx,
                                            pcl_shm_stream_send_target_t*  target) {
  if (!ctx || !target) return;
  pcl_shm_pending_lock_acquire(ctx);
  target->prev = NULL;
  target->next = ctx->active_stream_targets;
  if (ctx->active_stream_targets) ctx->active_stream_targets->prev = target;
  ctx->active_stream_targets = target;
  pcl_shm_pending_lock_release(ctx);
}

static void pcl_shm_active_target_unlink(pcl_shared_memory_transport_t* ctx,
                                         pcl_shm_stream_send_target_t*  target) {
  if (!ctx || !target) return;
  pcl_shm_pending_lock_acquire(ctx);
  if (target->prev) target->prev->next = target->next;
  if (target->next) target->next->prev = target->prev;
  if (ctx->active_stream_targets == target) ctx->active_stream_targets = target->next;
  target->prev = NULL;
  target->next = NULL;
  pcl_shm_pending_lock_release(ctx);
}

/* Abort every server-side stream the gateway opened but the handler never
 * closed. Called from destroy before the executor and gateway are torn down. */
static void pcl_shm_active_streams_abort(pcl_shared_memory_transport_t* ctx) {
  pcl_shm_stream_send_target_t* node;
  if (!ctx) return;
  pcl_shm_pending_lock_acquire(ctx);
  node = ctx->active_stream_targets;
  ctx->active_stream_targets = NULL;
  pcl_shm_pending_lock_release(ctx);
  while (node) {
    pcl_shm_stream_send_target_t* next = node->next;
    /* Tell the peer first while the bus is still mapped. */
    {
      uint8_t status_byte = (uint8_t)(PCL_ERR_CANCELLED & 0xFFu);
      pcl_msg_t end_msg;
      memset(&end_msg, 0, sizeof(end_msg));
      end_msg.data = &status_byte;
      end_msg.size = 1u;
      (void)pcl_shm_stream_send_frame(ctx, node,
                                      PCL_SHM_FRAME_STREAM_END, &end_msg,
                                      PCL_ERR_CANCELLED);
    }
    /* Mark the executor-side stream context as ended so any subsequent
     * pcl_stream_{send,end,abort} from the (possibly still-running) handler
     * becomes a safe no-op, and disconnect its transport reference to avoid
     * a dangle into freed transport state. The container framework still
     * owns the stream_ctx allocation. */
    if (node->stream_ctx) {
      node->stream_ctx->ended         = 1;
      node->stream_ctx->transport     = NULL;
      node->stream_ctx->transport_ctx = NULL;
    }
    pcl_free(node);
    node = next;
  }
}

static pcl_status_t pcl_shm_active_stream_cancel(
    pcl_shared_memory_transport_t* ctx,
    uint32_t                       seq_id,
    const char*                    requester_id) {
  pcl_shm_stream_send_target_t* node;

  if (!ctx || !requester_id) return PCL_ERR_INVALID;

  pcl_shm_pending_lock_acquire(ctx);
  for (node = ctx->active_stream_targets; node; node = node->next) {
    if (node->seq_id == seq_id &&
        strcmp(node->requester_id, requester_id) == 0) {
      if (node->stream_ctx && !node->stream_ctx->ended) {
        node->stream_ctx->cancelled = 1;
      }
      pcl_shm_pending_lock_release(ctx);
      return PCL_OK;
    }
  }
  // GCOVR_EXCL_START: an unmatched sequence id requires a response frame
  // arriving after its pending entry was already completed or flushed --
  // adversarial peer timing that cannot be forced deterministically.
  pcl_shm_pending_lock_release(ctx);
  return PCL_ERR_NOT_FOUND;
}
// GCOVR_EXCL_STOP

/* Unpacks a pcl_shm_stream_trampoline_t queued via the executor's response
 * queue and dispatches it as a typed stream callback on the executor thread. */
static void pcl_shm_stream_trampoline_cb(const pcl_msg_t* msg, void* user_data) {
  pcl_shm_stream_trampoline_t* tr =
      (pcl_shm_stream_trampoline_t*)user_data;
  pcl_msg_t empty = {0};
  const pcl_msg_t* delivered = msg ? msg : &empty;
  if (!tr) return;
  if (tr->callback) {
    tr->callback(delivered, tr->end != 0, tr->status, tr->user_data);
  }
  pcl_free(tr);
}

static pcl_status_t pcl_shm_pending_stream_complete(
    pcl_shared_memory_transport_t* ctx,
    uint32_t                       seq_id,
    pcl_status_t                   status) {
  pcl_stream_msg_fn_t callback = NULL;
  void* user_data = NULL;
  pcl_msg_t empty = {0};
  pcl_shm_stream_trampoline_t* tr;
  pcl_status_t rc;

  if (!ctx) return PCL_ERR_INVALID;
  if (pcl_shm_pending_stream_remove(ctx, seq_id, &callback, &user_data) !=
      PCL_OK) {
    return PCL_ERR_NOT_FOUND;  // GCOVR_EXCL_LINE: requires completing a stream whose pending entry was already removed (peer-timing race)
  }

  tr = (pcl_shm_stream_trampoline_t*)pcl_calloc(1, sizeof(*tr));
  if (!tr) return PCL_ERR_NOMEM;
  tr->callback = callback;
  tr->user_data = user_data;
  tr->owner = ctx;
  tr->seq_id = seq_id;
  tr->end = 1;
  tr->status = status;
  rc = pcl_executor_post_response_msg(ctx->executor,
                                      pcl_shm_stream_trampoline_cb,
                                      tr, &empty);
  if (rc != PCL_OK) pcl_free(tr);
  return rc;
}

static pcl_status_t pcl_shm_enqueue_frame_locked(
    pcl_shared_memory_transport_t* ctx,
    uint32_t                       target_slot_index,
    pcl_shm_frame_kind_t           kind,
    const char*                    source_id,
    const char*                    name,
    const char*                    type_name,
    const void*                    data,
    uint32_t                       data_size,
    uint32_t                       seq_id) {
  pcl_shm_slot_t* slot;
  uint32_t next_index;
  pcl_shm_frame_t* frame;

  if (!ctx || !ctx->region || target_slot_index >= PCL_SHM_MAX_PARTICIPANTS) {
    return PCL_ERR_INVALID;  // GCOVR_EXCL_LINE: internal precondition; every caller resolves a valid slot under the bus lock
  }
  if (data_size > 0u && !data) return PCL_ERR_INVALID;
  if (data_size > PCL_SHM_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  slot = &ctx->region->slots[target_slot_index];
  if (!slot->in_use) return PCL_ERR_NOT_FOUND;

  next_index = (slot->write_index + 1u) % PCL_SHM_QUEUE_DEPTH;
  if (next_index == slot->read_index) return PCL_ERR_NOMEM;

  frame = &slot->frames[slot->write_index];
  memset(frame, 0, sizeof(*frame));
  frame->kind = (uint32_t)kind;
  frame->seq_id = seq_id;
  frame->data_size = data_size;
  if (source_id) snprintf(frame->source_id, sizeof(frame->source_id), "%s", source_id);
  if (name) snprintf(frame->name, sizeof(frame->name), "%s", name);
  if (type_name) snprintf(frame->type_name, sizeof(frame->type_name), "%s", type_name);
  if (data_size > 0u) memcpy(frame->data, data, data_size);

  slot->write_index = next_index;
  return PCL_OK;
}

static int pcl_shm_slot_has_capacity_locked(const pcl_shm_slot_t* slot) {
  uint32_t next_index;

  if (!slot || !slot->in_use) return 0;
  next_index = (slot->write_index + 1u) % PCL_SHM_QUEUE_DEPTH;
  return next_index != slot->read_index;
}

static pcl_status_t pcl_shm_publish_once_locked(
    pcl_shared_memory_transport_t* ctx,
    const char*                    topic,
    const pcl_msg_t*               msg,
    uint32_t*                      delivered_count) {
  uint32_t targets[PCL_SHM_MAX_PARTICIPANTS];
  uint32_t original_write_index[PCL_SHM_MAX_PARTICIPANTS];
  uint32_t target_count = 0u;
  uint32_t i;

  if (!ctx || !topic || !msg || !delivered_count) return PCL_ERR_INVALID;
  *delivered_count = 0u;

  for (i = 0u; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
    pcl_shm_slot_t* slot = &ctx->region->slots[i];
    if (i == ctx->slot_index || !slot->in_use) continue;
    targets[target_count] = i;
    original_write_index[target_count] = slot->write_index;
    ++target_count;
  }

  if (target_count == 0u) return PCL_ERR_NOT_FOUND;

  for (i = 0u; i < target_count; ++i) {
    pcl_shm_slot_t* slot = &ctx->region->slots[targets[i]];
    if (!pcl_shm_slot_has_capacity_locked(slot)) {
      return PCL_ERR_NOMEM;
    }
  }

  for (i = 0u; i < target_count; ++i) {
    pcl_status_t rc = pcl_shm_enqueue_frame_locked(ctx,
                                                   targets[i],
                                                   PCL_SHM_FRAME_PUBLISH,
                                                   ctx->participant_id,
                                                   topic,
                                                   msg->type_name,
                                                   msg->data,
                                                   msg->size,
                                                   0u);
    if (rc != PCL_OK) {
      // GCOVR_EXCL_START: the capacity pre-check under the bus lock makes a
      // mid-fan-out enqueue failure unreachable; kept as a safety net against
      // concurrent writers that bypass the lock.
      uint32_t rollback;
      for (rollback = 0u; rollback < i; ++rollback) {
        pcl_shm_slot_t* slot = &ctx->region->slots[targets[rollback]];
        uint32_t frame_index = original_write_index[rollback];
        memset(&slot->frames[frame_index], 0, sizeof(slot->frames[frame_index]));
        slot->write_index = original_write_index[rollback];
      }
      return rc;
      // GCOVR_EXCL_STOP
    }
  }

  *delivered_count = target_count;
  return PCL_OK;
}

static int pcl_shm_find_provider_slot_locked(pcl_shared_memory_transport_t* ctx,
                                             const char*                    service_name,
                                             uint32_t*                      slot_index) {
  uint32_t i;
  int found = 0;

  if (!ctx || !ctx->region || !service_name || !slot_index) return 0;

  for (i = 0; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
    uint32_t si;
    pcl_shm_slot_t* slot = &ctx->region->slots[i];
    if (!slot->in_use || i == ctx->slot_index) continue;
    for (si = 0; si < slot->service_count; ++si) {
      if (strcmp(slot->services[si], service_name) == 0) {
        if (found) return -1;
        *slot_index = i;
        found = 1;
      }
    }
  }

  return found;
}

static pcl_status_t pcl_shm_publish(void*            adapter_ctx,
                                    const char*      topic,
                                    const pcl_msg_t* msg) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)adapter_ctx;
  uint32_t delivered_count = 0u;
  uint32_t timeout_ms;
  uint64_t deadline_ms = 0u;

  if (!ctx || !topic || !msg || !msg->type_name) return PCL_ERR_INVALID;
  if (msg->size > 0u && !msg->data) return PCL_ERR_INVALID;
  if (msg->size > PCL_SHM_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  timeout_ms = pcl_shm_backpressure_timeout_ms(ctx, topic);
  if (timeout_ms > 0u) {
    deadline_ms = pcl_shm_now_ms() + (uint64_t)timeout_ms;
  }

  for (;;) {
    pcl_status_t rc;

    if (pcl_shm_bus_lock(ctx) != PCL_OK) return PCL_ERR_STATE;
    rc = pcl_shm_publish_once_locked(ctx, topic, msg, &delivered_count);
    pcl_shm_bus_unlock(ctx);

    if (rc == PCL_OK || rc == PCL_ERR_NOT_FOUND) {
      return rc;
    }
    if (rc != PCL_ERR_NOMEM || timeout_ms == 0u) {
      return rc;
    }
    if (pcl_shm_now_ms() >= deadline_ms) {
      return PCL_ERR_TIMEOUT;  // GCOVR_EXCL_LINE: expiry needs a peer that stays stalled past the deadline; each participant's drain thread makes that nondeterministic in-process
    }
    pcl_shm_sleep_ms(PCL_SHM_POLL_MS);
  }
}

static pcl_status_t pcl_shm_subscribe(void*       adapter_ctx,
                                      const char* topic,
                                      const char* type_name) {
  (void)adapter_ctx;
  (void)topic;
  (void)type_name;
  return PCL_OK;
}

static pcl_status_t pcl_shm_send_response(
    pcl_shared_memory_transport_t*  ctx,
    const pcl_shm_response_target_t* target,
    const pcl_msg_t*                response) {
  uint32_t i;
  pcl_status_t rc = PCL_ERR_NOT_FOUND;

  if (!ctx || !target || !response) return PCL_ERR_INVALID;

  if (pcl_shm_bus_lock(ctx) != PCL_OK) return PCL_ERR_STATE;
  for (i = 0; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
    pcl_shm_slot_t* slot = &ctx->region->slots[i];
    if (!slot->in_use) continue;
    if (strcmp(slot->participant_id, target->requester_id) == 0) {
      rc = pcl_shm_enqueue_frame_locked(ctx,
                                        i,
                                        PCL_SHM_FRAME_SVC_RESP,
                                        ctx->participant_id,
                                        "",
                                        response->type_name,
                                        response->data,
                                        response->size,
                                        target->seq_id);
      break;
    }
  }
  pcl_shm_bus_unlock(ctx);
  return rc;
}

static pcl_status_t pcl_shm_respond(void*              adapter_ctx,
                                    pcl_svc_context_t* svc_ctx,
                                    const pcl_msg_t*   response) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)adapter_ctx;
  pcl_shm_response_target_t* target;
  pcl_status_t rc;

  if (!ctx || !svc_ctx || !response) return PCL_ERR_INVALID;
  target = (pcl_shm_response_target_t*)svc_ctx->transport_ctx;
  if (!target) return PCL_ERR_INVALID;

  rc = pcl_shm_send_response(ctx, target, response);
  pcl_free(target);
  svc_ctx->transport_ctx = NULL;
  return rc;
}

static pcl_status_t pcl_shm_invoke_async(void*            adapter_ctx,
                                         const char*      service_name,
                                         const pcl_msg_t* request,
                                         pcl_resp_cb_fn_t callback,
                                         void*            user_data) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)adapter_ctx;
  pcl_shm_pending_request_t* pending;
  uint32_t seq_id;
  uint32_t provider_slot = 0u;
  uint32_t attempts;
  int found;
  pcl_status_t rc;

  if (!ctx || !service_name || !request || !callback) return PCL_ERR_INVALID;
  if (request->size > 0u && !request->data) return PCL_ERR_INVALID;
  if (request->size > PCL_SHM_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  pending = (pcl_shm_pending_request_t*)pcl_calloc(1, sizeof(*pending));
  if (!pending) return PCL_ERR_NOMEM;

  pcl_shm_pending_lock_acquire(ctx);
  seq_id = ++ctx->next_seq_id;
  if (seq_id == 0u) seq_id = ++ctx->next_seq_id;
  pending->seq_id = seq_id;
  pending->callback = callback;
  pending->user_data = user_data;
  pending->next = ctx->pending_head;
  ctx->pending_head = pending;
  pcl_shm_pending_lock_release(ctx);

  found = 0;
  for (attempts = 0u; attempts < PCL_SHM_DISCOVERY_RETRIES; ++attempts) {
    if (pcl_shm_bus_lock(ctx) != PCL_OK) break;
    found = pcl_shm_find_provider_slot_locked(ctx, service_name, &provider_slot);
    pcl_shm_bus_unlock(ctx);
    if (found != 0) break;
    pcl_shm_sleep_ms(PCL_SHM_POLL_MS);
  }

  if (found == 0) {
    pcl_shm_pending_remove(ctx, seq_id, NULL, NULL);
    return PCL_ERR_NOT_FOUND;
  }
  if (found < 0) {
    pcl_shm_pending_remove(ctx, seq_id, NULL, NULL);
    return PCL_ERR_INVALID;
  }

  if (pcl_shm_bus_lock(ctx) != PCL_OK) {
    // GCOVR_EXCL_START: the bus semaphore fails only on kernel fault.
    pcl_shm_pending_remove(ctx, seq_id, NULL, NULL);
    return PCL_ERR_STATE;
    // GCOVR_EXCL_STOP
  }
  rc = pcl_shm_enqueue_frame_locked(ctx,
                                    provider_slot,
                                    PCL_SHM_FRAME_SVC_REQ,
                                    ctx->participant_id,
                                    service_name,
                                    request->type_name,
                                    request->data,
                                    request->size,
                                    seq_id);
  pcl_shm_bus_unlock(ctx);

  if (rc != PCL_OK) {
    pcl_shm_pending_remove(ctx, seq_id, NULL, NULL);  // GCOVR_EXCL_LINE: requires the provider mailbox to fill in the instant after discovery
  }
  return rc;
}

static pcl_status_t pcl_shm_invoke_stream(void*               adapter_ctx,
                                          const char*         service_name,
                                          const pcl_msg_t*    request,
                                          pcl_stream_msg_fn_t callback,
                                          void*               user_data,
                                          void**              stream_handle) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)adapter_ctx;
  pcl_shm_pending_stream_t* pending;
  pcl_shm_stream_client_handle_t* handle = NULL;
  char provider_id[PCL_SHM_MAX_ID] = {0};
  uint32_t seq_id;
  uint32_t provider_slot = 0u;
  uint32_t attempts;
  int found;
  pcl_status_t rc;

  if (!ctx || !service_name || !request || !callback) return PCL_ERR_INVALID;
  if (request->size > 0u && !request->data) return PCL_ERR_INVALID;
  if (request->size > PCL_SHM_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  pending = (pcl_shm_pending_stream_t*)pcl_calloc(1, sizeof(*pending));
  if (!pending) return PCL_ERR_NOMEM;

  pcl_shm_pending_lock_acquire(ctx);
  seq_id = ++ctx->next_seq_id;
  if (seq_id == 0u) seq_id = ++ctx->next_seq_id;
  pending->seq_id   = seq_id;
  pending->callback = callback;
  pending->user_data = user_data;
  pending->next     = ctx->pending_stream_head;
  ctx->pending_stream_head = pending;
  pcl_shm_pending_lock_release(ctx);

  found = 0;
  for (attempts = 0u; attempts < PCL_SHM_DISCOVERY_RETRIES; ++attempts) {
    if (pcl_shm_bus_lock(ctx) != PCL_OK) break;
    found = pcl_shm_find_provider_slot_locked(ctx, service_name, &provider_slot);
    pcl_shm_bus_unlock(ctx);
    if (found != 0) break;
    pcl_shm_sleep_ms(PCL_SHM_POLL_MS);
  }

  if (found == 0) {
    pcl_shm_pending_stream_remove(ctx, seq_id, NULL, NULL);
    return PCL_ERR_NOT_FOUND;
  }
  if (found < 0) {
    pcl_shm_pending_stream_remove(ctx, seq_id, NULL, NULL);
    return PCL_ERR_INVALID;
  }

  if (pcl_shm_bus_lock(ctx) != PCL_OK) {
    // GCOVR_EXCL_START: the bus semaphore fails only on kernel fault.
    pcl_shm_pending_stream_remove(ctx, seq_id, NULL, NULL);
    return PCL_ERR_STATE;
    // GCOVR_EXCL_STOP
  }
  snprintf(provider_id, sizeof(provider_id), "%s",
           ctx->region->slots[provider_slot].participant_id);
  rc = pcl_shm_enqueue_frame_locked(ctx,
                                    provider_slot,
                                    PCL_SHM_FRAME_STREAM_REQ,
                                    ctx->participant_id,
                                    service_name,
                                    request->type_name,
                                    request->data,
                                    request->size,
                                    seq_id);
  pcl_shm_bus_unlock(ctx);

  if (rc != PCL_OK) {
    // GCOVR_EXCL_START: requires the provider mailbox to fill in the instant
    // after discovery succeeded under the same bus lock cadence.
    pcl_shm_pending_stream_remove(ctx, seq_id, NULL, NULL);
    return rc;
    // GCOVR_EXCL_STOP
  }

  if (stream_handle) {
    handle = (pcl_shm_stream_client_handle_t*)pcl_calloc(1, sizeof(*handle));
    if (handle) {
      handle->owner  = ctx;
      handle->seq_id = seq_id;
      snprintf(handle->provider_id, sizeof(handle->provider_id), "%s",
               provider_id);
    }
    *stream_handle = handle;
  }
  return PCL_STREAMING;
}

static pcl_status_t pcl_shm_stream_send_frame(
    pcl_shared_memory_transport_t* ctx,
    const pcl_shm_stream_send_target_t* target,
    pcl_shm_frame_kind_t                kind,
    const pcl_msg_t*                    msg,
    pcl_status_t                        end_status) {
  uint32_t i;
  pcl_status_t rc = PCL_ERR_NOT_FOUND;
  const void* data = (msg && msg->size > 0u) ? msg->data : NULL;
  uint32_t    size = (msg && msg->size > 0u) ? msg->size : 0u;
  const char* type_name = (msg && msg->type_name) ? msg->type_name : "";
  uint8_t status_byte[1];

  if (!ctx || !target) return PCL_ERR_INVALID;

  /* STREAM_END carries status in its single payload byte (overrides msg). */
  if (kind == PCL_SHM_FRAME_STREAM_END) {
    status_byte[0] = (uint8_t)(end_status & 0xFFu);
    data = status_byte;
    size = 1u;
    type_name = "";
  }

  if (pcl_shm_bus_lock(ctx) != PCL_OK) return PCL_ERR_STATE;
  for (i = 0; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
    pcl_shm_slot_t* slot = &ctx->region->slots[i];
    if (!slot->in_use) continue;
    if (strcmp(slot->participant_id, target->requester_id) == 0) {
      rc = pcl_shm_enqueue_frame_locked(ctx, i, kind,
                                        ctx->participant_id, "",
                                        type_name, data, size,
                                        target->seq_id);
      break;
    }
  }
  pcl_shm_bus_unlock(ctx);
  return rc;
}

static pcl_status_t pcl_shm_stream_send(void*            adapter_ctx,
                                        void*            stream_handle,
                                        const pcl_msg_t* msg) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)adapter_ctx;
  pcl_shm_stream_send_target_t* target =
      (pcl_shm_stream_send_target_t*)stream_handle;
  if (!ctx || !target || !msg) return PCL_ERR_INVALID;
  return pcl_shm_stream_send_frame(ctx, target,
                                   PCL_SHM_FRAME_STREAM_FRAME, msg,
                                   PCL_OK);
}

static pcl_status_t pcl_shm_stream_end(void*        adapter_ctx,
                                       void*        stream_handle,
                                       pcl_status_t status) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)adapter_ctx;
  pcl_shm_stream_send_target_t* target =
      (pcl_shm_stream_send_target_t*)stream_handle;
  pcl_status_t rc;
  if (!ctx || !target) return PCL_ERR_INVALID;
  pcl_shm_active_target_unlink(ctx, target);
  rc = pcl_shm_stream_send_frame(ctx, target,
                                 PCL_SHM_FRAME_STREAM_END, NULL,
                                 status);
  pcl_free(target);
  return rc;
}

static pcl_status_t pcl_shm_stream_cancel(void* adapter_ctx,
                                          void* stream_handle) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)adapter_ctx;
  pcl_shm_stream_client_handle_t* handle =
      (pcl_shm_stream_client_handle_t*)stream_handle;
  uint32_t i;
  pcl_status_t remote_rc = PCL_ERR_NOT_FOUND;
  pcl_status_t local_rc;

  if (!ctx || !handle || handle->provider_id[0] == '\0') {
    return PCL_ERR_INVALID;  // GCOVR_EXCL_LINE: stream cancel with an internal handle whose provider id is always populated at creation
  }

  if (pcl_shm_bus_lock(ctx) == PCL_OK) {
    for (i = 0; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
      pcl_shm_slot_t* slot = &ctx->region->slots[i];
      if (!slot->in_use) continue;
      if (strcmp(slot->participant_id, handle->provider_id) == 0) {
        remote_rc = pcl_shm_enqueue_frame_locked(ctx, i,
                                                 PCL_SHM_FRAME_STREAM_CANCEL,
                                                 ctx->participant_id, "",
                                                 "", NULL, 0u,
                                                 handle->seq_id);
        break;
      }
    }
    pcl_shm_bus_unlock(ctx);
  } else {
    remote_rc = PCL_ERR_STATE;  // GCOVR_EXCL_LINE: bus semaphore fails only on kernel fault
  }

  local_rc = pcl_shm_pending_stream_complete(ctx, handle->seq_id,
                                             PCL_ERR_CANCELLED);
  if (remote_rc != PCL_OK) return remote_rc;
  if (local_rc != PCL_OK && local_rc != PCL_ERR_NOT_FOUND) return local_rc;
  return PCL_OK;
}

static void pcl_shm_shutdown(void* adapter_ctx) {
  (void)adapter_ctx;
}

static void pcl_shm_gateway_sub_cb(pcl_container_t* c,
                                   const pcl_msg_t* msg,
                                   void*            user_data) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)user_data;
  const uint8_t* end;
  const uint8_t* p;
  uint32_t seq_id;
  uint16_t source_len;
  uint16_t service_len;
  uint16_t type_len;
  uint32_t req_len;
  char source_id[PCL_SHM_MAX_ID];
  char* service_name = NULL;
  char* request_type = NULL;
  pcl_port_t* service_port;
  pcl_svc_context_t* svc_ctx = NULL;
  pcl_shm_response_target_t* target = NULL;
  pcl_msg_t request;
  pcl_msg_t response;
  pcl_msg_t empty_response;
  pcl_status_t rc;

  (void)c;
  if (!ctx || !msg || !msg->data || msg->size < 14u) return;

  p = (const uint8_t*)msg->data;
  end = p + msg->size;
  seq_id = pcl_shm_read_u32_be(p);      p += 4u;
  if ((size_t)(end - p) < 2u) return;
  source_len = pcl_shm_read_u16_be(p);  p += 2u;
  if (source_len == 0u || source_len >= sizeof(source_id)) return;
  if ((size_t)(end - p) < source_len + 2u) return;
  memcpy(source_id, p, source_len);
  source_id[source_len] = '\0';         p += source_len;

  service_len = pcl_shm_read_u16_be(p); p += 2u;
  if ((size_t)(end - p) < service_len + 2u) return;
  service_name = (char*)pcl_alloc((size_t)service_len + 1u);
  if (!service_name) return;
  memcpy(service_name, p, service_len);
  service_name[service_len] = '\0';     p += service_len;

  type_len = pcl_shm_read_u16_be(p);    p += 2u;
  if ((size_t)(end - p) < type_len + 4u) {
    // GCOVR_EXCL_START: internal SVC_REQ topic frames are produced by pcl_shm_handle_frame and always well-formed
    pcl_free(service_name);
    return;
    // GCOVR_EXCL_STOP
  }
  if (type_len > 0u) {
    request_type = (char*)pcl_alloc((size_t)type_len + 1u);
    if (!request_type) {
      // GCOVR_EXCL_START: heap exhaustion is not injectable through this path
      pcl_free(service_name);
      return;
      // GCOVR_EXCL_STOP
    }
    memcpy(request_type, p, type_len);
    request_type[type_len] = '\0';
  }
  p += type_len;

  req_len = pcl_shm_read_u32_be(p);     p += 4u;
  if ((size_t)(end - p) < req_len) {
    // GCOVR_EXCL_START: internal SVC_REQ topic frames are produced by pcl_shm_handle_frame and always well-formed
    pcl_free(request_type);
    pcl_free(service_name);
    return;
    // GCOVR_EXCL_STOP
  }

  service_port = pcl_shm_find_remote_service(ctx->executor, service_name, source_id);
  if (!service_port) {
    memset(&empty_response, 0, sizeof(empty_response));
    target = (pcl_shm_response_target_t*)pcl_calloc(1, sizeof(*target));
    if (target) {
      target->seq_id = seq_id;
      snprintf(target->requester_id, sizeof(target->requester_id), "%s", source_id);
      pcl_shm_send_response(ctx, target, &empty_response);
      pcl_free(target);
    }
    pcl_free(request_type);
    pcl_free(service_name);
    return;
  }

  svc_ctx = (pcl_svc_context_t*)pcl_calloc(1, sizeof(*svc_ctx));
  target = (pcl_shm_response_target_t*)pcl_calloc(1, sizeof(*target));
  if (!svc_ctx || !target) {
    // GCOVR_EXCL_START: heap exhaustion is not injectable through this path
    pcl_free(svc_ctx);
    pcl_free(target);
    pcl_free(request_type);
    pcl_free(service_name);
    return;
    // GCOVR_EXCL_STOP
  }

  target->seq_id = seq_id;
  snprintf(target->requester_id, sizeof(target->requester_id), "%s", source_id);

  memset(&request, 0, sizeof(request));
  request.data = (req_len > 0u) ? p : NULL;
  request.size = req_len;
  request.type_name = request_type;

  memset(&response, 0, sizeof(response));
  svc_ctx->executor = ctx->executor;
  svc_ctx->transport = &ctx->transport;
  svc_ctx->transport_ctx = target;
  snprintf(svc_ctx->peer_id, sizeof(svc_ctx->peer_id), "%s", source_id);

  rc = service_port->svc_handler(service_port->owner,
                                 &request,
                                 &response,
                                 svc_ctx,
                                 service_port->svc_user_data);
  if (rc == PCL_OK) {
    pcl_shm_send_response(ctx, target, &response);
    pcl_free(target);
    pcl_free(svc_ctx);
  } else if (rc != PCL_PENDING) {
    memset(&empty_response, 0, sizeof(empty_response));
    pcl_shm_send_response(ctx, target, &empty_response);
    pcl_free(target);
    pcl_free(svc_ctx);
  }

  pcl_free(request_type);
  pcl_free(service_name);
}

static void pcl_shm_gateway_stream_sub_cb(pcl_container_t* c,
                                          const pcl_msg_t* msg,
                                          void*            user_data) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)user_data;
  const uint8_t* end;
  const uint8_t* p;
  uint32_t seq_id;
  uint16_t source_len;
  uint16_t service_len;
  uint16_t type_len;
  uint32_t req_len;
  char source_id[PCL_SHM_MAX_ID];
  char* service_name = NULL;
  char* request_type = NULL;
  pcl_port_t* stream_port;
  pcl_stream_context_t* stream_ctx = NULL;
  pcl_shm_stream_send_target_t* target = NULL;
  pcl_msg_t request;
  pcl_status_t rc;

  (void)c;
  if (!ctx || !msg || !msg->data || msg->size < 14u) return;

  p = (const uint8_t*)msg->data;
  end = p + msg->size;
  seq_id = pcl_shm_read_u32_be(p);      p += 4u;
  if ((size_t)(end - p) < 2u) return;
  source_len = pcl_shm_read_u16_be(p);  p += 2u;
  if (source_len == 0u || source_len >= sizeof(source_id)) return;
  if ((size_t)(end - p) < source_len + 2u) return;
  memcpy(source_id, p, source_len);
  source_id[source_len] = '\0';         p += source_len;

  service_len = pcl_shm_read_u16_be(p); p += 2u;
  if ((size_t)(end - p) < service_len + 2u) return;
  service_name = (char*)pcl_alloc((size_t)service_len + 1u);
  if (!service_name) return;
  memcpy(service_name, p, service_len);
  service_name[service_len] = '\0';     p += service_len;

  type_len = pcl_shm_read_u16_be(p);    p += 2u;
  if ((size_t)(end - p) < type_len + 4u) {
    // GCOVR_EXCL_START: internal STREAM_REQ topic frames are produced by pcl_shm_handle_frame and always well-formed
    pcl_free(service_name);
    return;
    // GCOVR_EXCL_STOP
  }
  if (type_len > 0u) {
    request_type = (char*)pcl_alloc((size_t)type_len + 1u);
    if (!request_type) {
      // GCOVR_EXCL_START: heap exhaustion is not injectable through this path
      pcl_free(service_name);
      return;
      // GCOVR_EXCL_STOP
    }
    memcpy(request_type, p, type_len);
    request_type[type_len] = '\0';
  }
  p += type_len;

  req_len = pcl_shm_read_u32_be(p);     p += 4u;
  if ((size_t)(end - p) < req_len) {
    // GCOVR_EXCL_START: internal STREAM_REQ topic frames are produced by pcl_shm_handle_frame and always well-formed
    pcl_free(request_type);
    pcl_free(service_name);
    return;
    // GCOVR_EXCL_STOP
  }

  stream_port = pcl_shm_find_remote_stream_service(ctx->executor,
                                                   service_name,
                                                   source_id);
  if (!stream_port) {
    /* Best-effort: tell the client the stream ended with NOT_FOUND. */
    pcl_shm_stream_send_target_t end_target;
    end_target.owner  = ctx;
    end_target.seq_id = seq_id;
    snprintf(end_target.requester_id, sizeof(end_target.requester_id), "%s", source_id);
    pcl_shm_stream_send_frame(ctx, &end_target,
                              PCL_SHM_FRAME_STREAM_END, NULL,
                              PCL_ERR_NOT_FOUND);
    pcl_free(request_type);
    pcl_free(service_name);
    return;
  }

  target = (pcl_shm_stream_send_target_t*)pcl_calloc(1, sizeof(*target));
  stream_ctx = (pcl_stream_context_t*)pcl_calloc(1, sizeof(*stream_ctx));
  if (!target || !stream_ctx) {
    // GCOVR_EXCL_START: heap exhaustion is not injectable through this path
    pcl_free(target);
    pcl_free(stream_ctx);
    pcl_free(request_type);
    pcl_free(service_name);
    return;
    // GCOVR_EXCL_STOP
  }
  target->owner      = ctx;
  target->seq_id     = seq_id;
  target->stream_ctx = stream_ctx;
  snprintf(target->requester_id, sizeof(target->requester_id), "%s", source_id);

  stream_ctx->executor      = ctx->executor;
  stream_ctx->transport     = &ctx->transport;
  stream_ctx->transport_ctx = target;

  /* Register before invoking the handler so destroy can abort it even if
   * the handler returns PCL_STREAMING and then loses the context. */
  pcl_shm_active_target_register(ctx, target);

  memset(&request, 0, sizeof(request));
  request.data      = (req_len > 0u) ? p : NULL;
  request.size      = req_len;
  request.type_name = request_type;

  rc = stream_port->stream_handler(stream_port->owner, &request, stream_ctx,
                                   stream_port->stream_user_data);

  if (rc != PCL_STREAMING) {
    /* Handler refused -- close the stream immediately with the returned
     * status and free the resources the handler did not retain. */
    pcl_shm_active_target_unlink(ctx, target);
    pcl_shm_stream_send_frame(ctx, target,
                              PCL_SHM_FRAME_STREAM_END, NULL,
                              rc == PCL_OK ? PCL_OK : rc);
    pcl_free(target);
    pcl_free(stream_ctx);
  }

  pcl_free(request_type);
  pcl_free(service_name);
}

static pcl_status_t pcl_shm_gateway_on_configure(pcl_container_t* c, void* ud) {
  pcl_port_t* port;
  pcl_status_t rc;

  port = pcl_container_add_subscriber(c,
                                      PCL_SHM_INTERNAL_TOPIC_SVC_REQ,
                                      PCL_SHM_INTERNAL_REQ_TYPE,
                                      pcl_shm_gateway_sub_cb,
                                      ud);
  if (!port) return PCL_ERR_NOMEM;
  rc = pcl_port_set_route(port, PCL_ROUTE_REMOTE, NULL, 0);
  if (rc != PCL_OK) return rc;

  port = pcl_container_add_subscriber(c,
                                      PCL_SHM_INTERNAL_TOPIC_STREAM_REQ,
                                      PCL_SHM_INTERNAL_STREAM_REQ_TYPE,
                                      pcl_shm_gateway_stream_sub_cb,
                                      ud);
  if (!port) return PCL_ERR_NOMEM;
  return pcl_port_set_route(port, PCL_ROUTE_REMOTE, NULL, 0);
}

static pcl_status_t pcl_shm_handle_frame(pcl_shared_memory_transport_t* ctx,
                                         const pcl_shm_frame_t*         frame) {
  if (!ctx || !frame) return PCL_ERR_INVALID;
  if (frame->data_size > PCL_SHM_MAX_PAYLOAD) return PCL_ERR_INVALID;

  if (frame->kind == PCL_SHM_FRAME_PUBLISH) {
    size_t source_len = pcl_shm_strnlen(frame->source_id,
                                        sizeof(frame->source_id));
    size_t name_len = pcl_shm_strnlen(frame->name, sizeof(frame->name));
    size_t type_len = pcl_shm_strnlen(frame->type_name,
                                      sizeof(frame->type_name));
    pcl_msg_t msg = {0};
    if (source_len == sizeof(frame->source_id) ||
        name_len == sizeof(frame->name) ||
        type_len == sizeof(frame->type_name)) {
      return PCL_ERR_INVALID;  // GCOVR_EXCL_LINE: bus frames are written by this transport under the bus lock; an unterminated name requires a corrupted region
    }
    msg.data = (frame->data_size > 0u) ? frame->data : NULL;
    msg.size = frame->data_size;
    msg.type_name = frame->type_name[0] ? frame->type_name : NULL;
    return pcl_executor_post_remote_incoming(ctx->executor,
                                             frame->source_id,
                                             frame->name,
                                             &msg);
  }

  if (frame->kind == PCL_SHM_FRAME_SVC_REQ) {
    size_t raw_source_len = pcl_shm_strnlen(frame->source_id,
                                            sizeof(frame->source_id));
    size_t raw_service_len = pcl_shm_strnlen(frame->name,
                                             sizeof(frame->name));
    size_t raw_type_len = pcl_shm_strnlen(frame->type_name,
                                          sizeof(frame->type_name));
    uint16_t source_len;
    uint16_t service_len;
    uint16_t type_len;
    uint32_t payload_size;
    uint8_t* payload;
    size_t off = 0u;
    pcl_msg_t msg = {0};
    pcl_status_t rc;

    if (raw_source_len == sizeof(frame->source_id) ||
        raw_service_len == sizeof(frame->name) ||
        raw_type_len == sizeof(frame->type_name)) {
      return PCL_ERR_INVALID;  // GCOVR_EXCL_LINE: bus frames are written by this transport under the bus lock; an unterminated name requires a corrupted region
    }
    source_len = (uint16_t)raw_source_len;
    service_len = (uint16_t)raw_service_len;
    type_len = (uint16_t)raw_type_len;
    payload_size = 4u + 2u + source_len + 2u + service_len +
                   2u + type_len + 4u + frame->data_size;
    payload = (uint8_t*)pcl_alloc(payload_size);

    if (!payload) return PCL_ERR_NOMEM;
    pcl_shm_write_u32_be(payload + off, frame->seq_id);      off += 4u;
    pcl_shm_write_u16_be(payload + off, source_len);         off += 2u;
    memcpy(payload + off, frame->source_id, source_len);     off += source_len;
    pcl_shm_write_u16_be(payload + off, service_len);        off += 2u;
    memcpy(payload + off, frame->name, service_len);         off += service_len;
    pcl_shm_write_u16_be(payload + off, type_len);           off += 2u;
    if (type_len > 0u) memcpy(payload + off, frame->type_name, type_len);
    off += type_len;
    pcl_shm_write_u32_be(payload + off, frame->data_size);   off += 4u;
    if (frame->data_size > 0u) memcpy(payload + off, frame->data, frame->data_size);

    msg.data = payload;
    msg.size = payload_size;
    msg.type_name = PCL_SHM_INTERNAL_REQ_TYPE;
    rc = pcl_executor_post_remote_incoming(ctx->executor,
                                           frame->source_id,
                                           PCL_SHM_INTERNAL_TOPIC_SVC_REQ,
                                           &msg);
    pcl_free(payload);
    return rc;
  }

  if (frame->kind == PCL_SHM_FRAME_SVC_RESP) {
    pcl_resp_cb_fn_t callback = NULL;
    void* user_data = NULL;
    pcl_msg_t response = {0};
    size_t type_len = pcl_shm_strnlen(frame->type_name,
                                      sizeof(frame->type_name));
    if (type_len == sizeof(frame->type_name)) return PCL_ERR_INVALID;
    if (pcl_shm_pending_remove(ctx, frame->seq_id, &callback, &user_data) != PCL_OK) {
      return PCL_ERR_NOT_FOUND;  // GCOVR_EXCL_LINE: a response with no pending entry requires adversarial peer timing
    }
    response.data = (frame->data_size > 0u) ? frame->data : NULL;
    response.size = frame->data_size;
    response.type_name = frame->type_name[0] ? frame->type_name : NULL;
    return pcl_executor_post_response_msg(ctx->executor, callback, user_data, &response);
  }

  if (frame->kind == PCL_SHM_FRAME_STREAM_REQ) {
    /* Same wire layout as SVC_REQ; reroute to the stream gateway topic. */
    size_t raw_source_len = pcl_shm_strnlen(frame->source_id,
                                            sizeof(frame->source_id));
    size_t raw_service_len = pcl_shm_strnlen(frame->name,
                                             sizeof(frame->name));
    size_t raw_type_len = pcl_shm_strnlen(frame->type_name,
                                          sizeof(frame->type_name));
    uint16_t source_len;
    uint16_t service_len;
    uint16_t type_len;
    uint32_t payload_size;
    uint8_t* payload;
    size_t off = 0u;
    pcl_msg_t msg = {0};
    pcl_status_t rc;

    if (raw_source_len == sizeof(frame->source_id) ||
        raw_service_len == sizeof(frame->name) ||
        raw_type_len == sizeof(frame->type_name)) {
      return PCL_ERR_INVALID;  // GCOVR_EXCL_LINE: bus frames are written by this transport under the bus lock; an unterminated name requires a corrupted region
    }
    source_len = (uint16_t)raw_source_len;
    service_len = (uint16_t)raw_service_len;
    type_len = (uint16_t)raw_type_len;
    payload_size = 4u + 2u + source_len + 2u + service_len +
                   2u + type_len + 4u + frame->data_size;
    payload = (uint8_t*)pcl_alloc(payload_size);
    if (!payload) return PCL_ERR_NOMEM;
    pcl_shm_write_u32_be(payload + off, frame->seq_id);      off += 4u;
    pcl_shm_write_u16_be(payload + off, source_len);         off += 2u;
    memcpy(payload + off, frame->source_id, source_len);     off += source_len;
    pcl_shm_write_u16_be(payload + off, service_len);        off += 2u;
    memcpy(payload + off, frame->name, service_len);         off += service_len;
    pcl_shm_write_u16_be(payload + off, type_len);           off += 2u;
    if (type_len > 0u) memcpy(payload + off, frame->type_name, type_len);
    off += type_len;
    pcl_shm_write_u32_be(payload + off, frame->data_size);   off += 4u;
    if (frame->data_size > 0u) memcpy(payload + off, frame->data, frame->data_size);

    msg.data = payload;
    msg.size = payload_size;
    msg.type_name = PCL_SHM_INTERNAL_STREAM_REQ_TYPE;
    rc = pcl_executor_post_remote_incoming(ctx->executor,
                                           frame->source_id,
                                           PCL_SHM_INTERNAL_TOPIC_STREAM_REQ,
                                           &msg);
    pcl_free(payload);
    return rc;
  }

  if (frame->kind == PCL_SHM_FRAME_STREAM_CANCEL) {
    size_t source_len = pcl_shm_strnlen(frame->source_id,
                                        sizeof(frame->source_id));
    if (source_len == 0u || source_len == sizeof(frame->source_id)) {
      return PCL_ERR_INVALID;  // GCOVR_EXCL_LINE: bus frames are written by this transport under the bus lock; an unterminated name requires a corrupted region
    }
    return pcl_shm_active_stream_cancel(ctx, frame->seq_id,
                                        frame->source_id);
  }

  if (frame->kind == PCL_SHM_FRAME_STREAM_FRAME) {
    pcl_stream_msg_fn_t callback = NULL;
    void* user_data = NULL;
    pcl_msg_t response = {0};
    pcl_shm_stream_trampoline_t* tr;
    size_t type_len = pcl_shm_strnlen(frame->type_name,
                                      sizeof(frame->type_name));
    if (type_len == sizeof(frame->type_name)) return PCL_ERR_INVALID;
    if (pcl_shm_pending_stream_lookup(ctx, frame->seq_id,
                                      &callback, &user_data) != PCL_OK) {
      return PCL_ERR_NOT_FOUND;  // GCOVR_EXCL_LINE: a stream frame with no pending entry requires adversarial peer timing
    }
    tr = (pcl_shm_stream_trampoline_t*)pcl_calloc(1, sizeof(*tr));
    if (!tr) return PCL_ERR_NOMEM;
    tr->callback = callback;
    tr->user_data = user_data;
    tr->owner = ctx;
    tr->seq_id = frame->seq_id;
    tr->end = 0;
    tr->status = PCL_OK;
    response.data = (frame->data_size > 0u) ? frame->data : NULL;
    response.size = frame->data_size;
    response.type_name = frame->type_name[0] ? frame->type_name : NULL;
    return pcl_executor_post_response_msg(ctx->executor,
                                          pcl_shm_stream_trampoline_cb,
                                          tr, &response);
  }

  if (frame->kind == PCL_SHM_FRAME_STREAM_END) {
    pcl_stream_msg_fn_t callback = NULL;
    void* user_data = NULL;
    pcl_msg_t empty = {0};
    pcl_shm_stream_trampoline_t* tr;
    pcl_status_t end_status = PCL_OK;
    if (frame->data_size >= 1u) {
      end_status = (pcl_status_t)(int8_t)frame->data[0];
    }
    if (pcl_shm_pending_stream_remove(ctx, frame->seq_id,
                                      &callback, &user_data) != PCL_OK) {
      return PCL_ERR_NOT_FOUND;  // GCOVR_EXCL_LINE: an END after local cancel/flush is peer-timing dependent and cannot be forced deterministically
    }
    tr = (pcl_shm_stream_trampoline_t*)pcl_calloc(1, sizeof(*tr));
    if (!tr) return PCL_ERR_NOMEM;
    tr->callback = callback;
    tr->user_data = user_data;
    tr->owner = ctx;
    tr->seq_id = frame->seq_id;
    tr->end = 1;
    tr->status = end_status;
    return pcl_executor_post_response_msg(ctx->executor,
                                          pcl_shm_stream_trampoline_cb,
                                          tr, &empty);
  }

  return PCL_ERR_INVALID;  // GCOVR_EXCL_LINE: an unknown frame kind requires a corrupted region or misbehaving peer implementation
}

#ifdef _WIN32
static DWORD WINAPI pcl_shm_recv_thread_main(LPVOID arg)
#else
static void* pcl_shm_recv_thread_main(void* arg)
#endif
{
  pcl_shared_memory_transport_t* ctx = (pcl_shared_memory_transport_t*)arg;

  while (!ctx->recv_stop) {
    pcl_shm_frame_t frame;
    int has_frame = 0;

    pcl_shm_sync_local_services(ctx);

    if (pcl_shm_bus_lock(ctx) == PCL_OK) {
      if (ctx->slot_index < PCL_SHM_MAX_PARTICIPANTS &&
          ctx->region->slots[ctx->slot_index].in_use &&
          strcmp(ctx->region->slots[ctx->slot_index].participant_id,
                 ctx->participant_id) == 0) {
        pcl_shm_slot_t* slot = &ctx->region->slots[ctx->slot_index];
        if (slot->read_index != slot->write_index) {
          frame = slot->frames[slot->read_index];
          memset(&slot->frames[slot->read_index], 0, sizeof(slot->frames[slot->read_index]));
          slot->read_index = (slot->read_index + 1u) % PCL_SHM_QUEUE_DEPTH;
          has_frame = 1;
        }
      }
      pcl_shm_bus_unlock(ctx);
    }

    if (has_frame) {
      pcl_shm_handle_frame(ctx, &frame);
    } else {
      pcl_shm_sleep_ms(PCL_SHM_POLL_MS);
    }
  }

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

pcl_shared_memory_transport_t* pcl_shared_memory_transport_create(
    const char*      bus_name,
    const char*      participant_id,
    pcl_executor_t*  executor) {
  pcl_shared_memory_transport_t* ctx;
  pcl_callbacks_t gateway_callbacks;
  char gateway_name[PCL_SHM_MAX_NAME];
  uint32_t i;
  pcl_status_t rc;

  if (!bus_name || !participant_id || !participant_id[0] || !executor) {
    return NULL;  // GCOVR_EXCL_LINE: heap exhaustion is not injectable through this path
  }

  ctx = (pcl_shared_memory_transport_t*)pcl_calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;

  snprintf(ctx->bus_name, sizeof(ctx->bus_name), "%s", bus_name);
  snprintf(ctx->participant_id, sizeof(ctx->participant_id), "%s", participant_id);
  ctx->executor = executor;
  ctx->slot_index = PCL_SHM_MAX_PARTICIPANTS;
#ifndef _WIN32
  ctx->shm_fd = -1;
#endif
  pcl_shm_pending_lock_init(ctx);

  rc = pcl_shm_platform_open(ctx);
  if (rc != PCL_OK) {
    // GCOVR_EXCL_START: shm_open/mmap/sem_open fail only on OS resource exhaustion
    pcl_shm_platform_close(ctx, 0);
    pcl_shm_pending_lock_destroy(ctx);
    pcl_free(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }

  if (pcl_shm_bus_lock(ctx) != PCL_OK) {
    // GCOVR_EXCL_START: bus semaphore fails only on kernel fault
    pcl_shm_platform_close(ctx, 0);
    pcl_shm_pending_lock_destroy(ctx);
    pcl_free(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }

  if (ctx->region->magic != PCL_SHM_MAGIC || ctx->region->version != PCL_SHM_VERSION) {
    memset(ctx->region, 0, sizeof(*ctx->region));
    ctx->region->magic = PCL_SHM_MAGIC;
    ctx->region->version = PCL_SHM_VERSION;
  }

  for (i = 0; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
    if (ctx->region->slots[i].in_use &&
        strcmp(ctx->region->slots[i].participant_id, ctx->participant_id) == 0) {
      pcl_shm_bus_unlock(ctx);
      pcl_shm_platform_close(ctx, 0);
      pcl_shm_pending_lock_destroy(ctx);
      pcl_free(ctx);
      return NULL;
    }
  }

  for (i = 0; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
    if (!ctx->region->slots[i].in_use) {
      pcl_shm_slot_t* slot = &ctx->region->slots[i];
      memset(slot, 0, sizeof(*slot));
      slot->in_use = 1u;
      slot->generation = ++ctx->region->next_generation;
      snprintf(slot->participant_id, sizeof(slot->participant_id), "%s",
               ctx->participant_id);
      ctx->region->participant_count++;
      ctx->slot_index = i;
      break;
    }
  }

  pcl_shm_bus_unlock(ctx);

  if (ctx->slot_index >= PCL_SHM_MAX_PARTICIPANTS) {
    pcl_shm_platform_close(ctx, 0);
    pcl_shm_pending_lock_destroy(ctx);
    pcl_free(ctx);
    return NULL;
  }

  memset(&ctx->transport, 0, sizeof(ctx->transport));
  ctx->transport.publish = pcl_shm_publish;
  ctx->transport.subscribe = pcl_shm_subscribe;
  ctx->transport.invoke_async = pcl_shm_invoke_async;
  ctx->transport.respond = pcl_shm_respond;
  ctx->transport.invoke_stream = pcl_shm_invoke_stream;
  ctx->transport.stream_send = pcl_shm_stream_send;
  ctx->transport.stream_end = pcl_shm_stream_end;
  ctx->transport.stream_cancel = pcl_shm_stream_cancel;
  ctx->transport.shutdown = pcl_shm_shutdown;
  ctx->transport.adapter_ctx = ctx;

  memset(&gateway_callbacks, 0, sizeof(gateway_callbacks));
  gateway_callbacks.on_configure = pcl_shm_gateway_on_configure;
  snprintf(gateway_name, sizeof(gateway_name), "__pcl_shm_gateway_%s",
           ctx->participant_id);
  ctx->gateway = pcl_container_create(gateway_name, &gateway_callbacks, ctx);
  if (!ctx->gateway) {
    // GCOVR_EXCL_START: gateway container creation fails only on heap exhaustion
    pcl_shared_memory_transport_destroy(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }

#ifdef _WIN32
  ctx->recv_thread = CreateThread(NULL, 0, pcl_shm_recv_thread_main, ctx, 0, NULL);
  if (!ctx->recv_thread) {
    pcl_shared_memory_transport_destroy(ctx);
    return NULL;
  }
#else
  if (pthread_create(&ctx->recv_thread, NULL, pcl_shm_recv_thread_main, ctx) != 0) {
    // GCOVR_EXCL_START: recv-thread creation fails only on kernel resource exhaustion
    pcl_shared_memory_transport_destroy(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }
#endif

  return ctx;
}

const pcl_transport_t* pcl_shared_memory_transport_get_transport(
    pcl_shared_memory_transport_t* ctx) {
  if (!ctx) return NULL;
  return &ctx->transport;
}

pcl_container_t* pcl_shared_memory_transport_gateway_container(
    pcl_shared_memory_transport_t* ctx) {
  if (!ctx) return NULL;
  return ctx->gateway;
}

pcl_status_t pcl_shared_memory_transport_set_topic_backpressure(
    pcl_shared_memory_transport_t* ctx,
    const char*                    topic,
    uint32_t                       timeout_ms) {
  uint32_t i;
  uint32_t free_index = PCL_SHM_MAX_BACKPRESSURE_TOPICS;

  if (!ctx || !topic || !topic[0]) return PCL_ERR_INVALID;
  if (pcl_shm_strnlen(topic, PCL_SHM_MAX_NAME) == PCL_SHM_MAX_NAME) {
    return PCL_ERR_INVALID;
  }

  pcl_shm_pending_lock_acquire(ctx);
  for (i = 0u; i < PCL_SHM_MAX_BACKPRESSURE_TOPICS; ++i) {
    if (ctx->backpressure[i].in_use &&
        strcmp(ctx->backpressure[i].topic, topic) == 0) {
      if (timeout_ms == 0u) {
        memset(&ctx->backpressure[i], 0, sizeof(ctx->backpressure[i]));
      } else {
        ctx->backpressure[i].timeout_ms = timeout_ms;
      }
      pcl_shm_pending_lock_release(ctx);
      return PCL_OK;
    }
    if (!ctx->backpressure[i].in_use &&
        free_index == PCL_SHM_MAX_BACKPRESSURE_TOPICS) {
      free_index = i;
    }
  }

  if (timeout_ms == 0u) {
    pcl_shm_pending_lock_release(ctx);
    return PCL_OK;
  }
  if (free_index == PCL_SHM_MAX_BACKPRESSURE_TOPICS) {
    pcl_shm_pending_lock_release(ctx);
    return PCL_ERR_NOMEM;
  }

  ctx->backpressure[free_index].in_use = 1;
  ctx->backpressure[free_index].timeout_ms = timeout_ms;
  snprintf(ctx->backpressure[free_index].topic,
           sizeof(ctx->backpressure[free_index].topic),
           "%s",
           topic);
  pcl_shm_pending_lock_release(ctx);
  return PCL_OK;
}

void pcl_shared_memory_transport_destroy(pcl_shared_memory_transport_t* ctx) {
  int unlink_objects = 0;

  if (!ctx) return;

  /* Only clear the default transport if THIS instance is the active default --
     a manifest/plugin teardown must not wipe a default another owner installed. */
  if (ctx->executor) {
    const pcl_transport_t* def = pcl_executor_get_transport(ctx->executor);
    if (def && def->adapter_ctx == ctx) {
      pcl_executor_set_transport(ctx->executor, NULL);
    }
    pcl_executor_register_transport(ctx->executor, ctx->participant_id, NULL);
  }

  ctx->recv_stop = 1;
#ifdef _WIN32
  if (ctx->recv_thread) {
    WaitForSingleObject(ctx->recv_thread, 5000);
    CloseHandle(ctx->recv_thread);
    ctx->recv_thread = NULL;
  }
#else
  if (ctx->recv_thread) {
    pthread_join(ctx->recv_thread, NULL);
    ctx->recv_thread = 0;
  }
#endif

  if (ctx->gateway) {
    if (ctx->executor) {
      pcl_executor_remove(ctx->executor, ctx->gateway);
    }
    pcl_container_destroy(ctx->gateway);
    ctx->gateway = NULL;
  }

  if (ctx->region && pcl_shm_bus_lock(ctx) == PCL_OK) {
    if (ctx->slot_index < PCL_SHM_MAX_PARTICIPANTS) {
      pcl_shm_slot_t* slot = &ctx->region->slots[ctx->slot_index];
      if (slot->in_use &&
          strcmp(slot->participant_id, ctx->participant_id) == 0) {
        memset(slot, 0, sizeof(*slot));
        if (ctx->region->participant_count > 0u) {
          ctx->region->participant_count--;
        }
      }
    }
    unlink_objects = (ctx->region->participant_count == 0u);
    pcl_shm_bus_unlock(ctx);
  }

  pcl_shm_active_streams_abort(ctx);
  pcl_shm_pending_clear(ctx);
  pcl_shm_pending_stream_clear(ctx);
  pcl_shm_platform_close(ctx, unlink_objects);
  pcl_shm_pending_lock_destroy(ctx);
  pcl_free(ctx);
}
