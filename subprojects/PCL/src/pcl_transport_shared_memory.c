/// \file pcl_transport_shared_memory.c
/// \brief Inter-process shared-memory central bus transport for PCL.
///
/// Each transport instance joins a named bus backed by an OS shared-memory
/// region. The bus owns participant mailboxes in shared memory; a background
/// receive thread polls the local mailbox and posts work onto the executor
/// thread. Topic publish fans out across the bus, while async service
/// requests route to the unique participant advertising the requested service.
#include "pcl/pcl_transport_shared_memory.h"

#include "pcl_internal.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"

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
#define PCL_SHM_DISCOVERY_RETRIES 20u
#define PCL_SHM_POLL_MS 5u

typedef enum {
  PCL_SHM_FRAME_NONE = 0,
  PCL_SHM_FRAME_PUBLISH = 1,
  PCL_SHM_FRAME_SVC_REQ = 2,
  PCL_SHM_FRAME_SVC_RESP = 3,
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

typedef struct {
  uint32_t seq_id;
  char     requester_id[PCL_SHM_MAX_ID];
} pcl_shm_response_target_t;

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
  Sleep(ms);
#else
  struct timespec req;
  req.tv_sec = (time_t)(ms / 1000u);
  req.tv_nsec = (long)((ms % 1000u) * 1000000u);
  while (nanosleep(&req, &req) == -1 && errno == EINTR) {
  }
#endif
}

static void pcl_shm_copy_token(const char* src,
                               char*       dst,
                               size_t      dst_size) {
  size_t i;
  if (!dst || dst_size == 0u) return;
  if (!src) {
    dst[0] = '\0';
    return;
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
    if (errno != EINTR) return PCL_ERR_STATE;
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
    ctx->region = NULL;
    return PCL_ERR_STATE;
  }

  ctx->lock_sem = sem_open(ctx->lock_object_name, O_CREAT, 0600, 1);
  if (ctx->lock_sem == SEM_FAILED) {
    ctx->lock_sem = NULL;
    return PCL_ERR_STATE;
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
    case PCL_PORT_PUBLISHER:      return PCL_ENDPOINT_PUBLISHER;
    case PCL_PORT_SUBSCRIBER:     return PCL_ENDPOINT_SUBSCRIBER;
    case PCL_PORT_SERVICE:        return PCL_ENDPOINT_PROVIDED;
    case PCL_PORT_CLIENT:         return PCL_ENDPOINT_CONSUMED;
    case PCL_PORT_STREAM_SERVICE: return PCL_ENDPOINT_STREAM_PROVIDED;
    default:                      return PCL_ENDPOINT_PUBLISHER;
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
      return PCL_ROUTE_REMOTE;
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

  if (!e || !name || !source_peer_id) return NULL;

  for (ci = 0; ci < e->container_count; ++ci) {
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
      return port;
    }
  }

  return NULL;
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

  for (ci = 0; ci < ctx->executor->container_count; ++ci) {
    pcl_container_t* c = ctx->executor->containers[ci];
    for (pi = 0; pi < c->port_count; ++pi) {
      pcl_port_t* port = &c->ports[pi];
      uint32_t route_mode;

      if (port->type != PCL_PORT_SERVICE || port->svc_handler == NULL) {
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
        return;
      }

      snprintf(services[*count], PCL_SHM_MAX_NAME, "%s", port->name);
      (*count)++;
    }
  }
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
      free(node);
      pcl_shm_pending_lock_release(ctx);
      return PCL_OK;
    }
  }
  pcl_shm_pending_lock_release(ctx);
  return PCL_ERR_NOT_FOUND;
}

static void pcl_shm_pending_clear(pcl_shared_memory_transport_t* ctx) {
  pcl_shm_pending_request_t* node;

  if (!ctx) return;
  pcl_shm_pending_lock_acquire(ctx);
  node = ctx->pending_head;
  ctx->pending_head = NULL;
  pcl_shm_pending_lock_release(ctx);

  while (node) {
    pcl_shm_pending_request_t* next = node->next;
    free(node);
    node = next;
  }
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
    return PCL_ERR_INVALID;
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
  uint32_t i;
  int delivered = 0;
  pcl_status_t rc = PCL_OK;

  if (!ctx || !topic || !msg || !msg->type_name) return PCL_ERR_INVALID;
  if (msg->size > 0u && !msg->data) return PCL_ERR_INVALID;

  if (pcl_shm_bus_lock(ctx) != PCL_OK) return PCL_ERR_STATE;
  for (i = 0; i < PCL_SHM_MAX_PARTICIPANTS; ++i) {
    pcl_status_t enqueue_rc;
    if (i == ctx->slot_index || !ctx->region->slots[i].in_use) continue;
    enqueue_rc = pcl_shm_enqueue_frame_locked(ctx,
                                              i,
                                              PCL_SHM_FRAME_PUBLISH,
                                              ctx->participant_id,
                                              topic,
                                              msg->type_name,
                                              msg->data,
                                              msg->size,
                                              0u);
    if (enqueue_rc == PCL_OK) {
      delivered = 1;
    } else if (rc == PCL_OK) {
      rc = enqueue_rc;
    }
  }
  pcl_shm_bus_unlock(ctx);

  return delivered ? PCL_OK : (rc == PCL_OK ? PCL_ERR_NOT_FOUND : rc);
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
  free(target);
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

  pending = (pcl_shm_pending_request_t*)calloc(1, sizeof(*pending));
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
    pcl_shm_pending_remove(ctx, seq_id, NULL, NULL);
    return PCL_ERR_STATE;
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
    pcl_shm_pending_remove(ctx, seq_id, NULL, NULL);
  }
  return rc;
}

static void pcl_shm_shutdown(void* adapter_ctx) {
  (void)adapter_ctx;
}

static void pcl_shm_gateway_sub_cb(pcl_container_t* c,
                                   const pcl_msg_t* msg,
                                   void*            user_data) {
  pcl_shared_memory_transport_t* ctx =
      (pcl_shared_memory_transport_t*)user_data;
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
  seq_id = pcl_shm_read_u32_be(p);      p += 4u;
  source_len = pcl_shm_read_u16_be(p);  p += 2u;
  if (source_len == 0u || source_len >= sizeof(source_id)) return;
  memcpy(source_id, p, source_len);
  source_id[source_len] = '\0';         p += source_len;

  service_len = pcl_shm_read_u16_be(p); p += 2u;
  service_name = (char*)malloc((size_t)service_len + 1u);
  if (!service_name) return;
  memcpy(service_name, p, service_len);
  service_name[service_len] = '\0';     p += service_len;

  type_len = pcl_shm_read_u16_be(p);    p += 2u;
  if (type_len > 0u) {
    request_type = (char*)malloc((size_t)type_len + 1u);
    if (!request_type) {
      free(service_name);
      return;
    }
    memcpy(request_type, p, type_len);
    request_type[type_len] = '\0';
  }
  p += type_len;

  req_len = pcl_shm_read_u32_be(p);     p += 4u;
  if ((uint32_t)(p - (const uint8_t*)msg->data) + req_len > msg->size) {
    free(request_type);
    free(service_name);
    return;
  }

  service_port = pcl_shm_find_remote_service(ctx->executor, service_name, source_id);
  if (!service_port) {
    memset(&empty_response, 0, sizeof(empty_response));
    target = (pcl_shm_response_target_t*)calloc(1, sizeof(*target));
    if (target) {
      target->seq_id = seq_id;
      snprintf(target->requester_id, sizeof(target->requester_id), "%s", source_id);
      pcl_shm_send_response(ctx, target, &empty_response);
      free(target);
    }
    free(request_type);
    free(service_name);
    return;
  }

  svc_ctx = (pcl_svc_context_t*)calloc(1, sizeof(*svc_ctx));
  target = (pcl_shm_response_target_t*)calloc(1, sizeof(*target));
  if (!svc_ctx || !target) {
    free(svc_ctx);
    free(target);
    free(request_type);
    free(service_name);
    return;
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
    free(target);
    free(svc_ctx);
  } else if (rc != PCL_PENDING) {
    memset(&empty_response, 0, sizeof(empty_response));
    pcl_shm_send_response(ctx, target, &empty_response);
    free(target);
    free(svc_ctx);
  }

  free(request_type);
  free(service_name);
}

static pcl_status_t pcl_shm_gateway_on_configure(pcl_container_t* c, void* ud) {
  pcl_port_t* port;
  port = pcl_container_add_subscriber(c,
                                      PCL_SHM_INTERNAL_TOPIC_SVC_REQ,
                                      PCL_SHM_INTERNAL_REQ_TYPE,
                                      pcl_shm_gateway_sub_cb,
                                      ud);
  if (!port) return PCL_ERR_NOMEM;
  return pcl_port_set_route(port, PCL_ROUTE_REMOTE, NULL, 0);
}

static pcl_status_t pcl_shm_handle_frame(pcl_shared_memory_transport_t* ctx,
                                         const pcl_shm_frame_t*         frame) {
  if (!ctx || !frame) return PCL_ERR_INVALID;

  if (frame->kind == PCL_SHM_FRAME_PUBLISH) {
    pcl_msg_t msg = {0};
    msg.data = (frame->data_size > 0u) ? frame->data : NULL;
    msg.size = frame->data_size;
    msg.type_name = frame->type_name[0] ? frame->type_name : NULL;
    return pcl_executor_post_remote_incoming(ctx->executor,
                                             frame->source_id,
                                             frame->name,
                                             &msg);
  }

  if (frame->kind == PCL_SHM_FRAME_SVC_REQ) {
    uint16_t source_len = (uint16_t)strlen(frame->source_id);
    uint16_t service_len = (uint16_t)strlen(frame->name);
    uint16_t type_len = (uint16_t)strlen(frame->type_name);
    uint32_t payload_size = 4u + 2u + source_len + 2u + service_len +
                            2u + type_len + 4u + frame->data_size;
    uint8_t* payload = (uint8_t*)malloc(payload_size);
    size_t off = 0u;
    pcl_msg_t msg = {0};
    pcl_status_t rc;

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
    free(payload);
    return rc;
  }

  if (frame->kind == PCL_SHM_FRAME_SVC_RESP) {
    pcl_resp_cb_fn_t callback = NULL;
    void* user_data = NULL;
    pcl_msg_t response = {0};
    if (pcl_shm_pending_remove(ctx, frame->seq_id, &callback, &user_data) != PCL_OK) {
      return PCL_ERR_NOT_FOUND;
    }
    response.data = (frame->data_size > 0u) ? frame->data : NULL;
    response.size = frame->data_size;
    response.type_name = frame->type_name[0] ? frame->type_name : NULL;
    return pcl_executor_post_response_msg(ctx->executor, callback, user_data, &response);
  }

  return PCL_ERR_INVALID;
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
    return NULL;
  }

  ctx = (pcl_shared_memory_transport_t*)calloc(1, sizeof(*ctx));
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
    pcl_shm_platform_close(ctx, 0);
    pcl_shm_pending_lock_destroy(ctx);
    free(ctx);
    return NULL;
  }

  if (pcl_shm_bus_lock(ctx) != PCL_OK) {
    pcl_shm_platform_close(ctx, 0);
    pcl_shm_pending_lock_destroy(ctx);
    free(ctx);
    return NULL;
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
      free(ctx);
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
    free(ctx);
    return NULL;
  }

  memset(&ctx->transport, 0, sizeof(ctx->transport));
  ctx->transport.publish = pcl_shm_publish;
  ctx->transport.subscribe = pcl_shm_subscribe;
  ctx->transport.invoke_async = pcl_shm_invoke_async;
  ctx->transport.respond = pcl_shm_respond;
  ctx->transport.shutdown = pcl_shm_shutdown;
  ctx->transport.adapter_ctx = ctx;

  memset(&gateway_callbacks, 0, sizeof(gateway_callbacks));
  gateway_callbacks.on_configure = pcl_shm_gateway_on_configure;
  snprintf(gateway_name, sizeof(gateway_name), "__pcl_shm_gateway_%s",
           ctx->participant_id);
  ctx->gateway = pcl_container_create(gateway_name, &gateway_callbacks, ctx);
  if (!ctx->gateway) {
    pcl_shared_memory_transport_destroy(ctx);
    return NULL;
  }

#ifdef _WIN32
  ctx->recv_thread = CreateThread(NULL, 0, pcl_shm_recv_thread_main, ctx, 0, NULL);
  if (!ctx->recv_thread) {
    pcl_shared_memory_transport_destroy(ctx);
    return NULL;
  }
#else
  if (pthread_create(&ctx->recv_thread, NULL, pcl_shm_recv_thread_main, ctx) != 0) {
    pcl_shared_memory_transport_destroy(ctx);
    return NULL;
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

void pcl_shared_memory_transport_destroy(pcl_shared_memory_transport_t* ctx) {
  int unlink_objects = 0;

  if (!ctx) return;

  if (ctx->executor) {
    pcl_executor_set_transport(ctx->executor, NULL);
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

  pcl_shm_pending_clear(ctx);
  pcl_shm_platform_close(ctx, unlink_objects);
  pcl_shm_pending_lock_destroy(ctx);
  free(ctx);
}
