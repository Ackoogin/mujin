/// \file pcl_transport_udp.c
/// \brief UDP datagram transport — pub/sub only, no service RPC.
///
/// Each datagram carries exactly one PUBLISH message.  UDP preserves
/// message boundaries, so no 4-byte length prefix is used.  Packets
/// larger than PCL_UDP_MAX_PAYLOAD are rejected at send time.
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#  define _POSIX_C_SOURCE 200809L
#endif

#include "pcl/pcl_transport_udp.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_alloc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "Ws2_32.lib")
#  define PCL_SOCKET_T SOCKET
#  define PCL_INVALID_SOCKET INVALID_SOCKET
#  define pcl_close_socket closesocket
#else
#  include <sys/socket.h>
#  include <sys/time.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#  include <unistd.h>
#  include <errno.h>
#  include <pthread.h>
#  define PCL_SOCKET_T int
#  define PCL_INVALID_SOCKET (-1)
#  define pcl_close_socket close
#endif

// -- Wire protocol constants ----------------------------------------------

#define PCL_UDP_MSG_PUBLISH 0
/* Conservative single-datagram payload cap.  Below the standard 1500-byte
   Ethernet MTU minus IP/UDP headers so fragmentation is rare on LAN links. */
#define PCL_UDP_MAX_PAYLOAD 1400

// -- Main transport struct ------------------------------------------------

typedef struct pcl_udp_outbound_frame_t {
  uint8_t*                         data;
  uint32_t                         size;
  struct pcl_udp_outbound_frame_t* next;
} pcl_udp_outbound_frame_t;

struct pcl_udp_transport_t {
  pcl_executor_t* executor;
  pcl_transport_t transport;

  PCL_SOCKET_T     sock;
  uint16_t         local_port;
  struct sockaddr_in remote_addr;

  char             peer_id[64];

  volatile int     recv_stop;
  volatile int     recv_running;
  volatile int     send_stop;
  volatile uint64_t received_datagrams;

#ifdef _WIN32
  HANDLE           recv_thread;
  HANDLE           send_thread;
  CRITICAL_SECTION send_lock;
  HANDLE           send_event;
#else
  pthread_t        recv_thread;
  pthread_t        send_thread;
  pthread_mutex_t  send_lock;
  pthread_cond_t   send_cond;
#endif
  int              send_sync_ready;

  pcl_udp_outbound_frame_t* send_head;
  pcl_udp_outbound_frame_t* send_tail;
};

// -- Byte-order helpers ---------------------------------------------------

static uint16_t udp_read_u16_be(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

static void udp_write_u16_be(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

static uint32_t udp_read_u32_be(const uint8_t* p) {
  return (uint32_t)((p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

static void udp_write_u32_be(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)(v & 0xFF);
}

static void udp_free_outbound_frame(pcl_udp_outbound_frame_t* frame) {
  if (!frame) return;
  pcl_free(frame->data);
  pcl_free(frame);
}

static void udp_free_outbound_queue(struct pcl_udp_transport_t* ctx) {
  pcl_udp_outbound_frame_t* frame;
  if (!ctx) return;
  frame = ctx->send_head;
  ctx->send_head = NULL;
  ctx->send_tail = NULL;
  // GCOVR_EXCL_START: teardown reclaim. The send worker drains every queued
  // frame before it observes send_stop and exits, so the queue is already
  // empty by the time destroy calls this; the loop body only runs if a frame
  // is stranded (worker never started / exited early).
  while (frame) {
    pcl_udp_outbound_frame_t* next = frame->next;
    udp_free_outbound_frame(frame);
    frame = next;
  }
  // GCOVR_EXCL_STOP
}

static pcl_status_t udp_enqueue_outbound_frame(struct pcl_udp_transport_t* ctx,
                                               uint8_t*                    data,
                                               uint32_t                    size) {
  pcl_udp_outbound_frame_t* frame;

  if (!ctx || !data || size == 0u) return PCL_ERR_INVALID;
  if (ctx->send_stop) return PCL_ERR_STATE;

  frame = (pcl_udp_outbound_frame_t*)pcl_alloc(sizeof(*frame));
  if (!frame) return PCL_ERR_NOMEM;

  frame->data = data;
  frame->size = size;
  frame->next = NULL;

#ifdef _WIN32
  EnterCriticalSection(&ctx->send_lock);
  if (ctx->send_stop) {
    LeaveCriticalSection(&ctx->send_lock);
    pcl_free(frame);
    return PCL_ERR_STATE;
  }
  if (ctx->send_tail) {
    ctx->send_tail->next = frame;
  } else {
    ctx->send_head = frame;
  }
  ctx->send_tail = frame;
  LeaveCriticalSection(&ctx->send_lock);
  SetEvent(ctx->send_event);
#else
  pthread_mutex_lock(&ctx->send_lock);
  // GCOVR_EXCL_START: stop-race rollback. send_stop was re-checked without the
  // lock above; losing the race so it flips true only after acquiring the lock
  // here is a narrow window not reachable under deterministic testing.
  if (ctx->send_stop) {
    pthread_mutex_unlock(&ctx->send_lock);
    pcl_free(frame);
    return PCL_ERR_STATE;
  }
  // GCOVR_EXCL_STOP
  if (ctx->send_tail) {
    ctx->send_tail->next = frame;
  } else {
    ctx->send_head = frame;
  }
  ctx->send_tail = frame;
  pthread_cond_signal(&ctx->send_cond);
  pthread_mutex_unlock(&ctx->send_lock);
#endif

  return PCL_OK;
}

static void udp_signal_send_stop(struct pcl_udp_transport_t* ctx) {
  if (!ctx || !ctx->send_sync_ready) return;
  ctx->send_stop = 1;
#ifdef _WIN32
  SetEvent(ctx->send_event);
#else
  pthread_mutex_lock(&ctx->send_lock);
  pthread_cond_signal(&ctx->send_cond);
  pthread_mutex_unlock(&ctx->send_lock);
#endif
}

static void udp_destroy_send_sync(struct pcl_udp_transport_t* ctx) {
  if (!ctx || !ctx->send_sync_ready) return;
#ifdef _WIN32
  if (ctx->send_event) {
    CloseHandle(ctx->send_event);
    ctx->send_event = NULL;
  }
  DeleteCriticalSection(&ctx->send_lock);
#else
  pthread_cond_destroy(&ctx->send_cond);
  pthread_mutex_destroy(&ctx->send_lock);
#endif
  ctx->send_sync_ready = 0;
}

// -- Send thread -----------------------------------------------------------

#ifdef _WIN32
static DWORD WINAPI udp_send_thread_main(LPVOID arg)
#else
static void* udp_send_thread_main(void* arg)
#endif
{
  struct pcl_udp_transport_t* ctx = (struct pcl_udp_transport_t*)arg;

  for (;;) {
    pcl_udp_outbound_frame_t* frame = NULL;

#ifdef _WIN32
    for (;;) {
      EnterCriticalSection(&ctx->send_lock);
      frame = ctx->send_head;
      if (frame) {
        ctx->send_head = frame->next;
        if (!ctx->send_head) ctx->send_tail = NULL;
      }
      LeaveCriticalSection(&ctx->send_lock);

      if (frame || ctx->send_stop) break;
      WaitForSingleObject(ctx->send_event, INFINITE);
    }
#else
    pthread_mutex_lock(&ctx->send_lock);
    while (!ctx->send_stop && ctx->send_head == NULL) {
      pthread_cond_wait(&ctx->send_cond, &ctx->send_lock);
    }
    frame = ctx->send_head;
    if (frame) {
      ctx->send_head = frame->next;
      if (!ctx->send_head) ctx->send_tail = NULL;
    }
    pthread_mutex_unlock(&ctx->send_lock);
#endif

    if (!frame) {
      if (ctx->send_stop) break;
      continue;  // GCOVR_EXCL_LINE: guards a spurious condvar wakeup with no queued frame and no stop request.
    }

    if (ctx->sock != PCL_INVALID_SOCKET) {
#ifdef _WIN32
      (void)sendto(ctx->sock, (const char*)frame->data, (int)frame->size, 0,
                   (const struct sockaddr*)&ctx->remote_addr,
                   (int)sizeof(ctx->remote_addr));
#else
      (void)sendto(ctx->sock, frame->data, frame->size, 0,
                   (const struct sockaddr*)&ctx->remote_addr,
                   (socklen_t)sizeof(ctx->remote_addr));
#endif
    }

    udp_free_outbound_frame(frame);
  }

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

// -- Transport vtable: publish (called on PCL main thread) ----------------

static pcl_status_t udp_publish(void*            adapter_ctx,
                                const char*      topic,
                                const pcl_msg_t* msg) {
  struct pcl_udp_transport_t* ctx = (struct pcl_udp_transport_t*)adapter_ctx;
  uint16_t topic_len, type_len;
  uint32_t data_len, payload_size;
  uint8_t* buf;
  size_t   off;
  pcl_status_t rc;

  if (!ctx || ctx->sock == PCL_INVALID_SOCKET || !topic || !msg) return PCL_ERR_INVALID;
  /* A nonzero size with a null data pointer would reserve data_len bytes in the
     packet but skip the copy below, sending uninitialized heap bytes. Reject
     that shape (other ingress paths do the same). */
  if (msg->size && !msg->data) return PCL_ERR_INVALID;

  topic_len    = (uint16_t)strlen(topic);
  type_len     = (uint16_t)(msg->type_name ? strlen(msg->type_name) : 0);
  data_len     = msg->size;
  payload_size = 1u + 2u + topic_len + 2u + type_len + 4u + data_len;

  if (payload_size > PCL_UDP_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  buf = (uint8_t*)pcl_alloc(payload_size);
  if (!buf) return PCL_ERR_NOMEM;

  off = 0;
  buf[off++] = PCL_UDP_MSG_PUBLISH;
  udp_write_u16_be(buf + off, topic_len);               off += 2;
  memcpy(buf + off, topic, topic_len);                  off += topic_len;
  udp_write_u16_be(buf + off, type_len);                off += 2;
  if (type_len && msg->type_name) {
    memcpy(buf + off, msg->type_name, type_len);
  }
  off += type_len;
  udp_write_u32_be(buf + off, data_len);                off += 4;
  if (data_len && msg->data) {
    memcpy(buf + off, msg->data, data_len);
  }

  rc = udp_enqueue_outbound_frame(ctx, buf, payload_size);
  if (rc != PCL_OK) {
    pcl_free(buf);
  }
  return rc;
}

// -- Transport vtable: subscribe / shutdown -------------------------------

static pcl_status_t udp_subscribe(void*       adapter_ctx,
                                  const char* topic,
                                  const char* type_name) {
  (void)adapter_ctx; (void)topic; (void)type_name;
  return PCL_OK;
}

static void udp_shutdown(void* adapter_ctx) {
  struct pcl_udp_transport_t* ctx = (struct pcl_udp_transport_t*)adapter_ctx;
  if (!ctx) return;
  ctx->recv_stop = 1;
  udp_signal_send_stop(ctx);
}

// -- Receive thread -------------------------------------------------------

#ifdef _WIN32
static DWORD WINAPI udp_recv_thread_main(LPVOID arg)
#else
static void* udp_recv_thread_main(void* arg)
#endif
{
  struct pcl_udp_transport_t* ctx = (struct pcl_udp_transport_t*)arg;
  uint8_t buf[PCL_UDP_MAX_PAYLOAD];

  ctx->recv_running = 1;

  while (!ctx->recv_stop) {
#ifdef _WIN32
    int n = recvfrom(ctx->sock, (char*)buf, (int)sizeof(buf), 0, NULL, NULL);
#else
    ssize_t n = recvfrom(ctx->sock, buf, sizeof(buf), 0, NULL, NULL);
#endif
    if (n <= 0) {
      if (ctx->recv_stop) break;
      continue;
    }

    ++ctx->received_datagrams;

    if ((size_t)n < 1u + 2u + 2u + 4u) continue;
    if (buf[0] != PCL_UDP_MSG_PUBLISH) continue;

    {
      uint16_t topic_len = udp_read_u16_be(buf + 1);
      uint16_t type_len;
      uint32_t data_len;
      char*    topic_s;
      char*    type_s = NULL;
      size_t   payload_len = (size_t)n;
      pcl_msg_t msg;

      if (1u + 2u + topic_len + 2u + 4u > payload_len) continue;

      topic_s = (char*)pcl_alloc((size_t)topic_len + 1u);
      if (!topic_s) continue;
      memcpy(topic_s, buf + 3, topic_len);
      topic_s[topic_len] = '\0';

      type_len = udp_read_u16_be(buf + 3 + topic_len);
      if (1u + 2u + topic_len + 2u + type_len + 4u > payload_len) {
        pcl_free(topic_s);
        continue;
      }

      if (type_len > 0u) {
        type_s = (char*)pcl_alloc((size_t)type_len + 1u);
        if (!type_s) { pcl_free(topic_s); continue; }
        memcpy(type_s, buf + 5 + topic_len, type_len);
        type_s[type_len] = '\0';
      }

      data_len = udp_read_u32_be(buf + 5 + topic_len + type_len);
      if (1u + 2u + topic_len + 2u + type_len + 4u + data_len > payload_len) {
        pcl_free(topic_s); pcl_free(type_s);
        continue;
      }

      memset(&msg, 0, sizeof(msg));
      msg.data      = (data_len > 0u) ? (buf + 9 + topic_len + type_len) : NULL;
      msg.size      = data_len;
      msg.type_name = type_s;

      pcl_executor_post_remote_incoming(ctx->executor, ctx->peer_id, topic_s, &msg);

      pcl_free(topic_s);
      pcl_free(type_s);
    }
  }

  ctx->recv_running = 0;

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

// -- Remote address resolution -------------------------------------------

static int udp_resolve_remote(const char*        host,
                              uint16_t           port,
                              struct sockaddr_in* out) {
  struct addrinfo  hints;
  struct addrinfo* result = NULL;
  char             port_str[8];
  int              rc;

  if (!host || !out) return -1;

  snprintf(port_str, sizeof(port_str), "%u", (unsigned)port);
  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_INET;   /* UDP transport is v4-only for now */
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;

  rc = getaddrinfo(host, port_str, &hints, &result);
  if (rc != 0 || !result) return -1;

  memcpy(out, result->ai_addr, sizeof(*out));
  freeaddrinfo(result);
  return 0;
}

// -- Public API -----------------------------------------------------------

pcl_udp_transport_t* pcl_udp_transport_create(uint16_t        local_port,
                                              const char*     remote_host,
                                              uint16_t        remote_port,
                                              pcl_executor_t* executor) {
  struct pcl_udp_transport_t* ctx;
  struct sockaddr_in          bind_addr;
  int                         opt = 1;

  if (!remote_host || !executor) return NULL;

#ifdef _WIN32
  { WSADATA wsa; if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return NULL; }
#endif

  ctx = (struct pcl_udp_transport_t*)pcl_calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;

  ctx->executor   = executor;
  ctx->sock       = PCL_INVALID_SOCKET;
  ctx->local_port = local_port;
  snprintf(ctx->peer_id, sizeof(ctx->peer_id), "%s", "default");

  ctx->transport.publish      = udp_publish;
  ctx->transport.subscribe    = udp_subscribe;
  ctx->transport.shutdown     = udp_shutdown;
  ctx->transport.adapter_ctx  = ctx;

#ifdef _WIN32
  InitializeCriticalSection(&ctx->send_lock);
  ctx->send_event = CreateEvent(NULL, FALSE, FALSE, NULL);
  if (!ctx->send_event) {
    DeleteCriticalSection(&ctx->send_lock);
    pcl_free(ctx);
    return NULL;
  }
  ctx->send_sync_ready = 1;
#else
  // GCOVR_EXCL_START: pthread sync-primitive initialisation fails only on
  // kernel resource exhaustion, which cannot be provoked under normal testing.
  if (pthread_mutex_init(&ctx->send_lock, NULL) != 0) {
    pcl_free(ctx);
    return NULL;
  }
  if (pthread_cond_init(&ctx->send_cond, NULL) != 0) {
    pthread_mutex_destroy(&ctx->send_lock);
    pcl_free(ctx);
    return NULL;
  }
  // GCOVR_EXCL_STOP
  ctx->send_sync_ready = 1;
#endif

  if (udp_resolve_remote(remote_host, remote_port, &ctx->remote_addr) != 0) {
    udp_destroy_send_sync(ctx);
    pcl_free(ctx);
    return NULL;
  }

  ctx->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (ctx->sock == PCL_INVALID_SOCKET) {
    // GCOVR_EXCL_START: socket() fails only on fd/kernel exhaustion.
    udp_destroy_send_sync(ctx);
    pcl_free(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }

  setsockopt(ctx->sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

  memset(&bind_addr, 0, sizeof(bind_addr));
  bind_addr.sin_family      = AF_INET;
  bind_addr.sin_addr.s_addr = INADDR_ANY;
  bind_addr.sin_port        = htons(local_port);

  if (bind(ctx->sock, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) != 0) {
    // GCOVR_EXCL_START: with SO_REUSEADDR set, binding fails only on
    // platform-specific port conflicts that cannot be forced portably.
    pcl_close_socket(ctx->sock);
    udp_destroy_send_sync(ctx);
    pcl_free(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }

  if (local_port == 0) {
#ifdef _WIN32
    int len = (int)sizeof(bind_addr);
#else
    socklen_t len = sizeof(bind_addr);
#endif
    if (getsockname(ctx->sock, (struct sockaddr*)&bind_addr, &len) == 0)
      ctx->local_port = ntohs(bind_addr.sin_port);
  }

  /* Set 200 ms recv timeout so the thread loop can poll recv_stop without
     shutdown() (UDP has no connection to shut down). */
#ifdef _WIN32
  {
    DWORD tmo_ms = 200;
    setsockopt(ctx->sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tmo_ms, sizeof(tmo_ms));
  }
#else
  {
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 200 * 1000;
    setsockopt(ctx->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }
#endif

#ifdef _WIN32
  ctx->send_thread = CreateThread(NULL, 0, udp_send_thread_main, ctx, 0, NULL);
  if (!ctx->send_thread) {
    pcl_close_socket(ctx->sock);
    udp_destroy_send_sync(ctx);
    pcl_free(ctx);
    return NULL;
  }
  ctx->recv_thread = CreateThread(NULL, 0, udp_recv_thread_main, ctx, 0, NULL);
  if (!ctx->recv_thread) {
    udp_signal_send_stop(ctx);
    WaitForSingleObject(ctx->send_thread, 5000);
    CloseHandle(ctx->send_thread);
    pcl_close_socket(ctx->sock);
    udp_destroy_send_sync(ctx);
    pcl_free(ctx);
    return NULL;
  }
#else
  if (pthread_create(&ctx->send_thread, NULL, udp_send_thread_main, ctx) != 0) {
    // GCOVR_EXCL_START: thread creation fails only on kernel resource
    // exhaustion, not injectable in normal testing.
    pcl_close_socket(ctx->sock);
    udp_destroy_send_sync(ctx);
    pcl_free(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }
  if (pthread_create(&ctx->recv_thread, NULL, udp_recv_thread_main, ctx) != 0) {
    // GCOVR_EXCL_START: thread creation fails only on kernel resource
    // exhaustion, not injectable in normal testing.
    udp_signal_send_stop(ctx);
    pthread_join(ctx->send_thread, NULL);
    pcl_close_socket(ctx->sock);
    udp_destroy_send_sync(ctx);
    pcl_free(ctx);
    return NULL;
    // GCOVR_EXCL_STOP
  }
#endif

  return (pcl_udp_transport_t*)ctx;
}

uint16_t pcl_udp_transport_get_local_port(const pcl_udp_transport_t* ctx_opaque) {
  const struct pcl_udp_transport_t* ctx = (const struct pcl_udp_transport_t*)ctx_opaque;
  if (!ctx) return 0;
  return ctx->local_port;
}

uint64_t pcl_udp_transport_received_datagrams(const pcl_udp_transport_t* ctx_opaque) {
  const struct pcl_udp_transport_t* ctx =
      (const struct pcl_udp_transport_t*)ctx_opaque;
  if (!ctx) return 0u;
  return ctx->received_datagrams;
}

pcl_status_t pcl_udp_transport_set_peer_id(pcl_udp_transport_t* ctx_opaque,
                                           const char*          peer_id) {
  struct pcl_udp_transport_t* ctx = (struct pcl_udp_transport_t*)ctx_opaque;
  if (!ctx || !peer_id || !peer_id[0]) return PCL_ERR_INVALID;
  snprintf(ctx->peer_id, sizeof(ctx->peer_id), "%s", peer_id);
  return PCL_OK;
}

const pcl_transport_t* pcl_udp_transport_get_transport(pcl_udp_transport_t* ctx_opaque) {
  struct pcl_udp_transport_t* ctx = (struct pcl_udp_transport_t*)ctx_opaque;
  if (!ctx) return NULL;
  return &ctx->transport;
}

void pcl_udp_transport_destroy(pcl_udp_transport_t* ctx_opaque) {
  struct pcl_udp_transport_t* ctx = (struct pcl_udp_transport_t*)ctx_opaque;
  if (!ctx) return;

  /* Unregister from executor before freeing. Only clear the default transport
     if THIS instance is the active default -- a manifest/plugin teardown must
     not wipe a default another owner installed. */
  if (ctx->executor) {
    const pcl_transport_t* def = pcl_executor_get_transport(ctx->executor);
    if (def && def->adapter_ctx == ctx) {
      pcl_executor_set_transport(ctx->executor, NULL);
    }
    pcl_executor_register_transport(ctx->executor, ctx->peer_id, NULL);
  }

  ctx->recv_stop = 1;
  udp_signal_send_stop(ctx);

#ifdef _WIN32
  if (ctx->send_thread) {
    WaitForSingleObject(ctx->send_thread, 5000);
    CloseHandle(ctx->send_thread);
    ctx->send_thread = NULL;
  }
#else
  if (ctx->send_thread) {
    pthread_join(ctx->send_thread, NULL);
    ctx->send_thread = 0;
  }
#endif

  if (ctx->sock != PCL_INVALID_SOCKET) {
    pcl_close_socket(ctx->sock);
    ctx->sock = PCL_INVALID_SOCKET;
  }

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

  udp_free_outbound_queue(ctx);
  udp_destroy_send_sync(ctx);
  pcl_free(ctx);
}
