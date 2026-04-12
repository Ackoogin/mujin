/// \file pcl_transport_socket.c
/// \brief TCP socket transport — fully non-blocking PCL main thread.
///
/// All socket writes are serialised through a dedicated send_thread that
/// drains a mutex-protected FIFO queue.  recv_thread posts inbound
/// PUBLISH messages via pcl_executor_post_incoming and inbound SVC_RESP
/// frames via pcl_executor_post_response_cb — both non-blocking enqueues.
/// The PCL executor thread never calls send() or blocks on I/O.
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#  define _POSIX_C_SOURCE 200809L
#endif

#include "pcl/pcl_transport_socket.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "Ws2_32.lib")
#  define SHUT_RDWR SD_BOTH
#  define PCL_SOCKET_T SOCKET
#  define PCL_INVALID_SOCKET INVALID_SOCKET
#  define PCL_SOCKET_ERROR SOCKET_ERROR
#  define pcl_close_socket closesocket
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#  include <unistd.h>
#  include <errno.h>
#  include <pthread.h>
#  define PCL_SOCKET_T int
#  define PCL_INVALID_SOCKET (-1)
#  define PCL_SOCKET_ERROR (-1)
#  define pcl_close_socket close
#endif

// -- Wire protocol constants ----------------------------------------------

#define PCL_SOCKET_MSG_PUBLISH  0
#define PCL_SOCKET_MSG_SVC_REQ  1
#define PCL_SOCKET_MSG_SVC_RESP 2
#define PCL_SOCKET_TOPIC_SVC_REQ "__pcl_svc_req"
#define PCL_SOCKET_MAX_PAYLOAD  65536

// -- Helper FIFO node types -----------------------------------------------

typedef struct pcl_outbound_frame_t {
  uint8_t*                     data;
  uint32_t                     size;
  struct pcl_outbound_frame_t* next;
} pcl_outbound_frame_t;

typedef struct pcl_svc_pending_t {
  uint32_t                  seq_id;
  pcl_resp_cb_fn_t          cb;
  void*                     user_data;
  struct pcl_svc_pending_t* next;
} pcl_svc_pending_t;

// -- Main transport struct ------------------------------------------------

struct pcl_socket_transport_t {
  int               is_server;
  uint16_t          port;
  char              host[256];
  char              peer_id[64];
  pcl_executor_t*   executor;
  pcl_transport_t   transport;

  PCL_SOCKET_T      listen_sock;
  PCL_SOCKET_T      client_sock;

  volatile int      recv_running;
  volatile int      recv_stop;

#ifdef _WIN32
  HANDLE             recv_thread;
  HANDLE             send_thread;
  CRITICAL_SECTION   send_lock;    /* guards send_head/send_tail + send_cond */
  CONDITION_VARIABLE send_cond;
  CRITICAL_SECTION   pending_lock;
#else
  pthread_t          recv_thread;
  pthread_t          send_thread;
  pthread_mutex_t    send_lock;
  pthread_cond_t     send_cond;
  pthread_mutex_t    pending_lock;
#endif

  volatile int          send_stop;
  pcl_outbound_frame_t* send_head;
  pcl_outbound_frame_t* send_tail;

  volatile uint32_t     next_seq_id; /* 1-based, never wraps to 0 */
  pcl_svc_pending_t*    pending_head;

  pcl_container_t*  gateway;
};

// -- Byte-order helpers ---------------------------------------------------

static uint16_t read_u16_be(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

static void write_u16_be(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

static uint32_t read_u32_be(const uint8_t* p) {
  return (uint32_t)((p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

static void write_u32_be(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)(v & 0xFF);
}

// -- Socket I/O helpers ---------------------------------------------------

static int recv_all(PCL_SOCKET_T sock, void* buf, size_t len) {
  size_t got = 0;
  while (got < len) {
#ifdef _WIN32
    int r = recv(sock, (char*)buf + got, (int)(len - got), 0);
#else
    ssize_t r = recv(sock, (char*)buf + got, len - got, 0);
#endif
    if (r <= 0) return -1;
    got += (size_t)r;
  }
  return 0;
}

static int send_all(PCL_SOCKET_T sock, const void* buf, size_t len) {
  size_t sent = 0;
  while (sent < len) {
#ifdef _WIN32
    int r = send(sock, (const char*)buf + sent, (int)(len - sent), 0);
#else
    ssize_t r = send(sock, (const char*)buf + sent, len - sent, 0);
#endif
    if (r <= 0) return -1;
    sent += (size_t)r;
  }
  return 0;
}

// -- Outbound FIFO queue --------------------------------------------------
// Called from ANY thread (PCL main, recv_thread, application).
// The send_thread is the ONLY thread that calls send_all().

static pcl_status_t enqueue_outbound_frame(struct pcl_socket_transport_t* ctx,
                                           const uint8_t*                 data,
                                           uint32_t                       size) {
  pcl_outbound_frame_t* f;

  f = (pcl_outbound_frame_t*)malloc(sizeof(*f));
  if (!f) return PCL_ERR_NOMEM;

  f->data = (uint8_t*)malloc(size);
  if (!f->data) {
    free(f);
    return PCL_ERR_NOMEM;
  }
  memcpy(f->data, data, size);
  f->size = size;
  f->next = NULL;

#ifdef _WIN32
  EnterCriticalSection(&ctx->send_lock);
  if (ctx->send_tail) { ctx->send_tail->next = f; } else { ctx->send_head = f; }
  ctx->send_tail = f;
  WakeConditionVariable(&ctx->send_cond);
  LeaveCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_lock(&ctx->send_lock);
  if (ctx->send_tail) { ctx->send_tail->next = f; } else { ctx->send_head = f; }
  ctx->send_tail = f;
  pthread_cond_signal(&ctx->send_cond);
  pthread_mutex_unlock(&ctx->send_lock);
#endif

  return PCL_OK;
}

// -- Send thread ----------------------------------------------------------
// Sole writer to client_sock.

#ifdef _WIN32
static DWORD WINAPI send_thread_main(LPVOID arg)
#else
static void* send_thread_main(void* arg)
#endif
{
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)arg;

  for (;;) {
    pcl_outbound_frame_t* f;

#ifdef _WIN32
    EnterCriticalSection(&ctx->send_lock);
    while (ctx->send_head == NULL && !ctx->send_stop) {
      SleepConditionVariableCS(&ctx->send_cond, &ctx->send_lock, INFINITE);
    }
    f = ctx->send_head;
    if (f) {
      ctx->send_head = f->next;
      if (!ctx->send_head) ctx->send_tail = NULL;
    }
    LeaveCriticalSection(&ctx->send_lock);
#else
    pthread_mutex_lock(&ctx->send_lock);
    while (ctx->send_head == NULL && !ctx->send_stop) {
      pthread_cond_wait(&ctx->send_cond, &ctx->send_lock);
    }
    f = ctx->send_head;
    if (f) {
      ctx->send_head = f->next;
      if (!ctx->send_head) ctx->send_tail = NULL;
    }
    pthread_mutex_unlock(&ctx->send_lock);
#endif

    if (!f) break; /* send_stop set and queue drained */

    if (ctx->client_sock != PCL_INVALID_SOCKET) {
      send_all(ctx->client_sock, f->data, f->size);
    }
    free(f->data);
    free(f);
  }

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

// -- Transport vtable: publish (called on PCL main thread) ----------------

static pcl_status_t socket_publish(void*            adapter_ctx,
                                   const char*      topic,
                                   const pcl_msg_t* msg) {
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)adapter_ctx;
  uint16_t topic_len, type_len;
  uint32_t data_len, payload_size;
  uint8_t* frame;
  size_t   off;
  pcl_status_t rc;

  if (!ctx || ctx->client_sock == PCL_INVALID_SOCKET) return PCL_ERR_INVALID;

  topic_len    = (uint16_t)strlen(topic);
  type_len     = (uint16_t)(msg->type_name ? strlen(msg->type_name) : 0);
  data_len     = msg->size;
  payload_size = 1u + 2u + topic_len + 2u + type_len + 4u + data_len;

  if (payload_size > PCL_SOCKET_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  /* frame: [4:payload_size][1:type=PUBLISH][2:topic_len][topic]
            [2:type_len][type_name][4:data_len][data] */
  frame = (uint8_t*)malloc(4u + payload_size);
  if (!frame) return PCL_ERR_NOMEM;

  off = 0;
  write_u32_be(frame + off, payload_size);              off += 4;
  frame[off++] = PCL_SOCKET_MSG_PUBLISH;
  write_u16_be(frame + off, topic_len);                 off += 2;
  memcpy(frame + off, topic, topic_len);                off += topic_len;
  write_u16_be(frame + off, type_len);                  off += 2;
  if (type_len && msg->type_name) {
    memcpy(frame + off, msg->type_name, type_len);
  }
  off += type_len;
  write_u32_be(frame + off, data_len);                  off += 4;
  if (data_len && msg->data) {
    memcpy(frame + off, msg->data, data_len);
  }

  rc = enqueue_outbound_frame(ctx, frame, (uint32_t)(4u + payload_size));
  free(frame);
  return rc;
}

static pcl_status_t socket_invoke_async(void*            adapter_ctx,
                                        const char*      service_name,
                                        const pcl_msg_t* request,
                                        pcl_resp_cb_fn_t callback,
                                        void*            user_data) {
  return pcl_socket_transport_invoke_remote_async(
      (pcl_socket_transport_t*)adapter_ctx,
      service_name,
      request,
      callback,
      user_data);
}

// -- Gateway subscriber callback (PCL main thread) ------------------------
// Invoked when a SVC_REQ arrives from the wire.  Calls the service handler
// synchronously (fine — runs on executor thread), then enqueues the response
// to send_thread (non-blocking).

static void gateway_sub_cb(pcl_container_t* c,
                           const pcl_msg_t* msg,
                           void*            user_data) {
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)user_data;
  const uint8_t* p;
  uint32_t seq_id, svc_len, req_len;
  uint16_t req_type_len;
  char     svc_name[256];
  char     req_type_name[256];
  pcl_msg_t req, resp;
  uint8_t   resp_buf[PCL_SOCKET_MAX_PAYLOAD];
  uint8_t*  frame;
  uint16_t  resp_type_len;
  uint32_t  payload_size;
  size_t    off;

  (void)c;

  if (!ctx || !msg || msg->size < 10u) return;

  p = (const uint8_t*)msg->data;
  seq_id  = read_u32_be(p);                    p += 4;
  svc_len = (uint32_t)read_u16_be(p);          p += 2;
  if (svc_len >= sizeof(svc_name)) return;
  memcpy(svc_name, p, svc_len);
  svc_name[svc_len] = '\0';                    p += svc_len;
  req_type_len = read_u16_be(p);               p += 2;
  if (req_type_len >= sizeof(req_type_name)) return;
  if (req_type_len > 0u) {
    memcpy(req_type_name, p, req_type_len);
  }
  req_type_name[req_type_len] = '\0';          p += req_type_len;
  req_len = read_u32_be(p);                    p += 4;

  memset(&req, 0, sizeof(req));
  req.data      = (req_len > 0u) ? (const void*)p : NULL;
  req.size      = req_len;
  req.type_name = (req_type_len > 0u) ? req_type_name : NULL;

  memset(&resp, 0, sizeof(resp));
  resp.data      = resp_buf;
  resp.size      = (uint32_t)sizeof(resp_buf);
  resp.type_name = NULL;

  if (pcl_executor_invoke_service_remote(ctx->executor,
                                         ctx->peer_id,
                                         svc_name,
                                         &req,
                                         &resp) != PCL_OK) {
    resp.size = 0;
  }

  resp_type_len = (uint16_t)(resp.type_name ? strlen(resp.type_name) : 0u);

  /* frame: [4:payload_size][1:type=SVC_RESP][4:seq_id]
            [2:type_len][type_name][4:resp_size][resp_data] */
  payload_size = 1u + 4u + 2u + resp_type_len + 4u + resp.size;
  frame = (uint8_t*)malloc(4u + payload_size);
  if (!frame) return;

  off = 0;
  write_u32_be(frame + off, payload_size);      off += 4;
  frame[off++] = PCL_SOCKET_MSG_SVC_RESP;
  write_u32_be(frame + off, seq_id);            off += 4;
  write_u16_be(frame + off, resp_type_len);     off += 2;
  if (resp_type_len > 0u && resp.type_name) {
    memcpy(frame + off, resp.type_name, resp_type_len);
  }
  off += resp_type_len;
  write_u32_be(frame + off, resp.size);         off += 4;
  if (resp.size > 0u && resp.data) {
    memcpy(frame + off, resp.data, resp.size);
  }

  enqueue_outbound_frame(ctx, frame, 4u + payload_size);
  free(frame);
}

// -- Transport vtable: subscribe / shutdown -------------------------------

static pcl_status_t socket_subscribe(void*       adapter_ctx,
                                     const char* topic,
                                     const char* type_name) {
  (void)adapter_ctx; (void)topic; (void)type_name;
  return PCL_OK;
}

static void socket_shutdown(void* adapter_ctx) {
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)adapter_ctx;
  if (!ctx) return;
  ctx->recv_stop = 1;
}

// -- Receive thread -------------------------------------------------------
// recv_thread only calls pcl_executor_post_incoming and
// pcl_executor_post_response_cb — both are non-blocking enqueues.

#ifdef _WIN32
static DWORD WINAPI recv_thread_main(LPVOID arg)
#else
static void* recv_thread_main(void* arg)
#endif
{
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)arg;
  uint8_t  len_buf[4];
  uint32_t payload_len;
  uint8_t* payload;

  ctx->recv_running = 1;

  while (!ctx->recv_stop && ctx->client_sock != PCL_INVALID_SOCKET) {
    if (recv_all(ctx->client_sock, len_buf, 4) != 0) break;
    payload_len = read_u32_be(len_buf);
    if (payload_len == 0u || payload_len > PCL_SOCKET_MAX_PAYLOAD) break;

    payload = (uint8_t*)malloc(payload_len);
    if (!payload) break;

    if (recv_all(ctx->client_sock, payload, payload_len) != 0) {
      free(payload);
      break;
    }

    if (payload[0] == PCL_SOCKET_MSG_PUBLISH) {
      /* [0x00][2:topic_len][topic][2:type_len][type_name][4:data_len][data] */
      uint16_t topic_len, type_len;
      uint32_t data_len;
      char*    topic_s;
      char*    type_s;
      pcl_msg_t msg;

      if (payload_len < 1u + 2u + 2u + 4u) { free(payload); continue; }

      topic_len = read_u16_be(payload + 1);
      if (1u + 2u + topic_len + 2u + 4u > payload_len) { free(payload); continue; }

      topic_s = (char*)malloc(topic_len + 1u);
      if (!topic_s) { free(payload); continue; }
      memcpy(topic_s, payload + 3, topic_len);
      topic_s[topic_len] = '\0';

      type_len = read_u16_be(payload + 3 + topic_len);
      if (1u + 2u + topic_len + 2u + type_len + 4u > payload_len) {
        free(topic_s); free(payload); continue;
      }

      type_s = (char*)malloc(type_len + 1u);
      if (!type_s) { free(topic_s); free(payload); continue; }
      memcpy(type_s, payload + 5 + topic_len, type_len);
      type_s[type_len] = '\0';

      data_len = read_u32_be(payload + 5 + topic_len + type_len);

      memset(&msg, 0, sizeof(msg));
      msg.data      = (data_len > 0u) ? (payload + 9 + topic_len + type_len) : NULL;
      msg.size      = data_len;
      msg.type_name = type_s;

      pcl_executor_post_remote_incoming(ctx->executor, ctx->peer_id, topic_s, &msg);

      free(topic_s);
      free(type_s);
      free(payload);

    } else if (payload[0] == PCL_SOCKET_MSG_SVC_REQ && ctx->is_server) {
      /* [0x01][4:seq_id][2:svc_len][svc_name][2:type_len][type_name][4:req_len][req_data]
         Strip the type byte; forward
         [seq_id][svc_len][svc_name][type_len][type_name][req_len][req_data]
         as the message body for the gateway's __pcl_svc_req subscriber. */
      if (payload_len >= 1u + 4u + 2u + 2u + 4u) {
        uint16_t  svc_len  = read_u16_be(payload + 5);
        uint16_t  type_len = 0u;
        uint32_t  fwd_len = payload_len - 1u;
        uint8_t*  fwd     = (uint8_t*)malloc(fwd_len);
        char*     type_s   = NULL;

        if (payload_len >= 1u + 4u + 2u + svc_len + 2u) {
          type_len = read_u16_be(payload + 7u + svc_len);
          if (payload_len >= 1u + 4u + 2u + svc_len + 2u + type_len + 4u &&
              type_len > 0u) {
            type_s = (char*)malloc((size_t)type_len + 1u);
            if (type_s) {
              memcpy(type_s, payload + 9u + svc_len, type_len);
              type_s[type_len] = '\0';
            }
          }
        }

        if (fwd) {
          pcl_msg_t svc_msg;
          uint32_t seq_id = read_u32_be(payload + 1u);
          memcpy(fwd, payload + 1, fwd_len);
          memset(&svc_msg, 0, sizeof(svc_msg));
          svc_msg.data      = fwd;
          svc_msg.size      = fwd_len;
          svc_msg.type_name = type_s ? type_s : "pcl_socket_service_request";
          pcl_executor_post_incoming(ctx->executor, PCL_SOCKET_TOPIC_SVC_REQ, &svc_msg);
          free(fwd);
        }
        free(type_s);
      }
      free(payload);

    } else if (payload[0] == PCL_SOCKET_MSG_SVC_RESP && !ctx->is_server) {
      /* [0x02][4:seq_id][2:type_len][type_name][4:resp_len][resp_data]
         Match seq_id to a pending async call and deliver via post_response_cb. */
      if (payload_len >= 1u + 4u + 2u + 4u) {
        uint32_t           seq_id    = read_u32_be(payload + 1);
        uint16_t           type_len  = read_u16_be(payload + 5);
        const char*        type_data = (const char*)(payload + 7);
        uint32_t           resp_size;
        const void*        resp_data;
        char*              type_s    = NULL;
        pcl_svc_pending_t* pending   = NULL;
        pcl_svc_pending_t** pp;

        if (payload_len < 1u + 4u + 2u + type_len + 4u) {
          free(payload);
          continue;
        }
        resp_size = read_u32_be(payload + 7u + type_len);
        resp_data = (resp_size > 0u &&
                     payload_len >= 11u + type_len + resp_size)
                        ? (const void*)(payload + 11u + type_len)
                        : NULL;
        if (type_len > 0u) {
          type_s = (char*)malloc((size_t)type_len + 1u);
          if (!type_s) {
            free(payload);
            continue;
          }
          memcpy(type_s, type_data, type_len);
          type_s[type_len] = '\0';
        }

#ifdef _WIN32
        EnterCriticalSection(&ctx->pending_lock);
#else
        pthread_mutex_lock(&ctx->pending_lock);
#endif
        for (pp = &ctx->pending_head; *pp; pp = &(*pp)->next) {
          if ((*pp)->seq_id == seq_id) {
            pending  = *pp;
            *pp      = pending->next;
            break;
          }
        }
#ifdef _WIN32
        LeaveCriticalSection(&ctx->pending_lock);
#else
        pthread_mutex_unlock(&ctx->pending_lock);
#endif

        if (pending) {
          pcl_msg_t resp_msg;
          memset(&resp_msg, 0, sizeof(resp_msg));
          resp_msg.data = resp_data;
          resp_msg.size = resp_size;
          resp_msg.type_name = type_s;
          pcl_executor_post_response_msg(ctx->executor,
                                         pending->cb,
                                         pending->user_data,
                                         &resp_msg);
          free(pending);
        }
        free(type_s);
      }
      free(payload);

    } else {
      free(payload);
    }
  }

  ctx->recv_running = 0;

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

// -- Gateway container setup ----------------------------------------------

static pcl_status_t gateway_on_configure(pcl_container_t* c, void* ud) {
  struct pcl_socket_transport_t* t = (struct pcl_socket_transport_t*)ud;
  if (!pcl_container_add_subscriber(c,
        PCL_SOCKET_TOPIC_SVC_REQ,
        "SubscribeInterest_Request",
        gateway_sub_cb, t)) {
    return PCL_ERR_NOMEM;
  }
  return PCL_OK;
}

// -- Common initialisation ------------------------------------------------

static void socket_transport_create_common(struct pcl_socket_transport_t* ctx,
                                           pcl_executor_t*               executor) {
  memset(&ctx->transport, 0, sizeof(ctx->transport));
  ctx->transport.publish     = socket_publish;
  ctx->transport.subscribe   = socket_subscribe;
  ctx->transport.invoke_async = socket_invoke_async;
  ctx->transport.shutdown    = socket_shutdown;
  ctx->transport.adapter_ctx = ctx;

  ctx->listen_sock  = PCL_INVALID_SOCKET;
  ctx->client_sock  = PCL_INVALID_SOCKET;
  ctx->recv_running = 0;
  ctx->recv_stop    = 0;
  ctx->send_stop    = 0;
  ctx->send_head    = NULL;
  ctx->send_tail    = NULL;
  ctx->next_seq_id  = 1;
  ctx->pending_head = NULL;
  ctx->executor     = executor;
  snprintf(ctx->peer_id, sizeof(ctx->peer_id), "%s", "default");

#ifdef _WIN32
  InitializeCriticalSection(&ctx->send_lock);
  InitializeConditionVariable(&ctx->send_cond);
  InitializeCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_init(&ctx->send_lock, NULL);
  pthread_cond_init(&ctx->send_cond, NULL);
  pthread_mutex_init(&ctx->pending_lock, NULL);
#endif
}

// -- Public create: server ------------------------------------------------

pcl_socket_transport_t* pcl_socket_transport_create_server_ex(
    uint16_t           port,
    pcl_executor_t*    executor,
    volatile uint16_t* port_ready) {
  struct pcl_socket_transport_t* ctx;
  struct sockaddr_in addr;
  int opt = 1;

  if (!executor) return NULL;

#ifdef _WIN32
  { WSADATA wsa; if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return NULL; }
#endif

  ctx = (struct pcl_socket_transport_t*)calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;

  ctx->is_server = 1;
  ctx->port      = port;
  socket_transport_create_common(ctx, executor);

  ctx->listen_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (ctx->listen_sock == PCL_INVALID_SOCKET) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

  setsockopt(ctx->listen_sock, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port        = htons(port);

  if (bind(ctx->listen_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

  if (port == 0) {
#ifdef _WIN32
    int len = (int)sizeof(addr);
#else
    socklen_t len = sizeof(addr);
#endif
    if (getsockname(ctx->listen_sock, (struct sockaddr*)&addr, &len) == 0)
      ctx->port = ntohs(addr.sin_port);
  }

  /* Signal caller thread that the port is known (before accept blocks). */
  if (port_ready)
    *port_ready = ctx->port;

  if (listen(ctx->listen_sock, 1) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

  ctx->client_sock = accept(ctx->listen_sock, NULL, NULL);
  if (ctx->client_sock == PCL_INVALID_SOCKET) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

  pcl_close_socket(ctx->listen_sock);
  ctx->listen_sock = PCL_INVALID_SOCKET;

#ifdef _WIN32
  ctx->recv_thread = CreateThread(NULL, 0, recv_thread_main, ctx, 0, NULL);
  if (!ctx->recv_thread) { pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL; }
  ctx->send_thread = CreateThread(NULL, 0, send_thread_main, ctx, 0, NULL);
  if (!ctx->send_thread) { pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL; }
#else
  if (pthread_create(&ctx->recv_thread, NULL, recv_thread_main, ctx) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL;
  }
  if (pthread_create(&ctx->send_thread, NULL, send_thread_main, ctx) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL;
  }
#endif

  {
    pcl_callbacks_t cbs;
    memset(&cbs, 0, sizeof(cbs));
    cbs.on_configure = gateway_on_configure;
    ctx->gateway = pcl_container_create("__pcl_socket_gateway", &cbs, ctx);
  }

  return (pcl_socket_transport_t*)ctx;
}

pcl_socket_transport_t* pcl_socket_transport_create_server(uint16_t        port,
                                                           pcl_executor_t* executor) {
  return pcl_socket_transport_create_server_ex(port, executor, NULL);
}

uint16_t pcl_socket_transport_get_port(const pcl_socket_transport_t* ctx) {
  if (!ctx) return 0;
  return ctx->port;
}

// -- Public create: client ------------------------------------------------

pcl_socket_transport_t* pcl_socket_transport_create_client(const char*      host,
                                                           uint16_t         port,
                                                           pcl_executor_t*  executor) {
  struct pcl_socket_transport_t* ctx;
  struct sockaddr_in addr;
  struct hostent*    he;

  if (!host || !executor) return NULL;

#ifdef _WIN32
  { WSADATA wsa; if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return NULL; }
#endif

  ctx = (struct pcl_socket_transport_t*)calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;

  ctx->is_server = 0;
  ctx->port      = port;
  snprintf(ctx->host, sizeof(ctx->host), "%s", host);
  socket_transport_create_common(ctx, executor);

  he = gethostbyname(host);
  if (!he) { pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL; }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(port);
  memcpy(&addr.sin_addr, he->h_addr_list[0], (size_t)he->h_length);

  ctx->client_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (ctx->client_sock == PCL_INVALID_SOCKET) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL;
  }

  if (connect(ctx->client_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL;
  }

#ifdef _WIN32
  ctx->recv_thread = CreateThread(NULL, 0, recv_thread_main, ctx, 0, NULL);
  if (!ctx->recv_thread) { pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL; }
  ctx->send_thread = CreateThread(NULL, 0, send_thread_main, ctx, 0, NULL);
  if (!ctx->send_thread) { pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL; }
#else
  if (pthread_create(&ctx->recv_thread, NULL, recv_thread_main, ctx) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL;
  }
  if (pthread_create(&ctx->send_thread, NULL, send_thread_main, ctx) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx); return NULL;
  }
#endif

  return (pcl_socket_transport_t*)ctx;
}

// -- Accessors ------------------------------------------------------------

const pcl_transport_t* pcl_socket_transport_get_transport(pcl_socket_transport_t* ctx) {
  if (!ctx) return NULL;
  return &ctx->transport;
}

pcl_status_t pcl_socket_transport_set_peer_id(pcl_socket_transport_t* ctx,
                                              const char*             peer_id) {
  if (!ctx || !peer_id || !peer_id[0]) return PCL_ERR_INVALID;
  snprintf(ctx->peer_id, sizeof(ctx->peer_id), "%s", peer_id);
  return PCL_OK;
}

pcl_container_t* pcl_socket_transport_gateway_container(pcl_socket_transport_t* ctx) {
  if (!ctx) return NULL;
  return ctx->gateway;
}

// -- Async service invocation (client only) -------------------------------

pcl_status_t pcl_socket_transport_invoke_remote_async(
    pcl_socket_transport_t* ctx_opaque,
    const char*             service_name,
    const pcl_msg_t*        request,
    pcl_resp_cb_fn_t        callback,
    void*                   user_data) {

  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)ctx_opaque;
  pcl_svc_pending_t* pending;
  uint16_t  svc_len, req_type_len;
  uint32_t  req_len, seq_id, payload_size;
  uint8_t*  frame;
  size_t    off;
  pcl_status_t rc;

  if (!ctx || !service_name || !request || !callback) return PCL_ERR_INVALID;
  if (ctx->is_server)                                  return PCL_ERR_INVALID;
  if (ctx->client_sock == PCL_INVALID_SOCKET)          return PCL_ERR_INVALID;

  pending = (pcl_svc_pending_t*)malloc(sizeof(*pending));
  if (!pending) return PCL_ERR_NOMEM;

  svc_len      = (uint16_t)strlen(service_name);
  req_type_len = (uint16_t)(request->type_name ? strlen(request->type_name) : 0u);
  req_len      = request->size;
  payload_size = 1u + 4u + 2u + svc_len + 2u + req_type_len + 4u + req_len;

  if (payload_size > PCL_SOCKET_MAX_PAYLOAD) {
    free(pending);
    return PCL_ERR_NOMEM;
  }

  /* frame: [4:payload_size][1:type=SVC_REQ][4:seq_id][2:svc_len][svc_name]
            [2:type_len][type_name][4:req_len][req_data] */
  frame = (uint8_t*)malloc(4u + payload_size);
  if (!frame) {
    free(pending);
    return PCL_ERR_NOMEM;
  }

  /* Assign sequence ID and register pending record under pending_lock */
#ifdef _WIN32
  EnterCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_lock(&ctx->pending_lock);
#endif

  seq_id = ctx->next_seq_id++;
  if (ctx->next_seq_id == 0u) ctx->next_seq_id = 1u; /* skip 0 */

  pending->seq_id    = seq_id;
  pending->cb        = callback;
  pending->user_data = user_data;
  pending->next      = ctx->pending_head;
  ctx->pending_head  = pending;

#ifdef _WIN32
  LeaveCriticalSection(&ctx->pending_lock);
#else
  pthread_mutex_unlock(&ctx->pending_lock);
#endif

  off = 0;
  write_u32_be(frame + off, payload_size);              off += 4;
  frame[off++] = PCL_SOCKET_MSG_SVC_REQ;
  write_u32_be(frame + off, seq_id);                    off += 4;
  write_u16_be(frame + off, svc_len);                   off += 2;
  memcpy(frame + off, service_name, svc_len);           off += svc_len;
  write_u16_be(frame + off, req_type_len);              off += 2;
  if (req_type_len > 0u && request->type_name) {
    memcpy(frame + off, request->type_name, req_type_len);
  }
  off += req_type_len;
  write_u32_be(frame + off, req_len);                   off += 4;
  if (req_len > 0u && request->data) {
    memcpy(frame + off, request->data, req_len);
  }

  rc = enqueue_outbound_frame(ctx, frame, (uint32_t)(4u + payload_size));
  free(frame);

  if (rc != PCL_OK) {
    /* Remove pending record since frame was not enqueued */
    pcl_svc_pending_t** pp;
#ifdef _WIN32
    EnterCriticalSection(&ctx->pending_lock);
#else
    pthread_mutex_lock(&ctx->pending_lock);
#endif
    for (pp = &ctx->pending_head; *pp; pp = &(*pp)->next) {
      if ((*pp)->seq_id == seq_id) {
        pcl_svc_pending_t* dead = *pp;
        *pp = dead->next;
        free(dead);
        break;
      }
    }
#ifdef _WIN32
    LeaveCriticalSection(&ctx->pending_lock);
#else
    pthread_mutex_unlock(&ctx->pending_lock);
#endif
  }

  return rc;
}

// -- Destroy --------------------------------------------------------------

void pcl_socket_transport_destroy(pcl_socket_transport_t* ctx_opaque) {
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)ctx_opaque;
  if (!ctx) return;

  /* Unregister from executor before freeing so pcl_executor_destroy does not
     call transport.shutdown() on the already-freed ctx (use-after-free). */
  if (ctx->executor) {
    pcl_executor_set_transport(ctx->executor, NULL);
    pcl_executor_register_transport(ctx->executor, ctx->peer_id, NULL);
  }

  /* Signal threads to stop */
  ctx->recv_stop = 1;
  ctx->send_stop = 1;

  /* Unblock recv_thread: shutdown() reliably interrupts a blocked recv().
     Close socket so recv_thread exits its loop. */
  if (ctx->client_sock != PCL_INVALID_SOCKET) {
    shutdown(ctx->client_sock, SHUT_RDWR);
    pcl_close_socket(ctx->client_sock);
    ctx->client_sock = PCL_INVALID_SOCKET;
  }
  if (ctx->listen_sock != PCL_INVALID_SOCKET) {
    pcl_close_socket(ctx->listen_sock);
    ctx->listen_sock = PCL_INVALID_SOCKET;
  }

  /* Wake send_thread (may be waiting on empty queue) */
#ifdef _WIN32
  EnterCriticalSection(&ctx->send_lock);
  WakeConditionVariable(&ctx->send_cond);
  LeaveCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_lock(&ctx->send_lock);
  pthread_cond_signal(&ctx->send_cond);
  pthread_mutex_unlock(&ctx->send_lock);
#endif

  /* Join threads */
#ifdef _WIN32
  if (ctx->recv_thread) {
    WaitForSingleObject(ctx->recv_thread, 5000);
    CloseHandle(ctx->recv_thread);
    ctx->recv_thread = NULL;
  }
  if (ctx->send_thread) {
    WaitForSingleObject(ctx->send_thread, 5000);
    CloseHandle(ctx->send_thread);
    ctx->send_thread = NULL;
  }
  DeleteCriticalSection(&ctx->send_lock);
  DeleteCriticalSection(&ctx->pending_lock);
#else
  if (ctx->recv_thread) {
    pthread_join(ctx->recv_thread, NULL);
    ctx->recv_thread = 0;
  }
  if (ctx->send_thread) {
    pthread_join(ctx->send_thread, NULL);
    ctx->send_thread = 0;
  }
  pthread_mutex_destroy(&ctx->send_lock);
  pthread_cond_destroy(&ctx->send_cond);
  pthread_mutex_destroy(&ctx->pending_lock);
#endif

  /* Drain any unsent frames */
  {
    pcl_outbound_frame_t* f = ctx->send_head;
    while (f) {
      pcl_outbound_frame_t* next = f->next;
      free(f->data);
      free(f);
      f = next;
    }
  }

  /* Free any un-responded pending async calls */
  {
    pcl_svc_pending_t* p = ctx->pending_head;
    while (p) {
      pcl_svc_pending_t* next = p->next;
      free(p);
      p = next;
    }
  }

  if (ctx->gateway) {
    /* Remove from executor before freeing: pcl_executor_destroy iterates its
       containers array and writes c->executor = NULL; if the gateway is already
       freed that is a use-after-free / access violation. */
    if (ctx->executor) {
      pcl_executor_remove(ctx->executor, ctx->gateway);
    }
    pcl_container_destroy(ctx->gateway);
    ctx->gateway = NULL;
  }

  free(ctx);
}
