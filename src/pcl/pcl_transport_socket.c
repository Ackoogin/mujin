/// \file pcl_transport_socket.c
/// \brief TCP socket transport implementation.
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
#  define PCL_SOCKET_T int
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

#define PCL_SOCKET_MSG_PUBLISH 0
#define PCL_SOCKET_MSG_SVC_REQ 1
#define PCL_SOCKET_MSG_SVC_RESP 2
#define PCL_SOCKET_TOPIC_SVC_REQ "__pcl_svc_req"
#define PCL_SOCKET_MAX_PAYLOAD 65536

struct pcl_socket_transport_t {
  int                is_server;
  uint16_t           port;
  char               host[256];
  pcl_executor_t*    executor;
  pcl_transport_t    transport;

  PCL_SOCKET_T       listen_sock;
  PCL_SOCKET_T       client_sock;

  volatile int       recv_running;
  volatile int       recv_stop;

#ifdef _WIN32
  HANDLE             recv_thread;
  CRITICAL_SECTION   send_lock;
  CRITICAL_SECTION   resp_lock;
  CONDITION_VARIABLE resp_cond;
#else
  pthread_t          recv_thread;
  pthread_mutex_t    send_lock;
  pthread_mutex_t    resp_lock;
  pthread_cond_t     resp_cond;
#endif

  volatile int       resp_ready;
  uint32_t           resp_size;
  void*              resp_data;

  pcl_container_t*   gateway;
};

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

static pcl_status_t socket_publish(void* adapter_ctx,
                                   const char* topic,
                                   const pcl_msg_t* msg) {
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)adapter_ctx;
  uint8_t hdr[4];
  uint8_t type_hdr[1];
  uint16_t topic_len, type_len;
  uint32_t data_len;
  size_t total;
  uint8_t* buf;
  size_t off;

  if (!ctx || ctx->client_sock == PCL_INVALID_SOCKET) return PCL_ERR_INVALID;

  topic_len = (uint16_t)strlen(topic);
  type_len = (uint16_t)(msg->type_name ? strlen(msg->type_name) : 0);
  data_len = msg->size;

  total = 1 + 2 + topic_len + 2 + type_len + 4 + data_len;
  if (total > PCL_SOCKET_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  buf = (uint8_t*)malloc(total);
  if (!buf) return PCL_ERR_NOMEM;

  off = 0;
  buf[off++] = PCL_SOCKET_MSG_PUBLISH;
  write_u16_be(buf + off, topic_len); off += 2;
  memcpy(buf + off, topic, topic_len); off += topic_len;
  write_u16_be(buf + off, type_len); off += 2;
  if (type_len) {
    memcpy(buf + off, msg->type_name, type_len); off += type_len;
  }
  write_u32_be(buf + off, data_len); off += 4;
  if (data_len && msg->data) {
    memcpy(buf + off, msg->data, data_len);
  }

  write_u32_be(hdr, (uint32_t)total);
  type_hdr[0] = PCL_SOCKET_MSG_PUBLISH;

#ifdef _WIN32
  EnterCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_lock(&ctx->send_lock);
#endif

  if (send_all(ctx->client_sock, hdr, 4) != 0 ||
      send_all(ctx->client_sock, buf, total) != 0) {
#ifdef _WIN32
    LeaveCriticalSection(&ctx->send_lock);
#else
    pthread_mutex_unlock(&ctx->send_lock);
#endif
    free(buf);
    return PCL_ERR_NOMEM;
  }

#ifdef _WIN32
  LeaveCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_unlock(&ctx->send_lock);
#endif
  free(buf);
  return PCL_OK;
}

static void gateway_sub_cb(pcl_container_t* c,
                           const pcl_msg_t* msg,
                           void* user_data) {
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)user_data;
  const uint8_t* p;
  uint32_t conn_id, svc_len, req_len;
  char svc_name[256];
  pcl_msg_t req, resp;
  uint8_t resp_buf[PCL_SOCKET_MAX_PAYLOAD];
  uint8_t hdr[4];
  uint8_t type_byte;
  uint32_t total;

  (void)c;

  if (!ctx || !msg || msg->size < 10) return;

  p = (const uint8_t*)msg->data;
  conn_id = read_u32_be(p); p += 4;
  svc_len = read_u16_be(p); p += 2;
  if (svc_len >= sizeof(svc_name)) return;
  memcpy(svc_name, p, svc_len); svc_name[svc_len] = '\0'; p += svc_len;
  req_len = read_u32_be(p); p += 4;

  memset(&req, 0, sizeof(req));
  req.data = (req_len > 0) ? (const void*)p : NULL;
  req.size = req_len;
  req.type_name = "SubscribeInterest_Request";

  memset(&resp, 0, sizeof(resp));
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  resp.type_name = "SubscribeInterest_Response";

  if (pcl_executor_invoke_service(ctx->executor, svc_name, &req, &resp) != PCL_OK) {
    resp.size = 0;
  }

  type_byte = PCL_SOCKET_MSG_SVC_RESP;
  total = 1 + 4 + resp.size;
  write_u32_be(hdr, total);

#ifdef _WIN32
  EnterCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_lock(&ctx->send_lock);
#endif

  if (ctx->client_sock != PCL_INVALID_SOCKET) {
    uint8_t wire[9];
    wire[0] = type_byte;
    write_u32_be(wire + 1, resp.size);
    send_all(ctx->client_sock, hdr, 4);
    send_all(ctx->client_sock, wire, 5);
    if (resp.size > 0 && resp.data) {
      send_all(ctx->client_sock, resp.data, resp.size);
    }
  }

#ifdef _WIN32
  LeaveCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_unlock(&ctx->send_lock);
#endif
}

static pcl_status_t socket_subscribe(void* adapter_ctx,
                                     const char* topic,
                                     const char* type_name) {
  (void)adapter_ctx;
  (void)topic;
  (void)type_name;
  return PCL_OK;
}

static void socket_shutdown(void* adapter_ctx) {
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)adapter_ctx;
  if (!ctx) return;
  ctx->recv_stop = 1;
}

#ifdef _WIN32
static DWORD WINAPI recv_thread_main(LPVOID arg)
#else
static void* recv_thread_main(void* arg)
#endif
{
  struct pcl_socket_transport_t* ctx = (struct pcl_socket_transport_t*)arg;
  uint8_t len_buf[4];
  uint8_t type_buf[1];
  uint32_t payload_len;
  uint8_t* payload;
  uint16_t topic_len, type_len;
  uint32_t data_len;
  char* topic;
  char* type_name;
  pcl_msg_t msg;

  ctx->recv_running = 1;

  while (!ctx->recv_stop && ctx->client_sock != PCL_INVALID_SOCKET) {
    if (recv_all(ctx->client_sock, len_buf, 4) != 0) break;
    payload_len = read_u32_be(len_buf);
    if (payload_len == 0 || payload_len > PCL_SOCKET_MAX_PAYLOAD) break;

    payload = (uint8_t*)malloc(payload_len);
    if (!payload) break;

    if (recv_all(ctx->client_sock, payload, payload_len) != 0) {
      free(payload);
      break;
    }

    if (payload_len < 1) {
      free(payload);
      continue;
    }

    if (payload[0] == PCL_SOCKET_MSG_PUBLISH) {
      if (payload_len < 1 + 2 + 2 + 4) {
        free(payload);
        continue;
      }
      topic_len = read_u16_be(payload + 1);
      if (1 + 2 + topic_len + 2 + 4 > payload_len) {
        free(payload);
        continue;
      }
      topic = (char*)malloc(topic_len + 1);
      if (!topic) {
        free(payload);
        continue;
      }
      memcpy(topic, payload + 3, topic_len);
      topic[topic_len] = '\0';
      type_len = read_u16_be(payload + 3 + topic_len);
      if (1 + 2 + topic_len + 2 + type_len + 4 > payload_len) {
        free(topic);
        free(payload);
        continue;
      }
      type_name = (char*)malloc(type_len + 1);
      if (!type_name) {
        free(topic);
        free(payload);
        continue;
      }
      memcpy(type_name, payload + 5 + topic_len, type_len);
      type_name[type_len] = '\0';
      data_len = read_u32_be(payload + 5 + topic_len + type_len);

      memset(&msg, 0, sizeof(msg));
      msg.data = (data_len > 0) ? (payload + 9 + topic_len + type_len) : NULL;
      msg.size = data_len;
      msg.type_name = type_name;

      pcl_executor_post_incoming(ctx->executor, topic, &msg);

      free(topic);
      free(type_name);
      free(payload);
    } else if (payload[0] == PCL_SOCKET_MSG_SVC_REQ && ctx->is_server) {
      pcl_msg_t svc_msg;
      uint8_t* fwd;
      uint32_t fwd_len = 4 + payload_len - 1;
      fwd = (uint8_t*)malloc(fwd_len);
      if (!fwd) {
        free(payload);
        continue;
      }
      write_u32_be(fwd, 0);
      memcpy(fwd + 4, payload + 1, payload_len - 1);

      memset(&svc_msg, 0, sizeof(svc_msg));
      svc_msg.data = fwd;
      svc_msg.size = fwd_len;
      svc_msg.type_name = "SubscribeInterest_Request";

      pcl_executor_post_incoming(ctx->executor, PCL_SOCKET_TOPIC_SVC_REQ, &svc_msg);
      free(fwd);
      free(payload);
    } else if (payload[0] == PCL_SOCKET_MSG_SVC_RESP && !ctx->is_server) {
#ifdef _WIN32
      EnterCriticalSection(&ctx->resp_lock);
#else
      pthread_mutex_lock(&ctx->resp_lock);
#endif
      if (payload_len >= 5) {
        ctx->resp_size = read_u32_be(payload + 1);
        if (ctx->resp_size > 0 && ctx->resp_size <= payload_len - 5) {
          void* tmp = realloc(ctx->resp_data, ctx->resp_size);
          if (tmp) {
            ctx->resp_data = tmp;
            memcpy(ctx->resp_data, payload + 5, ctx->resp_size);
          }
        } else {
          ctx->resp_size = 0;
        }
      } else {
        ctx->resp_size = 0;
      }
      ctx->resp_ready = 1;
#ifdef _WIN32
      WakeConditionVariable(&ctx->resp_cond);
      LeaveCriticalSection(&ctx->resp_lock);
#else
      pthread_cond_signal(&ctx->resp_cond);
      pthread_mutex_unlock(&ctx->resp_lock);
#endif
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

static pcl_status_t gateway_on_configure(pcl_container_t* c, void* ud) {
  struct pcl_socket_transport_t* t = (struct pcl_socket_transport_t*)ud;
  if (!pcl_container_add_subscriber(c, PCL_SOCKET_TOPIC_SVC_REQ, "SubscribeInterest_Request",
                                    gateway_sub_cb, t)) {
    return PCL_ERR_NOMEM;
  }
  return PCL_OK;
}

static pcl_status_t socket_transport_create_common(struct pcl_socket_transport_t* ctx,
                                                   pcl_executor_t* executor) {
  memset(&ctx->transport, 0, sizeof(ctx->transport));
  ctx->transport.publish = socket_publish;
  ctx->transport.subscribe = socket_subscribe;
  ctx->transport.shutdown = socket_shutdown;
  ctx->transport.adapter_ctx = ctx;

  ctx->listen_sock = PCL_INVALID_SOCKET;
  ctx->client_sock = PCL_INVALID_SOCKET;
  ctx->recv_running = 0;
  ctx->recv_stop = 0;
  ctx->resp_ready = 0;
  ctx->resp_data = NULL;

#ifdef _WIN32
  InitializeCriticalSection(&ctx->send_lock);
  InitializeCriticalSection(&ctx->resp_lock);
  InitializeConditionVariable(&ctx->resp_cond);
#else
  pthread_mutex_init(&ctx->send_lock, NULL);
  pthread_mutex_init(&ctx->resp_lock, NULL);
  pthread_cond_init(&ctx->resp_cond, NULL);
#endif

  return PCL_OK;
}

pcl_socket_transport_t* pcl_socket_transport_create_server(uint16_t        port,
                                                          pcl_executor_t* executor) {
  struct pcl_socket_transport_t* ctx;
  struct sockaddr_in addr;
  int opt = 1;

  if (!executor) return NULL;

#ifdef _WIN32
  {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return NULL;
  }
#endif

  ctx = (struct pcl_socket_transport_t*)calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;

  ctx->is_server = 1;
  ctx->port = port;
  ctx->executor = executor;

  socket_transport_create_common(ctx, executor);

  ctx->listen_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (ctx->listen_sock == PCL_INVALID_SOCKET) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

  setsockopt(ctx->listen_sock, SOL_SOCKET, SO_REUSEADDR,
             (char*)&opt, sizeof(opt));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

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
    if (getsockname(ctx->listen_sock, (struct sockaddr*)&addr, &len) == 0) {
      ctx->port = ntohs(addr.sin_port);
    }
  }

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
  if (!ctx->recv_thread) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }
#else
  if (pthread_create(&ctx->recv_thread, NULL, recv_thread_main, ctx) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
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

pcl_socket_transport_t* pcl_socket_transport_create_client(const char*      host,
                                                           uint16_t        port,
                                                           pcl_executor_t* executor) {
  struct pcl_socket_transport_t* ctx;
  struct sockaddr_in addr;
  struct hostent* he;

  if (!host || !executor) return NULL;

#ifdef _WIN32
  {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return NULL;
  }
#endif

  ctx = (struct pcl_socket_transport_t*)calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;

  ctx->is_server = 0;
  ctx->port = port;
  snprintf(ctx->host, sizeof(ctx->host), "%s", host);
  ctx->executor = executor;

  socket_transport_create_common(ctx, executor);

  he = gethostbyname(host);
  if (!he) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  memcpy(&addr.sin_addr, he->h_addr_list[0], (size_t)he->h_length);

  ctx->client_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (ctx->client_sock == PCL_INVALID_SOCKET) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

  if (connect(ctx->client_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }

#ifdef _WIN32
  ctx->recv_thread = CreateThread(NULL, 0, recv_thread_main, ctx, 0, NULL);
  if (!ctx->recv_thread) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }
#else
  if (pthread_create(&ctx->recv_thread, NULL, recv_thread_main, ctx) != 0) {
    pcl_socket_transport_destroy((pcl_socket_transport_t*)ctx);
    return NULL;
  }
#endif

  return (pcl_socket_transport_t*)ctx;
}

const pcl_transport_t* pcl_socket_transport_get_transport(pcl_socket_transport_t* ctx) {
  if (!ctx) return NULL;
  return &ctx->transport;
}

pcl_container_t* pcl_socket_transport_gateway_container(pcl_socket_transport_t* ctx) {
  if (!ctx) return NULL;
  return ctx->gateway;
}

pcl_status_t pcl_socket_transport_invoke_remote(pcl_socket_transport_t* ctx,
                                                const char*             service_name,
                                                const pcl_msg_t*        request,
                                                pcl_msg_t*             response) {
  uint16_t svc_len;
  uint32_t req_len;
  size_t total;
  uint8_t* buf;
  uint8_t hdr[4];
  size_t off;

  if (!ctx || !service_name || !request || !response) return PCL_ERR_INVALID;
  if (ctx->is_server) return PCL_ERR_INVALID;
  if (ctx->client_sock == PCL_INVALID_SOCKET) return PCL_ERR_INVALID;

  svc_len = (uint16_t)strlen(service_name);
  req_len = request->size;

  total = 1 + 2 + svc_len + 4 + req_len;
  if (total > PCL_SOCKET_MAX_PAYLOAD) return PCL_ERR_NOMEM;

  buf = (uint8_t*)malloc(total);
  if (!buf) return PCL_ERR_NOMEM;

  off = 0;
  buf[off++] = PCL_SOCKET_MSG_SVC_REQ;
  write_u16_be(buf + off, svc_len); off += 2;
  memcpy(buf + off, service_name, svc_len); off += svc_len;
  write_u32_be(buf + off, req_len); off += 4;
  if (req_len && request->data) {
    memcpy(buf + off, request->data, req_len);
  }

  write_u32_be(hdr, (uint32_t)total);

#ifdef _WIN32
  EnterCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_lock(&ctx->send_lock);
#endif

  if (send_all(ctx->client_sock, hdr, 4) != 0 ||
      send_all(ctx->client_sock, buf, total) != 0) {
#ifdef _WIN32
    LeaveCriticalSection(&ctx->send_lock);
#else
    pthread_mutex_unlock(&ctx->send_lock);
#endif
    free(buf);
    return PCL_ERR_NOMEM;
  }

#ifdef _WIN32
  LeaveCriticalSection(&ctx->send_lock);
#else
  pthread_mutex_unlock(&ctx->send_lock);
#endif
  free(buf);

  ctx->resp_ready = 0;
#ifdef _WIN32
  EnterCriticalSection(&ctx->resp_lock);
  while (!ctx->resp_ready && !ctx->recv_stop) {
    SleepConditionVariableCS(&ctx->resp_cond, &ctx->resp_lock, INFINITE);
  }
  if (ctx->resp_data) {
    if (response->data && response->size >= ctx->resp_size) {
      memcpy((void*)response->data, ctx->resp_data, ctx->resp_size);
      response->size = ctx->resp_size;
    } else {
      response->size = ctx->resp_size;
    }
  }
  LeaveCriticalSection(&ctx->resp_lock);
#else
  pthread_mutex_lock(&ctx->resp_lock);
  while (!ctx->resp_ready && !ctx->recv_stop) {
    pthread_cond_wait(&ctx->resp_cond, &ctx->resp_lock);
  }
  if (ctx->resp_data) {
    if (response->data && response->size >= ctx->resp_size) {
      memcpy((void*)response->data, ctx->resp_data, ctx->resp_size);
      response->size = ctx->resp_size;
    } else {
      response->size = ctx->resp_size;
    }
  }
  pthread_mutex_unlock(&ctx->resp_lock);
#endif

  return PCL_OK;
}

void pcl_socket_transport_destroy(pcl_socket_transport_t* ctx) {
  if (!ctx) return;

  ctx->recv_stop = 1;

  /* shutdown() unblocks any thread blocked in recv() on this socket.
     close() alone does NOT reliably interrupt a blocked recv() on Linux. */
  if (ctx->client_sock != PCL_INVALID_SOCKET) {
    shutdown(ctx->client_sock, SHUT_RDWR);
    pcl_close_socket(ctx->client_sock);
    ctx->client_sock = PCL_INVALID_SOCKET;
  }
  if (ctx->listen_sock != PCL_INVALID_SOCKET) {
    pcl_close_socket(ctx->listen_sock);
    ctx->listen_sock = PCL_INVALID_SOCKET;
  }

#ifdef _WIN32
  if (ctx->recv_thread) {
    WaitForSingleObject(ctx->recv_thread, 5000);
    CloseHandle(ctx->recv_thread);
  }
  DeleteCriticalSection(&ctx->send_lock);
  DeleteCriticalSection(&ctx->resp_lock);
#else
  if (ctx->recv_thread) {
    pthread_join(ctx->recv_thread, NULL);
  }
  pthread_mutex_destroy(&ctx->send_lock);
  pthread_mutex_destroy(&ctx->resp_lock);
  pthread_cond_destroy(&ctx->resp_cond);
#endif

  free(ctx->resp_data);
  if (ctx->gateway) {
    pcl_container_destroy(ctx->gateway);
  }
  free(ctx);
}
