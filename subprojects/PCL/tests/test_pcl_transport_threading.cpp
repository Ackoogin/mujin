/// \file test_pcl_transport_threading.cpp
/// \brief Threading-model conformance suite for PCL transports.
///
/// Where `pcl_transport_conformance.hpp` proves *delivery*, this suite proves
/// *threading behaviour* -- the contract documented on `pcl_transport_t`:
///
///   - Group A: subscriber / response callbacks run on the executor thread,
///     never inline on a transport recv/worker thread.
///   - Group B: executor-facing egress (`publish`, `invoke_async`) returns
///     promptly even while the real send path is blocked.
///   - Group C: `invoke_async` does not fire its callback inline; the callback
///     is delivered on a later executor spin.
///   - Group D: destroy wakes and joins blocked worker threads.
///
/// The ROS2 / gRPC coupled-plugin threading cases named in the report live
/// with those plugins (they need the all-enabled build); this file covers the
/// in-tree transports: template, shared memory, UDP, and socket.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <unistd.h>
#endif

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_transport_shared_memory.h"
#include "pcl/pcl_transport_socket.h"
#include "pcl/pcl_transport_template.h"
#include "pcl/pcl_transport_udp.h"
}

#include "pcl_transport_conformance.hpp"

namespace {

using pcl_conformance::TransportPair;

void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}
void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

std::string unique_bus(const char* prefix) {
#ifdef _WIN32
  unsigned long pid = GetCurrentProcessId();
#else
  unsigned long pid = static_cast<unsigned long>(getpid());
#endif
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::string(prefix) + "_" + std::to_string(pid) + "_" +
         std::to_string(now);
}

uint16_t pick_free_udp_port() {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
  SOCKET s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s == INVALID_SOCKET) return 0;
#else
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) return 0;
#endif
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  uint16_t port = 0;
  if (bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == 0) {
#ifdef _WIN32
    int len = static_cast<int>(sizeof(addr));
#else
    socklen_t len = sizeof(addr);
#endif
    getsockname(s, reinterpret_cast<struct sockaddr*>(&addr), &len);
    port = ntohs(addr.sin_port);
  }
#ifdef _WIN32
  closesocket(s);
#else
  close(s);
#endif
  return port;
}

// ===========================================================================
// Template transport: loopback + gated send
// ===========================================================================
//
// A pair of in-memory frame queues wires two template transports back to back.
// A separate "gated" send hook lets a test wedge the send path behind a latch
// so egress-promptness and destroy-wake behaviour can be proven deterministically
// without real network timing.

struct Frame {
  pcl_template_frame_kind_t kind = PCL_TEMPLATE_FRAME_PUBLISH;
  uint32_t                  seq_id = 0;
  char*                     topic = nullptr;
  char*                     service_name = nullptr;
  char*                     type_name = nullptr;
  uint8_t*                  payload = nullptr;
  uint32_t                  payload_size = 0;

  Frame() = default;
  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;
  Frame(Frame&& o) noexcept { move_from(o); }
  Frame& operator=(Frame&& o) noexcept { if (this != &o) { reset(); move_from(o); } return *this; }
  ~Frame() { reset(); }

  void move_from(Frame& o) {
    kind = o.kind; seq_id = o.seq_id;
    topic = o.topic; o.topic = nullptr;
    service_name = o.service_name; o.service_name = nullptr;
    type_name = o.type_name; o.type_name = nullptr;
    payload = o.payload; o.payload = nullptr;
    payload_size = o.payload_size; o.payload_size = 0;
  }
  void reset() {
    free(topic); free(service_name); free(type_name); free(payload);
    topic = service_name = type_name = nullptr;
    payload = nullptr; payload_size = 0;
  }
  static char* dup(const char* s) {
    if (!s) return nullptr;
    size_t n = std::strlen(s);
    char* out = static_cast<char*>(malloc(n + 1));
    std::memcpy(out, s, n + 1);
    return out;
  }
  static Frame borrow(const pcl_template_frame_t& f) {
    Frame out;
    out.kind = f.kind; out.seq_id = f.seq_id;
    out.topic = dup(f.topic);
    out.service_name = dup(f.service_name);
    out.type_name = dup(f.type_name);
    if (f.payload && f.payload_size) {
      out.payload = static_cast<uint8_t*>(malloc(f.payload_size));
      std::memcpy(out.payload, f.payload, f.payload_size);
      out.payload_size = f.payload_size;
    }
    return out;
  }
};

struct Link {
  std::mutex mu;
  std::condition_variable cv;
  std::deque<Frame> q;
};

struct End {
  Link* out = nullptr;
  Link* in = nullptr;
  std::atomic<bool> stopped{false};

  // Optional send gate: when armed, send_blocking blocks until released/woken.
  std::mutex gate_mu;
  std::condition_variable gate_cv;
  bool gate_armed = false;
  bool gate_release = false;
  std::atomic<int> send_entered{0};
};

extern "C" pcl_status_t end_send(void* ud, const pcl_template_frame_t* f) {
  auto* e = static_cast<End*>(ud);
  if (!e || !f) return PCL_ERR_INVALID;
  e->send_entered.fetch_add(1);
  {
    std::unique_lock<std::mutex> lk(e->gate_mu);
    if (e->gate_armed) {
      e->gate_cv.wait(lk, [&] { return e->gate_release || e->stopped.load(); });
    }
  }
  Frame copy = Frame::borrow(*f);
  std::lock_guard<std::mutex> lk(e->out->mu);
  e->out->q.emplace_back(std::move(copy));
  e->out->cv.notify_all();
  return PCL_OK;
}

extern "C" pcl_status_t end_recv(void* ud, pcl_template_frame_t* out,
                                 uint32_t timeout_ms) {
  auto* e = static_cast<End*>(ud);
  if (!e || !out) return PCL_ERR_INVALID;
  std::unique_lock<std::mutex> lk(e->in->mu);
  e->in->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                     [&] { return !e->in->q.empty() || e->stopped.load(); });
  if (e->in->q.empty()) return PCL_ERR_TIMEOUT;
  Frame f = std::move(e->in->q.front());
  e->in->q.pop_front();
  lk.unlock();
  out->kind = f.kind; out->seq_id = f.seq_id;
  out->topic = f.topic; f.topic = nullptr;
  out->service_name = f.service_name; f.service_name = nullptr;
  out->type_name = f.type_name; f.type_name = nullptr;
  out->payload = f.payload; f.payload = nullptr;
  out->payload_size = f.payload_size;
  return PCL_OK;
}

extern "C" void end_wake(void* ud) {
  auto* e = static_cast<End*>(ud);
  if (!e) return;
  e->stopped = true;
  { std::lock_guard<std::mutex> lk(e->gate_mu); e->gate_cv.notify_all(); }
  if (e->in)  { std::lock_guard<std::mutex> lk(e->in->mu);  e->in->cv.notify_all(); }
  if (e->out) { std::lock_guard<std::mutex> lk(e->out->mu); e->out->cv.notify_all(); }
}

struct TemplatePair {
  Link ab, ba;
  End  a{ &ab, &ba };
  End  b{ &ba, &ab };
  pcl_executor_t* exec_a = nullptr;
  pcl_executor_t* exec_b = nullptr;
  pcl_transport_template_t* tpl_a = nullptr;
  pcl_transport_template_t* tpl_b = nullptr;

  TemplatePair() {
    exec_a = pcl_executor_create();
    exec_b = pcl_executor_create();
    pcl_template_io_hooks_t ha = {};
    ha.user_data = &a; ha.send_blocking = &end_send;
    ha.recv_blocking = &end_recv; ha.wake = &end_wake;
    pcl_template_io_hooks_t hb = {};
    hb.user_data = &b; hb.send_blocking = &end_send;
    hb.recv_blocking = &end_recv; hb.wake = &end_wake;
    tpl_a = pcl_transport_template_create(&ha, exec_a);
    tpl_b = pcl_transport_template_create(&hb, exec_b);
    pcl_transport_template_set_peer_id(tpl_a, "node_b");
    pcl_transport_template_set_peer_id(tpl_b, "node_a");
    pcl_executor_set_transport(exec_a, pcl_transport_template_get_transport(tpl_a));
    pcl_executor_set_transport(exec_b, pcl_transport_template_get_transport(tpl_b));
    pcl_executor_register_transport(exec_a, "node_b", pcl_transport_template_get_transport(tpl_a));
    pcl_executor_register_transport(exec_b, "node_a", pcl_transport_template_get_transport(tpl_b));
  }
  ~TemplatePair() {
    if (exec_a) { pcl_executor_set_transport(exec_a, nullptr);
                  pcl_executor_register_transport(exec_a, "node_b", nullptr); }
    if (exec_b) { pcl_executor_set_transport(exec_b, nullptr);
                  pcl_executor_register_transport(exec_b, "node_a", nullptr); }
    if (tpl_a) pcl_transport_template_destroy(tpl_a);
    if (tpl_b) pcl_transport_template_destroy(tpl_b);
    if (exec_a) pcl_executor_destroy(exec_a);
    if (exec_b) pcl_executor_destroy(exec_b);
  }
  TransportPair pair() const {
    TransportPair p;
    p.sender_exec = exec_a;
    p.receiver_exec = exec_b;
    p.sender_vtable = pcl_transport_template_get_transport(tpl_a);
    p.receiver_vtable = pcl_transport_template_get_transport(tpl_b);
    p.sender_peer_id = "node_a";
    p.receiver_peer_id = "node_b";
    return p;
  }
};

// ===========================================================================
// Shared-memory pair
// ===========================================================================

struct ShmPair {
  pcl_executor_t* sender_exec = nullptr;
  pcl_executor_t* receiver_exec = nullptr;
  pcl_shared_memory_transport_t* sender = nullptr;
  pcl_shared_memory_transport_t* receiver = nullptr;

  explicit ShmPair(const std::string& bus) {
    sender_exec = pcl_executor_create();
    receiver_exec = pcl_executor_create();
    sender = pcl_shared_memory_transport_create(bus.c_str(), "sender", sender_exec);
    receiver = pcl_shared_memory_transport_create(bus.c_str(), "receiver", receiver_exec);
    attach(sender_exec, sender);
    attach(receiver_exec, receiver);
  }
  static void attach(pcl_executor_t* e, pcl_shared_memory_transport_t* t) {
    pcl_executor_set_transport(e, pcl_shared_memory_transport_get_transport(t));
    pcl_container_t* gw = pcl_shared_memory_transport_gateway_container(t);
    pcl_container_configure(gw);
    pcl_container_activate(gw);
    pcl_executor_add(e, gw);
  }
  ~ShmPair() {
    if (sender) pcl_shared_memory_transport_destroy(sender);
    if (receiver) pcl_shared_memory_transport_destroy(receiver);
    if (sender_exec) pcl_executor_destroy(sender_exec);
    if (receiver_exec) pcl_executor_destroy(receiver_exec);
  }
  bool ok() const { return sender && receiver; }
  TransportPair pair() const {
    TransportPair p;
    p.sender_exec = sender_exec;
    p.receiver_exec = receiver_exec;
    p.sender_vtable = pcl_shared_memory_transport_get_transport(sender);
    p.receiver_vtable = pcl_shared_memory_transport_get_transport(receiver);
    p.sender_peer_id = "sender";
    p.receiver_peer_id = "receiver";
    return p;
  }
};

}  // namespace

// ===========================================================================
// Group A: ingress runs on the executor thread
// ===========================================================================

TEST(PclTransportThreading, TemplateIngressRunsOnExecutor) {
  silence_logs();
  TemplatePair tp;
  pcl_conformance::expectIngressRunsOnExecutorThread(
      tp.pair(), "telemetry/tick", "Tick", "payload");
  restore_logs();
}

TEST(PclTransportThreading, SharedMemoryIngressRunsOnExecutor) {
  silence_logs();
  ShmPair sp(unique_bus("thr_shm_ingress"));
  ASSERT_TRUE(sp.ok());
  pcl_conformance::expectIngressRunsOnExecutorThread(
      sp.pair(), "shm/tick", "Tick", "payload", /*retries=*/3);
  restore_logs();
}

TEST(PclTransportThreading, UdpIngressRunsOnExecutor) {
  silence_logs();
  const uint16_t recv_port = pick_free_udp_port();
  const uint16_t send_port = pick_free_udp_port();
  ASSERT_NE(recv_port, 0);
  ASSERT_NE(send_port, 0);
  ASSERT_NE(recv_port, send_port);

  auto* recv_exec = pcl_executor_create();
  auto* send_exec = pcl_executor_create();
  auto* recv_udp = pcl_udp_transport_create(recv_port, "127.0.0.1", send_port, recv_exec);
  auto* send_udp = pcl_udp_transport_create(send_port, "127.0.0.1", recv_port, send_exec);
  ASSERT_NE(recv_udp, nullptr);
  ASSERT_NE(send_udp, nullptr);
  // The receiver tags inbound datagrams with its transport peer_id, so the
  // subscriber (which allows "sender") sees them as coming from "sender".
  pcl_udp_transport_set_peer_id(recv_udp, "sender");
  pcl_udp_transport_set_peer_id(send_udp, "receiver");
  pcl_executor_register_transport(recv_exec, "sender",
      pcl_udp_transport_get_transport(recv_udp));

  TransportPair p;
  p.sender_exec = send_exec;
  p.receiver_exec = recv_exec;
  p.sender_vtable = pcl_udp_transport_get_transport(send_udp);
  p.receiver_vtable = pcl_udp_transport_get_transport(recv_udp);
  p.sender_peer_id = "sender";     // identity the receiver tags on inbound
  p.receiver_peer_id = "receiver";

  // UDP is best-effort: retransmit a few times to tolerate loss.
  pcl_conformance::expectIngressRunsOnExecutorThread(
      p, "udp/tick", "Tick", "payload", /*retries=*/5);

  pcl_udp_transport_destroy(send_udp);
  pcl_udp_transport_destroy(recv_udp);
  pcl_executor_destroy(send_exec);
  pcl_executor_destroy(recv_exec);
  restore_logs();
}

TEST(PclTransportThreading, SocketIngressRunsOnExecutor) {
  silence_logs();
  const uint16_t port = pick_free_udp_port();  // any free port works for TCP too
  ASSERT_NE(port, 0);

  auto* server_exec = pcl_executor_create();
  auto* client_exec = pcl_executor_create();
  pcl_socket_transport_t* server = nullptr;
  std::thread server_thread([&] {
    server = pcl_socket_transport_create_server(port, server_exec);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto* client = pcl_socket_transport_create_client("127.0.0.1", port, client_exec);
  server_thread.join();
  ASSERT_NE(server, nullptr);
  ASSERT_NE(client, nullptr);
  pcl_socket_transport_set_peer_id(server, "client");
  pcl_socket_transport_set_peer_id(client, "server");

  // Sender = server, receiver = client; the client tags inbound as "server".
  TransportPair p;
  p.sender_exec = server_exec;
  p.receiver_exec = client_exec;
  p.sender_vtable = pcl_socket_transport_get_transport(server);
  p.receiver_vtable = pcl_socket_transport_get_transport(client);
  p.sender_peer_id = "server";
  p.receiver_peer_id = "client";

  pcl_conformance::expectIngressRunsOnExecutorThread(
      p, "sock/tick", "Tick", "payload");

  pcl_socket_transport_destroy(client);
  pcl_socket_transport_destroy(server);
  pcl_executor_destroy(client_exec);
  pcl_executor_destroy(server_exec);
  restore_logs();
}

// ===========================================================================
// Groups A + C: response callback on executor thread, not inline
// ===========================================================================

TEST(PclTransportThreading, TemplateResponseCallbackNotInline) {
  silence_logs();
  TemplatePair tp;
  pcl_conformance::expectResponseCallbackOnExecutorNotInline(
      tp.pair(), "math/echo", "ping", "pong");
  restore_logs();
}

TEST(PclTransportThreading, SharedMemoryResponseCallbackNotInline) {
  silence_logs();
  ShmPair sp(unique_bus("thr_shm_notinline"));
  ASSERT_TRUE(sp.ok());
  pcl_conformance::expectResponseCallbackOnExecutorNotInline(
      sp.pair(), "shm/echo", "ping", "pong");
  restore_logs();
}

TEST(PclTransportThreading, SocketResponseCallbackNotInline) {
  silence_logs();
  const uint16_t port = pick_free_udp_port();
  ASSERT_NE(port, 0);
  auto* server_exec = pcl_executor_create();
  auto* client_exec = pcl_executor_create();
  pcl_socket_transport_t* server = nullptr;
  std::thread server_thread([&] {
    server = pcl_socket_transport_create_server(port, server_exec);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto* client = pcl_socket_transport_create_client("127.0.0.1", port, client_exec);
  server_thread.join();
  ASSERT_NE(server, nullptr);
  ASSERT_NE(client, nullptr);
  pcl_socket_transport_set_peer_id(server, "client");
  pcl_socket_transport_set_peer_id(client, "server");

  // Full service wiring: the server dispatches inbound SVC_REQ through its
  // gateway container, and both ends register the peer transport.
  pcl_executor_set_transport(server_exec, pcl_socket_transport_get_transport(server));
  pcl_executor_set_transport(client_exec, pcl_socket_transport_get_transport(client));
  pcl_executor_register_transport(server_exec, "client",
      pcl_socket_transport_get_transport(server));
  pcl_executor_register_transport(client_exec, "server",
      pcl_socket_transport_get_transport(client));
  pcl_container_t* gw = pcl_socket_transport_gateway_container(server);
  ASSERT_NE(gw, nullptr);
  pcl_container_configure(gw);
  pcl_container_activate(gw);
  pcl_executor_add(server_exec, gw);

  // Client invokes a service the server provides.
  TransportPair p;
  p.sender_exec = client_exec;
  p.receiver_exec = server_exec;
  p.sender_vtable = pcl_socket_transport_get_transport(client);
  p.receiver_vtable = pcl_socket_transport_get_transport(server);
  p.sender_peer_id = "client";
  p.receiver_peer_id = "server";

  pcl_conformance::expectResponseCallbackOnExecutorNotInline(
      p, "svc/echo", "ping", "pong");

  pcl_socket_transport_destroy(client);
  pcl_socket_transport_destroy(server);
  pcl_executor_destroy(client_exec);
  pcl_executor_destroy(server_exec);
  restore_logs();
}

// ===========================================================================
// Group B: egress returns promptly while the real send path is blocked
// ===========================================================================

TEST(PclTransportThreading, TemplatePublishQueuesWhenSendBlocks) {
  silence_logs();
  auto* exec = pcl_executor_create();
  End end;              // standalone end; send is gated
  Link out, in;
  end.out = &out; end.in = &in;
  end.gate_armed = true;   // wedge the send hook until released

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data = &end;
  hooks.send_blocking = &end_send;
  hooks.recv_blocking = &end_recv;
  hooks.wake = &end_wake;
  auto* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);
  const pcl_transport_t* vt = pcl_transport_template_get_transport(tpl);

  pcl_msg_t msg = {};
  msg.data = "x"; msg.size = 1u; msg.type_name = "T";

  // First publish wedges the send thread; the rest must still enqueue and
  // return promptly on the executor thread.
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/0", &msg), PCL_OK);
  for (int i = 0; i < 200 && end.send_entered.load() < 1; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GE(end.send_entered.load(), 1);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/1", &msg), PCL_OK);
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/2", &msg), PCL_OK);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
            100)
      << "publish blocked behind the wedged send thread";

  // Release and drain.
  { std::lock_guard<std::mutex> lk(end.gate_mu); end.gate_release = true; }
  end.gate_cv.notify_all();

  pcl_transport_template_destroy(tpl);
  pcl_executor_destroy(exec);
  restore_logs();
}

TEST(PclTransportThreading, TemplateInvokeQueuesWhenSendBlocks) {
  silence_logs();
  auto* exec = pcl_executor_create();
  End end;
  Link out, in;
  end.out = &out; end.in = &in;
  end.gate_armed = true;

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data = &end;
  hooks.send_blocking = &end_send;
  hooks.recv_blocking = &end_recv;
  hooks.wake = &end_wake;
  auto* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);
  const pcl_transport_t* vt = pcl_transport_template_get_transport(tpl);

  std::atomic<bool> cb_fired{false};
  pcl_msg_t req = {};
  req.data = "q"; req.size = 1u; req.type_name = "T";

  const auto start = std::chrono::steady_clock::now();
  pcl_status_t rc = vt->invoke_async(
      vt->adapter_ctx, "svc/blocked", &req,
      [](const pcl_msg_t*, void* ud) { *static_cast<std::atomic<bool>*>(ud) = true; },
      &cb_fired);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(rc, PCL_OK);
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
            100)
      << "invoke_async blocked behind the wedged send thread";
  EXPECT_FALSE(cb_fired.load()) << "callback fired inline before the response";

  { std::lock_guard<std::mutex> lk(end.gate_mu); end.gate_release = true; }
  end.gate_cv.notify_all();

  pcl_transport_template_destroy(tpl);
  pcl_executor_destroy(exec);
  restore_logs();
}

TEST(PclTransportThreading, SharedMemoryPublishDoesNotSleepForBackpressure) {
  silence_logs();
  auto* exec = pcl_executor_create();
  auto* t = pcl_shared_memory_transport_create(
      unique_bus("thr_shm_bp").c_str(), "solo", exec);
  ASSERT_NE(t, nullptr);
  pcl_executor_set_transport(exec, pcl_shared_memory_transport_get_transport(t));

  // A non-zero backpressure budget means the worker owns the mailbox-capacity
  // wait; the executor-facing publish must copy and enqueue promptly.
  ASSERT_EQ(pcl_shared_memory_transport_set_topic_backpressure(t, "bp/topic", 500u),
            PCL_OK);

  const pcl_transport_t* vt = pcl_shared_memory_transport_get_transport(t);
  pcl_msg_t msg = {};
  msg.data = "x"; msg.size = 1u; msg.type_name = "T";

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "bp/topic", &msg), PCL_OK);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
            50)
      << "backpressured publish slept on the executor thread";

  pcl_shared_memory_transport_destroy(t);
  pcl_executor_destroy(exec);
  restore_logs();
}

// ===========================================================================
// Group D: destroy wakes and joins a blocked send worker
// ===========================================================================

TEST(PclTransportThreading, TemplateDestroyWakesBlockedSendWorker) {
  silence_logs();
  auto* exec = pcl_executor_create();
  End end;
  Link out, in;
  end.out = &out; end.in = &in;
  end.gate_armed = true;   // send thread wedges; only wake() releases it

  pcl_template_io_hooks_t hooks = {};
  hooks.user_data = &end;
  hooks.send_blocking = &end_send;
  hooks.recv_blocking = &end_recv;
  hooks.wake = &end_wake;
  auto* tpl = pcl_transport_template_create(&hooks, exec);
  ASSERT_NE(tpl, nullptr);
  const pcl_transport_t* vt = pcl_transport_template_get_transport(tpl);

  pcl_msg_t msg = {};
  msg.data = "x"; msg.size = 1u; msg.type_name = "T";
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/0", &msg), PCL_OK);
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t/1", &msg), PCL_OK);
  for (int i = 0; i < 200 && end.send_entered.load() < 1; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GE(end.send_entered.load(), 1)
      << "send worker never entered the gated hook";

  // Destroy must call wake() (which releases the gate) and join the worker
  // without hanging.  Run it on a helper thread guarded by a timeout so a
  // regression surfaces as a failure, not a wedged test.
  std::atomic<bool> done{false};
  std::thread destroyer([&] {
    pcl_transport_template_destroy(tpl);
    done = true;
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!done.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(done.load()) << "destroy did not wake/join the blocked send worker";
  destroyer.join();

  pcl_executor_destroy(exec);
  restore_logs();
}
