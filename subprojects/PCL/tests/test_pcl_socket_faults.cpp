/// \file test_pcl_socket_faults.cpp
/// \brief GCC linker-wrap and raw-wire coverage tests for PCL socket transport.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "Ws2_32.lib")
using test_socket_t = SOCKET;
static constexpr test_socket_t k_invalid_socket = INVALID_SOCKET;
static void close_test_socket(test_socket_t s) { closesocket(s); }
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <unistd.h>
using test_socket_t = int;
static constexpr test_socket_t k_invalid_socket = -1;
static void close_test_socket(test_socket_t s) { close(s); }
#endif

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_transport_socket.h"
#include "pcl_internal.h"
}

// -- Linker-wrap fault injection -----------------------------------------

extern "C" void* __real_malloc(size_t size);
extern "C" void* __real_calloc(size_t nmemb, size_t size);

static std::atomic<int> g_malloc_countdown{-1};
static std::atomic<int> g_calloc_countdown{-1};

static bool consume_failure(std::atomic<int>& countdown) {
  int value = countdown.load();
  while (value >= 0) {
    if (value == 0) {
      if (countdown.compare_exchange_weak(value, -1)) return true;
    } else if (countdown.compare_exchange_weak(value, value - 1)) {
      return false;
    }
  }
  return false;
}

extern "C" void* __wrap_malloc(size_t size) {
  if (consume_failure(g_malloc_countdown)) return nullptr;
  return __real_malloc(size);
}

extern "C" void* __wrap_calloc(size_t nmemb, size_t size) {
  if (consume_failure(g_calloc_countdown)) return nullptr;
  return __real_calloc(nmemb, size);
}

// -- Helpers --------------------------------------------------------------

static void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}

static void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

static void write_u16(std::vector<uint8_t>& out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value >> 8));
  out.push_back(static_cast<uint8_t>(value & 0xffu));
}

static void write_u32(std::vector<uint8_t>& out, uint32_t value) {
  out.push_back(static_cast<uint8_t>(value >> 24));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xffu));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xffu));
  out.push_back(static_cast<uint8_t>(value & 0xffu));
}

static bool send_all_raw(test_socket_t s, const uint8_t* data, size_t size) {
  size_t sent = 0;
  while (sent < size) {
#ifdef _WIN32
    int rc = send(s, reinterpret_cast<const char*>(data + sent),
                  static_cast<int>(size - sent), 0);
#else
    ssize_t rc = send(s, data + sent, size - sent, 0);
#endif
    if (rc <= 0) return false;
    sent += static_cast<size_t>(rc);
  }
  return true;
}

static bool send_frame(test_socket_t s, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> len;
  write_u32(len, static_cast<uint32_t>(payload.size()));
  return send_all_raw(s, len.data(), len.size()) &&
         send_all_raw(s, payload.data(), payload.size());
}

static uint16_t pick_free_tcp_port() {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  test_socket_t tmp = socket(AF_INET, SOCK_STREAM, 0);
  if (tmp == k_invalid_socket) return 0;

  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(tmp, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close_test_socket(tmp);
    return 0;
  }

#ifdef _WIN32
  int len = static_cast<int>(sizeof(addr));
#else
  socklen_t len = sizeof(addr);
#endif
  getsockname(tmp, reinterpret_cast<sockaddr*>(&addr), &len);
  uint16_t port = ntohs(addr.sin_port);
  close_test_socket(tmp);
  return port;
}

struct LoopbackPair {
  pcl_executor_t* server_exec = nullptr;
  pcl_executor_t* client_exec = nullptr;
  pcl_socket_transport_t* server = nullptr;
  pcl_socket_transport_t* client = nullptr;
  uint16_t port = 0;

  bool create(bool auto_reconnect = false) {
    server_exec = pcl_executor_create();
    client_exec = pcl_executor_create();
    if (!server_exec || !client_exec) return false;

    port = pick_free_tcp_port();
    if (port == 0) return false;

    std::thread server_thread([&]() {
      server = pcl_socket_transport_create_server(port, server_exec);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    pcl_socket_client_opts_t opts = {};
    opts.auto_reconnect = auto_reconnect ? 1 : 0;
    opts.connect_timeout_ms = auto_reconnect ? 5000u : 0u;
    opts.max_retries = auto_reconnect ? 50u : 0u;
    client = auto_reconnect
                 ? pcl_socket_transport_create_client_ex("127.0.0.1", port,
                                                         client_exec, &opts)
                 : pcl_socket_transport_create_client("127.0.0.1", port,
                                                       client_exec);
    server_thread.join();
    return server && client;
  }

  void destroy_server_only() {
    if (server) {
      pcl_socket_transport_destroy(server);
      server = nullptr;
    }
    if (server_exec) {
      pcl_executor_destroy(server_exec);
      server_exec = nullptr;
    }
  }

  void destroy() {
    if (client) {
      pcl_socket_transport_destroy(client);
      client = nullptr;
    }
    destroy_server_only();
    if (client_exec) {
      pcl_executor_destroy(client_exec);
      client_exec = nullptr;
    }
  }
};

class RawFrameServer {
public:
  explicit RawFrameServer(std::vector<std::vector<uint8_t>> frames)
      : frames_(std::move(frames)) {}

  bool start(uint16_t port, bool wait_for_signal = false) {
    wait_for_signal_ = wait_for_signal;
    thread_ = std::thread([this, port]() { run(port); });
    for (int i = 0; i < 200 && !ready_.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready_.load();
  }

  void release() { released_ = true; }

  void join() {
    if (thread_.joinable()) thread_.join();
  }

  bool ok() const { return ok_.load(); }

private:
  void run(uint16_t port) {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    test_socket_t listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock == k_invalid_socket) {
      ready_ = true;
      return;
    }

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (bind(listen_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
        listen(listen_sock, 1) != 0) {
      close_test_socket(listen_sock);
      ready_ = true;
      return;
    }

    ready_ = true;
    test_socket_t client = accept(listen_sock, nullptr, nullptr);
    if (client == k_invalid_socket) {
      close_test_socket(listen_sock);
      return;
    }

    if (wait_for_signal_) {
      while (!released_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    bool sent = true;
    for (const auto& frame : frames_) {
      sent = send_frame(client, frame) && sent;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

#ifdef _WIN32
    shutdown(client, SD_BOTH);
#else
    shutdown(client, SHUT_RDWR);
#endif
    close_test_socket(client);
    close_test_socket(listen_sock);
    ok_ = sent;
  }

  std::vector<std::vector<uint8_t>> frames_;
  std::thread thread_;
  bool wait_for_signal_ = false;
  std::atomic<bool> ready_{false};
  std::atomic<bool> released_{false};
  std::atomic<bool> ok_{false};
};

// -- Tests ----------------------------------------------------------------

TEST(PclSocketFaults, GatewayConfigureFailsWhenPortsAlreadyFull) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_container_t* gateway = pcl_socket_transport_gateway_container(pair.server);
  ASSERT_NE(gateway, nullptr);
  gateway->port_count = PCL_MAX_PORTS;
  EXPECT_EQ(pcl_container_configure(gateway), PCL_ERR_NOMEM);

  pair.destroy();
  restore_logs();
}

TEST(PclSocketFaults, InvokeRemoteAsyncFrameMallocFails) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1;
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, void*) {};

  // Alloc #0 = pending record, #1 = wire frame.
  g_malloc_countdown = 1;
  pcl_status_t rc =
      pcl_socket_transport_invoke_remote_async(pair.client, "svc", &req, cb, nullptr);
  g_malloc_countdown = -1;
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pair.destroy();
  restore_logs();
}

TEST(PclSocketFaults, InvokeRemoteAsyncEnqueueDataMallocFailsRollsBackPending) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1;
  req.type_name = "Req";
  auto cb = [](const pcl_msg_t*, void*) {};

  // Alloc #0 = pending record, #1 = frame, #2 = outbound node,
  // #3 = outbound frame data copy.
  g_malloc_countdown = 3;
  pcl_status_t rc =
      pcl_socket_transport_invoke_remote_async(pair.client, "svc", &req, cb, nullptr);
  g_malloc_countdown = -1;
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pair.destroy();
  restore_logs();
}

TEST(PclSocketFaults, RecvThreadDropsMalformedPublishResponseAndUnknownFrames) {
  silence_logs();
  uint16_t port = pick_free_tcp_port();
  ASSERT_NE(port, 0);

  std::vector<uint8_t> malformed_publish;
  malformed_publish.push_back(0u);
  write_u16(malformed_publish, 1u);
  malformed_publish.push_back(static_cast<uint8_t>('t'));
  write_u16(malformed_publish, 100u);
  write_u32(malformed_publish, 0u);

  std::vector<uint8_t> malformed_response;
  malformed_response.push_back(2u);
  write_u32(malformed_response, 1u);
  write_u16(malformed_response, 100u);
  write_u32(malformed_response, 0u);

  std::vector<uint8_t> unknown_type;
  unknown_type.push_back(99u);

  RawFrameServer server({malformed_publish, malformed_response, unknown_type});
  ASSERT_TRUE(server.start(port));

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  auto* client = pcl_socket_transport_create_client("127.0.0.1", port, e);
  ASSERT_NE(client, nullptr);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  pcl_socket_transport_destroy(client);
  pcl_executor_destroy(e);
  server.join();
  EXPECT_TRUE(server.ok());
  restore_logs();
}

TEST(PclSocketFaults, RecvThreadHandlesResponseTypeAllocationFailure) {
  silence_logs();
  uint16_t port = pick_free_tcp_port();
  ASSERT_NE(port, 0);

  std::vector<uint8_t> response;
  response.push_back(2u);
  write_u32(response, 1u);
  write_u16(response, 1u);
  response.push_back(static_cast<uint8_t>('T'));
  write_u32(response, 0u);

  RawFrameServer server({response});
  ASSERT_TRUE(server.start(port, true));

  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  auto* client = pcl_socket_transport_create_client("127.0.0.1", port, e);
  ASSERT_NE(client, nullptr);

  // Alloc #0 = receive payload, #1 = response type string.
  g_malloc_countdown = 1;
  server.release();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  g_malloc_countdown = -1;

  pcl_socket_transport_destroy(client);
  pcl_executor_destroy(e);
  server.join();
  EXPECT_TRUE(server.ok());
  restore_logs();
}

TEST(PclSocketFaults, ClientRetryCoversBadHostAndBackoffSleep) {
  silence_logs();
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  pcl_socket_client_opts_t opts = {};
  opts.max_retries = 1u;
  EXPECT_EQ(pcl_socket_transport_create_client_ex("256.256.256.256", 9, e, &opts),
            nullptr);

  uint16_t port = pick_free_tcp_port();
  ASSERT_NE(port, 0);
  opts.max_retries = 1u;
  auto* t = pcl_socket_transport_create_client_ex("127.0.0.1", port, e, &opts);
  EXPECT_EQ(t, nullptr);

  pcl_executor_destroy(e);
  restore_logs();
}

TEST(PclSocketFaults, AutoReconnectBackoffRunsWhileServerStaysDown) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create(true));

  pair.destroy_server_only();
  std::this_thread::sleep_for(std::chrono::milliseconds(2600));
  EXPECT_NE(pcl_socket_transport_get_state(pair.client),
            PCL_SOCKET_STATE_CONNECTED);

  pair.destroy();
  restore_logs();
}
