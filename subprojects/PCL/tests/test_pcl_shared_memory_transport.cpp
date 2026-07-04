/// \file test_pcl_shared_memory_transport.cpp
/// \brief Tests for the PCL central shared-memory transport bus.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <signal.h>
#  include <sys/types.h>
#  include <sys/wait.h>
#  include <unistd.h>
#endif

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_transport_shared_memory.h"
}

static void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}

static void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

static std::string unique_token(const char* prefix) {
#ifdef _WIN32
  DWORD pid = GetCurrentProcessId();
#else
  pid_t pid = getpid();
#endif
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), "%s_%lu_%lld",
                prefix,
                static_cast<unsigned long>(pid),
                static_cast<long long>(now));
  return buffer;
}

static std::string temp_file_path(const char* prefix) {
#ifdef _WIN32
  char temp_dir[MAX_PATH];
  char file_name[MAX_PATH];
  GetTempPathA(MAX_PATH, temp_dir);
  GetTempFileNameA(temp_dir, prefix, 0, file_name);
  DeleteFileA(file_name);
  return file_name;
#else
  std::string pattern = std::string("/tmp/") + prefix + "_XXXXXX";
  std::vector<char> path(pattern.begin(), pattern.end());
  path.push_back('\0');
  int fd = mkstemp(path.data());
  if (fd >= 0) {
    close(fd);
    unlink(path.data());
  }
  return path.data();
#endif
}

static bool file_exists(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return in.good();
}

static bool wait_for_file(const std::string& path, uint32_t timeout_ms) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (file_exists(path)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return file_exists(path);
}

static std::string read_text(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

static void remove_if_exists(const std::string& path) {
  if (path.empty()) return;
  std::remove(path.c_str());
}

class ChildProcess {
public:
  ChildProcess() = default;
  ~ChildProcess() { terminate(); }

  bool start(const std::vector<std::string>& args) {
#ifdef _WIN32
    std::string command = "\"" PCL_SHM_PEER_HELPER_PATH "\"";
    STARTUPINFOA si = {};
    PROCESS_INFORMATION pi = {};
    si.cb = sizeof(si);

    for (const auto& arg : args) {
      command += " \"" + arg + "\"";
    }
    std::vector<char> buffer(command.begin(), command.end());
    buffer.push_back('\0');

    if (!CreateProcessA(nullptr, buffer.data(), nullptr, nullptr, FALSE, 0,
                        nullptr, nullptr, &si, &pi)) {
      return false;
    }

    process_handle_ = pi.hProcess;
    thread_handle_ = pi.hThread;
    CloseHandle(thread_handle_);
    thread_handle_ = nullptr;
    return true;
#else
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(PCL_SHM_PEER_HELPER_PATH));
    for (const auto& arg : args) {
      argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);

    pid_ = fork();
    if (pid_ < 0) return false;
    if (pid_ == 0) {
      execv(PCL_SHM_PEER_HELPER_PATH, argv.data());
      _exit(127);
    }
    return true;
#endif
  }

  int wait(uint32_t timeout_ms) {
#ifdef _WIN32
    if (!process_handle_) return -1;
    DWORD wait_rc = WaitForSingleObject(process_handle_, timeout_ms);
    if (wait_rc != WAIT_OBJECT_0) return -1;
    DWORD exit_code = 0;
    GetExitCodeProcess(process_handle_, &exit_code);
    CloseHandle(process_handle_);
    process_handle_ = nullptr;
    return static_cast<int>(exit_code);
#else
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    int status = 0;
    if (pid_ <= 0) return -1;
    while (std::chrono::steady_clock::now() < deadline) {
      pid_t rc = waitpid(pid_, &status, WNOHANG);
      if (rc == pid_) {
        pid_ = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return -1;
#endif
  }

  void terminate() {
#ifdef _WIN32
    if (process_handle_) {
      TerminateProcess(process_handle_, 1);
      WaitForSingleObject(process_handle_, 1000);
      CloseHandle(process_handle_);
      process_handle_ = nullptr;
    }
#else
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      waitpid(pid_, nullptr, 0);
      pid_ = -1;
    }
#endif
  }

private:
#ifdef _WIN32
  HANDLE process_handle_ = nullptr;
  HANDLE thread_handle_ = nullptr;
#else
  pid_t pid_ = -1;
#endif
};

struct BusNode {
  pcl_executor_t*                exec = nullptr;
  pcl_shared_memory_transport_t* transport = nullptr;

  bool create(const char* bus, const char* participant) {
    exec = pcl_executor_create();
    if (!exec) return false;
    transport = pcl_shared_memory_transport_create(bus, participant, exec);
    return transport != nullptr;
  }

  bool attach_transport(bool with_gateway = false) {
    if (!exec || !transport) return false;
    if (pcl_executor_set_transport(
            exec,
            pcl_shared_memory_transport_get_transport(transport)) != PCL_OK) {
      return false;
    }
    if (!with_gateway) return true;

    pcl_container_t* gateway =
        pcl_shared_memory_transport_gateway_container(transport);
    if (!gateway) return false;
    if (pcl_container_configure(gateway) != PCL_OK) return false;
    if (pcl_container_activate(gateway) != PCL_OK) return false;
    if (pcl_executor_add(exec, gateway) != PCL_OK) return false;
    return true;
  }

  void destroy() {
    if (transport) {
      pcl_shared_memory_transport_destroy(transport);
      transport = nullptr;
    }
    if (exec) {
      pcl_executor_destroy(exec);
      exec = nullptr;
    }
  }
};

TEST(PclSharedMemoryTransport, CreateAndDestroy) {
  silence_logs();

  const std::string bus = unique_token("unit_bus");
  BusNode alpha;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));

  const pcl_transport_t* transport =
      pcl_shared_memory_transport_get_transport(alpha.transport);
  ASSERT_NE(transport, nullptr);
  EXPECT_NE(transport->publish, nullptr);
  EXPECT_NE(transport->invoke_async, nullptr);
  EXPECT_NE(transport->respond, nullptr);
  EXPECT_NE(pcl_shared_memory_transport_gateway_container(alpha.transport), nullptr);

  alpha.destroy();
  restore_logs();
}

TEST(PclSharedMemoryTransport, DuplicateParticipantIdRejectedOnSameBus) {
  silence_logs();

  const std::string bus = unique_token("dup_bus");
  BusNode alpha;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));

  pcl_executor_t* other_exec = pcl_executor_create();
  ASSERT_NE(other_exec, nullptr);
  auto* duplicate =
      pcl_shared_memory_transport_create(bus.c_str(), "alpha", other_exec);
  EXPECT_EQ(duplicate, nullptr);

  pcl_executor_destroy(other_exec);
  alpha.destroy();
  restore_logs();
}

TEST(PclSharedMemoryTransport, PublishFansOutAcrossBusParticipants) {
  silence_logs();

  const std::string bus = unique_token("fanout_bus");
  BusNode alpha;
  BusNode bravo;
  BusNode charlie;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));
  ASSERT_TRUE(bravo.create(bus.c_str(), "bravo"));
  ASSERT_TRUE(charlie.create(bus.c_str(), "charlie"));
  ASSERT_TRUE(alpha.attach_transport());
  ASSERT_TRUE(bravo.attach_transport());
  ASSERT_TRUE(charlie.attach_transport());

  struct SubState {
    std::atomic<bool> received{false};
    std::string       payload;
  };

  SubState bravo_state;
  SubState charlie_state;

  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_port_t* port = pcl_container_add_subscriber(
        c,
        "bus/topic",
        "BusMsg",
        [](pcl_container_t*, const pcl_msg_t* msg, void* state_ud) {
          auto* state = static_cast<SubState*>(state_ud);
          state->payload.assign(static_cast<const char*>(msg->data), msg->size);
          state->received = true;
        },
        ud);
    if (!port) return PCL_ERR_NOMEM;
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, nullptr, 0);
  };

  pcl_container_t* bravo_sub =
      pcl_container_create("bravo_sub", &sub_cbs, &bravo_state);
  pcl_container_t* charlie_sub =
      pcl_container_create("charlie_sub", &sub_cbs, &charlie_state);
  ASSERT_NE(bravo_sub, nullptr);
  ASSERT_NE(charlie_sub, nullptr);
  ASSERT_EQ(pcl_container_configure(bravo_sub), PCL_OK);
  ASSERT_EQ(pcl_container_activate(bravo_sub), PCL_OK);
  ASSERT_EQ(pcl_executor_add(bravo.exec, bravo_sub), PCL_OK);
  ASSERT_EQ(pcl_container_configure(charlie_sub), PCL_OK);
  ASSERT_EQ(pcl_container_activate(charlie_sub), PCL_OK);
  ASSERT_EQ(pcl_executor_add(charlie.exec, charlie_sub), PCL_OK);

  const char* payload = "fanout";
  pcl_msg_t msg = {};
  msg.data = payload;
  msg.size = static_cast<uint32_t>(strlen(payload));
  msg.type_name = "BusMsg";

  const pcl_transport_t* alpha_transport =
      pcl_shared_memory_transport_get_transport(alpha.transport);
  ASSERT_NE(alpha_transport, nullptr);
  EXPECT_EQ(alpha_transport->publish(alpha_transport->adapter_ctx, "bus/topic", &msg),
            PCL_OK);

  for (int i = 0; i < 50 && (!bravo_state.received || !charlie_state.received); ++i) {
    pcl_executor_spin_once(bravo.exec, 0);
    pcl_executor_spin_once(charlie.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(bravo_state.received);
  EXPECT_TRUE(charlie_state.received);
  EXPECT_EQ(bravo_state.payload, "fanout");
  EXPECT_EQ(charlie_state.payload, "fanout");

  pcl_executor_remove(bravo.exec, bravo_sub);
  pcl_executor_remove(charlie.exec, charlie_sub);
  pcl_container_destroy(bravo_sub);
  pcl_container_destroy(charlie_sub);
  alpha.destroy();
  bravo.destroy();
  charlie.destroy();
  restore_logs();
}

TEST(PclSharedMemoryTransport, PublishHonoursSubscriberPeerFilter) {
  silence_logs();

  const std::string bus = unique_token("filter_bus");
  BusNode alpha;
  BusNode bravo;
  BusNode charlie;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));
  ASSERT_TRUE(bravo.create(bus.c_str(), "bravo"));
  ASSERT_TRUE(charlie.create(bus.c_str(), "charlie"));
  ASSERT_TRUE(alpha.attach_transport());
  ASSERT_TRUE(bravo.attach_transport());
  ASSERT_TRUE(charlie.attach_transport());

  struct SubState {
    std::atomic<int> hits{0};
    std::string      last_payload;
  } state;

  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = {"alpha"};
    pcl_port_t* port = pcl_container_add_subscriber(
        c,
        "peer/topic",
        "PeerMsg",
        [](pcl_container_t*, const pcl_msg_t* msg, void* state_ud) {
          auto* sub_state = static_cast<SubState*>(state_ud);
          sub_state->last_payload.assign(static_cast<const char*>(msg->data),
                                         msg->size);
          sub_state->hits.fetch_add(1);
        },
        ud);
    if (!port) return PCL_ERR_NOMEM;
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  };

  pcl_container_t* bravo_sub =
      pcl_container_create("bravo_filtered_sub", &sub_cbs, &state);
  ASSERT_NE(bravo_sub, nullptr);
  ASSERT_EQ(pcl_container_configure(bravo_sub), PCL_OK);
  ASSERT_EQ(pcl_container_activate(bravo_sub), PCL_OK);
  ASSERT_EQ(pcl_executor_add(bravo.exec, bravo_sub), PCL_OK);

  auto publish_from = [](BusNode& node, const char* text) {
    pcl_msg_t msg = {};
    msg.data = text;
    msg.size = static_cast<uint32_t>(strlen(text));
    msg.type_name = "PeerMsg";
    const pcl_transport_t* transport =
        pcl_shared_memory_transport_get_transport(node.transport);
    return transport->publish(transport->adapter_ctx, "peer/topic", &msg);
  };

  EXPECT_EQ(publish_from(charlie, "ignored"), PCL_OK);
  for (int i = 0; i < 10; ++i) {
    pcl_executor_spin_once(bravo.exec, 0);
  }
  EXPECT_EQ(state.hits.load(), 0);

  EXPECT_EQ(publish_from(alpha, "accepted"), PCL_OK);
  for (int i = 0; i < 30 && state.hits.load() == 0; ++i) {
    pcl_executor_spin_once(bravo.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(state.hits.load(), 1);
  EXPECT_EQ(state.last_payload, "accepted");

  pcl_executor_remove(bravo.exec, bravo_sub);
  pcl_container_destroy(bravo_sub);
  alpha.destroy();
  bravo.destroy();
  charlie.destroy();
  restore_logs();
}

static pcl_status_t publish_text(BusNode& node,
                                 const char* topic,
                                 const char* type_name,
                                 const std::string& payload) {
  pcl_msg_t msg = {};
  msg.data = payload.data();
  msg.size = static_cast<uint32_t>(payload.size());
  msg.type_name = type_name;
  const pcl_transport_t* transport =
      pcl_shared_memory_transport_get_transport(node.transport);
  if (!transport) return PCL_ERR_INVALID;
  return transport->publish(transport->adapter_ctx, topic, &msg);
}

static bool publish_until_mailbox_full(BusNode& filler,
                                       BusNode& publisher,
                                       const char* probe_topic,
                                       const std::string& probe_payload_prefix,
                                       std::string* failed_payload = nullptr) {
  const std::string big_payload(16384, 'x');
  for (int attempt = 0; attempt < 200; ++attempt) {
    const std::string probe_payload =
        probe_payload_prefix + "_" + std::to_string(attempt);
    for (int i = 0; i < 32; ++i) {
      (void)publish_text(filler, "shm/fill", "FillMsg", big_payload);
    }
    pcl_status_t rc = publish_text(publisher, probe_topic, "ProbeMsg", probe_payload);
    if (rc == PCL_ERR_NOMEM) {
      if (failed_payload) {
        *failed_payload = probe_payload;
      }
      return true;
    }
  }
  return false;
}

///< REQ_PCL_211: Shared-memory publish fan-out is all-or-nothing. PCL.036c, PCL.036g.
TEST(PclSharedMemoryTransport, PublishFanoutIsAtomicWhenAnyMailboxIsFull) {
  silence_logs();

  const std::string bus = unique_token("atomic_bus");
  BusNode alpha;
  BusNode bravo;
  BusNode charlie;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));
  ASSERT_TRUE(bravo.create(bus.c_str(), "bravo"));
  ASSERT_TRUE(charlie.create(bus.c_str(), "charlie"));
  ASSERT_TRUE(alpha.attach_transport());
  ASSERT_TRUE(bravo.attach_transport());
  ASSERT_TRUE(charlie.attach_transport());

  std::string failed_payload;
  ASSERT_TRUE(publish_until_mailbox_full(charlie,
                                         alpha,
                                         "atomic/topic",
                                         "atomic_sentinel",
                                         &failed_payload));

  struct SubState {
    std::atomic<bool> received_sentinel{false};
    std::string expected_payload;
  } state;
  state.expected_payload = failed_payload;

  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = {"alpha"};
    pcl_port_t* port = pcl_container_add_subscriber(
        c,
        "atomic/topic",
        "AtomicMsg",
        [](pcl_container_t*, const pcl_msg_t* msg, void* state_ud) {
          auto* sub_state = static_cast<SubState*>(state_ud);
          std::string payload(static_cast<const char*>(msg->data), msg->size);
          if (payload == sub_state->expected_payload) {
            sub_state->received_sentinel = true;
          }
        },
        ud);
    if (!port) return PCL_ERR_NOMEM;
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  };

  pcl_container_t* charlie_sub =
      pcl_container_create("charlie_atomic_sub", &sub_cbs, &state);
  ASSERT_NE(charlie_sub, nullptr);
  ASSERT_EQ(pcl_container_configure(charlie_sub), PCL_OK);
  ASSERT_EQ(pcl_container_activate(charlie_sub), PCL_OK);
  ASSERT_EQ(pcl_executor_add(charlie.exec, charlie_sub), PCL_OK);

  for (int i = 0; i < 100 && !state.received_sentinel; ++i) {
    pcl_executor_spin_once(charlie.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_FALSE(state.received_sentinel);

  pcl_executor_remove(charlie.exec, charlie_sub);
  pcl_container_destroy(charlie_sub);
  alpha.destroy();
  bravo.destroy();
  charlie.destroy();
  restore_logs();
}

///< REQ_PCL_212: Shared-memory topic backpressure can wait for mailbox capacity. PCL.036g.
TEST(PclSharedMemoryTransport, TopicBackpressureWaitsForMailboxCapacity) {
  silence_logs();

  const std::string bus = unique_token("backpressure_bus");
  BusNode alpha;
  BusNode bravo;
  BusNode charlie;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));
  ASSERT_TRUE(bravo.create(bus.c_str(), "bravo"));
  ASSERT_TRUE(charlie.create(bus.c_str(), "charlie"));
  ASSERT_TRUE(alpha.attach_transport());
  ASSERT_TRUE(bravo.attach_transport());
  ASSERT_TRUE(charlie.attach_transport());

  ASSERT_TRUE(publish_until_mailbox_full(charlie, alpha, "pressure/topic", "no_wait"));

  ASSERT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, "pressure/topic", 500),
            PCL_OK);

  ASSERT_TRUE(publish_until_mailbox_full(charlie,
                                         alpha,
                                         "generic/topic",
                                         "generic_no_wait"));

  EXPECT_EQ(publish_text(alpha, "pressure/topic", "PressureMsg", "wait"),
            PCL_OK);

  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, "pressure/topic", 0),
            PCL_OK);
  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, nullptr, 100),
            PCL_ERR_INVALID);

  alpha.destroy();
  bravo.destroy();
  charlie.destroy();
  restore_logs();
}

TEST(PclSharedMemoryTransport, AsyncDeferredServiceRoundTrip) {
  silence_logs();

  const std::string bus = unique_token("svc_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  struct ServiceState {
    std::atomic<bool> invoked{false};
    pcl_svc_context_t* saved_ctx = nullptr;
    std::string request_payload;
    std::string request_type_name;
  } service_state;

  pcl_callbacks_t svc_cbs = {};
  svc_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = {"client"};
    pcl_port_t* port = pcl_container_add_service(
        c,
        "echo_service",
        "EchoReq",
        [](pcl_container_t*, const pcl_msg_t* req, pcl_msg_t*,
           pcl_svc_context_t* ctx, void* state_ud) -> pcl_status_t {
          auto* state = static_cast<ServiceState*>(state_ud);
          state->invoked = true;
          if (req->data && req->size > 0u) {
            state->request_payload.assign(static_cast<const char*>(req->data),
                                          req->size);
          }
          if (req->type_name) {
            state->request_type_name = req->type_name;
          }
          state->saved_ctx = ctx;
          return PCL_PENDING;
        },
        ud);
    if (!port) return PCL_ERR_NOMEM;
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  };

  pcl_container_t* svc =
      pcl_container_create("echo_server", &svc_cbs, &service_state);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "echo_service";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  struct ResponseState {
    std::atomic<bool> received{false};
    std::string payload;
    std::string type_name;
  } response_state;

  const char* req_data = "ping";
  pcl_msg_t req = {};
  req.data = req_data;
  req.size = static_cast<uint32_t>(strlen(req_data));
  req.type_name = "application/protobuf";

  ASSERT_EQ(
      pcl_executor_invoke_async(
          client.exec,
          "echo_service",
          &req,
          [](const pcl_msg_t* resp, void* ud) {
            auto* state = static_cast<ResponseState*>(ud);
            if (resp && resp->data && resp->size > 0u) {
              state->payload.assign(static_cast<const char*>(resp->data),
                                    resp->size);
            }
            if (resp && resp->type_name) {
              state->type_name = resp->type_name;
            }
            state->received = true;
          },
          &response_state),
      PCL_OK);

  for (int i = 0; i < 50 && !service_state.invoked; ++i) {
    pcl_executor_spin_once(server.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(service_state.invoked);
  ASSERT_NE(service_state.saved_ctx, nullptr);
  EXPECT_EQ(service_state.request_payload, "ping");
  EXPECT_EQ(service_state.request_type_name, "application/protobuf");

  pcl_msg_t resp = {};
  resp.data = "pong";
  resp.size = 4;
  resp.type_name = "application/protobuf";
  ASSERT_EQ(pcl_service_respond(service_state.saved_ctx, &resp), PCL_OK);
  service_state.saved_ctx = nullptr;

  for (int i = 0; i < 50 && !response_state.received; ++i) {
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(response_state.received);
  EXPECT_EQ(response_state.payload, "pong");
  EXPECT_EQ(response_state.type_name, "application/protobuf");

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server.destroy();
  restore_logs();
}

///< REQ_PCL_186: Inter-process publish parent to child delivered. PCL.036b, PCL.036c.
TEST(PclSharedMemoryTransport, InterProcessPublishParentToChildDelivered) {
  silence_logs();

  const std::string bus = unique_token("shm_bus_parent_child");
  const std::string ready_file = temp_file_path("shmrdy");
  const std::string result_file = temp_file_path("shmres");

  BusNode parent;
  ChildProcess child;
  ASSERT_TRUE(parent.create(bus.c_str(), "parent_pub"));
  ASSERT_TRUE(parent.attach_transport());

  ASSERT_TRUE(child.start({
      "--mode=subscriber",
      "--bus=" + bus,
      "--participant=child_sub",
      "--topic=ipc/topic",
      "--type=IpcMsg",
      "--allowed-peer=parent_pub",
      "--ready-file=" + ready_file,
      "--result-file=" + result_file,
      "--timeout-ms=5000"}));
  ASSERT_TRUE(wait_for_file(ready_file, 5000));

  pcl_msg_t msg = {};
  msg.data = "cross-process";
  msg.size = 13;
  msg.type_name = "IpcMsg";
  const pcl_transport_t* transport =
      pcl_shared_memory_transport_get_transport(parent.transport);
  ASSERT_NE(transport, nullptr);
  EXPECT_EQ(transport->publish(transport->adapter_ctx, "ipc/topic", &msg), PCL_OK);

  EXPECT_EQ(child.wait(5000), 0);
  EXPECT_EQ(read_text(result_file), "cross-process");

  remove_if_exists(ready_file);
  remove_if_exists(result_file);
  parent.destroy();
  restore_logs();
}

///< REQ_PCL_187: Inter-process publish child to parent delivered. PCL.036b, PCL.036c.
TEST(PclSharedMemoryTransport, InterProcessPublishChildToParentDelivered) {
  silence_logs();

  const std::string bus = unique_token("shm_bus_child_parent");
  BusNode parent;
  ChildProcess child;

  struct SubState {
    std::atomic<bool> received{false};
    std::string payload;
  } state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = {"child_pub"};
    pcl_port_t* port = pcl_container_add_subscriber(
        c,
        "ipc/topic",
        "IpcMsg",
        [](pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
          auto* sub = static_cast<SubState*>(user_data);
          sub->payload.assign(static_cast<const char*>(msg->data), msg->size);
          sub->received = true;
        },
        ud);
    if (!port) return PCL_ERR_NOMEM;
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  };

  ASSERT_TRUE(parent.create(bus.c_str(), "parent_sub"));
  ASSERT_TRUE(parent.attach_transport());

  pcl_container_t* sub = pcl_container_create("parent_subscriber", &cbs, &state);
  ASSERT_NE(sub, nullptr);
  ASSERT_EQ(pcl_container_configure(sub), PCL_OK);
  ASSERT_EQ(pcl_container_activate(sub), PCL_OK);
  ASSERT_EQ(pcl_executor_add(parent.exec, sub), PCL_OK);

  ASSERT_TRUE(child.start({
      "--mode=publisher",
      "--bus=" + bus,
      "--participant=child_pub",
      "--topic=ipc/topic",
      "--type=IpcMsg",
      "--payload=from-child",
      "--delay-ms=200"}));

  for (int i = 0; i < 100 && !state.received; ++i) {
    pcl_executor_spin_once(parent.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(child.wait(5000), 0);
  EXPECT_TRUE(state.received);
  EXPECT_EQ(state.payload, "from-child");

  pcl_executor_remove(parent.exec, sub);
  pcl_container_destroy(sub);
  parent.destroy();
  restore_logs();
}

///< REQ_PCL_188: Inter-process async service round trip succeeds. PCL.036b, PCL.036d.
TEST(PclSharedMemoryTransport, InterProcessAsyncServiceRoundTrip) {
  silence_logs();

  const std::string bus = unique_token("shm_bus_service");
  const std::string ready_file = temp_file_path("shmsrv");
  const std::string result_file = temp_file_path("shmsvc");
  BusNode client;
  ChildProcess child;

  ASSERT_TRUE(client.create(bus.c_str(), "parent_client"));
  ASSERT_TRUE(client.attach_transport());

  ASSERT_TRUE(child.start({
      "--mode=service",
      "--bus=" + bus,
      "--participant=child_service",
      "--service=echo_service",
      "--request-type=EchoReq",
      "--response=pong",
      "--response-type=application/protobuf",
      "--allowed-peer=parent_client",
      "--ready-file=" + ready_file,
      "--result-file=" + result_file,
      "--timeout-ms=5000"}));
  ASSERT_TRUE(wait_for_file(ready_file, 5000));

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "echo_service";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  struct RespState {
    std::atomic<bool> received{false};
    std::string payload;
    std::string type_name;
  } response;

  pcl_msg_t req = {};
  req.data = "ping";
  req.size = 4;
  req.type_name = "application/protobuf";

  ASSERT_EQ(
      pcl_executor_invoke_async(
          client.exec,
          "echo_service",
          &req,
          [](const pcl_msg_t* resp, void* ud) {
            auto* state = static_cast<RespState*>(ud);
            if (resp && resp->data && resp->size > 0u) {
              state->payload.assign(static_cast<const char*>(resp->data), resp->size);
            }
            if (resp && resp->type_name) {
              state->type_name = resp->type_name;
            }
            state->received = true;
          },
          &response),
      PCL_OK);

  for (int i = 0; i < 100 && !response.received; ++i) {
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_EQ(child.wait(5000), 0);
  EXPECT_TRUE(response.received);
  EXPECT_EQ(response.payload, "pong");
  EXPECT_EQ(response.type_name, "application/protobuf");

  {
    const std::string helper_result = read_text(result_file);
    const std::string expected = std::string("ping\napplication/protobuf");
    EXPECT_EQ(helper_result, expected);
  }

  remove_if_exists(ready_file);
  remove_if_exists(result_file);
  client.destroy();
  restore_logs();
}

///< REQ_PCL_189: Shared-memory invoke async without provider returns not found. PCL.036d, PCL.045.
TEST(PclSharedMemoryTransport, InvokeAsyncReturnsNotFoundWithoutProvider) {
  silence_logs();

  BusNode client;
  ASSERT_TRUE(client.create("missing-svc-bus", "client"));
  ASSERT_TRUE(client.attach_transport(true));

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "missing_service";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1;
  req.type_name = "MissingReq";

  EXPECT_EQ(
      pcl_executor_invoke_async(client.exec,
                                "missing_service",
                                &req,
                                [](const pcl_msg_t*, void*) {},
                                nullptr),
      PCL_ERR_NOT_FOUND);

  client.destroy();
  restore_logs();
}

namespace {

struct StreamServerState {
  std::atomic<bool>     handler_invoked{false};
  pcl_stream_context_t* saved_ctx = nullptr;
  std::string           request_payload;
};

struct StreamClientState {
  std::vector<std::string> frames;
  std::atomic<bool>        ended{false};
  pcl_status_t             final_status = PCL_OK;
};

void spin_both_until(BusNode& a, BusNode& b,
                     const std::function<bool()>& done,
                     int max_iters = 200) {
  for (int i = 0; i < max_iters && !done(); ++i) {
    pcl_executor_spin_once(a.exec, 0);
    pcl_executor_spin_once(b.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void install_stream_service(BusNode& node,
                            const char*        service_name,
                            const char*        allowed_peer,
                            StreamServerState& state,
                            pcl_container_t**  out_container) {
  struct ConfigContext {
    const char*        service_name;
    const char*        allowed_peer;
    StreamServerState* state;
  };
  static thread_local ConfigContext config{};
  config.service_name = service_name;
  config.allowed_peer = allowed_peer;
  config.state        = &state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* cfg = &config;
    pcl_port_t* port = pcl_container_add_stream_service(
        c, cfg->service_name, "StreamReq",
        [](pcl_container_t*, const pcl_msg_t* req,
           pcl_stream_context_t* sctx, void* state_ud) -> pcl_status_t {
          auto* s = static_cast<StreamServerState*>(state_ud);
          s->saved_ctx = sctx;
          if (req && req->data && req->size > 0u) {
            s->request_payload.assign(static_cast<const char*>(req->data),
                                      req->size);
          }
          s->handler_invoked = true;
          return PCL_STREAMING;
        },
        ud);
    if (!port) return PCL_ERR_NOMEM;
    const char* peers[] = {cfg->allowed_peer};
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  };

  *out_container = pcl_container_create("stream_server_node", &cbs, &state);
  ASSERT_NE(*out_container, nullptr);
  ASSERT_EQ(pcl_container_configure(*out_container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(*out_container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(node.exec, *out_container), PCL_OK);
}

}  // namespace

///< Stream round-trip across the shared-memory bus (in-process two-executor).
TEST(PclSharedMemoryTransport, StreamRoundTripSendsAndEnds) {
  silence_logs();

  const std::string bus = unique_token("shm_bus_stream");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "stream_client"));
  ASSERT_TRUE(server.create(bus.c_str(), "stream_server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  StreamServerState server_state;
  pcl_container_t*  svc = nullptr;
  install_stream_service(server, "shm_echo_stream", "stream_client",
                         server_state, &svc);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "shm_echo_stream";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode    = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  StreamClientState client_state;
  pcl_stream_context_t* client_ctx = nullptr;

  pcl_msg_t req = {};
  req.data      = "go";
  req.size      = 2;
  req.type_name = "StreamReq";

  auto cb = [](const pcl_msg_t* msg, bool end, pcl_status_t status, void* ud) {
    auto* s = static_cast<StreamClientState*>(ud);
    if (!end && msg && msg->data && msg->size > 0u) {
      s->frames.emplace_back(static_cast<const char*>(msg->data), msg->size);
    }
    if (end) {
      s->ended        = true;
      s->final_status = status;
    }
  };

  ASSERT_EQ(pcl_executor_invoke_stream(client.exec, "shm_echo_stream",
                                       &req, cb, &client_state, &client_ctx),
            PCL_STREAMING);

  spin_both_until(client, server,
                  [&] { return server_state.handler_invoked.load(); });
  ASSERT_TRUE(server_state.handler_invoked);
  ASSERT_NE(server_state.saved_ctx, nullptr);
  EXPECT_EQ(server_state.request_payload, "go");

  const char* payloads[] = {"one", "two", "three"};
  for (const char* text : payloads) {
    pcl_msg_t out = {};
    out.data      = text;
    out.size      = static_cast<uint32_t>(strlen(text));
    out.type_name = "StreamFrame";
    EXPECT_EQ(pcl_stream_send(server_state.saved_ctx, &out), PCL_OK);
  }

  spin_both_until(client, server,
                  [&] { return client_state.frames.size() == 3u; });
  ASSERT_EQ(client_state.frames.size(), 3u);
  EXPECT_EQ(client_state.frames[0], "one");
  EXPECT_EQ(client_state.frames[1], "two");
  EXPECT_EQ(client_state.frames[2], "three");

  EXPECT_EQ(pcl_stream_end(server_state.saved_ctx), PCL_OK);
  spin_both_until(client, server, [&] { return client_state.ended.load(); });
  EXPECT_TRUE(client_state.ended);
  EXPECT_EQ(client_state.final_status, PCL_OK);

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  server.destroy();
  client.destroy();
  restore_logs();
}

///< Client-side stream cancel propagates to the shared-memory server context.
TEST(PclSharedMemoryTransport, StreamCancelAfterThreeFramesReachesServer) {
  silence_logs();

  const std::string bus = unique_token("shm_bus_stream_cancel");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "stream_client"));
  ASSERT_TRUE(server.create(bus.c_str(), "stream_server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  StreamServerState server_state;
  pcl_container_t*  svc = nullptr;
  install_stream_service(server, "cancel_stream", "stream_client",
                         server_state, &svc);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "cancel_stream";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode    = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  struct CancelClientState {
    std::vector<std::string> frames;
    std::atomic<bool>        cancel_called{false};
    std::atomic<bool>        ended{false};
    pcl_status_t             cancel_status = PCL_OK;
    pcl_status_t             final_status = PCL_OK;
    pcl_stream_context_t*    ctx = nullptr;
  } client_state;

  pcl_msg_t req = {};
  req.data      = "go";
  req.size      = 2;
  req.type_name = "StreamReq";

  auto cb = [](const pcl_msg_t* msg, bool end, pcl_status_t status, void* ud) {
    auto* s = static_cast<CancelClientState*>(ud);
    if (!end && msg && msg->data && msg->size > 0u) {
      s->frames.emplace_back(static_cast<const char*>(msg->data), msg->size);
      if (s->frames.size() == 3u) {
        s->cancel_status = pcl_stream_cancel(s->ctx);
        s->cancel_called = true;
      }
    }
    if (end) {
      s->ended = true;
      s->final_status = status;
    }
  };

  ASSERT_EQ(pcl_executor_invoke_stream(client.exec, "cancel_stream",
                                       &req, cb, &client_state,
                                       &client_state.ctx),
            PCL_STREAMING);
  ASSERT_NE(client_state.ctx, nullptr);

  spin_both_until(client, server,
                  [&] { return server_state.handler_invoked.load(); });
  ASSERT_TRUE(server_state.handler_invoked);
  ASSERT_NE(server_state.saved_ctx, nullptr);

  const char* payloads[] = {"one", "two", "three"};
  for (const char* text : payloads) {
    pcl_msg_t out = {};
    out.data      = text;
    out.size      = static_cast<uint32_t>(strlen(text));
    out.type_name = "StreamFrame";
    EXPECT_EQ(pcl_stream_send(server_state.saved_ctx, &out), PCL_OK);
  }

  spin_both_until(client, server,
                  [&] { return client_state.cancel_called.load(); });
  ASSERT_TRUE(client_state.cancel_called);
  EXPECT_EQ(client_state.cancel_status, PCL_OK);
  ASSERT_EQ(client_state.frames.size(), 3u);
  EXPECT_EQ(client_state.frames[0], "one");
  EXPECT_EQ(client_state.frames[1], "two");
  EXPECT_EQ(client_state.frames[2], "three");

  spin_both_until(client, server, [&] {
    return pcl_stream_is_cancelled(server_state.saved_ctx);
  });
  EXPECT_TRUE(pcl_stream_is_cancelled(server_state.saved_ctx));

  pcl_msg_t after_cancel = {};
  after_cancel.data      = "four";
  after_cancel.size      = 4;
  after_cancel.type_name = "StreamFrame";
  EXPECT_EQ(pcl_stream_send(server_state.saved_ctx, &after_cancel),
            PCL_ERR_CANCELLED);

  spin_both_until(client, server, [&] { return client_state.ended.load(); });
  EXPECT_TRUE(client_state.ended);
  EXPECT_EQ(client_state.final_status, PCL_ERR_CANCELLED);

  EXPECT_EQ(pcl_stream_abort(server_state.saved_ctx, PCL_ERR_CANCELLED),
            PCL_OK);
  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  server.destroy();
  client.destroy();
  restore_logs();
}

///< Stream invoked against a missing service returns NOT_FOUND.
TEST(PclSharedMemoryTransport, StreamReturnsNotFoundWithoutProvider) {
  silence_logs();

  BusNode client;
  ASSERT_TRUE(client.create(unique_token("shm_stream_none").c_str(),
                            "stream_only_client"));
  ASSERT_TRUE(client.attach_transport(true));

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "missing_stream";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode    = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  pcl_msg_t req = {};
  req.data      = "x";
  req.size      = 1;
  req.type_name = "StreamReq";

  EXPECT_EQ(
      pcl_executor_invoke_stream(client.exec, "missing_stream", &req,
                                 [](const pcl_msg_t*, bool, pcl_status_t, void*) {},
                                 nullptr, nullptr),
      PCL_ERR_NOT_FOUND);

  client.destroy();
  restore_logs();
}

///< Server-side stream aborted by the handler propagates the status to the client.
TEST(PclSharedMemoryTransport, StreamAbortPropagatesStatus) {
  silence_logs();

  const std::string bus = unique_token("shm_bus_stream_abort");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "stream_client"));
  ASSERT_TRUE(server.create(bus.c_str(), "stream_server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  StreamServerState server_state;
  pcl_container_t*  svc = nullptr;
  install_stream_service(server, "abort_stream", "stream_client",
                         server_state, &svc);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "abort_stream";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode    = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  StreamClientState client_state;
  pcl_msg_t req = {};
  req.data      = "go";
  req.size      = 2;
  req.type_name = "StreamReq";

  auto cb = [](const pcl_msg_t*, bool end, pcl_status_t status, void* ud) {
    auto* s = static_cast<StreamClientState*>(ud);
    if (end) {
      s->ended        = true;
      s->final_status = status;
    }
  };

  ASSERT_EQ(pcl_executor_invoke_stream(client.exec, "abort_stream",
                                       &req, cb, &client_state, nullptr),
            PCL_STREAMING);

  spin_both_until(client, server,
                  [&] { return server_state.handler_invoked.load(); });
  ASSERT_TRUE(server_state.handler_invoked);
  ASSERT_NE(server_state.saved_ctx, nullptr);

  EXPECT_EQ(pcl_stream_abort(server_state.saved_ctx, PCL_ERR_CALLBACK), PCL_OK);
  spin_both_until(client, server, [&] { return client_state.ended.load(); });
  EXPECT_TRUE(client_state.ended);
  EXPECT_EQ(client_state.final_status, PCL_ERR_CALLBACK);

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  server.destroy();
  client.destroy();
  restore_logs();
}

///< Destroy mid-stream aborts pending streams with PCL_ERR_CANCELLED.
TEST(PclSharedMemoryTransport, DestroyDuringActiveStreamAbortsServer) {
  silence_logs();

  const std::string bus = unique_token("shm_bus_stream_destroy");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "stream_client"));
  ASSERT_TRUE(server.create(bus.c_str(), "stream_server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  StreamServerState server_state;
  pcl_container_t*  svc = nullptr;
  install_stream_service(server, "destroy_stream", "stream_client",
                         server_state, &svc);

  pcl_endpoint_route_t route = {};
  route.endpoint_name = "destroy_stream";
  route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  route.route_mode    = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &route), PCL_OK);

  StreamClientState client_state;
  pcl_msg_t req = {};
  req.data      = "x";
  req.size      = 1;
  req.type_name = "StreamReq";

  auto cb = [](const pcl_msg_t*, bool end, pcl_status_t status, void* ud) {
    auto* s = static_cast<StreamClientState*>(ud);
    if (end) {
      s->ended        = true;
      s->final_status = status;
    }
  };

  ASSERT_EQ(pcl_executor_invoke_stream(client.exec, "destroy_stream",
                                       &req, cb, &client_state, nullptr),
            PCL_STREAMING);

  spin_both_until(client, server,
                  [&] { return server_state.handler_invoked.load(); });
  ASSERT_TRUE(server_state.handler_invoked);

  // Tear down the server while the stream is still open; the SHM transport
  // must send STREAM_END(PCL_ERR_CANCELLED) to the client before it goes away.
  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  server.destroy();

  spin_both_until(client, server, [&] { return client_state.ended.load(); });
  EXPECT_TRUE(client_state.ended);
  EXPECT_EQ(client_state.final_status, PCL_ERR_CANCELLED);

  client.destroy();
  restore_logs();
}

///< REQ_PCL_305: the shm vtable subscribe and shutdown entries are no-ops
///< (interest and teardown are managed through the bus region itself).
TEST(PclSharedMemoryTransport, SubscribeAndShutdownVtableAreNoOps) {
  silence_logs();
  const std::string bus = unique_token("noop_bus");
  BusNode alpha;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));

  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(alpha.transport);
  ASSERT_NE(vt, nullptr);
  ASSERT_NE(vt->subscribe, nullptr);
  EXPECT_EQ(vt->subscribe(vt->adapter_ctx, "topic", "Type"), PCL_OK);
  ASSERT_NE(vt->shutdown, nullptr);
  vt->shutdown(vt->adapter_ctx);

  alpha.destroy();
  restore_logs();
}

///< REQ_PCL_212: backpressure policies can be updated in place, cleared with
///< a zero timeout (including for topics never configured), and the fixed
///< policy table rejects overflow with PCL_ERR_NOMEM.
TEST(PclSharedMemoryTransport, TopicBackpressureConfigLifecycle) {
  silence_logs();
  const std::string bus = unique_token("bp_bus");
  BusNode alpha;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));

  // New policy, then in-place timeout update, then clear.
  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, "cmd/topic", 50u),
            PCL_OK);
  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, "cmd/topic", 75u),
            PCL_OK);
  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, "cmd/topic", 0u),
            PCL_OK);
  // Clearing a topic that has no policy is an accepted no-op.
  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, "never/configured", 0u),
            PCL_OK);

  // Fill every policy slot; one more distinct topic must fail closed.
  for (unsigned i = 0; i < 16u; ++i) {
    char topic[32];
    std::snprintf(topic, sizeof(topic), "bp/topic_%u", i);
    EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                  alpha.transport, topic, 10u),
              PCL_OK);
  }
  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, "bp/one_too_many", 10u),
            PCL_ERR_NOMEM);

  alpha.destroy();
  restore_logs();
}

///< REQ_PCL_326: a publisher with no explicit endpoint route publishing via
///< the default shm transport uses the legacy default route modes without
///< error, and local subscribers still receive the message.
TEST(PclSharedMemoryTransport, PublishWithoutRoutesUsesDefaultModes) {
  silence_logs();
  const std::string bus = unique_token("defroute_bus");
  BusNode alpha;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));

  ASSERT_EQ(pcl_executor_set_transport(
                alpha.exec,
                pcl_shared_memory_transport_get_transport(alpha.transport)),
            PCL_OK);

  struct State {
    pcl_port_t* pub = nullptr;
  } state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) {
    auto* s = static_cast<State*>(ud);
    s->pub = pcl_container_add_publisher(c, "defroute/topic", "Type");
    return s->pub ? PCL_OK : PCL_ERR_CALLBACK;
  };

  pcl_container_t* c = pcl_container_create("defroute_pub", &cbs, &state);
  ASSERT_NE(c, nullptr);
  ASSERT_EQ(pcl_executor_add(alpha.exec, c), PCL_OK);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);

  pcl_msg_t msg = {};
  msg.data = "hello";
  msg.size = 5u;
  msg.type_name = "Type";
  // No endpoint route installed: the publish path falls back to the legacy
  // default (remote via default transport for publishers).  Alpha is alone
  // on the bus, so the remote fan-out finds no target mailbox.
  EXPECT_EQ(pcl_port_publish(state.pub, &msg), PCL_ERR_NOT_FOUND);

  pcl_container_shutdown(c);
  pcl_executor_remove(alpha.exec, c);
  pcl_container_destroy(c);
  alpha.destroy();
  restore_logs();
}

// ---------------------------------------------------------------------------
// Additional coverage-driven shm scenarios
// ---------------------------------------------------------------------------

namespace {

// A service container whose single unary service is remote-routed with an
// explicit peer allow-list and a caller-selected handler behaviour.
struct FilteredSvcState {
  const char* service_name = nullptr;
  const char* allowed_peer = nullptr;
  pcl_status_t handler_rc  = PCL_OK;
  std::atomic<int> handler_calls{0};
};

pcl_status_t filtered_svc_configure(pcl_container_t* c, void* ud) {
  auto* st = static_cast<FilteredSvcState*>(ud);
  pcl_port_t* port = pcl_container_add_service(
      c, st->service_name, "Req",
      [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t* resp,
         pcl_svc_context_t*, void* state_ud) -> pcl_status_t {
        auto* state = static_cast<FilteredSvcState*>(state_ud);
        state->handler_calls.fetch_add(1);
        if (state->handler_rc == PCL_OK && resp) {
          resp->data = "ok";
          resp->size = 2u;
          resp->type_name = "Resp";
        }
        return state->handler_rc;
      },
      ud);
  if (!port) return PCL_ERR_NOMEM;
  const char* peers[] = {st->allowed_peer};
  return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
}

struct RespState {
  std::atomic<bool> received{false};
  uint32_t size = 0u;
};

void resp_recorder(const pcl_msg_t* resp, void* ud) {
  auto* st = static_cast<RespState*>(ud);
  st->size = resp ? resp->size : 0u;
  st->received = true;
}

}  // namespace

///< REQ_PCL_306: a remote service request from a peer outside the provider's
///< allow-list is answered with an empty response instead of dispatching the
///< handler.
TEST(PclSharedMemoryTransport, ServicePeerFilterYieldsEmptyResponse) {
  silence_logs();
  const std::string bus = unique_token("svcfilter_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  FilteredSvcState svc_state;
  svc_state.service_name = "locked_service";
  svc_state.allowed_peer = "someone_else";  // client is NOT allowed

  pcl_callbacks_t cbs = {};
  cbs.on_configure = filtered_svc_configure;
  pcl_container_t* svc =
      pcl_container_create("locked_server", &cbs, &svc_state);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  // Route-entry-based allow-list on the provider side too, so the executor
  // route table (not just the port) drives the peer decision.
  const char* entry_peers[] = {"someone_else"};
  pcl_endpoint_route_t provided_route = {};
  provided_route.endpoint_name = "locked_service";
  provided_route.endpoint_kind = PCL_ENDPOINT_PROVIDED;
  provided_route.route_mode = PCL_ROUTE_REMOTE;
  provided_route.peer_ids = entry_peers;
  provided_route.peer_count = 1;
  ASSERT_EQ(pcl_executor_set_endpoint_route(server.exec, &provided_route),
            PCL_OK);

  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "locked_service";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);

  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  ASSERT_EQ(pcl_executor_invoke_async(client.exec, "locked_service", &req,
                                      resp_recorder, &resp_state),
            PCL_OK);

  for (int i = 0; i < 100 && !resp_state.received; ++i) {
    pcl_executor_spin_once(server.exec, 0);
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_TRUE(resp_state.received);
  EXPECT_EQ(resp_state.size, 0u);              // empty response
  EXPECT_EQ(svc_state.handler_calls.load(), 0); // handler never dispatched

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server.destroy();
  restore_logs();
}

///< REQ_PCL_307: a remote unary handler failure is converted into an empty
///< response so the caller is never left waiting.
TEST(PclSharedMemoryTransport, ServiceHandlerErrorYieldsEmptyResponse) {
  silence_logs();
  const std::string bus = unique_token("svcerr_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  FilteredSvcState svc_state;
  svc_state.service_name = "failing_service";
  svc_state.allowed_peer = "client";
  svc_state.handler_rc = PCL_ERR_CALLBACK;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = filtered_svc_configure;
  pcl_container_t* svc =
      pcl_container_create("failing_server", &cbs, &svc_state);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "failing_service";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);

  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  ASSERT_EQ(pcl_executor_invoke_async(client.exec, "failing_service", &req,
                                      resp_recorder, &resp_state),
            PCL_OK);

  for (int i = 0; i < 100 && !resp_state.received; ++i) {
    pcl_executor_spin_once(server.exec, 0);
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_TRUE(resp_state.received);
  EXPECT_EQ(resp_state.size, 0u);
  EXPECT_EQ(svc_state.handler_calls.load(), 1);

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server.destroy();
  restore_logs();
}

///< REQ_PCL_308: two providers advertising the same service make discovery
///< ambiguous; unary and streaming invocation both fail closed.
TEST(PclSharedMemoryTransport, AmbiguousProviderFailsClosed) {
  silence_logs();
  const std::string bus = unique_token("ambig_bus");
  BusNode client;
  BusNode server_a;
  BusNode server_b;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server_a.create(bus.c_str(), "server_a"));
  ASSERT_TRUE(server_b.create(bus.c_str(), "server_b"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server_a.attach_transport(true));
  ASSERT_TRUE(server_b.attach_transport(true));

  FilteredSvcState state_a;
  state_a.service_name = "dup_service";
  state_a.allowed_peer = "client";
  FilteredSvcState state_b;
  state_b.service_name = "dup_service";
  state_b.allowed_peer = "client";

  pcl_callbacks_t cbs = {};
  cbs.on_configure = filtered_svc_configure;
  pcl_container_t* ca = pcl_container_create("dup_a", &cbs, &state_a);
  pcl_container_t* cb = pcl_container_create("dup_b", &cbs, &state_b);
  ASSERT_NE(ca, nullptr);
  ASSERT_NE(cb, nullptr);
  ASSERT_EQ(pcl_container_configure(ca), PCL_OK);
  ASSERT_EQ(pcl_container_activate(ca), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server_a.exec, ca), PCL_OK);
  ASSERT_EQ(pcl_container_configure(cb), PCL_OK);
  ASSERT_EQ(pcl_container_activate(cb), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server_b.exec, cb), PCL_OK);

  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "dup_service";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);

  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  EXPECT_EQ(pcl_executor_invoke_async(client.exec, "dup_service", &req,
                                      resp_recorder, &resp_state),
            PCL_ERR_INVALID);

  // Streaming discovery hits the same ambiguity.
  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(client.transport);
  ASSERT_NE(vt->invoke_stream, nullptr);
  auto stream_cb = [](const pcl_msg_t*, bool, pcl_status_t, void*) {};
  EXPECT_EQ(vt->invoke_stream(vt->adapter_ctx, "dup_service", &req, stream_cb,
                              nullptr, nullptr),
            PCL_ERR_INVALID);

  pcl_executor_remove(server_a.exec, ca);
  pcl_executor_remove(server_b.exec, cb);
  pcl_container_destroy(ca);
  pcl_container_destroy(cb);
  client.destroy();
  server_a.destroy();
  server_b.destroy();
  restore_logs();
}

///< REQ_PCL_309: destroying a participant with un-answered unary and stream
///< requests frees the pending records and fires stream callbacks with
///< PCL_ERR_CANCELLED.
TEST(PclSharedMemoryTransport, DestroyWithPendingRequestsCancelsThem) {
  silence_logs();
  const std::string bus = unique_token("pend_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  // Server advertises a unary and a stream service but never spins, so no
  // request is ever answered.
  struct SilentState {
    std::atomic<int> calls{0};
  } silent;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    (void)ud;
    const char* peers[] = {"client"};
    pcl_port_t* unary = pcl_container_add_service(
        c, "quiet_service", "Req",
        [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t*, pcl_svc_context_t*,
           void*) -> pcl_status_t { return PCL_OK; },
        nullptr);
    if (!unary) return PCL_ERR_NOMEM;
    if (pcl_port_set_route(unary, PCL_ROUTE_REMOTE, peers, 1) != PCL_OK) {
      return PCL_ERR_CALLBACK;
    }
    pcl_port_t* stream = pcl_container_add_stream_service(
        c, "quiet_stream", "Req",
        [](pcl_container_t*, const pcl_msg_t*, pcl_stream_context_t*, void*)
            -> pcl_status_t { return PCL_STREAMING; },
        nullptr);
    if (!stream) return PCL_ERR_NOMEM;
    return pcl_port_set_route(stream, PCL_ROUTE_REMOTE, peers, 1);
  };
  pcl_container_t* svc = pcl_container_create("quiet_server", &cbs, &silent);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "quiet_service";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);

  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  ASSERT_EQ(pcl_executor_invoke_async(client.exec, "quiet_service", &req,
                                      resp_recorder, &resp_state),
            PCL_OK);

  struct StreamState {
    std::atomic<bool> ended{false};
    pcl_status_t end_status = PCL_OK;
  } stream_state;
  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(client.transport);
  ASSERT_EQ(vt->invoke_stream(
                vt->adapter_ctx, "quiet_stream", &req,
                [](const pcl_msg_t*, bool end, pcl_status_t status, void* ud) {
                  auto* st = static_cast<StreamState*>(ud);
                  if (end) {
                    st->end_status = status;
                    st->ended = true;
                  }
                },
                &stream_state, nullptr),
            PCL_STREAMING);

  // Destroy the client with both requests pending: the unary pending list is
  // freed and the stream pending fires its callback with PCL_ERR_CANCELLED.
  client.destroy();
  EXPECT_FALSE(resp_state.received);

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  server.destroy();
  restore_logs();
}

///< REQ_PCL_310: the bus supports at most 8 participants; the ninth join
///< fails closed.
TEST(PclSharedMemoryTransport, ParticipantSlotsExhausted) {
  silence_logs();
  const std::string bus = unique_token("full_bus");

  BusNode nodes[8];
  for (int i = 0; i < 8; ++i) {
    char id[16];
    std::snprintf(id, sizeof(id), "p%d", i);
    ASSERT_TRUE(nodes[i].create(bus.c_str(), id)) << i;
  }

  pcl_executor_t* extra_exec = pcl_executor_create();
  ASSERT_NE(extra_exec, nullptr);
  EXPECT_EQ(pcl_shared_memory_transport_create(bus.c_str(), "p8", extra_exec),
            nullptr);
  pcl_executor_destroy(extra_exec);

  for (int i = 0; i < 8; ++i) nodes[i].destroy();
  restore_logs();
}

///< REQ_PCL_212: a topic with a backpressure policy waits for mailbox
///< capacity instead of failing with the non-blocking PCL_ERR_NOMEM, while
///< topics without a policy keep the non-blocking error semantics.
TEST(PclSharedMemoryTransport, TopicBackpressureWaitsInsteadOfFailingFast) {
  silence_logs();
  const std::string bus = unique_token("bpwait_bus");
  BusNode sender;
  BusNode receiver;
  ASSERT_TRUE(sender.create(bus.c_str(), "sender"));
  ASSERT_TRUE(receiver.create(bus.c_str(), "receiver"));

  ASSERT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                sender.transport, "bp/pressured", 500u),
            PCL_OK);

  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(sender.transport);
  pcl_msg_t msg = {};
  msg.data = "y";
  msg.size = 1u;
  msg.type_name = "T";

  // Saturate the receiver's mailbox with non-backpressured traffic until the
  // non-blocking path reports congestion (the receiver's drain thread races
  // us, so keep hammering until we observe NOMEM).
  bool saw_congestion = false;
  for (int i = 0; i < 4096 && !saw_congestion; ++i) {
    saw_congestion =
        (vt->publish(vt->adapter_ctx, "bp/filler", &msg) == PCL_ERR_NOMEM);
  }
  EXPECT_TRUE(saw_congestion);

  // The pressured topic rides out the congestion within its bounded wait.
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "bp/pressured", &msg), PCL_OK);

  sender.destroy();
  receiver.destroy();
  restore_logs();
}

///< REQ_PCL_212: backpressure configuration rejects an unterminated-length
///< (over-long) topic name.
TEST(PclSharedMemoryTransport, TopicBackpressureRejectsOverlongTopic) {
  silence_logs();
  const std::string bus = unique_token("bplong_bus");
  BusNode alpha;
  ASSERT_TRUE(alpha.create(bus.c_str(), "alpha"));

  const std::string long_topic(200, 't');  // >= PCL_SHM_MAX_NAME (128)
  EXPECT_EQ(pcl_shared_memory_transport_set_topic_backpressure(
                alpha.transport, long_topic.c_str(), 10u),
            PCL_ERR_INVALID);

  alpha.destroy();
  restore_logs();
}

///< REQ_PCL_327: subscriber ports with no explicit route fall back to the
///< legacy defaults -- remote ingress is accepted when a default transport is
///< attached and refused when the executor has no transport at all.
TEST(PclSharedMemoryTransport, RemoteIngressDefaultRouteModes) {
  silence_logs();
  const std::string bus = unique_token("ingdef_bus");
  BusNode sender;
  BusNode receiver;
  ASSERT_TRUE(sender.create(bus.c_str(), "sender"));
  ASSERT_TRUE(receiver.create(bus.c_str(), "receiver"));
  ASSERT_TRUE(sender.attach_transport(false));
  ASSERT_TRUE(receiver.attach_transport(false));  // default transport attached

  // An unrelated route entry so the route-entry lookup loop is exercised
  // without matching this subscriber.
  pcl_endpoint_route_t unrelated = {};
  unrelated.endpoint_name = "some/other_endpoint";
  unrelated.endpoint_kind = PCL_ENDPOINT_PUBLISHER;
  unrelated.route_mode = PCL_ROUTE_LOCAL;
  ASSERT_EQ(pcl_executor_set_endpoint_route(receiver.exec, &unrelated), PCL_OK);

  struct SubState {
    std::atomic<int> received{0};
  } sub_state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_port_t* sub = pcl_container_add_subscriber(
        c, "ingdef/topic", "T",
        [](pcl_container_t*, const pcl_msg_t*, void* state_ud) {
          static_cast<SubState*>(state_ud)->received.fetch_add(1);
        },
        ud);
    return sub ? PCL_OK : PCL_ERR_NOMEM;  // deliberately NO route config
  };
  pcl_container_t* c = pcl_container_create("ingdef_sub", &cbs, &sub_state);
  ASSERT_NE(c, nullptr);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(c), PCL_OK);
  ASSERT_EQ(pcl_executor_add(receiver.exec, c), PCL_OK);

  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(sender.transport);
  pcl_msg_t msg = {};
  msg.data = "z";
  msg.size = 1u;
  msg.type_name = "T";
  ASSERT_EQ(vt->publish(vt->adapter_ctx, "ingdef/topic", &msg), PCL_OK);

  for (int i = 0; i < 100 && sub_state.received.load() == 0; ++i) {
    pcl_executor_spin_once(receiver.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GT(sub_state.received.load(), 0);

  pcl_executor_remove(receiver.exec, c);
  pcl_container_destroy(c);
  sender.destroy();
  receiver.destroy();
  restore_logs();
}

namespace {

struct StreamEndState {
  std::atomic<bool> ended{false};
  pcl_status_t end_status = PCL_OK;
};

void stream_end_recorder(const pcl_msg_t*, bool end, pcl_status_t status,
                         void* ud) {
  auto* st = static_cast<StreamEndState*>(ud);
  if (end) {
    st->end_status = status;
    st->ended = true;
  }
}

struct StreamSvcState {
  const char* service_name = nullptr;
  const char* allowed_peer = nullptr;
  pcl_status_t handler_rc  = PCL_STREAMING;
  std::atomic<int> handler_calls{0};
};

pcl_status_t stream_svc_configure(pcl_container_t* c, void* ud) {
  auto* st = static_cast<StreamSvcState*>(ud);
  pcl_port_t* port = pcl_container_add_stream_service(
      c, st->service_name, "Req",
      [](pcl_container_t*, const pcl_msg_t*, pcl_stream_context_t* stream,
         void* state_ud) -> pcl_status_t {
        auto* state = static_cast<StreamSvcState*>(state_ud);
        state->handler_calls.fetch_add(1);
        if (state->handler_rc == PCL_STREAMING) {
          pcl_stream_end(stream);
        }
        return state->handler_rc;
      },
      ud);
  if (!port) return PCL_ERR_NOMEM;
  const char* peers[] = {st->allowed_peer};
  return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
}

}  // namespace

///< REQ_PCL_311: a stream request from a peer outside the provider's
///< allow-list is terminated with an error END frame instead of dispatching
///< the stream handler.
TEST(PclSharedMemoryTransport, StreamPeerFilterEndsWithError) {
  silence_logs();
  const std::string bus = unique_token("strfilter_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  StreamSvcState svc_state;
  svc_state.service_name = "locked_stream";
  svc_state.allowed_peer = "someone_else";  // client NOT allowed

  pcl_callbacks_t cbs = {};
  cbs.on_configure = stream_svc_configure;
  pcl_container_t* svc =
      pcl_container_create("locked_stream_srv", &cbs, &svc_state);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  StreamEndState end_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(client.transport);
  ASSERT_EQ(vt->invoke_stream(vt->adapter_ctx, "locked_stream", &req,
                              stream_end_recorder, &end_state, nullptr),
            PCL_STREAMING);

  for (int i = 0; i < 100 && !end_state.ended; ++i) {
    pcl_executor_spin_once(server.exec, 0);
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_TRUE(end_state.ended);
  EXPECT_NE(end_state.end_status, PCL_OK);
  EXPECT_EQ(svc_state.handler_calls.load(), 0);

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server.destroy();
  restore_logs();
}

///< REQ_PCL_312: a stream handler that fails instead of streaming causes an
///< END frame carrying the handler's error status.
TEST(PclSharedMemoryTransport, StreamHandlerErrorEndsStream) {
  silence_logs();
  const std::string bus = unique_token("strerr_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  StreamSvcState svc_state;
  svc_state.service_name = "failing_stream";
  svc_state.allowed_peer = "client";
  svc_state.handler_rc = PCL_ERR_CALLBACK;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = stream_svc_configure;
  pcl_container_t* svc =
      pcl_container_create("failing_stream_srv", &cbs, &svc_state);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  StreamEndState end_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(client.transport);
  ASSERT_EQ(vt->invoke_stream(vt->adapter_ctx, "failing_stream", &req,
                              stream_end_recorder, &end_state, nullptr),
            PCL_STREAMING);

  for (int i = 0; i < 100 && !end_state.ended; ++i) {
    pcl_executor_spin_once(server.exec, 0);
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_TRUE(end_state.ended);
  EXPECT_NE(end_state.end_status, PCL_OK);
  EXPECT_EQ(svc_state.handler_calls.load(), 1);

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server.destroy();
  restore_logs();
}

///< REQ_PCL_313: the per-participant service table deduplicates repeated
///< service names and caps at 16 advertised services.
TEST(PclSharedMemoryTransport, ServiceTableDedupesAndCaps) {
  silence_logs();
  const std::string bus = unique_token("svctable_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  // 19 remote service ports; two share a name (dedupe -> 17 unique names)
  // and the table caps at 16 entries.  One extra local-only service must not
  // be advertised at all.
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    (void)ud;
    auto handler = [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t*,
                      pcl_svc_context_t*, void*) -> pcl_status_t {
      return PCL_OK;
    };
    for (int i = 0; i < 18; ++i) {
      char name[32];
      // Index 1 repeats index 0's name to exercise deduplication.
      std::snprintf(name, sizeof(name), "table_svc_%d", i == 1 ? 0 : i);
      pcl_port_t* port =
          pcl_container_add_service(c, name, "Req", handler, nullptr);
      if (!port) return PCL_ERR_NOMEM;
      if (pcl_port_set_route(port, PCL_ROUTE_REMOTE, nullptr, 0) != PCL_OK) {
        return PCL_ERR_CALLBACK;
      }
    }
    pcl_port_t* local_only =
        pcl_container_add_service(c, "table_local", "Req", handler, nullptr);
    if (!local_only) return PCL_ERR_NOMEM;
    return pcl_port_set_route(local_only, PCL_ROUTE_LOCAL, nullptr, 0);
  };
  pcl_container_t* svc = pcl_container_create("table_srv", &cbs, nullptr);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  // The local-only service is not discoverable from the client.
  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "table_local";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);
  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  EXPECT_EQ(pcl_executor_invoke_async(client.exec, "table_local", &req,
                                      resp_recorder, &resp_state),
            PCL_ERR_NOT_FOUND);

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server.destroy();
  restore_logs();
}

///< REQ_PCL_314: a request that arrives after the provider re-routed its
///< service to local-only is refused at dispatch time -- unary requests get
///< an empty response, stream requests an error END frame.
TEST(PclSharedMemoryTransport, StaleAdvertisementRefusedAtDispatch) {
  silence_logs();
  const std::string bus = unique_token("stale_bus");
  BusNode client;
  BusNode server;
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server.create(bus.c_str(), "server"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server.attach_transport(true));

  struct Ports {
    pcl_port_t* unary = nullptr;
    pcl_port_t* stream = nullptr;
    std::atomic<int> calls{0};
  } ports;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* p = static_cast<Ports*>(ud);
    p->unary = pcl_container_add_service(
        c, "stale_service", "Req",
        [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t*, pcl_svc_context_t*,
           void* state_ud) -> pcl_status_t {
          static_cast<Ports*>(state_ud)->calls.fetch_add(1);
          return PCL_OK;
        },
        ud);
    p->stream = pcl_container_add_stream_service(
        c, "stale_stream", "Req",
        [](pcl_container_t*, const pcl_msg_t*, pcl_stream_context_t*,
           void* state_ud) -> pcl_status_t {
          static_cast<Ports*>(state_ud)->calls.fetch_add(1);
          return PCL_ERR_CALLBACK;
        },
        ud);
    if (!p->unary || !p->stream) return PCL_ERR_NOMEM;
    if (pcl_port_set_route(p->unary, PCL_ROUTE_REMOTE, nullptr, 0) != PCL_OK) {
      return PCL_ERR_CALLBACK;
    }
    return pcl_port_set_route(p->stream, PCL_ROUTE_REMOTE, nullptr, 0);
  };
  pcl_container_t* svc = pcl_container_create("stale_srv", &cbs, &ports);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server.exec, svc), PCL_OK);

  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "stale_service";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);

  // Send the requests while the services are still advertised...
  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  ASSERT_EQ(pcl_executor_invoke_async(client.exec, "stale_service", &req,
                                      resp_recorder, &resp_state),
            PCL_OK);
  StreamEndState end_state;
  const pcl_transport_t* vt =
      pcl_shared_memory_transport_get_transport(client.transport);
  ASSERT_EQ(vt->invoke_stream(vt->adapter_ctx, "stale_stream", &req,
                              stream_end_recorder, &end_state, nullptr),
            PCL_STREAMING);

  // ...then flip both routes to local-only before the server dispatches.
  ASSERT_EQ(pcl_port_set_route(ports.unary, PCL_ROUTE_LOCAL, nullptr, 0),
            PCL_OK);
  ASSERT_EQ(pcl_port_set_route(ports.stream, PCL_ROUTE_LOCAL, nullptr, 0),
            PCL_OK);

  for (int i = 0;
       i < 100 && !(resp_state.received && end_state.ended); ++i) {
    pcl_executor_spin_once(server.exec, 0);
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_TRUE(resp_state.received);
  EXPECT_EQ(resp_state.size, 0u);
  EXPECT_TRUE(end_state.ended);
  EXPECT_NE(end_state.end_status, PCL_OK);
  EXPECT_EQ(ports.calls.load(), 0);  // neither handler dispatched

  pcl_executor_remove(server.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server.destroy();
  restore_logs();
}

///< REQ_PCL_315: a service port with no explicit route falls back to the
///< legacy defaults -- advertised when a default transport is attached,
///< local-only (not advertised) when the executor has no transport.
TEST(PclSharedMemoryTransport, ServiceWithoutRouteUsesDefaultModes) {
  silence_logs();
  const std::string bus = unique_token("svcdef_bus");
  BusNode client;
  BusNode server_with;     // attaches its transport as default
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server_with.create(bus.c_str(), "server_with"));
  ASSERT_TRUE(client.attach_transport(true));
  ASSERT_TRUE(server_with.attach_transport(true));

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    (void)ud;
    pcl_port_t* port = pcl_container_add_service(
        c, "defmode_service", "Req",
        [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t* resp,
           pcl_svc_context_t*, void*) -> pcl_status_t {
          resp->data = "ok";
          resp->size = 2u;
          resp->type_name = "Resp";
          return PCL_OK;
        },
        nullptr);
    // Deliberately no pcl_port_set_route and no executor route entry.
    return port ? PCL_OK : PCL_ERR_NOMEM;
  };
  pcl_container_t* svc = pcl_container_create("defmode_srv", &cbs, nullptr);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server_with.exec, svc), PCL_OK);

  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "defmode_service";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);

  // Default transport attached: the routeless service is advertised and the
  // round trip completes.
  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  ASSERT_EQ(pcl_executor_invoke_async(client.exec, "defmode_service", &req,
                                      resp_recorder, &resp_state),
            PCL_OK);
  for (int i = 0; i < 100 && !resp_state.received; ++i) {
    pcl_executor_spin_once(server_with.exec, 0);
    pcl_executor_spin_once(client.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(resp_state.received);
  EXPECT_EQ(resp_state.size, 2u);

  pcl_executor_remove(server_with.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server_with.destroy();
  restore_logs();
}

///< REQ_PCL_316: without a default transport attached, a routeless service
///< stays local-only and is never advertised on the bus.
TEST(PclSharedMemoryTransport, ServiceWithoutRouteOrTransportStaysLocal) {
  silence_logs();
  const std::string bus = unique_token("svclocal_bus");
  BusNode client;
  BusNode server_bare;  // transport created but NOT attached as default
  ASSERT_TRUE(client.create(bus.c_str(), "client"));
  ASSERT_TRUE(server_bare.create(bus.c_str(), "server_bare"));
  ASSERT_TRUE(client.attach_transport(true));

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    (void)ud;
    pcl_port_t* port = pcl_container_add_service(
        c, "hidden_service", "Req",
        [](pcl_container_t*, const pcl_msg_t*, pcl_msg_t*, pcl_svc_context_t*,
           void*) -> pcl_status_t { return PCL_OK; },
        nullptr);
    return port ? PCL_OK : PCL_ERR_NOMEM;
  };
  pcl_container_t* svc = pcl_container_create("hidden_srv", &cbs, nullptr);
  ASSERT_NE(svc, nullptr);
  ASSERT_EQ(pcl_container_configure(svc), PCL_OK);
  ASSERT_EQ(pcl_container_activate(svc), PCL_OK);
  ASSERT_EQ(pcl_executor_add(server_bare.exec, svc), PCL_OK);

  // Give the bare server's recv thread a chance to run a service sync.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  pcl_endpoint_route_t consumed_route = {};
  consumed_route.endpoint_name = "hidden_service";
  consumed_route.endpoint_kind = PCL_ENDPOINT_CONSUMED;
  consumed_route.route_mode = PCL_ROUTE_REMOTE;
  ASSERT_EQ(pcl_executor_set_endpoint_route(client.exec, &consumed_route),
            PCL_OK);

  RespState resp_state;
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1u;
  req.type_name = "Req";
  EXPECT_EQ(pcl_executor_invoke_async(client.exec, "hidden_service", &req,
                                      resp_recorder, &resp_state),
            PCL_ERR_NOT_FOUND);

  pcl_executor_remove(server_bare.exec, svc);
  pcl_container_destroy(svc);
  client.destroy();
  server_bare.destroy();
  restore_logs();
}
