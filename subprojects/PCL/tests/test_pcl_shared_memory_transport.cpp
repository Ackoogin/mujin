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
