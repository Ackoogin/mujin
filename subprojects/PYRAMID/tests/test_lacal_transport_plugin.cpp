/// \file test_lacal_transport_plugin.cpp
/// \brief Phase 2 of doc/plans/PYRAMID/la_cal_integration_plan.md -- the LA-CAL
///        transport plugin (libpyramid_lacal_transport_plugin.so).
///
/// Unlike the point-to-point UDP plugin, LA-CAL routes through a broker, so the
/// loopback proof runs a small in-process owp routing mock (validate/INIT/INFO
/// + SUB/PUB->MSG fan-out) and joins two executors through it in one process:
/// a publisher executor PUBs a topic, the broker routes it to the subscriber
/// executor's SUB, and the message lands in a container subscriber callback.
///
/// Covers: ABI/caps/QoS declaration; fail-closed INIT (D3); the content-type
/// guard (D1); identity from INFO (D6); the broker round-trip; and the
/// RELIABLE-over-BEST_EFFORT compose-time negative (QoS floor unmet).

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_capabilities.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_transport_routing.h>
}

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <unistd.h>
#endif

#ifndef PYRAMID_LACAL_PLUGIN_PATH
#error "PYRAMID_LACAL_PLUGIN_PATH must be defined by the build"
#endif

namespace {
using namespace std::chrono_literals;
using Server = websocketpp::server<websocketpp::config::asio>;
constexpr const char* kPluginPath = PYRAMID_LACAL_PLUGIN_PATH;

// -- In-process owp routing broker ------------------------------------------

class BrokerMock {
 public:
  enum class Mode { Happy, RejectInit };

  explicit BrokerMock(Mode mode = Mode::Happy) : mode_(mode) {
    server_.clear_access_channels(websocketpp::log::alevel::all);
    server_.clear_error_channels(websocketpp::log::elevel::all);
    server_.init_asio();
    server_.set_validate_handler([this](websocketpp::connection_hdl hdl) {
      auto con = server_.get_con_from_hdl(hdl);
      if (con->get_requested_subprotocols().empty()) return false;
      con->select_subprotocol("owp");
      return true;
    });
    server_.set_message_handler(
        [this](websocketpp::connection_hdl hdl, Server::message_ptr msg) {
          onMessage(hdl, msg->get_payload());
        });
    server_.listen(websocketpp::lib::asio::ip::tcp::v4(), 0);
    server_.start_accept();
    websocketpp::lib::error_code ec;
    port_ = server_.get_local_endpoint(ec).port();
    worker_ = std::thread([this] { server_.run(); });
  }
  ~BrokerMock() {
    server_.stop_listening();
    server_.stop();
    if (worker_.joinable()) worker_.join();
  }
  std::string url() const { return "ws://127.0.0.1:" + std::to_string(port_); }

 private:
  struct Sub {
    websocketpp::connection_hdl hdl;
    std::string sid;
    std::string topic;
  };
  static std::vector<std::string> tokens(const std::string& s, size_t max) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size() && out.size() + 1 < max) {
      while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
      size_t start = i;
      while (i < s.size() && s[i] != ' ' && s[i] != '\t') ++i;
      if (i > start) out.push_back(s.substr(start, i - start));
    }
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    if (i < s.size()) out.push_back(s.substr(i));  // verbatim remainder
    return out;
  }
  void send(websocketpp::connection_hdl hdl, const std::string& frame) {
    websocketpp::lib::error_code ec;
    server_.send(hdl, frame, websocketpp::frame::opcode::text, ec);
  }
  void onMessage(websocketpp::connection_hdl hdl, const std::string& text) {
    // Handlers all run on the single server io thread, so subs_ needs no lock.
    if (text.rfind("INIT ", 0) == 0) {
      if (mode_ == Mode::RejectInit) {
        send(hdl, "-ERR Unsupported-Service service denied");
        return;
      }
      send(hdl, "+OK");
      send(hdl,
           "INFO {\"version\":\"1.0\",\"server_id\":\"broker-1\",\"uuids\":"
           "{\"system\":\"aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa\",\"service\":"
           "\"bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb\"},\"system_label\":\"broker\"}");
      return;
    }
    if (text.rfind("SUB ", 0) == 0) {
      auto t = tokens(text, 5);  // SUB sid name topic [group]
      if (t.size() >= 4) subs_.push_back({hdl, t[1], t[3]});
      return;
    }
    if (text.rfind("PUB ", 0) == 0) {
      auto t = tokens(text, 3);  // PUB topic <verbatim body>
      if (t.size() >= 3) {
        for (const auto& s : subs_) {
          if (s.topic == t[1]) send(s.hdl, "MSG " + s.sid + " " + t[2]);
        }
      }
      return;
    }
  }
  Server server_;
  Mode mode_;
  uint16_t port_ = 0;
  std::thread worker_;
  std::vector<Sub> subs_;
};

std::string plugin_config(const std::string& url, const std::string& service_id,
                          pcl_executor_t* exec, const char* extra = "") {
  std::ostringstream os;
  os << "{\"executor\":"
     << static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(exec))
     << ",\"url\":\"" << url << "\",\"service_id\":\"" << service_id
     << "\",\"peer_id\":\"asb\",\"connect_timeout_ms\":2000" << extra << "}";
  return os.str();
}

std::string unique_temp_path() {
#ifdef _WIN32
  char temp_dir[MAX_PATH];
  char file_name[MAX_PATH];
  GetTempPathA(MAX_PATH, temp_dir);
  GetTempFileNameA(temp_dir, "lcr", 0, file_name);
  return file_name;
#else
  char path[] = "/tmp/lacal_routing_XXXXXX";
  const int fd = mkstemp(path);
  if (fd != -1) close(fd);
  return path;
#endif
}

std::string write_manifest(const std::string& body) {
  const std::string path = unique_temp_path();
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.good()) return {};
  out.write(body.data(), static_cast<std::streamsize>(body.size()));
  out.close();
  return path;
}

}  // namespace

TEST(LacalPlugin, DeclaresAbiCapsAndBestEffortQos) {
  pcl_plugin_handle_t* handle = pcl_plugin_open(kPluginPath);
  ASSERT_NE(handle, nullptr);

  auto abi = reinterpret_cast<pcl_transport_abi_version_fn>(
      pcl_plugin_symbol(handle, PCL_TRANSPORT_ABI_VERSION_SYMBOL));
  ASSERT_NE(abi, nullptr);
  EXPECT_EQ(abi(), PCL_TRANSPORT_ABI_VERSION);

  auto caps = reinterpret_cast<pcl_transport_plugin_caps_fn>(
      pcl_plugin_symbol(handle, PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL));
  ASSERT_NE(caps, nullptr);
  EXPECT_EQ(caps(nullptr), PCL_CAP_PUBSUB);

  auto qos = reinterpret_cast<pcl_transport_plugin_qos_fn>(
      pcl_plugin_symbol(handle, PCL_TRANSPORT_PLUGIN_QOS_SYMBOL));
  ASSERT_NE(qos, nullptr);
  EXPECT_EQ(qos(nullptr).reliability, PCL_QOS_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(qos("{\"declare_reliability\":\"reliable\"}").reliability,
            PCL_QOS_RELIABILITY_RELIABLE);

  pcl_plugin_unload(handle);
}

TEST(LacalPlugin, FailsClosedWhenInitUnanswered) {
  // No broker at this URL: INIT is never answered -> entry returns NULL (D3),
  // the surrogate for a CAL server silently rejecting an unregistered service.
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);
  const std::string config =
      plugin_config("ws://127.0.0.1:9", "unregistered", exec);
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(kPluginPath, config.c_str(), &handle,
                                      &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(transport, nullptr);
  pcl_executor_destroy(exec);
}

TEST(LacalPlugin, FailsClosedWhenInitRejected) {
  BrokerMock broker(BrokerMock::Mode::RejectInit);
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);
  const std::string config =
      plugin_config(broker.url(), "unregistered", exec);
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(kPluginPath, config.c_str(), &handle,
                                      &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(transport, nullptr);
  pcl_executor_destroy(exec);
}

TEST(LacalPlugin, RejectsMismatchedContentType) {
  // D1: a config declaring a non-OMS-JSON content type fails closed at entry,
  // before any connection, so the transport can't be paired with the wrong codec.
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);
  const std::string config = plugin_config(
      "ws://127.0.0.1:9", "svc", exec, ",\"content_type\":\"application/json\"");
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  EXPECT_EQ(pcl_plugin_load_transport(kPluginPath, config.c_str(), &handle,
                                      &transport),
            PCL_ERR_STATE);
  EXPECT_EQ(transport, nullptr);
  pcl_executor_destroy(exec);
}

TEST(LacalPlugin, IdentityFromInfoAndBrokerRoundTrip) {
  BrokerMock broker;

  // --- Subscriber side ---
  pcl_executor_t* sub_exec = pcl_executor_create();
  ASSERT_NE(sub_exec, nullptr);
  pcl_plugin_handle_t* sub_handle = nullptr;
  const pcl_transport_t* sub_transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(
                kPluginPath,
                plugin_config(broker.url(), "subscriber", sub_exec).c_str(),
                &sub_handle, &sub_transport),
            PCL_OK);
  ASSERT_NE(sub_transport, nullptr);

  // D6: identity taken from INFO, not invented.
  auto system_uuid = reinterpret_cast<const char* (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(sub_handle, "pyramid_lacal_transport_system_uuid"));
  auto service_uuid = reinterpret_cast<const char* (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(sub_handle, "pyramid_lacal_transport_service_uuid"));
  ASSERT_NE(system_uuid, nullptr);
  ASSERT_NE(service_uuid, nullptr);
  EXPECT_STREQ(system_uuid(sub_transport),
               "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa");
  EXPECT_STREQ(service_uuid(sub_transport),
               "bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb");

  ASSERT_EQ(pcl_executor_register_transport_caps(sub_exec, "asb", sub_transport,
                                                 PCL_CAP_PUBSUB),
            PCL_OK);

  struct SubState {
    std::mutex m;
    std::condition_variable cv;
    std::vector<std::string> bodies;
    std::string expected_body;
    std::chrono::steady_clock::time_point completed_at;
    bool timing_armed = false;
  } sub_state;
  auto sub_on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto cb = [](pcl_container_t*, const pcl_msg_t* msg, void* u) {
      auto* st = static_cast<SubState*>(u);
      std::lock_guard<std::mutex> lock(st->m);
      st->bodies.emplace_back(static_cast<const char*>(msg->data), msg->size);
      if (st->timing_armed && st->bodies.back() == st->expected_body) {
        st->completed_at = std::chrono::steady_clock::now();
        st->timing_armed = false;
        st->cv.notify_one();
      }
    };
    pcl_port_t* sub =
        pcl_container_add_subscriber(c, "MA_Action", "MA_Action", cb, ud);
    if (!sub) return PCL_ERR_NOMEM;
    const char* peers[] = {"asb"};
    pcl_port_set_route(sub, PCL_ROUTE_REMOTE, peers, 1);
    return PCL_OK;
  };
  pcl_callbacks_t sub_cbs{};
  sub_cbs.on_configure = sub_on_configure;
  pcl_container_t* sub_c = pcl_container_create("lacal_sub", &sub_cbs, &sub_state);
  ASSERT_NE(sub_c, nullptr);
  ASSERT_EQ(pcl_container_configure(sub_c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(sub_c), PCL_OK);
  ASSERT_EQ(pcl_executor_add(sub_exec, sub_c), PCL_OK);

  // --- Publisher side ---
  pcl_executor_t* pub_exec = pcl_executor_create();
  ASSERT_NE(pub_exec, nullptr);
  pcl_plugin_handle_t* pub_handle = nullptr;
  const pcl_transport_t* pub_transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(
                kPluginPath,
                plugin_config(broker.url(), "publisher", pub_exec).c_str(),
                &pub_handle, &pub_transport),
            PCL_OK);
  ASSERT_EQ(pcl_executor_register_transport_caps(pub_exec, "asb", pub_transport,
                                                 PCL_CAP_PUBSUB),
            PCL_OK);

  pcl_port_t* pub_port = nullptr;
  auto pub_on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto** out = static_cast<pcl_port_t**>(ud);
    *out = pcl_container_add_publisher(c, "MA_Action", "MA_Action");
    if (!*out) return PCL_ERR_NOMEM;
    const char* peers[] = {"asb"};
    pcl_port_set_route(*out, PCL_ROUTE_REMOTE, peers, 1);
    return PCL_OK;
  };
  pcl_callbacks_t pub_cbs{};
  pub_cbs.on_configure = pub_on_configure;
  pcl_container_t* pub_c = pcl_container_create("lacal_pub", &pub_cbs, &pub_port);
  ASSERT_NE(pub_c, nullptr);
  ASSERT_EQ(pcl_container_configure(pub_c), PCL_OK);
  ASSERT_EQ(pcl_container_activate(pub_c), PCL_OK);
  ASSERT_EQ(pcl_executor_add(pub_exec, pub_c), PCL_OK);
  ASSERT_NE(pub_port, nullptr);

  // Give the broker a moment to register the SUB, then publish a body that
  // contains spaces and a newline (verbatim-body integrity through the wire).
  std::this_thread::sleep_for(200ms);
  const std::string body = "{\"MA_Action\": {\"id\": \"a 1\"}}\nx";
  pcl_msg_t msg;
  msg.data = body.data();
  msg.size = static_cast<uint32_t>(body.size());
  msg.type_name = "MA_Action";

  bool got = false;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && !got) {
    ASSERT_EQ(pcl_port_publish(pub_port, &msg), PCL_OK);
    for (int i = 0; i < 20 && !got; ++i) {
      pcl_executor_spin_once(sub_exec, 0);
      {
        std::lock_guard<std::mutex> lock(sub_state.m);
        got = !sub_state.bodies.empty() && sub_state.bodies.front() == body;
      }
      std::this_thread::sleep_for(10ms);
    }
  }
  EXPECT_TRUE(got) << "subscriber received the published body verbatim";

  // Measure one-way application delivery separately from the readiness proof
  // above. The completion timestamp is recorded in the subscriber callback;
  // the test thread waits only after that timestamp. It never polls the
  // executor while a timed sample is in flight.
  std::atomic<bool> stop_subscriber{false};
  std::thread subscriber_runner([&] {
    while (!stop_subscriber.load(std::memory_order_relaxed)) {
      pcl_executor_spin_once(sub_exec, 0);
      std::this_thread::yield();
    }
  });

  constexpr int kWarmups = 50;
  constexpr int kSamples = 500;
  std::vector<double> latencies_us;
  latencies_us.reserve(kSamples);
  const std::string padding(480, 'x');
  bool samples_ok = true;
  for (int i = -kWarmups; i < kSamples; ++i) {
    const std::string timed_body =
        "{\"PositionReport\":{\"id\":" + std::to_string(i) +
        ",\"padding\":\"" + padding + "\"}}";
    pcl_msg_t timed_msg{timed_body.data(),
                        static_cast<uint32_t>(timed_body.size()),
                        "PositionReport"};
    std::unique_lock<std::mutex> lock(sub_state.m);
    sub_state.expected_body = timed_body;
    sub_state.timing_armed = true;
    const auto started_at = std::chrono::steady_clock::now();
    const pcl_status_t publish_status = pcl_port_publish(pub_port, &timed_msg);
    const bool completed = publish_status == PCL_OK &&
        sub_state.cv.wait_for(lock, 2s, [&] {
          return !sub_state.timing_armed;
        });
    if (!completed) {
      ADD_FAILURE() << "LA-CAL sample " << i << " failed: publish status "
                    << publish_status;
      samples_ok = false;
      break;
    }
    const auto completed_at = sub_state.completed_at;
    lock.unlock();
    if (i >= 0) {
      latencies_us.push_back(
          std::chrono::duration<double, std::micro>(completed_at - started_at)
              .count());
    }
  }

  stop_subscriber.store(true, std::memory_order_relaxed);
  subscriber_runner.join();
  ASSERT_TRUE(samples_ok);
  ASSERT_EQ(latencies_us.size(), static_cast<size_t>(kSamples));
  std::sort(latencies_us.begin(), latencies_us.end());
  double sum = 0.0;
  for (const double value : latencies_us) sum += value;
  std::printf(
      "[PERF] raw-pcl / LA-CAL / OMS JSON      avg=%7.1f us min=%7.1f us "
      "p50=%7.1f us p99=%7.1f us payload=%zu B\n",
      sum / latencies_us.size(), latencies_us.front(),
      latencies_us[latencies_us.size() / 2],
      latencies_us[static_cast<size_t>(latencies_us.size() * 0.99)],
      sub_state.expected_body.size());

  pcl_executor_remove(pub_exec, pub_c);
  pcl_container_destroy(pub_c);
  pcl_executor_remove(sub_exec, sub_c);
  pcl_container_destroy(sub_c);
  pcl_plugin_unload_transport(pub_handle, pub_transport);
  pcl_plugin_unload_transport(sub_handle, sub_transport);
  pcl_executor_destroy(pub_exec);
  pcl_executor_destroy(sub_exec);
}

TEST(LacalPlugin, ReliableFloorOverBestEffortFailsClosed) {
  BrokerMock broker;
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  std::ostringstream manifest;
  manifest << "transport asb " << kPluginPath << " {\"url\":\""
           << broker.url()
           << "\",\"service_id\":\"svc\",\"peer_id\":\"asb\","
              "\"connect_timeout_ms\":2000}\n"
           << "route MA_Action publisher asb reliable\n";
  const std::string path = write_manifest(manifest.str());
  ASSERT_FALSE(path.empty());

  pcl_transport_routing_t* routing = nullptr;
  char diag[256] = "";
  EXPECT_EQ(pcl_transport_routing_load(exec, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_EQ(routing, nullptr);
  EXPECT_NE(std::string(diag).find("reliable"), std::string::npos) << diag;
  EXPECT_EQ(pcl_executor_get_transport_for_peer(exec, "asb"), nullptr);

  std::remove(path.c_str());
  pcl_executor_destroy(exec);
}

// Phase 5 capability negative: an RPC-shaped endpoint (PROVIDED requires
// PCL_CAP_RPC_UNARY) cannot be realized over LA-CAL, which advertises
// PCL_CAP_PUBSUB only. Routing must fail closed at compose time -- the seam's
// Request legs can be realized pub/sub over this peer but never RPC.
TEST(LacalPlugin, RpcEndpointOverPubsubPeerFailsClosed) {
  BrokerMock broker;
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  std::ostringstream manifest;
  manifest << "transport asb " << kPluginPath << " {\"url\":\""
           << broker.url()
           << "\",\"service_id\":\"svc\",\"peer_id\":\"asb\","
              "\"connect_timeout_ms\":2000}\n"
           << "route ActionCommand_Service provided asb\n";
  const std::string path = write_manifest(manifest.str());
  ASSERT_FALSE(path.empty());

  pcl_transport_routing_t* routing = nullptr;
  char diag[256] = "";
  EXPECT_EQ(pcl_transport_routing_load(exec, path.c_str(), &routing, diag,
                                       sizeof(diag)),
            PCL_ERR_STATE);
  EXPECT_EQ(routing, nullptr);
  // Pin the intended path: PROVIDED requires RPC_UNARY, and the failure is the
  // caps-mismatch diagnostic ("... provides caps 0x1" = PUBSUB) -- not merely
  // some generic "requires" that another required-cap regression could match.
  const std::string diag_text(diag);
  EXPECT_NE(diag_text.find("requires RPC_UNARY"), std::string::npos) << diag;
  EXPECT_NE(diag_text.find("provides caps"), std::string::npos) << diag;
  EXPECT_EQ(pcl_executor_get_transport_for_peer(exec, "asb"), nullptr);

  std::remove(path.c_str());
  pcl_executor_destroy(exec);
}
