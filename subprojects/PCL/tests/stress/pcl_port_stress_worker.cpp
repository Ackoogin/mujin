/// \file pcl_port_stress_worker.cpp
/// \brief One process of the PYRAMID port-system stress test.
///
/// The worker is a small command-line program that plays exactly one role
/// (publisher, subscriber, service provider, service client, or
/// attach-and-hold) against one runtime-loaded transport plugin.  A driver
/// script (run_port_stress.py) launches many workers at once, coordinates
/// them through marker files, and aggregates the JSON result each worker
/// writes on exit.
///
/// The transport is always loaded through the production plugin loader
/// (pcl_plugin_load_transport on a dlopen'd .so), so the stress test
/// exercises the same path a deployed PYRAMID component uses, not a
/// test-only shortcut.
///
/// Coordination protocol (all files are created by exactly one side):
///   --ready-file   worker creates it once the transport is attached and
///                  all ports are active (driver waits for every worker).
///   --start-file   driver creates it when every worker is ready; load
///                  generation begins only after it appears.
///   --stop-file    driver creates it to ask long-running roles
///                  (subscriber, service) to finish up and report.
///   --result-file  worker writes a single JSON object here on success.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_plugin_loader.h"
#include "pcl/pcl_transport.h"
}

namespace {

using Clock = std::chrono::steady_clock;

int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch())
      .count();
}

std::string get_option(int argc, char** argv, const char* prefix,
                       const char* fallback = "") {
  const size_t prefix_len = std::strlen(prefix);
  for (int i = 1; i < argc; ++i) {
    if (std::strncmp(argv[i], prefix, prefix_len) == 0) {
      return std::string(argv[i] + prefix_len);
    }
  }
  return fallback;
}

uint64_t get_option_u64(int argc, char** argv, const char* prefix,
                        uint64_t fallback) {
  const std::string text = get_option(argc, argv, prefix);
  if (text.empty()) return fallback;
  return std::strtoull(text.c_str(), nullptr, 10);
}

bool file_exists(const std::string& path) {
  std::ifstream in(path);
  return in.good();
}

bool write_text(const std::string& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  out << text;
  return out.good();
}

/// Reads the resident set size of this process in kilobytes (Linux only;
/// returns 0 elsewhere or on parse failure).  Used to observe memory growth
/// of the unbounded executor ingress queue under sustained overload.
uint64_t read_vm_rss_kb() {
  std::ifstream in("/proc/self/status");
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      return std::strtoull(line.c_str() + 6, nullptr, 10);
    }
  }
  return 0u;
}

void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}

/// Everything one worker owns: the executor, the plugin handle, and the
/// transport vtable the plugin returned.
struct Node {
  pcl_executor_t*        exec = nullptr;
  pcl_plugin_handle_t*   plugin = nullptr;
  const pcl_transport_t* transport = nullptr;

  bool load(const std::string& plugin_path, const std::string& config_json) {
    exec = pcl_executor_create();
    if (!exec) return false;
    const pcl_status_t rc = pcl_plugin_load_transport(
        plugin_path.c_str(), config_json.c_str(), &plugin, &transport);
    if (rc != PCL_OK || !transport) {
      std::fprintf(stderr, "plugin load failed: rc=%d path=%s\n", (int)rc,
                   plugin_path.c_str());
      return false;
    }
    return pcl_executor_set_transport(exec, transport) == PCL_OK;
  }

  /// Activates the shared-memory service gateway so this participant can be
  /// reached as a provider.  Resolved through the plugin's exported symbol so
  /// the worker stays independent of the transport's concrete type.
  bool activate_shm_gateway() {
    using GatewayFn = pcl_container_t* (*)(const pcl_transport_t*);
    auto gateway_fn = reinterpret_cast<GatewayFn>(
        pcl_plugin_symbol(plugin, "pcl_shm_transport_plugin_gateway"));
    if (!gateway_fn) return false;
    pcl_container_t* gateway = gateway_fn(transport);
    if (!gateway) return false;
    if (pcl_container_configure(gateway) != PCL_OK) return false;
    if (pcl_container_activate(gateway) != PCL_OK) return false;
    return pcl_executor_add(exec, gateway) == PCL_OK;
  }

  void destroy() {
    if (plugin) {
      pcl_plugin_unload_transport(plugin, transport);
      plugin = nullptr;
      transport = nullptr;
    }
    if (exec) {
      pcl_executor_destroy(exec);
      exec = nullptr;
    }
  }
};

/// Builds the transport plugin's opaque config_json from worker options.
/// The executor pointer is threaded through as a decimal integer, matching
/// what every PCL transport plugin's read_executor() expects.
std::string build_config_json(int argc, char** argv, pcl_executor_t* exec) {
  const std::string transport = get_option(argc, argv, "--transport=", "shm");
  char exec_field[64];
  std::snprintf(exec_field, sizeof(exec_field), "\"executor\":%" PRIuPTR,
                (uintptr_t)exec);

  std::string json = "{";
  if (transport == "shm") {
    json += "\"bus_name\":\"" + get_option(argc, argv, "--bus=") + "\",";
    json += "\"participant_id\":\"" +
            get_option(argc, argv, "--participant=") + "\",";
  } else if (transport == "socket") {
    json += "\"role\":\"" + get_option(argc, argv, "--role=") + "\",";
    json += "\"host\":\"" + get_option(argc, argv, "--host=", "127.0.0.1") +
            "\",";
    json += "\"port\":" + get_option(argc, argv, "--port=", "0") + ",";
    const std::string peer = get_option(argc, argv, "--peer-id=");
    if (!peer.empty()) json += "\"peer_id\":\"" + peer + "\",";
  } else if (transport == "udp") {
    json += "\"remote_host\":\"" +
            get_option(argc, argv, "--remote-host=", "127.0.0.1") + "\",";
    json += "\"remote_port\":" +
            get_option(argc, argv, "--remote-port=", "0") + ",";
    json += "\"local_port\":" + get_option(argc, argv, "--local-port=", "0") +
            ",";
    const std::string peer = get_option(argc, argv, "--peer-id=");
    if (!peer.empty()) json += "\"peer_id\":\"" + peer + "\",";
  }
  json += exec_field;
  json += "}";
  return json;
}

/// Shared counters a role fills in and report() serialises.
struct Report {
  const char* mode = "";
  uint64_t    attempts = 0;
  uint64_t    ok = 0;
  uint64_t    err_nomem = 0;
  uint64_t    err_not_found = 0;
  uint64_t    err_other = 0;
  uint64_t    received = 0;
  uint64_t    bytes = 0;
  uint64_t    gaps = 0;         // sequence discontinuities seen by a subscriber
  uint64_t    out_of_window = 0;
  int64_t     first_rx_ns = 0;
  int64_t     last_rx_ns = 0;
  int64_t     active_ns = 0;    // publisher/client active window
  uint64_t    rss_kb = 0;

  std::string to_json(const std::string& participant) const {
    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "{\"mode\":\"%s\",\"participant\":\"%s\",\"attempts\":%" PRIu64
        ",\"ok\":%" PRIu64 ",\"err_nomem\":%" PRIu64
        ",\"err_not_found\":%" PRIu64 ",\"err_other\":%" PRIu64
        ",\"received\":%" PRIu64 ",\"bytes\":%" PRIu64 ",\"gaps\":%" PRIu64
        ",\"first_rx_ns\":%" PRId64 ",\"last_rx_ns\":%" PRId64
        ",\"active_ns\":%" PRId64 ",\"rss_kb\":%" PRIu64 "}",
        mode, participant.c_str(), attempts, ok, err_nomem, err_not_found,
        err_other, received, bytes, gaps, first_rx_ns, last_rx_ns, active_ns,
        rss_kb);
    return std::string(buf);
  }
};

bool wait_for_file(const std::string& path, uint32_t timeout_ms) {
  const auto deadline =
      Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (Clock::now() < deadline) {
    if (file_exists(path)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

// -- publisher --------------------------------------------------------------

int run_pub(int argc, char** argv, Node& node) {
  const std::string topic = get_option(argc, argv, "--topic=", "stress_topic");
  const std::string start_file = get_option(argc, argv, "--start-file=");
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const uint64_t duration_ms = get_option_u64(argc, argv, "--duration-ms=", 3000);
  const uint64_t payload_size = get_option_u64(argc, argv, "--size=", 64);
  const uint64_t rate = get_option_u64(argc, argv, "--rate=", 0);  // 0 = max

  std::vector<uint8_t> payload(payload_size > 8 ? payload_size : 8, 0xA5);

  if (!start_file.empty() && !wait_for_file(start_file, 30000)) return 4;

  Report rep;
  rep.mode = "pub";
  const int64_t t0 = now_ns();
  const int64_t t_end = t0 + (int64_t)duration_ms * 1000000;
  const double ns_per_msg = rate ? 1e9 / (double)rate : 0.0;
  uint64_t seq = 0;

  while (now_ns() < t_end) {
    // Sequence number in the first 8 bytes so subscribers can detect loss.
    uint64_t wire_seq = seq;
    std::memcpy(payload.data(), &wire_seq, sizeof(wire_seq));

    pcl_msg_t msg = {};
    msg.data = payload.data();
    msg.size = (uint32_t)payload.size();
    msg.type_name = "stress_bytes";

    ++rep.attempts;
    const pcl_status_t rc = node.transport->publish(
        node.transport->adapter_ctx, topic.c_str(), &msg);
    if (rc == PCL_OK) {
      ++rep.ok;
      ++seq;
    } else if (rc == PCL_ERR_NOMEM) {
      // Fail-fast backpressure: some target mailbox was full. Yield briefly
      // so the bus lock is not hammered while receivers catch up.
      ++rep.err_nomem;
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    } else if (rc == PCL_ERR_NOT_FOUND) {
      ++rep.err_not_found;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } else {
      ++rep.err_other;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (rate) {
      const int64_t next = t0 + (int64_t)(ns_per_msg * (double)rep.attempts);
      while (now_ns() < next) {
        std::this_thread::sleep_for(std::chrono::microseconds(20));
      }
    }
  }
  rep.active_ns = now_ns() - t0;
  rep.rss_kb = read_vm_rss_kb();
  return write_text(result_file, rep.to_json(participant)) ? 0 : 5;
}

// -- subscriber -------------------------------------------------------------

struct SubState {
  Report   rep;
  uint64_t next_seq = 0;
  bool     have_seq = false;
};

int run_sub(int argc, char** argv, Node& node) {
  const std::string topic = get_option(argc, argv, "--topic=", "stress_topic");
  const std::string ready_file = get_option(argc, argv, "--ready-file=");
  const std::string stop_file = get_option(argc, argv, "--stop-file=");
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const uint64_t timeout_ms = get_option_u64(argc, argv, "--timeout-ms=", 60000);
  const uint64_t spin_sleep_us =
      get_option_u64(argc, argv, "--spin-sleep-us=", 0);

  SubState state;
  state.rep.mode = "sub";

  struct Cfg {
    const char* topic;
    SubState*   state;
  } cfg{topic.c_str(), &state};

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* cc = static_cast<Cfg*>(ud);
    pcl_port_t* port = pcl_container_add_subscriber(
        c, cc->topic, "stress_bytes",
        [](pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
          auto* st = static_cast<SubState*>(user_data);
          const int64_t t = now_ns();
          if (st->rep.received == 0) st->rep.first_rx_ns = t;
          st->rep.last_rx_ns = t;
          ++st->rep.received;
          if (msg && msg->data) {
            st->rep.bytes += msg->size;
            if (msg->size >= 8) {
              uint64_t seq = 0;
              std::memcpy(&seq, msg->data, sizeof(seq));
              if (st->have_seq && seq != st->next_seq) ++st->rep.gaps;
              st->next_seq = seq + 1;
              st->have_seq = true;
            }
          }
        },
        cc->state);
    if (!port) return PCL_ERR_NOMEM;
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, nullptr, 0);
  };

  pcl_container_t* sub = pcl_container_create("stress_sub", &cbs, &cfg);
  if (!sub) return 2;
  if (pcl_container_configure(sub) != PCL_OK ||
      pcl_container_activate(sub) != PCL_OK ||
      pcl_executor_add(node.exec, sub) != PCL_OK) {
    return 2;
  }

  if (!ready_file.empty()) write_text(ready_file, "ready");

  const auto deadline =
      Clock::now() + std::chrono::milliseconds(timeout_ms);
  int64_t quiet_since = 0;
  while (Clock::now() < deadline) {
    pcl_executor_spin_once(node.exec, 0);
    if (spin_sleep_us) {
      std::this_thread::sleep_for(std::chrono::microseconds(spin_sleep_us));
    }
    if (!stop_file.empty() && file_exists(stop_file)) {
      // Drain: leave once no message has arrived for 300 ms.
      const int64_t t = now_ns();
      const int64_t last = state.rep.received ? state.rep.last_rx_ns : 0;
      if (quiet_since == 0) quiet_since = t;
      if (last > quiet_since) quiet_since = last;
      if (t - quiet_since > 300000000LL) break;
    }
  }

  state.rep.rss_kb = read_vm_rss_kb();
  const bool wrote = write_text(result_file, state.rep.to_json(participant));
  pcl_executor_remove(node.exec, sub);
  pcl_container_destroy(sub);
  return wrote ? 0 : 5;
}

// -- service provider -------------------------------------------------------

int run_svc(int argc, char** argv, Node& node) {
  const std::string service = get_option(argc, argv, "--service=", "stress_svc");
  const std::string ready_file = get_option(argc, argv, "--ready-file=");
  const std::string stop_file = get_option(argc, argv, "--stop-file=");
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const uint64_t timeout_ms = get_option_u64(argc, argv, "--timeout-ms=", 60000);

  if (!node.activate_shm_gateway()) {
    std::fprintf(stderr, "gateway activation failed\n");
    return 2;
  }

  Report rep;
  rep.mode = "svc";

  struct Cfg {
    const char* service;
    Report*     rep;
  } cfg{service.c_str(), &rep};

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* cc = static_cast<Cfg*>(ud);
    pcl_port_t* port = pcl_container_add_service(
        c, cc->service, "stress_bytes",
        [](pcl_container_t*, const pcl_msg_t* req, pcl_msg_t* resp,
           pcl_svc_context_t*, void* user_data) -> pcl_status_t {
          auto* r = static_cast<Report*>(user_data);
          const int64_t t = now_ns();
          if (r->received == 0) r->first_rx_ns = t;
          r->last_rx_ns = t;
          ++r->received;
          if (req && req->data) r->bytes += req->size;
          static const char kAck[] = "ack";
          resp->data = kAck;
          resp->size = 3;
          resp->type_name = "stress_bytes";
          return PCL_OK;
        },
        cc->rep);
    if (!port) return PCL_ERR_NOMEM;
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, nullptr, 0);
  };

  pcl_container_t* svc = pcl_container_create("stress_svc", &cbs, &cfg);
  if (!svc) return 2;
  if (pcl_container_configure(svc) != PCL_OK ||
      pcl_container_activate(svc) != PCL_OK ||
      pcl_executor_add(node.exec, svc) != PCL_OK) {
    return 2;
  }

  // Give the receive thread one service-table sync before declaring ready.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!ready_file.empty()) write_text(ready_file, "ready");

  const auto deadline =
      Clock::now() + std::chrono::milliseconds(timeout_ms);
  int64_t quiet_since = 0;
  while (Clock::now() < deadline) {
    pcl_executor_spin_once(node.exec, 0);
    if (!stop_file.empty() && file_exists(stop_file)) {
      const int64_t t = now_ns();
      if (quiet_since == 0) quiet_since = t;
      if (rep.received && rep.last_rx_ns > quiet_since) {
        quiet_since = rep.last_rx_ns;
      }
      if (t - quiet_since > 300000000LL) break;
    }
  }

  rep.rss_kb = read_vm_rss_kb();
  const bool wrote = write_text(result_file, rep.to_json(participant));
  pcl_executor_remove(node.exec, svc);
  pcl_container_destroy(svc);
  return wrote ? 0 : 5;
}

// -- service client ---------------------------------------------------------

int run_client(int argc, char** argv, Node& node) {
  const std::string service = get_option(argc, argv, "--service=", "stress_svc");
  const std::string start_file = get_option(argc, argv, "--start-file=");
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const uint64_t duration_ms = get_option_u64(argc, argv, "--duration-ms=", 3000);
  const uint64_t payload_size = get_option_u64(argc, argv, "--size=", 64);
  const uint64_t max_inflight = get_option_u64(argc, argv, "--inflight=", 8);

  std::vector<uint8_t> payload(payload_size > 8 ? payload_size : 8, 0x5A);

  if (!start_file.empty() && !wait_for_file(start_file, 30000)) return 4;

  Report rep;
  rep.mode = "client";
  std::atomic<uint64_t> completed{0};
  std::atomic<uint64_t> failed{0};

  struct RespCtx {
    std::atomic<uint64_t>* completed;
    std::atomic<uint64_t>* failed;
  } resp_ctx{&completed, &failed};

  const int64_t t0 = now_ns();
  const int64_t t_end = t0 + (int64_t)duration_ms * 1000000;

  while (now_ns() < t_end) {
    if (rep.attempts - completed.load() - failed.load() < max_inflight) {
      pcl_msg_t msg = {};
      msg.data = payload.data();
      msg.size = (uint32_t)payload.size();
      msg.type_name = "stress_bytes";
      ++rep.attempts;
      const pcl_status_t rc = pcl_executor_invoke_async(
          node.exec, service.c_str(), &msg,
          [](const pcl_msg_t* resp, void* user_data) {
            auto* ctx = static_cast<RespCtx*>(user_data);
            if (resp && resp->data) {
              ctx->completed->fetch_add(1);
            } else {
              ctx->failed->fetch_add(1);
            }
          },
          &resp_ctx);
      if (rc != PCL_OK) {
        --rep.attempts;
        ++rep.err_other;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    pcl_executor_spin_once(node.exec, 0);
  }

  // Drain outstanding responses (bounded).
  const auto drain_deadline = Clock::now() + std::chrono::seconds(5);
  while (completed.load() + failed.load() + rep.err_other < rep.attempts &&
         Clock::now() < drain_deadline) {
    pcl_executor_spin_once(node.exec, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  rep.ok = completed.load();
  rep.err_not_found = failed.load();
  rep.active_ns = now_ns() - t0;
  rep.rss_kb = read_vm_rss_kb();
  return write_text(result_file, rep.to_json(participant)) ? 0 : 5;
}

// -- attach / hold / detach -------------------------------------------------

int run_attach(int argc, char** argv, Node& node) {
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const uint64_t hold_ms = get_option_u64(argc, argv, "--hold-ms=", 100);

  // Reaching this point means the plugin load (and bus attach) succeeded;
  // hold the slot briefly, then detach cleanly.
  Report rep;
  rep.mode = "attach";
  rep.ok = 1;
  std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
  rep.rss_kb = read_vm_rss_kb();
  return write_text(result_file, rep.to_json(participant)) ? 0 : 5;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string mode = get_option(argc, argv, "--mode=");
  const std::string plugin_path = get_option(argc, argv, "--plugin=");
  const std::string ready_file = get_option(argc, argv, "--ready-file=");
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string participant = get_option(argc, argv, "--participant=");

  if (get_option(argc, argv, "--verbose=").empty()) silence_logs();

  Node node;
  node.exec = pcl_executor_create();
  if (!node.exec) return 2;

  const std::string config_json = build_config_json(argc, argv, node.exec);

  {
    pcl_status_t rc = pcl_plugin_load_transport(
        plugin_path.c_str(), config_json.c_str(), &node.plugin,
        &node.transport);
    if (rc != PCL_OK || !node.transport) {
      // Report attach failures as data, not just a process error: the
      // participant-limit scenario asserts on this.
      if (!result_file.empty()) {
        Report rep;
        rep.mode = "attach";
        rep.err_other = 1;
        write_text(result_file, rep.to_json(participant));
      }
      std::fprintf(stderr, "plugin load failed rc=%d\n", (int)rc);
      pcl_executor_destroy(node.exec);
      return 3;
    }
  }
  if (pcl_executor_set_transport(node.exec, node.transport) != PCL_OK) {
    return 2;
  }

  // Publisher-style roles signal readiness here; subscriber/service roles
  // signal after their ports are active inside their run function.
  int rc = 1;
  if (mode == "pub" || mode == "client" || mode == "attach") {
    if (!ready_file.empty()) write_text(ready_file, "ready");
  }

  if (mode == "pub") rc = run_pub(argc, argv, node);
  else if (mode == "sub") rc = run_sub(argc, argv, node);
  else if (mode == "svc") rc = run_svc(argc, argv, node);
  else if (mode == "client") rc = run_client(argc, argv, node);
  else if (mode == "attach") rc = run_attach(argc, argv, node);
  else std::fprintf(stderr, "unknown --mode=%s\n", mode.c_str());

  node.destroy();
  return rc;
}
