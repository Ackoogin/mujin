#include <chrono>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_transport_shared_memory.h"
}

namespace {

void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}

std::string get_option(int argc, char** argv, const char* prefix) {
  const size_t prefix_len = std::strlen(prefix);
  for (int i = 1; i < argc; ++i) {
    if (std::strncmp(argv[i], prefix, prefix_len) == 0) {
      return std::string(argv[i] + prefix_len);
    }
  }
  return {};
}

bool write_text(const std::string& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  out << text;
  return out.good();
}

struct BusNode {
  pcl_executor_t*                exec = nullptr;
  pcl_shared_memory_transport_t* transport = nullptr;

  bool create(const std::string& bus, const std::string& participant) {
    exec = pcl_executor_create();
    if (!exec) return false;
    transport = pcl_shared_memory_transport_create(bus.c_str(),
                                                   participant.c_str(),
                                                   exec);
    return transport != nullptr;
  }

  bool attach(bool with_gateway) {
    if (!exec || !transport) return false;
    if (pcl_executor_set_transport(
            exec, pcl_shared_memory_transport_get_transport(transport)) != PCL_OK) {
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

struct SubscriberConfig {
  const char* topic = nullptr;
  const char* type_name = nullptr;
  const char* allowed_peer = nullptr;
  bool received = false;
  std::string payload;
};

struct ServiceConfig {
  const char* service_name = nullptr;
  const char* request_type = nullptr;
  const char* allowed_peer = nullptr;
  const char* response_payload = nullptr;
  const char* response_type = nullptr;
  bool invoked = false;
  std::string request_payload;
  std::string request_type_name;
};

pcl_status_t subscriber_on_configure(pcl_container_t* c, void* ud) {
  auto* cfg = static_cast<SubscriberConfig*>(ud);
  pcl_port_t* port = pcl_container_add_subscriber(
      c,
      cfg->topic,
      cfg->type_name,
      [](pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
        auto* state = static_cast<SubscriberConfig*>(user_data);
        if (msg && msg->data && msg->size > 0u) {
          state->payload.assign(static_cast<const char*>(msg->data), msg->size);
        }
        state->received = true;
      },
      cfg);
  if (!port) return PCL_ERR_NOMEM;
  if (cfg->allowed_peer && cfg->allowed_peer[0]) {
    const char* peers[] = {cfg->allowed_peer};
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  }
  return pcl_port_set_route(port, PCL_ROUTE_REMOTE, nullptr, 0);
}

pcl_status_t service_on_configure(pcl_container_t* c, void* ud) {
  auto* cfg = static_cast<ServiceConfig*>(ud);
  pcl_port_t* port = pcl_container_add_service(
      c,
      cfg->service_name,
      cfg->request_type,
      [](pcl_container_t*, const pcl_msg_t* req, pcl_msg_t* resp,
         pcl_svc_context_t*, void* user_data) -> pcl_status_t {
        auto* state = static_cast<ServiceConfig*>(user_data);
        if (req && req->data && req->size > 0u) {
          state->request_payload.assign(static_cast<const char*>(req->data), req->size);
        }
        if (req && req->type_name) {
          state->request_type_name = req->type_name;
        }
        state->invoked = true;
        resp->data = state->response_payload;
        resp->size = static_cast<uint32_t>(std::strlen(state->response_payload));
        resp->type_name = state->response_type;
        return PCL_OK;
      },
      cfg);
  if (!port) return PCL_ERR_NOMEM;
  if (cfg->allowed_peer && cfg->allowed_peer[0]) {
    const char* peers[] = {cfg->allowed_peer};
    return pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
  }
  return pcl_port_set_route(port, PCL_ROUTE_REMOTE, nullptr, 0);
}

int run_subscriber(int argc, char** argv) {
  const std::string bus = get_option(argc, argv, "--bus=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const std::string topic = get_option(argc, argv, "--topic=");
  const std::string type_name = get_option(argc, argv, "--type=");
  const std::string ready_file = get_option(argc, argv, "--ready-file=");
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string peer = get_option(argc, argv, "--allowed-peer=");
  const uint32_t timeout_ms =
      static_cast<uint32_t>(std::stoul(get_option(argc, argv, "--timeout-ms=")));

  BusNode node;
  SubscriberConfig cfg;
  pcl_callbacks_t cbs = {};
  pcl_container_t* sub = nullptr;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  if (!node.create(bus, participant) || !node.attach(false)) return 2;

  cfg.topic = topic.c_str();
  cfg.type_name = type_name.c_str();
  cfg.allowed_peer = peer.empty() ? nullptr : peer.c_str();
  cbs.on_configure = subscriber_on_configure;

  sub = pcl_container_create("shm_helper_sub", &cbs, &cfg);
  if (!sub) {
    node.destroy();
    return 2;
  }
  if (pcl_container_configure(sub) != PCL_OK ||
      pcl_container_activate(sub) != PCL_OK ||
      pcl_executor_add(node.exec, sub) != PCL_OK) {
    pcl_container_destroy(sub);
    node.destroy();
    return 2;
  }

  write_text(ready_file, "ready");

  while (std::chrono::steady_clock::now() < deadline && !cfg.received) {
    pcl_executor_spin_once(node.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (cfg.received) {
    write_text(result_file, cfg.payload);
  }

  pcl_executor_remove(node.exec, sub);
  pcl_container_destroy(sub);
  node.destroy();
  return cfg.received ? 0 : 3;
}

int run_publisher(int argc, char** argv) {
  const std::string bus = get_option(argc, argv, "--bus=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const std::string topic = get_option(argc, argv, "--topic=");
  const std::string type_name = get_option(argc, argv, "--type=");
  const std::string payload = get_option(argc, argv, "--payload=");
  const uint32_t delay_ms =
      static_cast<uint32_t>(std::stoul(get_option(argc, argv, "--delay-ms=")));

  BusNode node;
  pcl_msg_t msg = {};
  const pcl_transport_t* transport;

  if (!node.create(bus, participant) || !node.attach(false)) return 2;

  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

  msg.data = payload.data();
  msg.size = static_cast<uint32_t>(payload.size());
  msg.type_name = type_name.c_str();
  transport = pcl_shared_memory_transport_get_transport(node.transport);
  if (!transport || transport->publish(transport->adapter_ctx, topic.c_str(), &msg) != PCL_OK) {
    node.destroy();
    return 2;
  }

  node.destroy();
  return 0;
}

int run_service(int argc, char** argv) {
  const std::string bus = get_option(argc, argv, "--bus=");
  const std::string participant = get_option(argc, argv, "--participant=");
  const std::string service_name = get_option(argc, argv, "--service=");
  const std::string request_type = get_option(argc, argv, "--request-type=");
  const std::string response_payload = get_option(argc, argv, "--response=");
  const std::string response_type = get_option(argc, argv, "--response-type=");
  const std::string ready_file = get_option(argc, argv, "--ready-file=");
  const std::string result_file = get_option(argc, argv, "--result-file=");
  const std::string peer = get_option(argc, argv, "--allowed-peer=");
  const uint32_t timeout_ms =
      static_cast<uint32_t>(std::stoul(get_option(argc, argv, "--timeout-ms=")));

  BusNode node;
  ServiceConfig cfg;
  pcl_callbacks_t cbs = {};
  pcl_container_t* svc = nullptr;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  if (!node.create(bus, participant) || !node.attach(true)) return 2;

  cfg.service_name = service_name.c_str();
  cfg.request_type = request_type.c_str();
  cfg.allowed_peer = peer.empty() ? nullptr : peer.c_str();
  cfg.response_payload = response_payload.c_str();
  cfg.response_type = response_type.c_str();
  cbs.on_configure = service_on_configure;

  svc = pcl_container_create("shm_helper_service", &cbs, &cfg);
  if (!svc) {
    node.destroy();
    return 2;
  }
  if (pcl_container_configure(svc) != PCL_OK ||
      pcl_container_activate(svc) != PCL_OK ||
      pcl_executor_add(node.exec, svc) != PCL_OK) {
    pcl_container_destroy(svc);
    node.destroy();
    return 2;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  write_text(ready_file, "ready");

  while (std::chrono::steady_clock::now() < deadline && !cfg.invoked) {
    pcl_executor_spin_once(node.exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (cfg.invoked) {
    write_text(result_file, cfg.request_payload + "\n" + cfg.request_type_name);
  }

  pcl_executor_remove(node.exec, svc);
  pcl_container_destroy(svc);
  node.destroy();
  return cfg.invoked ? 0 : 3;
}

} // namespace

int main(int argc, char** argv) {
  const std::string mode = get_option(argc, argv, "--mode=");
  silence_logs();

  if (mode == "subscriber") return run_subscriber(argc, argv);
  if (mode == "publisher") return run_publisher(argc, argv);
  if (mode == "service") return run_service(argc, argv);
  return 1;
}
