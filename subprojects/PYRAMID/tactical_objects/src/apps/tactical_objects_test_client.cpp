#include "pyramid_services_tactical_objects_json_codec.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_socket.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

namespace JsonCodec = pyramid::services::tactical_objects::json_codec;
namespace Provided = pyramid::services::tactical_objects::provided;
using namespace pyramid::data_model;

struct ClientState {
  std::atomic<bool> response_ready{false};
  std::atomic<bool> interest_id_received{false};
  std::atomic<int> match_count{0};
};

void onEntityMatches(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* state = static_cast<ClientState*>(user_data);
  if (!msg || !msg->data || msg->size == 0) {
    return;
  }

  const std::string payload(static_cast<const char*>(msg->data), msg->size);
  const auto matches = JsonCodec::entityMatchesFromJson(payload);
  state->match_count.store(static_cast<int>(matches.size()));

  std::fprintf(stderr, "[tactical_objects_test_client] standard.entity_matches: %s\n",
               payload.c_str());
}

void onCreateRequirementResponse(const pcl_msg_t* resp, void* user_data) {
  auto* state = static_cast<ClientState*>(user_data);
  state->response_ready.store(true);

  if (!resp || !resp->data || resp->size == 0) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] create_requirement returned empty response\n");
    return;
  }

  const std::string payload(static_cast<const char*>(resp->data), resp->size);
  std::fprintf(stderr,
               "[tactical_objects_test_client] create_requirement response: %s\n",
               payload.c_str());

  const auto response = JsonCodec::createRequirementResponseFromJson(payload);
  if (!response.interest_id.empty()) {
    state->interest_id_received.store(true);
  }
}

pcl_status_t onConfigure(pcl_container_t* container, void* user_data) {
  Provided::subscribeEntityMatches(container, onEntityMatches, user_data);
  return PCL_OK;
}

} // namespace

int main(int argc, char* argv[]) {
  const char* host = "127.0.0.1";
  uint16_t port = 19123;
  int timeout_ms = 4000;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
      host = argv[++i];
    } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--timeout-ms") == 0 && i + 1 < argc) {
      timeout_ms = std::atoi(argv[++i]);
    }
  }

  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) {
    std::fprintf(stderr, "[tactical_objects_test_client] Failed to create executor\n");
    return 1;
  }

  pcl_socket_transport_t* transport =
      pcl_socket_transport_create_client(host, port, exec);
  if (!transport) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] Failed to connect to %s:%u\n",
                 host, port);
    pcl_executor_destroy(exec);
    return 1;
  }

  pcl_executor_set_transport(exec, pcl_socket_transport_get_transport(transport));

  ClientState state;
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigure;
  pcl_container_t* container =
      pcl_container_create("tactical_objects_test_client", &callbacks, &state);
  if (!container) {
    std::fprintf(stderr, "[tactical_objects_test_client] Failed to create container\n");
    pcl_socket_transport_destroy(transport);
    pcl_executor_destroy(exec);
    return 1;
  }

  pcl_container_configure(container);
  pcl_container_activate(container);
  pcl_executor_add(exec, container);

  JsonCodec::CreateRequirementRequest request;
  request.policy = DataPolicy::Obtain;
  request.identity = StandardIdentity::Hostile;
  request.dimension = BattleDimension::Unspecified;
  request.min_lat_rad = 50.0 * 0.017453292519943295;
  request.max_lat_rad = 52.0 * 0.017453292519943295;
  request.min_lon_rad = -1.0 * 0.017453292519943295;
  request.max_lon_rad = 1.0 * 0.017453292519943295;

  const std::string request_payload = JsonCodec::toJson(request);
  pcl_msg_t request_msg{};
  request_msg.data = request_payload.data();
  request_msg.size = static_cast<uint32_t>(request_payload.size());
  request_msg.type_name = "application/json";

  const pcl_status_t invoke_rc = pcl_executor_invoke_async(
      exec, Provided::kSvcCreateRequirement, &request_msg,
      onCreateRequirementResponse, &state);
  if (invoke_rc != PCL_OK) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] Failed to invoke %s\n",
                 Provided::kSvcCreateRequirement);
    pcl_container_destroy(container);
    pcl_socket_transport_destroy(transport);
    pcl_executor_destroy(exec);
    return 1;
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(exec, 0);
    if (state.response_ready.load() && state.match_count.load() > 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::fprintf(stderr,
               "[tactical_objects_test_client] interest_id_received=%s matches=%d\n",
               state.interest_id_received.load() ? "true" : "false",
               state.match_count.load());

  const bool ok = state.interest_id_received.load();
  pcl_container_destroy(container);
  pcl_socket_transport_destroy(transport);
  pcl_executor_destroy(exec);
  return ok ? 0 : 1;
}
