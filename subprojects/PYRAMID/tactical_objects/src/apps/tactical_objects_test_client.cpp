#include "pyramid_services_tactical_objects_provided.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#endif
#if defined(PYRAMID_ENABLE_PROTOBUF)
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"
#endif

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_socket.h>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

#if defined(PYRAMID_ENABLE_FLATBUFFERS)
namespace FlatCodec = pyramid::services::tactical_objects::flatbuffers_codec;
#endif
#if defined(PYRAMID_ENABLE_PROTOBUF)
namespace ProtobufCodec = pyramid::services::tactical_objects::protobuf_codec;
#endif
namespace Provided = pyramid::services::tactical_objects::provided;
namespace TacticalCodec = pyramid::domain_model::tactical;
namespace Common = pyramid::domain_model::common;
using namespace pyramid::domain_model;

struct ClientState {
    std::atomic<bool> response_ready{false};
    std::atomic<bool> interest_id_received{false};
    std::atomic<int> match_count{0};
    std::string content_type = "application/json";
};

void onEntityMatches(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* state = static_cast<ClientState*>(user_data);
  if (!msg || !msg->data || msg->size == 0) {
    return;
  }

  std::vector<ObjectMatch> matches;
  std::string payload;
  try {
    if (false) {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
    } else if (msg->type_name &&
               std::strcmp(msg->type_name, FlatCodec::kContentType) == 0) {
      matches = FlatCodec::fromBinaryObjectMatchArray(msg->data, msg->size);
      payload = "<flatbuffers>";
#endif
#if defined(PYRAMID_ENABLE_PROTOBUF)
    } else if (msg->type_name &&
               std::strcmp(msg->type_name, "application/protobuf") == 0) {
      matches = ProtobufCodec::fromBinaryObjectMatchArray(msg->data, msg->size);
      payload = "<protobuf>";
#endif
    } else {
      payload.assign(static_cast<const char*>(msg->data), msg->size);
      auto arr = nlohmann::json::parse(payload);
      if (!arr.is_array()) {
        return;
      }
      for (const auto& item : arr) {
        matches.push_back(TacticalCodec::fromJson(
            item.dump(), static_cast<ObjectMatch*>(nullptr)));
      }
    }
  } catch (...) {
    return;
  }
  state->match_count.store(static_cast<int>(matches.size()));

  std::fprintf(stderr,
               "[tactical_objects_test_client] standard.entity_matches type=%s count=%d payload=%s\n",
               msg->type_name ? msg->type_name : "<null>",
               static_cast<int>(matches.size()), payload.c_str());
}

void onCreateRequirementResponse(const pcl_msg_t* resp, void* user_data) {
  auto* state = static_cast<ClientState*>(user_data);
  state->response_ready.store(true);

  if (!resp || !resp->data || resp->size == 0) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] create_requirement returned empty response\n");
    return;
  }

  try {
    std::string identifier;
    if (false) {
#if defined(PYRAMID_ENABLE_FLATBUFFERS)
    } else if (resp->type_name &&
               std::strcmp(resp->type_name, FlatCodec::kContentType) == 0) {
      identifier = FlatCodec::fromBinaryIdentifier(resp->data, resp->size);
#endif
#if defined(PYRAMID_ENABLE_PROTOBUF)
    } else if (resp->type_name &&
               std::strcmp(resp->type_name, "application/protobuf") == 0) {
      identifier = ProtobufCodec::fromBinaryIdentifier(resp->data, resp->size);
#endif
    } else {
      const std::string payload(static_cast<const char*>(resp->data), resp->size);
      const auto response = nlohmann::json::parse(payload);
      if (response.is_string()) {
        identifier = response.get<std::string>();
      }
    }
    std::fprintf(stderr,
                 "[tactical_objects_test_client] create_requirement identifier: %s\n",
                 identifier.c_str());
    if (!identifier.empty()) {
      state->interest_id_received.store(true);
    }
  } catch (...) {
  }

  if (state->interest_id_received.load()) {
    state->interest_id_received.store(true);
  }
}

pcl_status_t onConfigure(pcl_container_t* container, void* user_data) {
  auto* state = static_cast<ClientState*>(user_data);
  Provided::subscribeEntityMatches(
      container, onEntityMatches, user_data, state->content_type.c_str());
  return PCL_OK;
}

} // namespace

int main(int argc, char* argv[]) {
  const char* host = "127.0.0.1";
  uint16_t port = 19123;
  int timeout_ms = 4000;
  std::string content_type = "application/json";

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
      host = argv[++i];
    } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--timeout-ms") == 0 && i + 1 < argc) {
      timeout_ms = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--content-type") == 0 && i + 1 < argc) {
      content_type = argv[++i];
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
  state.content_type = content_type;
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

  ObjectInterestRequirement request;
  request.source = ObjectSource::Local;
  request.policy = DataPolicy::Obtain;
  request.dimension.push_back(BattleDimension::Unspecified);
  request.point = Common::Point{};
  request.point->position.latitude = 50.0 * 0.017453292519943295;
  request.point->position.longitude = -1.0 * 0.017453292519943295;

  const pcl_status_t invoke_rc = Provided::invokeCreateRequirement(
      exec, request, onCreateRequirementResponse, &state, nullptr,
      state.content_type.c_str());
  if (invoke_rc != PCL_OK) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] Failed to invoke %s\n",
                 Provided::kSvcCreateRequirement);
    pcl_socket_transport_destroy(transport);
    pcl_executor_destroy(exec);
    pcl_container_destroy(container);
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
  pcl_socket_transport_destroy(transport);
  pcl_executor_destroy(exec);
  pcl_container_destroy(container);
  return ok ? 0 : 1;
}
