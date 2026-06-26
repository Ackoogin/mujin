// Tactical Objects test client.
//
// Exercises the Tactical Objects provided services purely through the
// generated, fully-typed bindings: it subscribes to standard.entity_matches
// and standard.evidence_requirements, issues object_of_interest.create_requirement,
// and publishes standard.object_evidence through typed generated helpers.
// All wire encoding/decoding and plugin/codec selection is hidden inside the
// generated facade -- this client holds zero marshalling or wire knowledge.

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided_components.hpp"
#include "tactical_objects_codec_plugin_paths.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
}

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace Provided = pyramid::components::tactical_objects::services::provided;
namespace Consumed = pyramid::components::tactical_objects::services::consumed;
namespace Common = pyramid::domain_model::common;
using namespace pyramid::domain_model;

constexpr double kDegToRad = 0.017453292519943295;

struct CodecPluginDeleter {
  void operator()(pcl_plugin_handle_t* handle) const {
    if (handle) {
      pcl_plugin_unload(handle);
    }
  }
};

using CodecPlugin = std::unique_ptr<pcl_plugin_handle_t, CodecPluginDeleter>;

// The transport is supplied entirely by a runtime .so: the client links no
// transport library. The plugin is configured through the same opaque
// config_json that codec plugins receive (uniform pass-through), with the
// executor handle threaded through as an integer pointer the plugin reinstalls.
using TransportDestroyFn = void (*)(const pcl_transport_t*);

class LoadedTransport {
public:
  LoadedTransport() = default;
  LoadedTransport(const LoadedTransport&) = delete;
  LoadedTransport& operator=(const LoadedTransport&) = delete;
  ~LoadedTransport() {
    if (destroy_ && vtable_) {
      destroy_(vtable_);
    }
    if (handle_) {
      pcl_plugin_unload(handle_);
    }
  }

  bool load(const char* path, const char* config_json) {
    if (pcl_plugin_load_transport(path, config_json, &handle_, &vtable_) !=
            PCL_OK ||
        !vtable_) {
      return false;
    }
    destroy_ = reinterpret_cast<TransportDestroyFn>(
        pcl_plugin_symbol(handle_, "pcl_socket_transport_plugin_destroy"));
    return true;
  }

  const pcl_transport_t* vtable() const { return vtable_; }

private:
  pcl_plugin_handle_t* handle_ = nullptr;
  const pcl_transport_t* vtable_ = nullptr;
  TransportDestroyFn destroy_ = nullptr;
};

std::string makeSocketTransportConfig(const char* host,
                                      uint16_t port,
                                      pcl_executor_t* executor) {
  std::ostringstream out;
  out << "{\"role\":\"client\",\"host\":\"" << host << "\",\"port\":" << port
      << ",\"executor\":"
      << static_cast<unsigned long long>(
             reinterpret_cast<std::uintptr_t>(executor))
      << "}";
  return out.str();
}

class TacticalObjectsTestClient : public pcl::Component {
public:
  TacticalObjectsTestClient(pcl::Executor& executor, std::string content_type)
      : pcl::Component("tactical_objects_test_client"),
        content_type_(std::move(content_type)),
        consumer_(*this, executor, content_type_) {}

  std::future<Provided::Result<Identifier>>
  createRequirementAsync(const ObjectInterestRequirement& request) {
    return consumer_.objectOfInterestCreateRequirementAsync(request);
  }

  bool responseReady() const { return response_ready_; }
  bool interestIdReceived() const { return interest_id_received_; }
  int matchCount() const { return match_count_; }
  bool evidenceRequirementReceived() const {
    return evidence_requirement_received_;
  }
  bool evidencePublished() const { return evidence_published_; }

  void acceptCreateRequirementResult(Provided::Result<Identifier> result) {
    response_ready_ = true;
    if (!result.ok()) {
      std::fprintf(stderr,
                   "[tactical_objects_test_client] create_requirement failed: %d\n",
                   static_cast<int>(result.status));
      return;
    }

    std::fprintf(stderr,
                 "[tactical_objects_test_client] create_requirement identifier: %s\n",
                 result.value.c_str());
    if (!result.value.empty()) {
      interest_id_received_ = true;
    }
  }

protected:
  pcl_status_t on_configure() override {
    const pcl_port_t* matches_port = consumer_.subscribeEntityMatches(
        [this](const std::vector<ObjectMatch>& matches) {
          onEntityMatches(matches);
        });
    const pcl_port_t* requirements_port = consumer_.subscribeEvidenceRequirements(
        [this](const ObjectEvidenceRequirement& requirement) {
          onEvidenceRequirement(requirement);
        });
    publisher_ =
        addPublisher(Consumed::kTopicObjectEvidence, content_type_.c_str());
    return matches_port && requirements_port && publisher_ ? PCL_OK
                                                           : PCL_ERR_NOMEM;
  }

private:
  void onEntityMatches(const std::vector<ObjectMatch>& matches) {
    match_count_ = static_cast<int>(matches.size());
    std::fprintf(stderr,
                 "[tactical_objects_test_client] standard.entity_matches count=%d\n",
                 match_count_);
  }

  void onEvidenceRequirement(const ObjectEvidenceRequirement& requirement) {
    evidence_requirement_received_ = true;
    if (evidence_published_ || !publisher_) {
      return;
    }

    ObjectDetail evidence;
    evidence.id = "tactical-objects-test-evidence";
    evidence.entity_source = "tactical-objects-test-client";
    evidence.source.push_back(ObjectSource::Local);
    evidence.identity = StandardIdentity::Hostile;
    evidence.dimension = evidenceDimension(requirement);
    evidence.position = evidencePosition(requirement);
    evidence.quality = 0.95;
    evidence.creation_time = 1.0;

    const pcl_status_t rc =
        Consumed::publishObjectEvidence(publisher_, evidence,
                                        content_type_.c_str());
    if (rc == PCL_OK) {
      evidence_published_ = true;
      std::fprintf(stderr,
                   "[tactical_objects_test_client] standard.object_evidence "
                   "published id=%s\n",
                   evidence.id.c_str());
    } else {
      std::fprintf(stderr,
                   "[tactical_objects_test_client] standard.object_evidence "
                   "publish failed: %d\n",
                   static_cast<int>(rc));
    }
  }

  BattleDimension
  evidenceDimension(const ObjectEvidenceRequirement& requirement) const {
    if (!requirement.dimension.empty() &&
        requirement.dimension.front() != BattleDimension::Unspecified) {
      return requirement.dimension.front();
    }
    return BattleDimension::SeaSurface;
  }

  Common::GeodeticPosition
  evidencePosition(const ObjectEvidenceRequirement& requirement) const {
    Common::GeodeticPosition position{};
    position.latitude = 51.0 * kDegToRad;
    position.longitude = 0.0;

    if (requirement.point.has_value()) {
      return requirement.point->position;
    }

    if (requirement.poly_area.has_value() &&
        !requirement.poly_area->points.empty()) {
      double lat_sum = 0.0;
      double lon_sum = 0.0;
      for (const auto& point : requirement.poly_area->points) {
        lat_sum += point.latitude;
        lon_sum += point.longitude;
      }
      const auto count =
          static_cast<double>(requirement.poly_area->points.size());
      position.latitude = lat_sum / count;
      position.longitude = lon_sum / count;
    }

    return position;
  }

  std::string content_type_;
  Provided::ConsumedService consumer_;
  pcl_port_t* publisher_ = nullptr;
  bool response_ready_ = false;
  bool interest_id_received_ = false;
  bool evidence_requirement_received_ = false;
  bool evidence_published_ = false;
  int match_count_ = 0;
};

ObjectInterestRequirement makeRequirement() {
  ObjectInterestRequirement request;
  request.source = ObjectSource::Local;
  request.policy = DataPolicy::Obtain;
  request.dimension.push_back(BattleDimension::Unspecified);
  request.point = Common::Point{};
  request.point->position.latitude = 50.0 * kDegToRad;
  request.point->position.longitude = -1.0 * kDegToRad;
  return request;
}

}  // namespace

int main(int argc, char* argv[]) {
  const char* host = "127.0.0.1";
  uint16_t port = 19123;
  int timeout_ms = 4000;
  std::string content_type = "application/json";
  std::vector<std::string> codec_plugin_paths;
  std::string codec_config;
  std::string codec_manifest;
  std::string transport_plugin_path =
      kTacticalObjectsSocketTransportPlugin ? kTacticalObjectsSocketTransportPlugin
                                            : "";

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
      host = argv[++i];
    } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--timeout-ms") == 0 && i + 1 < argc) {
      timeout_ms = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--content-type") == 0 && i + 1 < argc) {
      content_type = argv[++i];
    } else if (std::strcmp(argv[i], "--codec-plugin") == 0 && i + 1 < argc) {
      codec_plugin_paths.push_back(argv[++i]);
    } else if (std::strcmp(argv[i], "--codec-config") == 0 && i + 1 < argc) {
      codec_config = argv[++i];
    } else if (std::strcmp(argv[i], "--transport-plugin") == 0 && i + 1 < argc) {
      transport_plugin_path = argv[++i];
    } else if (std::strcmp(argv[i], "--codec-manifest") == 0 && i + 1 < argc) {
      codec_manifest = argv[++i];
    }
  }
  const char* codec_config_json =
      codec_config.empty() ? nullptr : codec_config.c_str();
  if (codec_manifest.empty()) {
    if (const char* env = std::getenv("PCL_CODEC_MANIFEST")) {
      codec_manifest = env;
    }
  }

  if (transport_plugin_path.empty()) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] no transport plugin: pass "
                 "--transport-plugin <path>\n");
    return 2;
  }

  if (codec_plugin_paths.empty()) {
    // Deployment default: a manifest (from --codec-manifest or
    // PCL_CODEC_MANIFEST) lets the client run without naming each plugin. Fall
    // back to the build-tree default plugin set when no manifest is supplied.
    if (!codec_manifest.empty()) {
      if (pcl_codec_registry_load_plugins_from_manifest(
              pcl_codec_registry_default(), codec_manifest.c_str()) != PCL_OK) {
        std::fprintf(stderr, "failed to load codec manifest: %s\n",
                     codec_manifest.c_str());
        return 2;
      }
    } else {
      const auto rc = pcl_codec_registry_load_plugins_from_paths(
          pcl_codec_registry_default(),
          kTacticalObjectsDefaultCodecPlugins.data(),
          kTacticalObjectsDefaultCodecPlugins.size());
      if (rc != PCL_OK) {
        std::fprintf(stderr, "failed to load default codec plugins\n");
        return 2;
      }
    }
  }

  // Deployment wiring: load the requested codec plugin(s) into the default
  // registry. The generated facade picks them up transparently.
  std::vector<CodecPlugin> codec_plugins;
  for (const auto& path : codec_plugin_paths) {
    pcl_plugin_handle_t* handle = nullptr;
    if (pcl_plugin_load_codec(path.c_str(), codec_config_json,
                              pcl_codec_registry_default(),
                              &handle) != PCL_OK) {
      std::fprintf(stderr, "failed to load codec plugin: %s\n", path.c_str());
      return 2;
    }
    codec_plugins.emplace_back(handle);
  }

  pcl::Executor executor;
  if (!executor.handle()) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] Failed to create executor\n");
    return 1;
  }

  // Load the transport entirely from a runtime .so. The client links no
  // transport library: the socket implementation arrives as a plugin, composed
  // at runtime exactly like the codec.
  LoadedTransport transport;
  const std::string transport_config =
      makeSocketTransportConfig(host, port, executor.handle());
  if (!transport.load(transport_plugin_path.c_str(),
                      transport_config.c_str())) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] Failed to load transport "
                 "plugin %s or connect to %s:%u\n",
                 transport_plugin_path.c_str(), host, port);
    return 1;
  }

  executor.setTransport(transport.vtable());

  TacticalObjectsTestClient client{executor, content_type};
  if (client.configure() != PCL_OK ||
      client.activate() != PCL_OK ||
      executor.add(client) != PCL_OK) {
    std::fprintf(stderr,
                 "[tactical_objects_test_client] Failed to create container\n");
    return 1;
  }

  auto response = client.createRequirementAsync(makeRequirement());

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spinOnce(0);
    if (!client.responseReady() &&
        response.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
      client.acceptCreateRequirementResult(response.get());
    }
    if (client.responseReady() && client.matchCount() > 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::fprintf(stderr,
               "[tactical_objects_test_client] interest_id_received=%s "
               "evidence_requirement_received=%s evidence_published=%s "
               "matches=%d\n",
               client.interestIdReceived() ? "true" : "false",
               client.evidenceRequirementReceived() ? "true" : "false",
               client.evidencePublished() ? "true" : "false",
               client.matchCount());

  return client.interestIdReceived() && client.matchCount() > 0 ? 0 : 1;
}
