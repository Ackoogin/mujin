/// \file kittyhawk_consumer_test.cpp
/// \brief Live Kitty Hawk generated-P1 information-topic consumer.

#include "pyramid_services_uci_information_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

namespace info = pyramid::components::uci::information::services::provided;
namespace uci = pyramid::domain_model::uci;

namespace {
using namespace std::chrono_literals;
constexpr int kMinimumSamples = 3;
constexpr auto kTrafficTimeout = 20s;
constexpr auto kSignalTimeout = 30s;
constexpr double kPi = 3.14159265358979323846;

struct Results {
  int position_samples = 0;
  int observation_samples = 0;
  int signal_samples = 0;
  bool rf_normal = false;
  bool ir_normal = false;
  bool malformed = false;
};

bool plausibleWgs84(double latitude, double longitude) {
  return std::isfinite(latitude) && std::isfinite(longitude) &&
         latitude >= -kPi / 2.0 && latitude <= kPi / 2.0 &&
         longitude >= -kPi && longitude <= kPi;
}

bool contains(const std::string& value, const char* needle) {
  return value.find(needle) != std::string::npos;
}

std::filesystem::path writeManifest(const std::string& url,
                                    const std::filesystem::path& directory) {
  const auto path = directory / "kittyhawk_consumer.manifest";
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << "transport asb " << PYRAMID_LACAL_PLUGIN_PATH << " {\"url\":\"" << url
      << "\",\"service_id\":\"kittyhawk-consumer\",\"schema\":\"002.5.0\","
         "\"content_type\":\"application/oms-json\",\"peer_id\":\"asb\","
         "\"connect_timeout_ms\":3000,\"declare_reliability\":\"reliable\"}\n"
      << "route mission.position-report subscriber asb reliable\n"
      << "route mission.signal-report subscriber asb reliable\n"
      << "route mission.observation-measurement-report subscriber asb reliable\n"
      << "route mission.service-status subscriber asb reliable\n";
  return path;
}

struct CodecOwner {
  pcl_plugin_handle_t* handle = nullptr;
  bool load() {
    return pcl_plugin_load_codec(PYRAMID_OMS_JSON_CODEC_PATH, nullptr,
                                 pcl_codec_registry_default(), &handle) == PCL_OK &&
           pcl_codec_registry_get(pcl_codec_registry_default(),
                                  "application/oms-json") != nullptr;
  }
  ~CodecOwner() {
    if (handle) pcl_plugin_unload(handle);
    pcl_codec_registry_clear(pcl_codec_registry_default());
  }
};

class ConsumerComponent final : public pcl::Component {
 public:
  ConsumerComponent(pcl::Executor& executor, Results& results)
      : pcl::Component("kittyhawk_consumer"), service_(*this, executor,
                                                         "application/oms-json"),
        results_(results) {}

  info::ConsumedService& service() { return service_; }

 protected:
  pcl_status_t on_configure() override {
    if (!service_.subscribeMissionPositionReport([this](const auto& frame) {
          if (!frame.position_report) { results_.malformed = true; return; }
          const auto& point = frame.position_report->message_data.inertial_state.position;
          if (!plausibleWgs84(point.latitude, point.longitude)) {
            results_.malformed = true;
            return;
          }
          ++results_.position_samples;
        })) return PCL_ERR_NOMEM;
    if (!service_.subscribeMissionSignalReport([this](const auto& frame) {
          if (!frame.signal_report) { results_.malformed = true; return; }
          ++results_.signal_samples;
        })) return PCL_ERR_NOMEM;
    if (!service_.subscribeMissionObservationMeasurementReport([this](const auto& frame) {
          if (!frame.observation_measurement_report) {
            results_.malformed = true;
            return;
          }
          bool valid = false;
          for (const auto& observation : frame.observation_measurement_report->message_data.observation_measurement) {
            for (const auto& measurement : observation.measurements) {
              if (!measurement.reference_kinematics || !measurement.reference_kinematics->kinematics) continue;
              const auto& point = measurement.reference_kinematics->kinematics->platform_kinematics.position.fixed_point;
              valid = plausibleWgs84(point.latitude, point.longitude);
              if (valid) break;
            }
            if (valid) break;
          }
          if (!valid) { results_.malformed = true; return; }
          ++results_.observation_samples;
        })) return PCL_ERR_NOMEM;
    if (!service_.subscribeMissionServiceStatus([this](const auto& frame) {
          if (!frame.service_status) { results_.malformed = true; return; }
          const auto& status = frame.service_status->message_data;
          if (status.service_state != uci::ServiceStateEnum::Normal) return;
          const auto& label = status.service_id.descriptive_label;
          results_.rf_normal |= contains(label, "rf-fm-demod");
          results_.ir_normal |= contains(label, "ir-search-and-track");
        })) return PCL_ERR_NOMEM;
    return PCL_OK;
  }

 private:
  info::ConsumedService service_;
  Results& results_;
};

int run(const std::string& url, const std::filesystem::path& directory) {
  CodecOwner codec;
  if (!codec.load()) { std::fprintf(stderr, "FAIL: generated OMS JSON codec load failed\n"); return 2; }
  pcl::Executor executor;
  const auto manifest = writeManifest(url, directory);
  pcl_transport_routing_t* routing = nullptr;
  char diag[512] = "";
  if (pcl_transport_routing_load(executor.handle(), manifest.c_str(), &routing, diag,
                                 sizeof(diag)) != PCL_OK) {
    std::fprintf(stderr, "FAIL: routing load failed: %s\n", diag);
    return 2;
  }
  Results results;
  ConsumerComponent component(executor, results);
  const bool configured = component.configure() == PCL_OK &&
      component.service().configurePubSubTransport("{\"transport\":\"remote\",\"peer\":\"asb\"}") == PCL_OK &&
      component.activate() == PCL_OK && executor.add(component) == PCL_OK;
  if (!configured) { std::fprintf(stderr, "FAIL: consumer component configuration rejected\n"); return 2; }

  const auto traffic_deadline = std::chrono::steady_clock::now() + kTrafficTimeout;
  while (std::chrono::steady_clock::now() < traffic_deadline && !results.malformed &&
         (results.position_samples < kMinimumSamples || results.observation_samples < kMinimumSamples ||
          !results.rf_normal || !results.ir_normal)) executor.spinOnce(10);
  const auto signal_deadline = std::chrono::steady_clock::now() + kSignalTimeout;
  while (std::chrono::steady_clock::now() < signal_deadline && !results.malformed &&
         results.signal_samples == 0) executor.spinOnce(10);
  executor.remove(component);
  pcl_transport_routing_destroy(routing);
  std::filesystem::remove(manifest);

  std::printf("PositionReport: %s (%d valid samples)\n", results.position_samples >= kMinimumSamples ? "PASS" : "FAIL", results.position_samples);
  std::printf("ObservationMeasurementReport: %s (%d valid samples)\n", results.observation_samples >= kMinimumSamples ? "PASS" : "FAIL", results.observation_samples);
  std::printf("ServiceStatus: %s (rf-fm-demod=%d ir-search-and-track=%d)\n", (results.rf_normal && results.ir_normal) ? "PASS" : "FAIL", results.rf_normal, results.ir_normal);
  if (results.signal_samples == 0) std::printf("SignalReport: inconclusive -- no RF lock observed in this run\n");
  else std::printf("SignalReport: PASS (%d decoded samples)\n", results.signal_samples);
  const bool ok = !results.malformed && results.position_samples >= kMinimumSamples &&
      results.observation_samples >= kMinimumSamples && results.rf_normal && results.ir_normal;
  std::printf("%s: Kitty Hawk generated-P1 consumer over Sleet\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 2;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <sleet-url> <scratch-directory>\n", argv[0]);
    return 2;
  }
  return run(argv[1], argv[2]);
}
