#include "pyramid/data_model/base.pb.h"
#include "pyramid/data_model/common.pb.h"
#include "pyramid/data_model/tactical.pb.h"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

namespace pyramid::services::tactical_objects::provided {

class GrpcServer {
public:
  GrpcServer();
  GrpcServer(GrpcServer&&) noexcept;
  GrpcServer& operator=(GrpcServer&&) noexcept;
  GrpcServer(const GrpcServer&) = delete;
  GrpcServer& operator=(const GrpcServer&) = delete;
  ~GrpcServer();

  bool started() const;
  explicit operator bool() const { return started(); }
  void shutdown();

private:
  struct Impl;
  explicit GrpcServer(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
  friend GrpcServer buildGrpcServer(const std::string& listen_address,
                                    pcl_executor_t* executor);
};

GrpcServer buildGrpcServer(const std::string& listen_address,
                           pcl_executor_t* executor);

}  // namespace pyramid::services::tactical_objects::provided

namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_tactical = pyramid::data_model::tactical;
namespace provided = pyramid::services::tactical_objects::provided;
namespace {

constexpr const char* kSvcCreateRequirement =
    "object_of_interest.create_requirement";

constexpr const char* kDefaultAddress = "127.0.0.1:50111";
constexpr const char* kExpectedInterestId = "ada-grpc-interest";
constexpr const char* kUnexpectedIdentifier = "grpc-invalid-request";

std::atomic<bool> g_shutdown{false};

void signal_handler(int) { g_shutdown.store(true); }

std::string serialize(const google::protobuf::MessageLite& message) {
  std::string bytes;
  if (!message.SerializeToString(&bytes)) {
    return {};
  }
  return bytes;
}

bool requestMatchesExpectedShape(
    const proto_tactical::ObjectInterestRequirement& request) {
  if (!request.has_base() || !request.base().has_base() ||
      !request.base().base().has_id()) {
    return false;
  }
  if (request.base().base().id().value() != kExpectedInterestId) {
    return false;
  }
  if (!request.base().has_status() ||
      request.base().status().status() != proto_common::PROGRESS_IN_PROGRESS ||
      !request.base().status().has_quality() ||
      std::fabs(request.base().status().quality().value() - 0.75) > 1e-9) {
    return false;
  }
  if (!request.has_source() ||
      request.source() != proto_tactical::OBJECT_SOURCE_RADAR) {
    return false;
  }
  if (request.policy() != proto_common::DATA_POLICY_OBTAIN) {
    return false;
  }
  if (request.dimension_size() != 2 ||
      request.dimension(0) != proto_common::BATTLE_DIMENSION_SEA_SURFACE ||
      request.dimension(1) != proto_common::BATTLE_DIMENSION_AIR) {
    return false;
  }
  if (!request.has_poly_area() || request.poly_area().points_size() != 4) {
    return false;
  }
  return true;
}

std::string buildIdentifierValue(
    const proto_tactical::ObjectInterestRequirement& request) {
  return std::string("grpc-interest-") + request.base().base().id().value() +
         "-" + std::to_string(request.poly_area().points_size()) + "-" +
         std::to_string(request.dimension_size());
}

pcl_status_t handleCreateRequirement(pcl_container_t*,
                                     const pcl_msg_t* request,
                                     pcl_msg_t* response,
                                     pcl_svc_context_t*,
                                     void*) {
  if (!request || !response || !request->data || request->size == 0 ||
      !request->type_name ||
      std::strcmp(request->type_name, "application/protobuf") != 0) {
    return PCL_ERR_INVALID;
  }

  proto_tactical::ObjectInterestRequirement decoded;
  if (!decoded.ParseFromArray(request->data, static_cast<int>(request->size))) {
    return PCL_ERR_INVALID;
  }

  proto_base::Identifier encoded;
  encoded.set_value(requestMatchesExpectedShape(decoded)
                        ? buildIdentifierValue(decoded)
                        : kUnexpectedIdentifier);

  const auto bytes = serialize(encoded);
  auto* storage = std::malloc(bytes.size());
  if (!storage) {
    return PCL_ERR_NOMEM;
  }
  std::memcpy(storage, bytes.data(), bytes.size());
  response->data = storage;
  response->size = static_cast<uint32_t>(bytes.size());
  response->type_name = "application/protobuf";
  return PCL_OK;
}

pcl_status_t onConfigure(pcl_container_t* container, void*) {
  auto* port = pcl_container_add_service(
      container, kSvcCreateRequirement, "application/protobuf",
      handleCreateRequirement, nullptr);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string listen_address = kDefaultAddress;
  std::string ready_file;
  int timeout_secs = 20;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--address") == 0 && i + 1 < argc) {
      listen_address = argv[++i];
    } else if (std::strcmp(argv[i], "--ready-file") == 0 && i + 1 < argc) {
      ready_file = argv[++i];
    } else if (std::strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
      timeout_secs = std::atoi(argv[++i]);
    }
  }

  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT, signal_handler);

  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigure;

  pcl_executor_t* executor = pcl_executor_create();
  if (!executor) {
    std::fprintf(stderr, "[grpc_server] FAIL: could not create executor\n");
    return 1;
  }

  pcl_container_t* container =
      pcl_container_create("grpc_tactical_objects_service", &callbacks, nullptr);
  if (!container || pcl_container_configure(container) != PCL_OK ||
      pcl_container_activate(container) != PCL_OK ||
      pcl_executor_add(executor, container) != PCL_OK) {
    std::fprintf(stderr, "[grpc_server] FAIL: could not configure container\n");
    if (container) {
      pcl_container_destroy(container);
    }
    pcl_executor_destroy(executor);
    return 1;
  }

  std::atomic<bool> spin_stop{false};
  std::thread executor_thread([&] {
    while (!spin_stop.load()) {
      pcl_executor_spin_once(executor, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  std::fprintf(stderr, "[grpc_server] Starting on %s\n", listen_address.c_str());
  auto host = provided::buildGrpcServer(listen_address, executor);
  if (!host.started()) {
    std::fprintf(stderr, "[grpc_server] FAIL: could not start server\n");
    spin_stop.store(true);
    executor_thread.join();
    pcl_container_destroy(container);
    pcl_executor_destroy(executor);
    return 1;
  }

  if (!ready_file.empty()) {
    std::ofstream out(ready_file);
    out << listen_address << std::endl;
    out.close();
    std::fprintf(stderr, "[grpc_server] Wrote ready file %s\n",
                 ready_file.c_str());
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(timeout_secs);
  while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::fprintf(stderr, "[grpc_server] Shutting down\n");
  host.shutdown();
  spin_stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
  return 0;
}
