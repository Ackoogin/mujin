// Fully worked C++ PYRAMID service-binding example.
//
// Demonstrates the intended user-facing path:
//   1. subclass the generated ServiceHandler,
//   2. register generated service channels with a PCL component,
//   3. run two components on separate executors joined by shared memory,
//   4. call create_requirement -> read_requirement -> delete_requirement.

#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/pcl_transport_shared_memory.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace svc = pyramid::components::tactical_objects::services::provided;
namespace model = pyramid::domain_model;

namespace {

constexpr auto kTimeout = std::chrono::seconds(3);

std::string uniqueBusName() {
  const auto ticks = std::chrono::steady_clock::now()
                         .time_since_epoch()
                         .count();
  return "tobj_service_binding_example_" + std::to_string(ticks);
}

class DemoObjectInterestHandler : public svc::ServiceHandler {
public:
  model::Identifier handleObjectOfInterestCreateRequirement(
      const model::ObjectInterestRequirement& request) override {
    model::ObjectInterestRequirement stored = request;
    if (stored.base.id.empty()) {
      stored.base.id = "interest-" + std::to_string(next_id_++);
    }
    requirements_[stored.base.id] = stored;
    ++create_count_;
    return stored.base.id;
  }

  std::vector<model::ObjectInterestRequirement>
  handleObjectOfInterestReadRequirement(const model::Query& request) override {
    ++read_count_;
    std::vector<model::ObjectInterestRequirement> result;
    if (request.id.empty()) {
      for (const auto& entry : requirements_) {
        result.push_back(entry.second);
      }
      return result;
    }

    for (const auto& id : request.id) {
      const auto found = requirements_.find(id);
      if (found != requirements_.end()) {
        result.push_back(found->second);
      }
    }
    return result;
  }

  model::Ack handleObjectOfInterestDeleteRequirement(
      const model::Identifier& request) override {
    ++delete_count_;
    return model::Ack{requirements_.erase(request) == 1u};
  }

  int createCount() const { return create_count_; }
  int readCount() const { return read_count_; }
  int deleteCount() const { return delete_count_; }
  bool empty() const { return requirements_.empty(); }

private:
  int next_id_ = 1;
  int create_count_ = 0;
  int read_count_ = 0;
  int delete_count_ = 0;
  std::map<model::Identifier, model::ObjectInterestRequirement> requirements_;
};

class ObjectInterestServiceComponent final : public pcl::Component {
public:
  ObjectInterestServiceComponent(std::string content_type,
                                 svc::ServiceHandler& handler)
      : pcl::Component("object_interest_service"),
        content_type_(std::move(content_type)),
        handler_(handler) {
    bindings_.reserve(3);
    ports_.reserve(3);
  }

protected:
  pcl_status_t on_configure() override {
    if (!svc::supportsContentType(content_type_.c_str())) {
      return PCL_ERR_INVALID;
    }

    if (!addGeneratedService(
            svc::kSvcObjectOfInterestCreateRequirement,
            svc::ServiceChannel::ObjectOfInterestCreateRequirement)) {
      return PCL_ERR_NOMEM;
    }
    if (!addGeneratedService(
            svc::kSvcObjectOfInterestReadRequirement,
            svc::ServiceChannel::ObjectOfInterestReadRequirement)) {
      return PCL_ERR_NOMEM;
    }
    if (!addGeneratedService(
            svc::kSvcObjectOfInterestDeleteRequirement,
            svc::ServiceChannel::ObjectOfInterestDeleteRequirement)) {
      return PCL_ERR_NOMEM;
    }
    return PCL_OK;
  }

private:
  struct ServiceBinding {
    ObjectInterestServiceComponent* self = nullptr;
    svc::ServiceChannel channel =
        svc::ServiceChannel::ObjectOfInterestCreateRequirement;
  };

  bool addGeneratedService(const char* service_name,
                           svc::ServiceChannel channel) {
    bindings_.push_back(ServiceBinding{this, channel});
    pcl::Port port = addService(
        service_name,
        content_type_.c_str(),
        &ObjectInterestServiceComponent::dispatchGeneratedService,
        &bindings_.back());
    if (!port) {
      return false;
    }
    ports_.push_back(port);
    return ports_.back().routeRemote("client") == PCL_OK;
  }

  static pcl_status_t dispatchGeneratedService(pcl_container_t*,
                                               const pcl_msg_t* request,
                                               pcl_msg_t* response,
                                               pcl_svc_context_t*,
                                               void* user_data) {
    auto* binding = static_cast<ServiceBinding*>(user_data);
    if (!binding || !binding->self || !response) {
      return PCL_ERR_INVALID;
    }

    ObjectInterestServiceComponent& self = *binding->self;
    void* response_buf = nullptr;
    size_t response_size = 0;
    svc::dispatch(
        self.handler_,
        binding->channel,
        request ? request->data : nullptr,
        request ? request->size : 0u,
        request && request->type_name ? request->type_name
                                      : self.content_type_.c_str(),
        &response_buf,
        &response_size);

    self.response_storage_.clear();
    if (response_buf && response_size > 0u) {
      self.response_storage_.assign(static_cast<const char*>(response_buf),
                                    response_size);
      std::free(response_buf);
    }

    response->data = self.response_storage_.empty()
                         ? nullptr
                         : const_cast<char*>(self.response_storage_.data());
    response->size = static_cast<uint32_t>(self.response_storage_.size());
    response->type_name = request && request->type_name
                              ? request->type_name
                              : self.content_type_.c_str();
    return PCL_OK;
  }

  std::string content_type_;
  svc::ServiceHandler& handler_;
  std::vector<ServiceBinding> bindings_;
  std::vector<pcl::Port> ports_;
  std::string response_storage_;
};

class ObjectInterestClientComponent final : public pcl::Component {
public:
  explicit ObjectInterestClientComponent(std::string content_type)
      : pcl::Component("object_interest_client"),
        content_type_(std::move(content_type)) {}

  bool runCreateReadDelete(pcl_executor_t* executor) {
    model::ObjectInterestRequirement request;
    request.base.id = "interest-demo-001";
    request.base.source = "example-client";
    request.policy = model::DataPolicy::Obtain;
    request.dimension.push_back(model::BattleDimension::Air);
    model::Point point;
    point.position.latitude = 51.477811;
    point.position.longitude = -0.001475;
    request.point = point;

    model::Identifier created_id;
    if (!invokeCreate(executor, request, &created_id) ||
        created_id != request.base.id) {
      return false;
    }

    std::vector<model::ObjectInterestRequirement> read_back;
    model::Query query;
    query.id.push_back(created_id);
    if (!invokeRead(executor, query, &read_back) ||
        read_back.size() != 1u ||
        read_back.front().base.id != created_id ||
        read_back.front().policy != model::DataPolicy::Obtain) {
      return false;
    }

    model::Ack deleted;
    return invokeDelete(executor, created_id, &deleted) && deleted.success;
  }

protected:
  pcl_status_t on_configure() override {
    return svc::supportsContentType(content_type_.c_str()) ? PCL_OK
                                                           : PCL_ERR_INVALID;
  }

private:
  struct ResponseState {
    std::atomic<bool> done{false};
    bool decoded = false;
    model::Identifier id;
    std::vector<model::ObjectInterestRequirement> requirements;
    model::Ack ack{};
  };

  bool waitForResponse(pcl_executor_t* executor, ResponseState& state) {
    const auto deadline = std::chrono::steady_clock::now() + kTimeout;
    while (!state.done.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(executor, 10);
      std::this_thread::yield();
    }
    return state.done.load(std::memory_order_acquire) && state.decoded;
  }

  bool invokeCreate(pcl_executor_t* executor,
                    const model::ObjectInterestRequirement& request,
                    model::Identifier* out) {
    ResponseState state;
    const pcl_status_t rc = svc::invokeObjectOfInterestCreateRequirement(
        executor,
        request,
        [](const pcl_msg_t* msg, void* user_data) {
          auto* response = static_cast<ResponseState*>(user_data);
          response->decoded =
              svc::decodeObjectOfInterestCreateRequirementResponse(
                  msg,
                  &response->id);
          response->done.store(true, std::memory_order_release);
        },
        &state,
        nullptr,
        content_type_.c_str());
    if (rc != PCL_OK || !waitForResponse(executor, state)) {
      return false;
    }
    *out = state.id;
    return true;
  }

  bool invokeRead(pcl_executor_t* executor,
                  const model::Query& request,
                  std::vector<model::ObjectInterestRequirement>* out) {
    ResponseState state;
    const pcl_status_t rc = svc::invokeObjectOfInterestReadRequirement(
        executor,
        request,
        [](const pcl_msg_t* msg, void* user_data) {
          auto* response = static_cast<ResponseState*>(user_data);
          response->decoded =
              svc::decodeObjectOfInterestReadRequirementResponse(
                  msg,
                  &response->requirements);
          response->done.store(true, std::memory_order_release);
        },
        &state,
        nullptr,
        content_type_.c_str());
    if (rc != PCL_OK || !waitForResponse(executor, state)) {
      return false;
    }
    *out = state.requirements;
    return true;
  }

  bool invokeDelete(pcl_executor_t* executor,
                    const model::Identifier& request,
                    model::Ack* out) {
    ResponseState state;
    const pcl_status_t rc = svc::invokeObjectOfInterestDeleteRequirement(
        executor,
        request,
        [](const pcl_msg_t* msg, void* user_data) {
          auto* response = static_cast<ResponseState*>(user_data);
          response->decoded =
              svc::decodeObjectOfInterestDeleteRequirementResponse(
                  msg,
                  &response->ack);
          response->done.store(true, std::memory_order_release);
        },
        &state,
        nullptr,
        content_type_.c_str());
    if (rc != PCL_OK || !waitForResponse(executor, state)) {
      return false;
    }
    *out = state.ack;
    return true;
  }

  std::string content_type_;
};

class SharedMemoryParticipant {
public:
  SharedMemoryParticipant(const char* bus_name, const char* participant_id)
      : transport_(pcl_shared_memory_transport_create(
            bus_name,
            participant_id,
            executor_.handle())) {}

  ~SharedMemoryParticipant() {
    if (transport_) {
      pcl_shared_memory_transport_destroy(transport_);
    }
  }

  bool valid() const { return transport_ != nullptr; }
  pcl::Executor& executor() { return executor_; }

  bool attachTransport(bool with_gateway) {
    if (!transport_) {
      return false;
    }
    if (executor_.setTransport(
            pcl_shared_memory_transport_get_transport(transport_)) !=
        PCL_OK) {
      return false;
    }
    if (!with_gateway) {
      return true;
    }

    pcl_container_t* gateway =
        pcl_shared_memory_transport_gateway_container(transport_);
    return gateway && pcl_container_configure(gateway) == PCL_OK &&
           pcl_container_activate(gateway) == PCL_OK &&
           executor_.add(gateway) == PCL_OK;
  }

private:
  pcl::Executor executor_;
  pcl_shared_memory_transport_t* transport_ = nullptr;
};

bool routeClientToSharedMemoryProvider(pcl::Executor& executor) {
  return executor.setEndpointRoute(
             svc::kSvcObjectOfInterestCreateRequirement,
             PCL_ENDPOINT_CONSUMED,
             PCL_ROUTE_REMOTE) == PCL_OK &&
         executor.setEndpointRoute(
             svc::kSvcObjectOfInterestReadRequirement,
             PCL_ENDPOINT_CONSUMED,
             PCL_ROUTE_REMOTE) == PCL_OK &&
         executor.setEndpointRoute(
             svc::kSvcObjectOfInterestDeleteRequirement,
             PCL_ENDPOINT_CONSUMED,
             PCL_ROUTE_REMOTE) == PCL_OK;
}

bool runSharedMemoryExample(const char* content_type) {
  const std::string bus = uniqueBusName();

  SharedMemoryParticipant server(bus.c_str(), "server");
  SharedMemoryParticipant client(bus.c_str(), "client");
  if (!server.valid() || !client.valid() ||
      !server.attachTransport(true) ||
      !client.attachTransport(false)) {
    return false;
  }

  DemoObjectInterestHandler handler;
  ObjectInterestServiceComponent service(content_type, handler);
  ObjectInterestClientComponent client_component(content_type);
  if (service.configure() != PCL_OK ||
      service.activate() != PCL_OK ||
      server.executor().add(service) != PCL_OK ||
      client_component.configure() != PCL_OK ||
      client_component.activate() != PCL_OK ||
      client.executor().add(client_component) != PCL_OK ||
      !routeClientToSharedMemoryProvider(client.executor())) {
    return false;
  }

  std::atomic<bool> stop_server{false};
  std::thread server_thread([&] {
    while (!stop_server.load(std::memory_order_acquire)) {
      server.executor().spinOnce(10);
      std::this_thread::yield();
    }
  });

  const bool ok =
      client_component.runCreateReadDelete(client.executor().handle());
  stop_server.store(true, std::memory_order_release);
  server_thread.join();

  const bool final_ok =
      ok &&
      handler.createCount() == 1 &&
      handler.readCount() == 1 &&
      handler.deleteCount() == 1 &&
      handler.empty();

  client.executor().remove(client_component);
  server.executor().remove(service);
  return final_ok;
}

}  // namespace

int main() {
  if (!runSharedMemoryExample(svc::kFlatBuffersContentType)) {
    std::cerr << "shared-memory service-binding example failed\n";
    return 1;
  }

  std::cout << "service-binding example completed\n";
  return 0;
}
