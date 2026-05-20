#include "tobj_shared_memory_example.hpp"

#include "tobj_service_binding_handler.hpp"
#include "tobj_service_client_component.hpp"
#include "tobj_service_provider_component.hpp"

#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/executor.hpp>
#include <pcl/pcl_transport_shared_memory.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

namespace tobj_example {

namespace {

std::string uniqueBusName() {
  const auto ticks = std::chrono::steady_clock::now()
                         .time_since_epoch()
                         .count();
  return "tobj_service_binding_example_" + std::to_string(ticks);
}

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

    // All outbound service calls use this participant's shared-memory
    // transport unless an endpoint route selects something else.
    if (executor_.setTransport(
            pcl_shared_memory_transport_get_transport(transport_)) !=
        PCL_OK) {
      return false;
    }

    // A participant that provides services must install the transport gateway.
    // The gateway receives bus service frames and invokes service callbacks on
    // the PCL executor thread.
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

bool routeClientToProvider(pcl::Executor& executor) {
  namespace svc = pyramid::components::tactical_objects::services::provided;

  // Consumed endpoints are routed remote. With the shared-memory bus the
  // transport discovers the unique provider for each advertised service.
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

}  // namespace

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
      !routeClientToProvider(client.executor())) {
    return false;
  }

  // Spin the provider executor in the background while the client performs
  // blocking request/response waits on its own executor.
  std::atomic<bool> stop_server{false};
  std::thread server_thread([&] {
    while (!stop_server.load(std::memory_order_acquire)) {
      server.executor().spinOnce(10);
      std::this_thread::yield();
    }
  });

  const bool sequence_ok =
      client_component.runCreateReadDelete(client.executor().handle());

  stop_server.store(true, std::memory_order_release);
  server_thread.join();

  const bool final_ok =
      sequence_ok &&
      handler.createCount() == 1 &&
      handler.streamCount() == 1 &&
      handler.deleteCount() == 1 &&
      handler.empty();

  client.executor().remove(client_component);
  server.executor().remove(service);
  return final_ok;
}

}  // namespace tobj_example
