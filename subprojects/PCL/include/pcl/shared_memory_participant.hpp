/// \file shared_memory_participant.hpp
/// \brief RAII C++ wrapper around pcl_shared_memory_transport_t.
///
/// Bundles the executor, transport, and (optionally) gateway-container
/// configuration that a shared-memory bus participant needs. Replaces the
/// per-example boilerplate previously inlined in tests/examples.
#ifndef PCL_SHARED_MEMORY_PARTICIPANT_HPP
#define PCL_SHARED_MEMORY_PARTICIPANT_HPP

#include "pcl/executor.hpp"
#include "pcl/pcl_transport_shared_memory.h"

#include <string>
#include <string_view>

namespace pcl {

enum class WithGateway { No, Yes };

/// \brief One participant on a shared-memory bus.
///
/// Construction joins the bus, sets the executor's outbound transport to the
/// shared-memory adapter, and (when \p with_gateway == Yes) configures and
/// activates the gateway container so inbound service requests can dispatch
/// to provided services. Destruction tears everything down.
///
/// After construction call valid() to check status. Use executor() to add
/// components and either spin manually or pass to pcl::SpinThread.
class SharedMemoryParticipant {
public:
  SharedMemoryParticipant(std::string_view bus_name,
                          std::string_view participant_id,
                          WithGateway with_gateway = WithGateway::No)
      : bus_(bus_name), participant_id_(participant_id) {
    transport_ = pcl_shared_memory_transport_create(
        bus_.c_str(), participant_id_.c_str(), executor_.handle());
    if (!transport_) return;

    if (executor_.setTransport(
            pcl_shared_memory_transport_get_transport(transport_)) != PCL_OK) {
      destroyTransport();
      return;
    }

    if (with_gateway == WithGateway::No) {
      ready_ = true;
      return;
    }

    pcl_container_t* gateway =
        pcl_shared_memory_transport_gateway_container(transport_);
    if (!gateway ||
        pcl_container_configure(gateway) != PCL_OK ||
        pcl_container_activate(gateway) != PCL_OK ||
        executor_.add(gateway) != PCL_OK) {
      destroyTransport();
      return;
    }

    gateway_ = gateway;
    ready_ = true;
  }

  ~SharedMemoryParticipant() { destroyTransport(); }

  SharedMemoryParticipant(const SharedMemoryParticipant&) = delete;
  SharedMemoryParticipant& operator=(const SharedMemoryParticipant&) = delete;
  SharedMemoryParticipant(SharedMemoryParticipant&&) = delete;
  SharedMemoryParticipant& operator=(SharedMemoryParticipant&&) = delete;

  bool valid() const { return ready_; }
  explicit operator bool() const { return ready_; }

  Executor& executor() { return executor_; }
  const Executor& executor() const { return executor_; }

  /// \brief Underlying transport handle (rarely needed by users).
  pcl_shared_memory_transport_t* transport() const { return transport_; }

  /// \brief Bus name passed at construction.
  const std::string& busName() const { return bus_; }

  /// \brief Participant id passed at construction.
  const std::string& participantId() const { return participant_id_; }

private:
  void destroyTransport() {
    if (gateway_) {
      executor_.remove(gateway_);
      gateway_ = nullptr;
    }
    if (transport_) {
      pcl_shared_memory_transport_destroy(transport_);
      transport_ = nullptr;
    }
    ready_ = false;
  }

  std::string bus_;
  std::string participant_id_;
  Executor executor_;
  pcl_shared_memory_transport_t* transport_ = nullptr;
  pcl_container_t* gateway_ = nullptr;
  bool ready_ = false;
};

}  // namespace pcl

#endif  // PCL_SHARED_MEMORY_PARTICIPANT_HPP
