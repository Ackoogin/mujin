/// \file executor.hpp
/// \brief Header-only C++ convenience wrapper for pcl_executor_t.
///
/// RAII wrapper that manages the executor lifecycle and provides
/// type-safe add() accepting pcl::Component references.
#ifndef PCL_EXECUTOR_HPP
#define PCL_EXECUTOR_HPP

#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport.h"
#include "pcl/component.hpp"

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace pcl {

/// \brief C++ wrapper for pcl_executor_t.
class Executor {
public:
  Executor() : handle_(pcl_executor_create()) {}

  ~Executor() {
    if (handle_) pcl_executor_destroy(handle_);
  }

  // non-copyable, movable
  Executor(const Executor&) = delete;
  Executor& operator=(const Executor&) = delete;

  Executor(Executor&& other) noexcept : handle_(other.handle_) {
    other.handle_ = nullptr;
  }

  Executor& operator=(Executor&& other) noexcept {
    if (this != &other) {
      if (handle_) pcl_executor_destroy(handle_);
      handle_ = other.handle_;
      other.handle_ = nullptr;
    }
    return *this;
  }

  /// \brief Add a Component to the executor.
  pcl_status_t add(Component& component) {
    return pcl_executor_add(handle_, component.handle());
  }

  /// \brief Add a raw container handle to the executor.
  pcl_status_t add(pcl_container_t* c) {
    return pcl_executor_add(handle_, c);
  }

  /// \brief Remove a Component from the executor without destroying it.
  pcl_status_t remove(Component& component) {
    return pcl_executor_remove(handle_, component.handle());
  }

  /// \brief Remove a raw container handle from the executor.
  pcl_status_t remove(pcl_container_t* c) {
    return pcl_executor_remove(handle_, c);
  }

  /// \brief Block and run the tick loop.
  pcl_status_t spin() {
    return pcl_executor_spin(handle_);
  }

  /// \brief Process one round of pending work then return.
  pcl_status_t spinOnce(uint32_t timeout_ms = 0) {
    return pcl_executor_spin_once(handle_, timeout_ms);
  }

  /// \brief Signal-safe shutdown request.
  void requestShutdown() {
    pcl_executor_request_shutdown(handle_);
  }

  /// \brief Graceful shutdown with deadline.
  pcl_status_t shutdownGraceful(uint32_t timeout_ms = 5000) {
    return pcl_executor_shutdown_graceful(handle_, timeout_ms);
  }

  /// \brief Set a custom transport adapter.
  pcl_status_t setTransport(const pcl_transport_t* transport) {
    return pcl_executor_set_transport(handle_, transport);
  }

  /// \brief Register a named peer transport.
  pcl_status_t registerTransport(std::string_view peer_id,
                                 const pcl_transport_t* transport) {
    const std::string peer(peer_id);
    return pcl_executor_register_transport(handle_, peer.c_str(), transport);
  }

  /// \brief Dispatch an incoming message (for custom transport adapters).
  pcl_status_t dispatchIncoming(const char* topic, const pcl_msg_t* msg) {
    return pcl_executor_dispatch_incoming(handle_, topic, msg);
  }

  /// \brief Queue an incoming message from an external I/O thread.
  pcl_status_t postIncoming(const char* topic, const pcl_msg_t* msg) {
    return pcl_executor_post_incoming(handle_, topic, msg);
  }

  /// \brief Queue an incoming message from a named remote peer.
  pcl_status_t postRemoteIncoming(std::string_view peer_id,
                                  std::string_view topic,
                                  const pcl_msg_t* msg) {
    const std::string peer(peer_id);
    const std::string topic_name(topic);
    return pcl_executor_post_remote_incoming(handle_, peer.c_str(),
                                             topic_name.c_str(), msg);
  }

  /// \brief Invoke a service asynchronously through the configured transport.
  pcl_status_t invokeAsync(const char*      service_name,
                           const pcl_msg_t* request,
                           pcl_resp_cb_fn_t callback,
                           void*            user_data = nullptr) {
    return pcl_executor_invoke_async(handle_, service_name, request,
                                     callback, user_data);
  }

  /// \brief Publish a message through the executor.
  pcl_status_t publish(std::string_view topic, const pcl_msg_t* msg) {
    const std::string topic_name(topic);
    return pcl_executor_publish(handle_, topic_name.c_str(), msg);
  }

  /// \brief Configure a route for a consumed/provided endpoint.
  pcl_status_t setEndpointRoute(
      std::string_view endpoint_name,
      pcl_endpoint_kind_t kind,
      uint32_t route_mode,
      std::initializer_list<std::string_view> peer_ids = {}) {
    const std::string endpoint(endpoint_name);
    std::vector<std::string> storage;
    std::vector<const char*> peers;
    storage.reserve(peer_ids.size());
    peers.reserve(peer_ids.size());

    for (const auto peer_id : peer_ids) {
      storage.emplace_back(peer_id);
      peers.push_back(storage.back().c_str());
    }

    pcl_endpoint_route_t route = {};
    route.endpoint_name = endpoint.c_str();
    route.endpoint_kind = kind;
    route.route_mode = route_mode;
    route.peer_ids = peers.empty() ? nullptr : peers.data();
    route.peer_count = static_cast<uint32_t>(peers.size());
    return pcl_executor_set_endpoint_route(handle_, &route);
  }

  pcl_status_t routeLocal(std::string_view endpoint_name,
                          pcl_endpoint_kind_t kind = PCL_ENDPOINT_CONSUMED) {
    return setEndpointRoute(endpoint_name, kind, PCL_ROUTE_LOCAL);
  }

  pcl_status_t routeRemote(
      std::string_view endpoint_name,
      std::string_view peer_id,
      pcl_endpoint_kind_t kind = PCL_ENDPOINT_CONSUMED) {
    return setEndpointRoute(endpoint_name, kind, PCL_ROUTE_REMOTE, {peer_id});
  }

  pcl_status_t routeRemote(
      std::string_view endpoint_name,
      std::initializer_list<std::string_view> peer_ids,
      pcl_endpoint_kind_t kind = PCL_ENDPOINT_CONSUMED) {
    return setEndpointRoute(endpoint_name, kind, PCL_ROUTE_REMOTE, peer_ids);
  }

  pcl_status_t routeLocalAndRemote(
      std::string_view endpoint_name,
      std::string_view peer_id,
      pcl_endpoint_kind_t kind) {
    return setEndpointRoute(endpoint_name, kind,
                            PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, {peer_id});
  }

  pcl_status_t routeLocalAndRemote(
      std::string_view endpoint_name,
      std::initializer_list<std::string_view> peer_ids,
      pcl_endpoint_kind_t kind) {
    return setEndpointRoute(endpoint_name, kind,
                            PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, peer_ids);
  }

  /// \brief Raw handle access.
  pcl_executor_t* handle() { return handle_; }
  const pcl_executor_t* handle() const { return handle_; }

private:
  pcl_executor_t* handle_ = nullptr;
};

} // namespace pcl

#endif // PCL_EXECUTOR_HPP
