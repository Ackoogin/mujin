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

  /// \brief Dispatch an incoming message (for custom transport adapters).
  pcl_status_t dispatchIncoming(const char* topic, const pcl_msg_t* msg) {
    return pcl_executor_dispatch_incoming(handle_, topic, msg);
  }

  /// \brief Queue an incoming message from an external I/O thread.
  pcl_status_t postIncoming(const char* topic, const pcl_msg_t* msg) {
    return pcl_executor_post_incoming(handle_, topic, msg);
  }

  /// \brief Raw handle access.
  pcl_executor_t* handle() { return handle_; }
  const pcl_executor_t* handle() const { return handle_; }

private:
  pcl_executor_t* handle_ = nullptr;
};

} // namespace pcl

#endif // PCL_EXECUTOR_HPP
