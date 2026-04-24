/// \file component.hpp
/// \brief Header-only C++ convenience wrapper for pcl_container_t.
///
/// Provides a virtual-method base class that trampolines through the
/// C callback API.  Users subclass pcl::Component and override lifecycle
/// methods.  RAII: destructor calls pcl_container_destroy().
#ifndef PCL_COMPONENT_HPP
#define PCL_COMPONENT_HPP

#include <pcl/pcl_container.h>
#include <pcl/pcl_log.h>

#include <string>
#include <string_view>
#include <initializer_list>
#include <functional>
#include <cstdarg>
#include <cstdio>
#include <utility>
#include <vector>

namespace pcl {

/// \brief Lightweight convenience wrapper around a concrete pcl_port_t.
class Port {
public:
  Port() = default;

  Port(pcl_port_t* handle, std::string_view default_type_name = {})
      : handle_(handle), default_type_name_(default_type_name) {}

  bool valid() const { return handle_ != nullptr; }
  explicit operator bool() const { return valid(); }
  operator pcl_port_t*() const { return handle_; }

  pcl_port_t* handle() const { return handle_; }
  std::string_view defaultTypeName() const { return default_type_name_; }

  pcl_status_t setRoute(uint32_t route_mode,
                        std::initializer_list<std::string_view> peer_ids = {}) const {
    if (!handle_) return PCL_ERR_INVALID;

    std::vector<std::string> storage;
    std::vector<const char*> peers;
    storage.reserve(peer_ids.size());
    peers.reserve(peer_ids.size());

    for (const auto peer_id : peer_ids) {
      storage.emplace_back(peer_id);
      peers.push_back(storage.back().c_str());
    }

    return pcl_port_set_route(handle_,
                              route_mode,
                              peers.empty() ? nullptr : peers.data(),
                              static_cast<uint32_t>(peers.size()));
  }

  pcl_status_t routeLocal() const {
    return setRoute(PCL_ROUTE_LOCAL);
  }

  pcl_status_t routeRemote(std::string_view peer_id) const {
    return setRoute(PCL_ROUTE_REMOTE, {peer_id});
  }

  pcl_status_t routeRemote(
      std::initializer_list<std::string_view> peer_ids) const {
    return setRoute(PCL_ROUTE_REMOTE, peer_ids);
  }

  pcl_status_t routeLocalAndRemote(std::string_view peer_id) const {
    return setRoute(PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, {peer_id});
  }

  pcl_status_t routeLocalAndRemote(
      std::initializer_list<std::string_view> peer_ids) const {
    return setRoute(PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, peer_ids);
  }

  pcl_status_t publish(const pcl_msg_t* msg) const {
    return handle_ ? pcl_port_publish(handle_, msg) : PCL_ERR_INVALID;
  }

  pcl_status_t publish(std::string_view payload,
                       std::string_view type_name = {}) const {
    if (!handle_) return PCL_ERR_INVALID;

    const std::string effective_type =
        type_name.empty() ? default_type_name_ : std::string(type_name);
    pcl_msg_t msg = {};
    msg.data = payload.empty() ? nullptr : payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = effective_type.empty() ? nullptr : effective_type.c_str();
    return pcl_port_publish(handle_, &msg);
  }

private:
  pcl_port_t* handle_ = nullptr;
  std::string default_type_name_;
};

/// \brief C++ base class wrapping a pcl_container_t.
///
/// Subclass and override on_configure(), on_activate(), on_tick(), etc.
/// All methods are called on the executor thread — no synchronization
/// needed in user code.
class Component {
public:
  explicit Component(std::string_view name) {
    pcl_callbacks_t cbs = {};
    cbs.on_configure  = trampoline_configure;
    cbs.on_activate   = trampoline_activate;
    cbs.on_deactivate = trampoline_deactivate;
    cbs.on_cleanup    = trampoline_cleanup;
    cbs.on_shutdown   = trampoline_shutdown;
    cbs.on_tick       = trampoline_tick;

    handle_ = pcl_container_create(std::string(name).c_str(), &cbs, this);
  }

  virtual ~Component() {
    if (handle_) pcl_container_destroy(handle_);
  }

  // non-copyable, movable
  Component(const Component&) = delete;
  Component& operator=(const Component&) = delete;

  Component(Component&& other) noexcept : handle_(other.handle_) {
    other.handle_ = nullptr;
  }

  Component& operator=(Component&& other) noexcept {
    if (this != &other) {
      if (handle_) pcl_container_destroy(handle_);
      handle_ = other.handle_;
      other.handle_ = nullptr;
    }
    return *this;
  }

  // -- Lifecycle (call or let executor manage) -------------------------

  pcl_status_t configure()  { return pcl_container_configure(handle_); }
  pcl_status_t activate()   { return pcl_container_activate(handle_); }
  pcl_status_t deactivate() { return pcl_container_deactivate(handle_); }
  pcl_status_t cleanup()    { return pcl_container_cleanup(handle_); }
  pcl_status_t shutdown()   { return pcl_container_shutdown(handle_); }

  pcl_state_t state() const { return pcl_container_state(handle_); }
  const char* name() const  { return pcl_container_name(handle_); }

  // -- Tick rate -------------------------------------------------------

  void setTickRateHz(double hz) {
    pcl_container_set_tick_rate_hz(handle_, hz);
  }

  double tickRateHz() const {
    return pcl_container_get_tick_rate_hz(handle_);
  }

  // -- Parameters ------------------------------------------------------

  void setParam(const char* key, const char* value) {
    pcl_container_set_param_str(handle_, key, value);
  }

  void setParam(const char* key, double value) {
    pcl_container_set_param_f64(handle_, key, value);
  }

  void setParam(const char* key, int64_t value) {
    pcl_container_set_param_i64(handle_, key, value);
  }

  void setParam(const char* key, bool value) {
    pcl_container_set_param_bool(handle_, key, value);
  }

  std::string paramStr(const char* key,
                       const char* default_val = "") const {
    return pcl_container_get_param_str(handle_, key, default_val);
  }

  double paramF64(const char* key, double default_val = 0.0) const {
    return pcl_container_get_param_f64(handle_, key, default_val);
  }

  int64_t paramI64(const char* key, int64_t default_val = 0) const {
    return pcl_container_get_param_i64(handle_, key, default_val);
  }

  bool paramBool(const char* key, bool default_val = false) const {
    return pcl_container_get_param_bool(handle_, key, default_val);
  }

  // -- Port creation (call during on_configure) ------------------------

  Port addPublisher(const char* topic, const char* type_name) {
    return Port(pcl_container_add_publisher(handle_, topic, type_name),
                type_name ? type_name : "");
  }

  Port addSubscriber(const char* topic, const char* type_name,
                     pcl_sub_callback_t cb, void* user_data) {
    return Port(pcl_container_add_subscriber(handle_, topic, type_name,
                                             cb, user_data),
                type_name ? type_name : "");
  }

  Port addService(const char* service_name, const char* type_name,
                  pcl_service_handler_t handler, void* user_data) {
    return Port(pcl_container_add_service(handle_, service_name, type_name,
                                          handler, user_data),
                type_name ? type_name : "");
  }

  Port addStreamService(const char* service_name, const char* type_name,
                        pcl_stream_handler_t handler, void* user_data) {
    return Port(pcl_container_add_stream_service(handle_, service_name, type_name,
                                                 handler, user_data),
                type_name ? type_name : "");
  }

  // -- Service invocation (call from on_tick or callbacks) -------------

  /// \brief Invoke a service asynchronously.
  ///
  /// Routes through the executor's transport, or falls back to intra-process
  /// dispatch.  Callback fires on the executor thread.
  pcl_status_t invokeAsync(const char*      service_name,
                           const pcl_msg_t* request,
                           pcl_resp_cb_fn_t callback,
                           void*            user_data = nullptr) {
    return pcl_container_invoke_async(handle_, service_name, request,
                                      callback, user_data);
  }

  // -- Logging ---------------------------------------------------------

  void logDebug(const char* fmt, ...) const {
    va_list args;
    va_start(args, fmt);
    logv(PCL_LOG_DEBUG, fmt, args);
    va_end(args);
  }

  void logInfo(const char* fmt, ...) const {
    va_list args;
    va_start(args, fmt);
    logv(PCL_LOG_INFO, fmt, args);
    va_end(args);
  }

  void logWarn(const char* fmt, ...) const {
    va_list args;
    va_start(args, fmt);
    logv(PCL_LOG_WARN, fmt, args);
    va_end(args);
  }

  void logError(const char* fmt, ...) const {
    va_list args;
    va_start(args, fmt);
    logv(PCL_LOG_ERROR, fmt, args);
    va_end(args);
  }

  // -- Raw handle access -----------------------------------------------

  pcl_container_t* handle() { return handle_; }
  const pcl_container_t* handle() const { return handle_; }

protected:
  // -- Override these --------------------------------------------------

  virtual pcl_status_t on_configure()        { return PCL_OK; }
  virtual pcl_status_t on_activate()         { return PCL_OK; }
  virtual pcl_status_t on_deactivate()       { return PCL_OK; }
  virtual pcl_status_t on_cleanup()          { return PCL_OK; }
  virtual pcl_status_t on_shutdown()         { return PCL_OK; }
  virtual pcl_status_t on_tick(double dt)    { (void)dt; return PCL_OK; }

private:
  pcl_container_t* handle_ = nullptr;

  // -- C → C++ trampolines ---------------------------------------------

  static pcl_status_t trampoline_configure(pcl_container_t*, void* ud) {
    return static_cast<Component*>(ud)->on_configure();
  }
  static pcl_status_t trampoline_activate(pcl_container_t*, void* ud) {
    return static_cast<Component*>(ud)->on_activate();
  }
  static pcl_status_t trampoline_deactivate(pcl_container_t*, void* ud) {
    return static_cast<Component*>(ud)->on_deactivate();
  }
  static pcl_status_t trampoline_cleanup(pcl_container_t*, void* ud) {
    return static_cast<Component*>(ud)->on_cleanup();
  }
  static pcl_status_t trampoline_shutdown(pcl_container_t*, void* ud) {
    return static_cast<Component*>(ud)->on_shutdown();
  }
  static pcl_status_t trampoline_tick(pcl_container_t*, double dt, void* ud) {
    return static_cast<Component*>(ud)->on_tick(dt);
  }

  void logv(pcl_log_level_t level, const char* fmt, va_list args) const {
    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    pcl_log(handle_, level, "%s", buf);
  }
};

} // namespace pcl

#endif // PCL_COMPONENT_HPP
