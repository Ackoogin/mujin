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
#include <functional>
#include <cstdarg>
#include <cstdio>
#include <utility>

namespace pcl {

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

  pcl_port_t* addPublisher(const char* topic, const char* type_name) {
    return pcl_container_add_publisher(handle_, topic, type_name);
  }

  pcl_port_t* addSubscriber(const char* topic, const char* type_name,
                            pcl_sub_callback_t cb, void* user_data) {
    return pcl_container_add_subscriber(handle_, topic, type_name,
                                        cb, user_data);
  }

  pcl_port_t* addService(const char* service_name, const char* type_name,
                         pcl_service_handler_t handler, void* user_data) {
    return pcl_container_add_service(handle_, service_name, type_name,
                                     handler, user_data);
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
