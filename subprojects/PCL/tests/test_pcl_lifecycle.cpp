/// \file test_pcl_lifecycle.cpp
/// \brief Tests for PCL container lifecycle, parameters, and port creation.
#include <gtest/gtest.h>

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_log.h"
}

// -- Test helpers --------------------------------------------------------

struct LifecycleLog {
  int configure_count  = 0;
  int activate_count   = 0;
  int deactivate_count = 0;
  int cleanup_count    = 0;
  int shutdown_count   = 0;
  int tick_count       = 0;
  double last_dt       = 0.0;
};

static pcl_status_t on_configure(pcl_container_t*, void* ud) {
  static_cast<LifecycleLog*>(ud)->configure_count++;
  return PCL_OK;
}

static pcl_status_t on_activate(pcl_container_t*, void* ud) {
  static_cast<LifecycleLog*>(ud)->activate_count++;
  return PCL_OK;
}

static pcl_status_t on_deactivate(pcl_container_t*, void* ud) {
  static_cast<LifecycleLog*>(ud)->deactivate_count++;
  return PCL_OK;
}

static pcl_status_t on_cleanup(pcl_container_t*, void* ud) {
  static_cast<LifecycleLog*>(ud)->cleanup_count++;
  return PCL_OK;
}

static pcl_status_t on_shutdown(pcl_container_t*, void* ud) {
  static_cast<LifecycleLog*>(ud)->shutdown_count++;
  return PCL_OK;
}

static pcl_status_t on_tick(pcl_container_t*, double dt, void* ud) {
  auto* log = static_cast<LifecycleLog*>(ud);
  log->tick_count++;
  log->last_dt = dt;
  return PCL_OK;
}

static pcl_callbacks_t make_callbacks() {
  pcl_callbacks_t cbs = {};
  cbs.on_configure  = on_configure;
  cbs.on_activate   = on_activate;
  cbs.on_deactivate = on_deactivate;
  cbs.on_cleanup    = on_cleanup;
  cbs.on_shutdown   = on_shutdown;
  cbs.on_tick       = on_tick;
  return cbs;
}

// -- Lifecycle Tests -----------------------------------------------------

TEST(PclLifecycle, CreateDestroy) {
  pcl_callbacks_t cbs = make_callbacks();
  LifecycleLog log;
  auto* c = pcl_container_create("test", &cbs, &log);
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_UNCONFIGURED);
  EXPECT_STREQ(pcl_container_name(c), "test");
  pcl_container_destroy(c);
}

TEST(PclLifecycle, FullTransitionCycle) {
  pcl_callbacks_t cbs = make_callbacks();
  LifecycleLog log;
  auto* c = pcl_container_create("cycle", &cbs, &log);

  // unconfigured -> configured
  EXPECT_EQ(pcl_container_configure(c), PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_CONFIGURED);
  EXPECT_EQ(log.configure_count, 1);

  // configured -> active
  EXPECT_EQ(pcl_container_activate(c), PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_ACTIVE);
  EXPECT_EQ(log.activate_count, 1);

  // active -> configured
  EXPECT_EQ(pcl_container_deactivate(c), PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_CONFIGURED);
  EXPECT_EQ(log.deactivate_count, 1);

  // configured -> unconfigured
  EXPECT_EQ(pcl_container_cleanup(c), PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_UNCONFIGURED);
  EXPECT_EQ(log.cleanup_count, 1);

  // unconfigured -> finalized
  EXPECT_EQ(pcl_container_shutdown(c), PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_FINALIZED);
  EXPECT_EQ(log.shutdown_count, 1);

  pcl_container_destroy(c);
}

TEST(PclLifecycle, InvalidTransitionsRejected) {
  pcl_callbacks_t cbs = make_callbacks();
  LifecycleLog log;
  auto* c = pcl_container_create("invalid", &cbs, &log);

  // can't activate from unconfigured
  EXPECT_EQ(pcl_container_activate(c), PCL_ERR_STATE);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_UNCONFIGURED);

  // can't deactivate from unconfigured
  EXPECT_EQ(pcl_container_deactivate(c), PCL_ERR_STATE);

  // can't cleanup from unconfigured
  EXPECT_EQ(pcl_container_cleanup(c), PCL_ERR_STATE);

  // configure then try invalid transitions
  EXPECT_EQ(pcl_container_configure(c), PCL_OK);

  // can't configure from configured
  EXPECT_EQ(pcl_container_configure(c), PCL_ERR_STATE);

  // can't deactivate from configured
  EXPECT_EQ(pcl_container_deactivate(c), PCL_ERR_STATE);

  pcl_container_destroy(c);
}

TEST(PclLifecycle, ShutdownFromAnyState) {
  // shutdown should work from active
  pcl_callbacks_t cbs = make_callbacks();
  LifecycleLog log;
  auto* c = pcl_container_create("sd", &cbs, &log);

  pcl_container_configure(c);
  pcl_container_activate(c);
  EXPECT_EQ(pcl_container_shutdown(c), PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_FINALIZED);

  // can't shutdown from finalized
  EXPECT_EQ(pcl_container_shutdown(c), PCL_ERR_STATE);

  pcl_container_destroy(c);
}

TEST(PclLifecycle, CallbackFailureAbortsTransition) {
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t*, void*) -> pcl_status_t {
    return PCL_ERR_CALLBACK;
  };

  auto* c = pcl_container_create("fail", &cbs, nullptr);
  EXPECT_EQ(pcl_container_configure(c), PCL_ERR_CALLBACK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_UNCONFIGURED);

  pcl_container_destroy(c);
}

TEST(PclLifecycle, NullCallbacksAreNoOp) {
  auto* c = pcl_container_create("null_cbs", nullptr, nullptr);
  ASSERT_NE(c, nullptr);

  EXPECT_EQ(pcl_container_configure(c), PCL_OK);
  EXPECT_EQ(pcl_container_activate(c), PCL_OK);
  EXPECT_EQ(pcl_container_deactivate(c), PCL_OK);
  EXPECT_EQ(pcl_container_cleanup(c), PCL_OK);
  EXPECT_EQ(pcl_container_shutdown(c), PCL_OK);

  pcl_container_destroy(c);
}

// -- Parameter Tests -----------------------------------------------------

TEST(PclParams, StringRoundTrip) {
  auto* c = pcl_container_create("params", nullptr, nullptr);

  EXPECT_EQ(pcl_container_set_param_str(c, "key1", "hello"), PCL_OK);
  EXPECT_STREQ(pcl_container_get_param_str(c, "key1", ""), "hello");
  EXPECT_STREQ(pcl_container_get_param_str(c, "missing", "default"), "default");

  pcl_container_destroy(c);
}

TEST(PclParams, NumericTypes) {
  auto* c = pcl_container_create("nums", nullptr, nullptr);

  pcl_container_set_param_f64(c, "rate", 42.5);
  EXPECT_DOUBLE_EQ(pcl_container_get_param_f64(c, "rate", 0.0), 42.5);

  pcl_container_set_param_i64(c, "count", 99);
  EXPECT_EQ(pcl_container_get_param_i64(c, "count", 0), 99);

  pcl_container_set_param_bool(c, "flag", true);
  EXPECT_EQ(pcl_container_get_param_bool(c, "flag", false), true);

  pcl_container_destroy(c);
}

TEST(PclParams, OverwriteExisting) {
  auto* c = pcl_container_create("ow", nullptr, nullptr);

  pcl_container_set_param_str(c, "key", "v1");
  EXPECT_STREQ(pcl_container_get_param_str(c, "key", ""), "v1");

  pcl_container_set_param_str(c, "key", "v2");
  EXPECT_STREQ(pcl_container_get_param_str(c, "key", ""), "v2");

  pcl_container_destroy(c);
}

// -- Port Creation Tests -------------------------------------------------

static pcl_status_t configure_with_ports(pcl_container_t* c, void*) {
  auto* pub = pcl_container_add_publisher(c, "state", "State");
  return pub ? PCL_OK : PCL_ERR_CALLBACK;
}

TEST(PclPorts, CreatedDuringConfigure) {
  pcl_callbacks_t cbs = {};
  cbs.on_configure = configure_with_ports;

  auto* c = pcl_container_create("ports", &cbs, nullptr);
  EXPECT_EQ(pcl_container_configure(c), PCL_OK);
  pcl_container_destroy(c);
}

TEST(PclPorts, RejectedOutsideConfigure) {
  auto* c = pcl_container_create("badport", nullptr, nullptr);

  // not in on_configure -- should fail
  auto* p = pcl_container_add_publisher(c, "topic", "Type");
  EXPECT_EQ(p, nullptr);

  pcl_container_destroy(c);
}

// -- Tick Rate Tests -----------------------------------------------------

TEST(PclTickRate, DefaultIs100Hz) {
  auto* c = pcl_container_create("rate", nullptr, nullptr);
  EXPECT_DOUBLE_EQ(pcl_container_get_tick_rate_hz(c), 100.0);
  pcl_container_destroy(c);
}

TEST(PclTickRate, SetAndGet) {
  auto* c = pcl_container_create("rate2", nullptr, nullptr);
  EXPECT_EQ(pcl_container_set_tick_rate_hz(c, 50.0), PCL_OK);
  EXPECT_DOUBLE_EQ(pcl_container_get_tick_rate_hz(c), 50.0);

  // invalid rate
  EXPECT_EQ(pcl_container_set_tick_rate_hz(c, -1.0), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_tick_rate_hz(c, 0.0), PCL_ERR_INVALID);

  pcl_container_destroy(c);
}

// -- Null Safety ---------------------------------------------------------

TEST(PclNull, NullHandlesReturnError) {
  EXPECT_EQ(pcl_container_configure(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_activate(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_deactivate(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_cleanup(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_shutdown(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_state(nullptr), PCL_STATE_FINALIZED);
  EXPECT_STREQ(pcl_container_name(nullptr), "");
  EXPECT_EQ(pcl_container_create(nullptr, nullptr, nullptr), nullptr);
}
