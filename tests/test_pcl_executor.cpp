/// \file test_pcl_executor.cpp
/// \brief Tests for PCL executor — spin, dispatch, multi-container, shutdown.
#include <gtest/gtest.h>

#include <thread>
#include <chrono>

extern "C" {
#include "pcl/pcl_executor.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_log.h"
}

// ── Helpers ─────────────────────────────────────────────────────────────

struct TickCounter {
  int tick_count = 0;
};

static pcl_status_t counting_tick(pcl_container_t*, double, void* ud) {
  static_cast<TickCounter*>(ud)->tick_count++;
  return PCL_OK;
}

static pcl_callbacks_t counting_callbacks() {
  pcl_callbacks_t cbs = {};
  cbs.on_tick = counting_tick;
  return cbs;
}

// ── Basic Executor Tests ────────────────────────────────────────────────

TEST(PclExecutor, CreateDestroy) {
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);
  pcl_executor_destroy(e);
}

TEST(PclExecutor, SpinOnceTicksActiveContainer) {
  TickCounter counter;
  pcl_callbacks_t cbs = counting_callbacks();
  auto* c = pcl_container_create("ticker", &cbs, &counter);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // spin_once should tick the active container
  pcl_executor_spin_once(e, 0);

  // with default 100 Hz and the first tick, the accumulator may not
  // trigger immediately, so tick multiple times
  for (int i = 0; i < 5; i++) {
    pcl_executor_spin_once(e, 0);
  }

  // at least one tick should have fired
  EXPECT_GT(counter.tick_count, 0);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, InactiveContainerNotTicked) {
  TickCounter counter;
  pcl_callbacks_t cbs = counting_callbacks();
  auto* c = pcl_container_create("inactive", &cbs, &counter);
  // leave unconfigured — should not tick

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);
  pcl_executor_spin_once(e, 0);

  EXPECT_EQ(counter.tick_count, 0);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, MultipleContainers) {
  TickCounter c1_count, c2_count;
  pcl_callbacks_t cbs = counting_callbacks();

  auto* c1 = pcl_container_create("c1", &cbs, &c1_count);
  auto* c2 = pcl_container_create("c2", &cbs, &c2_count);

  pcl_container_configure(c1);
  pcl_container_activate(c1);
  pcl_container_configure(c2);
  pcl_container_activate(c2);

  // set different tick rates
  pcl_container_set_tick_rate_hz(c1, 10.0);
  pcl_container_set_tick_rate_hz(c2, 50.0);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c1);
  pcl_executor_add(e, c2);

  for (int i = 0; i < 20; i++) {
    pcl_executor_spin_once(e, 0);
  }

  // both should have been ticked
  EXPECT_GT(c1_count.tick_count, 0);
  EXPECT_GT(c2_count.tick_count, 0);

  pcl_executor_destroy(e);
  pcl_container_destroy(c1);
  pcl_container_destroy(c2);
}

// ── Shutdown Tests ──────────────────────────────────────────────────────

TEST(PclExecutor, RequestShutdownStopsSpin) {
  auto* c = pcl_container_create("spin_test", nullptr, nullptr);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // spin in background, request shutdown after a short delay
  std::thread spinner([e]() {
    pcl_executor_spin(e);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pcl_executor_request_shutdown(e);

  spinner.join(); // should return promptly

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclExecutor, GracefulShutdownFinalizesContainers) {
  int deactivate_count = 0;
  int shutdown_count   = 0;

  pcl_callbacks_t cbs = {};
  cbs.on_deactivate = [](pcl_container_t*, void* ud) -> pcl_status_t {
    (*static_cast<int*>(ud))++;
    return PCL_OK;
  };

  // slightly hacky but works for this test — we need separate counters
  // so we use two containers
  auto* c = pcl_container_create("grace", &cbs, &deactivate_count);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  auto rc = pcl_executor_shutdown_graceful(e, 5000);
  EXPECT_EQ(rc, PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_FINALIZED);
  EXPECT_EQ(deactivate_count, 1);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// ── Intra-process Dispatch Tests ────────────────────────────────────────

struct SubReceived {
  bool received = false;
  uint32_t size = 0;
};

static void sub_callback(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
  auto* r = static_cast<SubReceived*>(ud);
  r->received = true;
  r->size = msg->size;
}

TEST(PclExecutor, IntraProcessPubSub) {
  SubReceived sub_data;
  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "topic_a", "TestMsg",
                                  sub_callback, ud);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("subscriber", &sub_cbs, &sub_data);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, sub_c);

  // dispatch a message manually (simulating a publisher)
  int payload = 42;
  pcl_msg_t msg = {};
  msg.data = &payload;
  msg.size = sizeof(payload);
  msg.type_name = "TestMsg";

  auto rc = pcl_executor_dispatch_incoming(e, "topic_a", &msg);
  EXPECT_EQ(rc, PCL_OK);
  EXPECT_TRUE(sub_data.received);
  EXPECT_EQ(sub_data.size, sizeof(int));

  pcl_executor_destroy(e);
  pcl_container_destroy(sub_c);
}

TEST(PclExecutor, DispatchToUnknownTopicReturnsNotFound) {
  auto* e = pcl_executor_create();

  pcl_msg_t msg = {};
  msg.data = nullptr;
  msg.size = 0;
  msg.type_name = "X";

  EXPECT_EQ(pcl_executor_dispatch_incoming(e, "no_such_topic", &msg),
            PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
}

// ── Null Safety ─────────────────────────────────────────────────────────

TEST(PclExecutor, NullSafety) {
  EXPECT_EQ(pcl_executor_add(nullptr, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_spin(nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_spin_once(nullptr, 0), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_shutdown_graceful(nullptr, 0), PCL_ERR_INVALID);
}
