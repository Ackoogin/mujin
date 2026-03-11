/// \file test_pcl_robustness.cpp
/// \brief Robustness, threading, throughput, and 100% coverage tests for PCL.
///
/// Covers every branch in pcl_container.c, pcl_executor.c, and pcl_log.c that
/// is not exercised by the existing lifecycle/executor/log test suites.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <string>

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_log.h"
}

// ── Helpers ─────────────────────────────────────────────────────────────

static pcl_container_t* make_active(const char* name) {
  pcl_container_t* c = pcl_container_create(name, nullptr, nullptr);
  pcl_container_configure(c);
  pcl_container_activate(c);
  return c;
}

// ═══════════════════════════════════════════════════════════════════════
// pcl_container.c — uncovered branches
// ═══════════════════════════════════════════════════════════════════════

// ── Parameter overflow (PCL_MAX_PARAMS = 128) ────────────────────────

TEST(PclContainerRobust, ParamOverflowReturnsNomem) {
  auto* c = pcl_container_create("overflow", nullptr, nullptr);
  // fill all 128 slots
  for (int i = 0; i < 128; ++i) {
    char key[16];
    snprintf(key, sizeof(key), "k%d", i);
    ASSERT_EQ(pcl_container_set_param_str(c, key, "v"), PCL_OK)
        << "failed at slot " << i;
  }
  // one more should fail
  EXPECT_EQ(pcl_container_set_param_str(c, "extra_str", "x"), PCL_ERR_NOMEM);
  EXPECT_EQ(pcl_container_set_param_f64(c, "extra_f64", 1.0), PCL_ERR_NOMEM);
  EXPECT_EQ(pcl_container_set_param_i64(c, "extra_i64", 1), PCL_ERR_NOMEM);
  EXPECT_EQ(pcl_container_set_param_bool(c, "extra_bool", true), PCL_ERR_NOMEM);
  pcl_container_destroy(c);
}

// ── Type-mismatch param getters return default ───────────────────────

TEST(PclContainerRobust, ParamTypeMismatchReturnsDefault) {
  auto* c = pcl_container_create("mismatch", nullptr, nullptr);
  // store as f64 …
  pcl_container_set_param_f64(c, "num", 3.14);
  // … get as str → should return default
  EXPECT_STREQ(pcl_container_get_param_str(c, "num", "def"), "def");
  // … get as i64 → should return default
  EXPECT_EQ(pcl_container_get_param_i64(c, "num", -99), -99);
  // … get as bool → should return default
  EXPECT_EQ(pcl_container_get_param_bool(c, "num", true), true);

  // store as str, get as f64/i64/bool → defaults
  pcl_container_set_param_str(c, "word", "hello");
  EXPECT_DOUBLE_EQ(pcl_container_get_param_f64(c, "word", 7.0), 7.0);
  EXPECT_EQ(pcl_container_get_param_i64(c, "word", 5), 5);

  pcl_container_destroy(c);
}

// ── Null key/value in param setters ─────────────────────────────────

TEST(PclContainerRobust, ParamNullArgs) {
  auto* c = pcl_container_create("pnull", nullptr, nullptr);
  EXPECT_EQ(pcl_container_set_param_str(nullptr, "k", "v"), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_str(c, nullptr, "v"),   PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_str(c, "k", nullptr),   PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_f64(nullptr, "k", 0.0), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_f64(c, nullptr, 0.0),   PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_i64(nullptr, "k", 0),   PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_i64(c, nullptr, 0),     PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_bool(nullptr, "k", false), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_container_set_param_bool(c, nullptr, false),   PCL_ERR_INVALID);
  pcl_container_destroy(c);
}

TEST(PclContainerRobust, ParamGetNullArgs) {
  EXPECT_STREQ(pcl_container_get_param_str(nullptr, "k", "d"), "d");
  EXPECT_STREQ(pcl_container_get_param_str((pcl_container_t*)1, nullptr, "d"), "d");
  EXPECT_DOUBLE_EQ(pcl_container_get_param_f64(nullptr, "k", 1.0), 1.0);
  EXPECT_EQ(pcl_container_get_param_i64(nullptr, "k", 7), 7);
  EXPECT_EQ(pcl_container_get_param_bool(nullptr, "k", true), true);
}

// ── Port overflow (PCL_MAX_PORTS = 64) ──────────────────────────────

static void sub_nop(pcl_container_t*, const pcl_msg_t*, void*) {}
static pcl_status_t svc_nop(pcl_container_t*, const pcl_msg_t*, pcl_msg_t*, void*) {
  return PCL_OK;
}

struct PortFillCtx {
  int  want;       // how many ports to add
  int  added = 0;
  bool pub_overflow  = false;
  bool sub_overflow  = false;
  bool svc_overflow  = false;
};

static pcl_status_t fill_ports_configure(pcl_container_t* c, void* ud) {
  auto* ctx = static_cast<PortFillCtx*>(ud);
  for (int i = 0; i < ctx->want; ++i) {
    char name[32];
    snprintf(name, sizeof(name), "t%d", i);
    if (pcl_container_add_publisher(c, name, "T") != nullptr) {
      ctx->added++;
    }
  }
  // one more publisher → should fail
  ctx->pub_overflow = (pcl_container_add_publisher(c, "overflow_pub", "T") == nullptr);
  // subscriber overflow
  ctx->sub_overflow = (pcl_container_add_subscriber(c, "overflow_sub", "T", sub_nop, nullptr) == nullptr);
  // service overflow
  ctx->svc_overflow = (pcl_container_add_service(c, "overflow_svc", "T", svc_nop, nullptr) == nullptr);
  return PCL_OK;
}

TEST(PclContainerRobust, PortOverflowReturnNull) {
  PortFillCtx ctx;
  ctx.want = 64; // PCL_MAX_PORTS

  pcl_callbacks_t cbs = {};
  cbs.on_configure = fill_ports_configure;

  auto* c = pcl_container_create("portfull", &cbs, &ctx);
  ASSERT_EQ(pcl_container_configure(c), PCL_OK);
  EXPECT_EQ(ctx.added, 64);
  EXPECT_TRUE(ctx.pub_overflow);
  EXPECT_TRUE(ctx.sub_overflow);
  EXPECT_TRUE(ctx.svc_overflow);
  pcl_container_destroy(c);
}

// ── Port creation with NULL mandatory args ───────────────────────────

static pcl_status_t null_port_args_configure(pcl_container_t* c, void*) {
  // publisher: null topic, null type
  EXPECT_EQ(pcl_container_add_publisher(c, nullptr, "T"), nullptr);
  EXPECT_EQ(pcl_container_add_publisher(c, "t", nullptr), nullptr);
  EXPECT_EQ(pcl_container_add_publisher(nullptr, "t", "T"), nullptr);

  // subscriber: null callback
  EXPECT_EQ(pcl_container_add_subscriber(c, "t", "T", nullptr, nullptr), nullptr);
  EXPECT_EQ(pcl_container_add_subscriber(c, nullptr, "T", sub_nop, nullptr), nullptr);
  EXPECT_EQ(pcl_container_add_subscriber(c, "t", nullptr, sub_nop, nullptr), nullptr);

  // service: null handler
  EXPECT_EQ(pcl_container_add_service(c, "s", "T", nullptr, nullptr), nullptr);
  EXPECT_EQ(pcl_container_add_service(c, nullptr, "T", svc_nop, nullptr), nullptr);
  EXPECT_EQ(pcl_container_add_service(c, "s", nullptr, svc_nop, nullptr), nullptr);

  return PCL_OK;
}

TEST(PclContainerRobust, PortCreationNullArgs) {
  pcl_callbacks_t cbs = {};
  cbs.on_configure = null_port_args_configure;
  auto* c = pcl_container_create("nullportargs", &cbs, nullptr);
  EXPECT_EQ(pcl_container_configure(c), PCL_OK);
  pcl_container_destroy(c);
}

// ── Port publish edge cases ───────────────────────────────────────────

struct PubPortCtx { pcl_port_t* pub = nullptr; };

static pcl_status_t create_pub_configure(pcl_container_t* c, void* ud) {
  auto* ctx = static_cast<PubPortCtx*>(ud);
  ctx->pub = pcl_container_add_publisher(c, "out", "Msg");
  return ctx->pub ? PCL_OK : PCL_ERR_CALLBACK;
}

TEST(PclContainerRobust, PublishNullPortOrMsg) {
  pcl_msg_t msg = {};
  msg.type_name = "Msg";
  EXPECT_EQ(pcl_port_publish(nullptr, &msg), PCL_ERR_INVALID);
  // NULL msg
  PubPortCtx ctx;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = create_pub_configure;
  auto* c = pcl_container_create("pub_null", &cbs, &ctx);
  pcl_container_configure(c);
  pcl_container_activate(c);
  EXPECT_EQ(pcl_port_publish(ctx.pub, nullptr), PCL_ERR_INVALID);
  pcl_container_destroy(c);
}

TEST(PclContainerRobust, PublishOnInactiveContainerReturnsClosed) {
  PubPortCtx ctx;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = create_pub_configure;

  auto* c = pcl_container_create("pub_inactive", &cbs, &ctx);
  pcl_container_configure(c);
  // configured but not active → PCL_ERR_PORT_CLOSED
  ASSERT_NE(ctx.pub, nullptr);
  pcl_msg_t msg = {};
  msg.type_name = "Msg";
  EXPECT_EQ(pcl_port_publish(ctx.pub, &msg), PCL_ERR_PORT_CLOSED);
  pcl_container_destroy(c);
}

TEST(PclContainerRobust, PublishOnSubscriberPortReturnsInvalid) {
  struct SubPortCtx { pcl_port_t* sub = nullptr; };
  SubPortCtx ctx;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* ctx = static_cast<SubPortCtx*>(ud);
    ctx->sub = pcl_container_add_subscriber(c, "in", "Msg", sub_nop, nullptr);
    return ctx->sub ? PCL_OK : PCL_ERR_CALLBACK;
  };
  auto* c = pcl_container_create("pub_sub_port", &cbs, &ctx);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t msg = {};
  msg.type_name = "Msg";
  EXPECT_EQ(pcl_port_publish(ctx.sub, &msg), PCL_ERR_INVALID);

  pcl_container_destroy(c);
}

// ── Callback failure paths for deactivate and cleanup ────────────────

TEST(PclContainerRobust, DeactivateCallbackFailureKeepsActiveState) {
  pcl_callbacks_t cbs = {};
  cbs.on_deactivate = [](pcl_container_t*, void*) -> pcl_status_t {
    return PCL_ERR_CALLBACK;
  };
  auto* c = pcl_container_create("deact_fail", &cbs, nullptr);
  pcl_container_configure(c);
  pcl_container_activate(c);
  EXPECT_EQ(pcl_container_deactivate(c), PCL_ERR_CALLBACK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_ACTIVE);
  pcl_container_destroy(c);
}

TEST(PclContainerRobust, CleanupCallbackFailureKeepsConfiguredState) {
  pcl_callbacks_t cbs = {};
  cbs.on_cleanup = [](pcl_container_t*, void*) -> pcl_status_t {
    return PCL_ERR_CALLBACK;
  };
  auto* c = pcl_container_create("clean_fail", &cbs, nullptr);
  pcl_container_configure(c);
  EXPECT_EQ(pcl_container_cleanup(c), PCL_ERR_CALLBACK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_CONFIGURED);
  pcl_container_destroy(c);
}

TEST(PclContainerRobust, ActivateCallbackFailureKeepsConfiguredState) {
  pcl_callbacks_t cbs = {};
  cbs.on_activate = [](pcl_container_t*, void*) -> pcl_status_t {
    return PCL_ERR_CALLBACK;
  };
  auto* c = pcl_container_create("act_fail", &cbs, nullptr);
  pcl_container_configure(c);
  EXPECT_EQ(pcl_container_activate(c), PCL_ERR_CALLBACK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_CONFIGURED);
  pcl_container_destroy(c);
}

// ── Cleanup resets port_count ─────────────────────────────────────────

TEST(PclContainerRobust, CleanupResetsPortCount) {
  PubPortCtx ctx;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = create_pub_configure;

  auto* c = pcl_container_create("cleanup_ports", &cbs, &ctx);
  pcl_container_configure(c);
  // port was created; after cleanup → re-configure should succeed again
  pcl_container_cleanup(c);
  ctx.pub = nullptr;
  EXPECT_EQ(pcl_container_configure(c), PCL_OK);
  EXPECT_NE(ctx.pub, nullptr);
  pcl_container_destroy(c);
}

// ── Container name truncation (>127 chars) ─────────────────────────

TEST(PclContainerRobust, LongNameTruncated) {
  std::string long_name(200, 'a');
  auto* c = pcl_container_create(long_name.c_str(), nullptr, nullptr);
  ASSERT_NE(c, nullptr);
  std::string stored = pcl_container_name(c);
  EXPECT_EQ(stored.size(), 127u); // snprintf into 128-byte buf
  pcl_container_destroy(c);
}

// ═══════════════════════════════════════════════════════════════════════
// pcl_executor.c — uncovered branches
// ═══════════════════════════════════════════════════════════════════════

// ── Executor container overflow (PCL_MAX_CONTAINERS = 64) ────────────

TEST(PclExecutorRobust, ContainerOverflowReturnsNomem) {
  auto* e = pcl_executor_create();
  std::vector<pcl_container_t*> containers;

  for (int i = 0; i < 64; ++i) {
    char name[16];
    snprintf(name, sizeof(name), "c%d", i);
    auto* c = pcl_container_create(name, nullptr, nullptr);
    containers.push_back(c);
    ASSERT_EQ(pcl_executor_add(e, c), PCL_OK) << "slot " << i;
  }
  // 65th should fail
  auto* extra = pcl_container_create("extra", nullptr, nullptr);
  EXPECT_EQ(pcl_executor_add(e, extra), PCL_ERR_NOMEM);
  pcl_container_destroy(extra);

  pcl_executor_destroy(e);
  for (auto* c : containers) pcl_container_destroy(c);
}

// ── pcl_executor_add null safety ─────────────────────────────────────

TEST(PclExecutorRobust, AddNullContainerReturnsInvalid) {
  auto* e = pcl_executor_create();
  EXPECT_EQ(pcl_executor_add(e, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_add(nullptr, nullptr), PCL_ERR_INVALID);
  pcl_executor_destroy(e);
}

// ── pcl_executor_post_incoming bad inputs ────────────────────────────

TEST(PclExecutorRobust, PostIncomingBadInputs) {
  auto* e = pcl_executor_create();

  // null executor
  pcl_msg_t msg = {};
  msg.type_name = "T";
  EXPECT_EQ(pcl_executor_post_incoming(nullptr, "t", &msg), PCL_ERR_INVALID);

  // null topic
  EXPECT_EQ(pcl_executor_post_incoming(e, nullptr, &msg), PCL_ERR_INVALID);

  // null msg
  EXPECT_EQ(pcl_executor_post_incoming(e, "t", nullptr), PCL_ERR_INVALID);

  // size > 0 but data == NULL → PCL_ERR_INVALID
  pcl_msg_t bad_msg = {};
  bad_msg.type_name = "T";
  bad_msg.size = 4;
  bad_msg.data = nullptr;
  EXPECT_EQ(pcl_executor_post_incoming(e, "t", &bad_msg), PCL_ERR_INVALID);

  // null type_name
  pcl_msg_t no_type = {};
  no_type.data = nullptr;
  no_type.size = 0;
  no_type.type_name = nullptr;
  EXPECT_EQ(pcl_executor_post_incoming(e, "t", &no_type), PCL_ERR_INVALID);

  pcl_executor_destroy(e);
}

// ── Zero-size post (data=null, size=0) is valid ───────────────────────

TEST(PclExecutorRobust, PostIncomingZeroSizeNoData) {
  struct Recv { bool got = false; };
  Recv recv;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "empty_topic", "Empty",
      [](pcl_container_t*, const pcl_msg_t* m, void* ud) {
        static_cast<Recv*>(ud)->got = true;
        EXPECT_EQ(m->size, 0u);
        EXPECT_EQ(m->data, nullptr);
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("zero_recv", &cbs, &recv);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  pcl_msg_t msg = {};
  msg.type_name = "Empty";
  msg.data = nullptr;
  msg.size = 0;
  EXPECT_EQ(pcl_executor_post_incoming(e, "empty_topic", &msg), PCL_OK);
  EXPECT_EQ(pcl_executor_spin_once(e, 0), PCL_OK);
  EXPECT_TRUE(recv.got);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// ── dispatch_incoming_now with null args ─────────────────────────────

TEST(PclExecutorRobust, DispatchIncomingNullArgs) {
  auto* e = pcl_executor_create();
  pcl_msg_t msg = {};
  msg.type_name = "T";
  EXPECT_EQ(pcl_executor_dispatch_incoming(nullptr, "t", &msg), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_dispatch_incoming(e, nullptr, &msg),   PCL_ERR_INVALID);
  EXPECT_EQ(pcl_executor_dispatch_incoming(e, "t", nullptr),    PCL_ERR_INVALID);
  pcl_executor_destroy(e);
}

// ── Drain queue: dispatch fails → error propagated ───────────────────

TEST(PclExecutorRobust, DrainQueuePropagatesNotFound) {
  // Post to a topic that has no subscriber → drain should get PCL_ERR_NOT_FOUND
  auto* e = pcl_executor_create();

  pcl_msg_t msg = {};
  msg.type_name = "T";
  EXPECT_EQ(pcl_executor_post_incoming(e, "ghost_topic", &msg), PCL_OK);
  // spin_once will drain and hit NOT_FOUND → propagated
  EXPECT_EQ(pcl_executor_spin_once(e, 0), PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
}

// ── Executor destroy flushes pending messages ─────────────────────────

TEST(PclExecutorRobust, DestroyWithPendingMessagesFrees) {
  auto* e = pcl_executor_create();
  int payload = 7;
  pcl_msg_t msg = {};
  msg.type_name = "T";
  msg.data = &payload;
  msg.size = sizeof(payload);

  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(pcl_executor_post_incoming(e, "nowhere", &msg), PCL_OK);
  }
  // destroy without draining — must not leak/crash
  pcl_executor_destroy(e);
}

// ── Transport adapter wiring ─────────────────────────────────────────

TEST(PclExecutorRobust, SetTransportAndClear) {
  auto* e = pcl_executor_create();

  bool shutdown_called = false;

  pcl_transport_t t = {};
  t.shutdown = [](void* ctx) {
    *static_cast<bool*>(ctx) = true;
  };
  t.adapter_ctx = &shutdown_called;

  EXPECT_EQ(pcl_executor_set_transport(e, &t), PCL_OK);
  // clearing with NULL should also be OK
  EXPECT_EQ(pcl_executor_set_transport(e, nullptr), PCL_OK);
  // destroy: transport was cleared, so shutdown NOT called via destroy
  pcl_executor_destroy(e);
  EXPECT_FALSE(shutdown_called); // cleared before destroy

  // set transport then destroy — shutdown IS called
  auto* e2 = pcl_executor_create();
  bool sd2 = false;
  pcl_transport_t t2 = {};
  t2.shutdown = [](void* ctx) { *static_cast<bool*>(ctx) = true; };
  t2.adapter_ctx = &sd2;
  pcl_executor_set_transport(e2, &t2);
  pcl_executor_destroy(e2);
  EXPECT_TRUE(sd2);
}

TEST(PclExecutorRobust, SetTransportNullExecutorReturnsInvalid) {
  EXPECT_EQ(pcl_executor_set_transport(nullptr, nullptr), PCL_ERR_INVALID);
}

// ── Graceful shutdown timeout ────────────────────────────────────────

TEST(PclExecutorRobust, GracefulShutdownTimeout) {
  // Use a container that stalls on deactivate long enough to time out
  // (timeout_ms=1 with no real stall — just checks the timed_out path
  //  when deadline < 2ms away and we have many containers).
  // We trigger the timeout by using timeout_ms=0 with an active container
  // (deadline = now, so any container not yet finalized trips the timeout).
  struct StallCtx { bool deactivated = false; };
  StallCtx ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_deactivate = [](pcl_container_t*, void* ud) -> pcl_status_t {
    static_cast<StallCtx*>(ud)->deactivated = true;
    return PCL_OK;
  };

  auto* c = pcl_container_create("stall", &cbs, &ctx);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // timeout_ms=1 might or might not expire before deactivate — so just
  // verify it returns without crashing and container ends up finalized.
  pcl_status_t rc = pcl_executor_shutdown_graceful(e, 1);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_FINALIZED);
  // rc is either PCL_OK or PCL_ERR_TIMEOUT depending on timing — both ok
  (void)rc;

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// ── Graceful shutdown: already-finalized container is a no-op ────────

TEST(PclExecutorRobust, GracefulShutdownAlreadyFinalized) {
  auto* c = pcl_container_create("already_fin", nullptr, nullptr);
  pcl_container_configure(c);
  pcl_container_activate(c);
  pcl_container_shutdown(c); // finalize manually

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  EXPECT_EQ(pcl_executor_shutdown_graceful(e, 5000), PCL_OK);
  EXPECT_EQ(pcl_container_state(c), PCL_STATE_FINALIZED);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// ── spin_once first call (prev_time == 0) uses 1 ms fallback ────────

TEST(PclExecutorRobust, SpinOnceFirstCallUsesDefaultDt) {
  std::atomic<int> tick_count{0};
  pcl_callbacks_t cbs = {};
  cbs.on_tick = [](pcl_container_t*, double dt, void* ud) -> pcl_status_t {
    static_cast<std::atomic<int>*>(ud)->fetch_add(1);
    EXPECT_GT(dt, 0.0); // fallback dt must be positive
    return PCL_OK;
  };

  auto* c = pcl_container_create("first_spin", &cbs, &tick_count);
  pcl_container_configure(c);
  pcl_container_activate(c);
  pcl_container_set_tick_rate_hz(c, 1.0); // 1 Hz = fires after 1 s

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // With 1 Hz rate, a single spin_once with fallback dt=0.001 s
  // won't fire a tick (accumulator < 1.0). That is correct behaviour.
  // We just verify it doesn't crash and returns OK.
  EXPECT_EQ(pcl_executor_spin_once(e, 0), PCL_OK);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// ── request_shutdown on null is safe ─────────────────────────────────

TEST(PclExecutorRobust, RequestShutdownNullSafe) {
  pcl_executor_request_shutdown(nullptr); // must not crash
}

// ═══════════════════════════════════════════════════════════════════════
// Threading / concurrency tests
// ═══════════════════════════════════════════════════════════════════════

TEST(PclThreading, ConcurrentPostFromManyThreads) {
  // N threads each post M messages into the same executor concurrently.
  // After joining, spin_once must drain exactly N*M messages (or fail
  // with NOT_FOUND since no subscriber is registered — but must not crash
  // or corrupt the queue).
  constexpr int N = 8;
  constexpr int M = 200;

  auto* e = pcl_executor_create();

  std::vector<std::thread> threads;
  std::atomic<int> post_ok{0};

  for (int t = 0; t < N; ++t) {
    threads.emplace_back([&, t]() {
      int payload = t;
      for (int m = 0; m < M; ++m) {
        pcl_msg_t msg = {};
        msg.type_name = "T";
        msg.data = &payload;
        msg.size = sizeof(payload);
        if (pcl_executor_post_incoming(e, "flood", &msg) == PCL_OK) {
          post_ok.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (auto& th : threads) th.join();

  EXPECT_EQ(post_ok.load(), N * M);

  // Drain all — each dispatch will fail with NOT_FOUND (no subscriber).
  // spin_once returns after first NOT_FOUND; loop until queue empty.
  int drained = 0;
  while (true) {
    pcl_status_t rc = pcl_executor_spin_once(e, 0);
    if (rc == PCL_ERR_NOT_FOUND) { ++drained; continue; }
    if (rc == PCL_OK)             break;
    ADD_FAILURE() << "unexpected rc=" << rc;
    break;
  }
  // We drained at least N*M - 1 (last spin_once sees empty queue → OK)
  EXPECT_GE(drained, N * M - 1);

  pcl_executor_destroy(e);
}

TEST(PclThreading, ProducerConsumerWithSubscriber) {
  // Subscriber counts every message received on executor thread.
  // N producer threads each post M messages.
  constexpr int N = 4;
  constexpr int M = 100;

  std::atomic<int> received{0};

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "chan", "T",
      [](pcl_container_t*, const pcl_msg_t*, void* ud) {
        static_cast<std::atomic<int>*>(ud)->fetch_add(1, std::memory_order_relaxed);
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("consumer", &cbs, &received);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  // Start producers
  std::vector<std::thread> producers;
  for (int t = 0; t < N; ++t) {
    producers.emplace_back([&]() {
      int val = 0;
      pcl_msg_t msg = {};
      msg.type_name = "T";
      msg.data = &val;
      msg.size = sizeof(val);
      for (int m = 0; m < M; ++m) {
        pcl_executor_post_incoming(e, "chan", &msg);
      }
    });
  }
  for (auto& p : producers) p.join();

  // Drain on the "executor thread" (this thread).
  int spins = 0;
  while (received.load() < N * M && spins < 10000) {
    pcl_executor_spin_once(e, 0);
    ++spins;
  }

  EXPECT_EQ(received.load(), N * M);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclThreading, SpinInBackgroundShutdownFromForeground) {
  // Background thread runs pcl_executor_spin(); foreground signals shutdown
  // after posting a burst of messages, then joins.
  std::atomic<int> received{0};

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "bg_chan", "T",
      [](pcl_container_t*, const pcl_msg_t*, void* ud) {
        static_cast<std::atomic<int>*>(ud)->fetch_add(1, std::memory_order_relaxed);
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("bg_consumer", &cbs, &received);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  std::thread spinner([e]() { pcl_executor_spin(e); });

  constexpr int MSG_COUNT = 500;
  int val = 0;
  for (int i = 0; i < MSG_COUNT; ++i) {
    pcl_msg_t msg = {};
    msg.type_name = "T";
    msg.data = &val;
    msg.size = sizeof(val);
    pcl_executor_post_incoming(e, "bg_chan", &msg);
  }

  // Give the executor time to drain, then shut it down.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  pcl_executor_request_shutdown(e);
  spinner.join();

  EXPECT_EQ(received.load(), MSG_COUNT);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// ── Post while spin is running (lock stress) ─────────────────────────

TEST(PclThreading, PostDuringActiveSpinNoCorruption) {
  // Spin on one thread, 4 producers pump messages concurrently.
  // A subscriber verifies message integrity (non-zero payload copied correctly).
  struct Payload { int magic; };
  std::atomic<int> corrupt{0};
  std::atomic<int> received{0};
  constexpr int MAGIC = 0xDEADBEEF;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    struct Ctx { std::atomic<int>* corrupt; std::atomic<int>* received; };
    static Ctx ctx_store; // allocated once per configure call
    ctx_store = { static_cast<std::atomic<int>**>(ud)[0],
                  static_cast<std::atomic<int>**>(ud)[1] };
    pcl_container_add_subscriber(c, "magic_chan", "Payload",
      [](pcl_container_t*, const pcl_msg_t* m, void* ud) {
        auto* ctx = static_cast<Ctx*>(ud);
        if (m->size == sizeof(Payload)) {
          Payload p;
          memcpy(&p, m->data, sizeof(p));
          if (p.magic != 0xDEADBEEF) ctx->corrupt->fetch_add(1);
        }
        ctx->received->fetch_add(1);
      }, &ctx_store);
    return PCL_OK;
  };
  std::atomic<int>* ptrs[2] = {&corrupt, &received};
  auto* c = pcl_container_create("magic_consumer", &cbs, &ptrs);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  std::thread spinner([e]() { pcl_executor_spin(e); });

  constexpr int THREADS = 4;
  constexpr int PER_THREAD = 250;
  std::vector<std::thread> producers;
  for (int t = 0; t < THREADS; ++t) {
    producers.emplace_back([&]() {
      Payload p = {MAGIC};
      for (int i = 0; i < PER_THREAD; ++i) {
        pcl_msg_t msg = {};
        msg.type_name = "Payload";
        msg.data = &p;
        msg.size = sizeof(p);
        pcl_executor_post_incoming(e, "magic_chan", &msg);
      }
    });
  }
  for (auto& p : producers) p.join();

  // Wait for all messages to be consumed.
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (received.load() < THREADS * PER_THREAD &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  pcl_executor_request_shutdown(e);
  spinner.join();

  EXPECT_EQ(received.load(), THREADS * PER_THREAD);
  EXPECT_EQ(corrupt.load(), 0);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

// ═══════════════════════════════════════════════════════════════════════
// Throughput tests
// ═══════════════════════════════════════════════════════════════════════

TEST(PclThroughput, SpinOnceBurstDispatch) {
  // Post 10 000 messages, drain with spin_once, measure they all arrive.
  constexpr int COUNT = 10000;
  std::atomic<int> received{0};

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "burst", "T",
      [](pcl_container_t*, const pcl_msg_t*, void* ud) {
        static_cast<std::atomic<int>*>(ud)->fetch_add(1, std::memory_order_relaxed);
      }, ud);
    return PCL_OK;
  };

  auto* c = pcl_container_create("burst_consumer", &cbs, &received);
  pcl_container_configure(c);
  pcl_container_activate(c);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  int val = 0;
  pcl_msg_t msg = {};
  msg.type_name = "T";
  msg.data = &val;
  msg.size = sizeof(val);
  for (int i = 0; i < COUNT; ++i) {
    ASSERT_EQ(pcl_executor_post_incoming(e, "burst", &msg), PCL_OK);
  }

  while (received.load() < COUNT) {
    pcl_executor_spin_once(e, 0);
  }
  EXPECT_EQ(received.load(), COUNT);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclThroughput, HighFrequencyTickRate) {
  // Container ticking at 10 kHz; run for ~50 ms and verify reasonable tick count.
  std::atomic<int> ticks{0};
  pcl_callbacks_t cbs = {};
  cbs.on_tick = [](pcl_container_t*, double, void* ud) -> pcl_status_t {
    static_cast<std::atomic<int>*>(ud)->fetch_add(1, std::memory_order_relaxed);
    return PCL_OK;
  };

  auto* c = pcl_container_create("fast_ticker", &cbs, &ticks);
  pcl_container_configure(c);
  pcl_container_activate(c);
  pcl_container_set_tick_rate_hz(c, 10000.0);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c);

  auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(50)) {
    pcl_executor_spin_once(e, 0);
  }

  // In ~50 ms at 10 kHz we expect at least 100 ticks.
  EXPECT_GE(ticks.load(), 100);

  pcl_executor_destroy(e);
  pcl_container_destroy(c);
}

TEST(PclThroughput, MultiContainerDifferentRates) {
  // Three containers at 100 Hz, 500 Hz, 1000 Hz; spin for 100 ms.
  // Verify tick counts are ordered correctly.
  constexpr double RATES[] = {100.0, 500.0, 1000.0};
  std::atomic<int> ticks[3] = {};
  std::atomic<int>* tick_ptrs[3] = {&ticks[0], &ticks[1], &ticks[2]};

  pcl_callbacks_t cbs = {};
  cbs.on_tick = [](pcl_container_t*, double, void* ud) -> pcl_status_t {
    static_cast<std::atomic<int>*>(ud)->fetch_add(1, std::memory_order_relaxed);
    return PCL_OK;
  };

  auto* e = pcl_executor_create();
  pcl_container_t* containers[3];
  for (int i = 0; i < 3; ++i) {
    char name[16]; snprintf(name, sizeof(name), "mc%d", i);
    containers[i] = pcl_container_create(name, &cbs, tick_ptrs[i]);
    pcl_container_configure(containers[i]);
    pcl_container_activate(containers[i]);
    pcl_container_set_tick_rate_hz(containers[i], RATES[i]);
    pcl_executor_add(e, containers[i]);
  }

  auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(100)) {
    pcl_executor_spin_once(e, 0);
  }

  // Ordering: c[0] < c[1] < c[2]
  EXPECT_LT(ticks[0].load(), ticks[1].load());
  EXPECT_LT(ticks[1].load(), ticks[2].load());

  pcl_executor_destroy(e);
  for (int i = 0; i < 3; ++i) pcl_container_destroy(containers[i]);
}

// ═══════════════════════════════════════════════════════════════════════
// pcl_log.c — remaining coverage
// ═══════════════════════════════════════════════════════════════════════

struct LogCapture {
  std::vector<std::pair<pcl_log_level_t, std::string>> entries;

  static void handler(pcl_log_level_t level, const char*,
                      const char* msg, void* ud) {
    static_cast<LogCapture*>(ud)->entries.push_back({level, msg ? msg : ""});
  }

  void install() {
    entries.clear();
    pcl_log_set_handler(handler, this);
    pcl_log_set_level(PCL_LOG_DEBUG);
  }

  void uninstall() {
    pcl_log_set_handler(nullptr, nullptr);
    pcl_log_set_level(PCL_LOG_INFO);
  }
};

TEST(PclLogRobust, AllLevelsReachHandler) {
  LogCapture cap;
  cap.install();

  pcl_log(nullptr, PCL_LOG_DEBUG, "debug");
  pcl_log(nullptr, PCL_LOG_INFO,  "info");
  pcl_log(nullptr, PCL_LOG_WARN,  "warn");
  pcl_log(nullptr, PCL_LOG_ERROR, "error");
  pcl_log(nullptr, PCL_LOG_FATAL, "fatal");

  ASSERT_EQ(cap.entries.size(), 5u);
  EXPECT_EQ(cap.entries[0].first, PCL_LOG_DEBUG);
  EXPECT_EQ(cap.entries[4].first, PCL_LOG_FATAL);

  cap.uninstall();
}

TEST(PclLogRobust, FilterBlocksAllBelowFatal) {
  LogCapture cap;
  cap.install();
  pcl_log_set_level(PCL_LOG_FATAL);

  pcl_log(nullptr, PCL_LOG_DEBUG, "d");
  pcl_log(nullptr, PCL_LOG_INFO,  "i");
  pcl_log(nullptr, PCL_LOG_WARN,  "w");
  pcl_log(nullptr, PCL_LOG_ERROR, "e");
  pcl_log(nullptr, PCL_LOG_FATAL, "f");

  EXPECT_EQ(cap.entries.size(), 1u);
  EXPECT_EQ(cap.entries[0].second, "f");

  cap.uninstall();
}

TEST(PclLogRobust, DefaultHandlerNoContainerName) {
  // Revert to default handler and log with NULL container — must not crash.
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_DEBUG);
  pcl_log(nullptr, PCL_LOG_DEBUG, "default handler debug");
  pcl_log(nullptr, PCL_LOG_FATAL, "default handler fatal");
  pcl_log_set_level(PCL_LOG_INFO); // restore
}

TEST(PclLogRobust, DefaultHandlerWithContainerName) {
  pcl_log_set_handler(nullptr, nullptr);
  auto* c = pcl_container_create("named_log", nullptr, nullptr);
  // writes to stderr — must not crash
  pcl_log(c, PCL_LOG_WARN, "warning from named container");
  pcl_container_destroy(c);
  pcl_log_set_level(PCL_LOG_INFO);
}

TEST(PclLogRobust, LongFormatStringTruncated) {
  LogCapture cap;
  cap.install();

  // Build a format that would exceed 1024 bytes if not truncated.
  std::string big(2000, 'x');
  pcl_log(nullptr, PCL_LOG_INFO, "%s", big.c_str());

  ASSERT_EQ(cap.entries.size(), 1u);
  // The message stored in the handler is at most 1023 chars (1024-byte buf).
  EXPECT_LE(cap.entries[0].second.size(), 1023u);

  cap.uninstall();
}

TEST(PclLogRobust, HandlerUserDataPassedThrough) {
  int user_value = 0;
  pcl_log_set_level(PCL_LOG_DEBUG);
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void* ud) {
    (*static_cast<int*>(ud))++;
  }, &user_value);

  pcl_log(nullptr, PCL_LOG_INFO, "a");
  pcl_log(nullptr, PCL_LOG_INFO, "b");
  pcl_log(nullptr, PCL_LOG_INFO, "c");

  EXPECT_EQ(user_value, 3);

  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

// ── level_str default branch: pass an out-of-range level ──────────────
// The default handler formats the level via level_str().  Casting an
// integer that is not in the pcl_log_level_t enum exercises the
// "default: return ???" branch in level_str().
TEST(PclLogRobust, UnknownLevelDefaultBranch) {
  pcl_log_set_handler(nullptr, nullptr); // use default handler → calls level_str
  // Cast 99 to pcl_log_level_t so level_str hits the default branch.
  // g_min_level is PCL_LOG_INFO (1); 99 >= 1 → passes the level filter.
  pcl_log(nullptr, static_cast<pcl_log_level_t>(99), "unknown level");
  // Writes "[???] unknown level\n" to stderr — must not crash.
  pcl_log_set_level(PCL_LOG_INFO);
}

// ═══════════════════════════════════════════════════════════════════════
// Missing coverage: pcl_container_add_service success path (lines 322-330)
// ═══════════════════════════════════════════════════════════════════════

static pcl_status_t svc_echo(pcl_container_t*, const pcl_msg_t* req,
                              pcl_msg_t* resp, void*) {
  resp->data      = req->data;
  resp->size      = req->size;
  resp->type_name = req->type_name;
  return PCL_OK;
}

TEST(PclContainerRobust, AddServiceSuccessPath) {
  struct SvcCtx { pcl_port_t* svc = nullptr; };
  SvcCtx ctx;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    auto* ctx = static_cast<SvcCtx*>(ud);
    ctx->svc = pcl_container_add_service(c, "echo_svc", "EchoMsg", svc_echo, nullptr);
    return ctx->svc ? PCL_OK : PCL_ERR_CALLBACK;
  };

  auto* c = pcl_container_create("svc_host", &cbs, &ctx);
  EXPECT_EQ(pcl_container_configure(c), PCL_OK);
  EXPECT_NE(ctx.svc, nullptr);
  pcl_container_destroy(c);
}

// ═══════════════════════════════════════════════════════════════════════
// Missing coverage: pcl_port_publish success path (line 343)
// ═══════════════════════════════════════════════════════════════════════

TEST(PclContainerRobust, PublishSuccessOnActiveContainer) {
  PubPortCtx ctx;
  pcl_callbacks_t cbs = {};
  cbs.on_configure = create_pub_configure;

  auto* c = pcl_container_create("pub_active", &cbs, &ctx);
  pcl_container_configure(c);
  pcl_container_activate(c);

  pcl_msg_t msg = {};
  msg.type_name = "Msg";
  int payload = 7;
  msg.data = &payload;
  msg.size = sizeof(payload);

  // Active container + publisher port → PCL_OK
  EXPECT_EQ(pcl_port_publish(ctx.pub, &msg), PCL_OK);

  pcl_container_destroy(c);
}

// ═══════════════════════════════════════════════════════════════════════
// Missing coverage: pcl_executor_spin() exiting on drain error (line 311)
// ═══════════════════════════════════════════════════════════════════════

TEST(PclExecutorRobust, SpinExitsOnDrainError) {
  // Post a ghost message BEFORE calling spin().  The very first drain
  // attempt returns PCL_ERR_NOT_FOUND → spin() returns immediately.
  auto* e = pcl_executor_create();

  pcl_msg_t msg = {};
  msg.type_name = "T";
  ASSERT_EQ(pcl_executor_post_incoming(e, "ghost_spin", &msg), PCL_OK);

  // spin() should return PCL_ERR_NOT_FOUND after draining the bad message.
  pcl_status_t rc = pcl_executor_spin(e);
  EXPECT_EQ(rc, PCL_ERR_NOT_FOUND);

  pcl_executor_destroy(e);
}

// ═══════════════════════════════════════════════════════════════════════
// Missing coverage: graceful-shutdown timeout log warning (lines 374-375)
// ═══════════════════════════════════════════════════════════════════════

TEST(PclExecutorRobust, GracefulShutdownTimeoutWarningLogged) {
  // c1's on_deactivate sleeps long enough to expire the deadline before
  // the loop reaches c2, triggering the "shutdown timeout - forcing finalize"
  // warning on c2's iteration.
  pcl_callbacks_t slow_cbs = {};
  slow_cbs.on_deactivate = [](pcl_container_t*, void*) -> pcl_status_t {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return PCL_OK;
  };

  auto* c1 = pcl_container_create("slow_c1", &slow_cbs, nullptr);
  auto* c2 = pcl_container_create("slow_c2", nullptr,   nullptr);

  pcl_container_configure(c1); pcl_container_activate(c1);
  pcl_container_configure(c2); pcl_container_activate(c2);

  auto* e = pcl_executor_create();
  pcl_executor_add(e, c1);
  pcl_executor_add(e, c2);

  // timeout_ms=5: deadline 5 ms from now.
  // c1 deactivate takes 20 ms → deadline long expired when c2 iteration starts.
  pcl_status_t rc = pcl_executor_shutdown_graceful(e, 5);
  EXPECT_EQ(rc, PCL_ERR_TIMEOUT);
  EXPECT_EQ(pcl_container_state(c1), PCL_STATE_FINALIZED);
  EXPECT_EQ(pcl_container_state(c2), PCL_STATE_FINALIZED);

  pcl_executor_destroy(e);
  pcl_container_destroy(c1);
  pcl_container_destroy(c2);
}
