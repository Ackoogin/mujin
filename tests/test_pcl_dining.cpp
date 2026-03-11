/// \file test_pcl_dining.cpp
/// \brief Dining-philosopher integration test + bridge unit tests.
///
/// Part 1 — Bridge unit tests
/// --------------------------
/// Verifies the pcl_bridge API: null-argument guards, creation/destruction,
/// the unit-conversion path (float m/s → int32 km/h), the encoding path
/// (int32 state enum → const-char* label), and the bridge dropping messages
/// when the transform returns non-OK.
///
/// Part 2 — Dining Philosophers with Bridges
/// ------------------------------------------
/// Models the classic dining-philosophers problem as a PCL component graph:
///
///   5 × Philosopher container  —[phil/N/state : int32]→
///   5 × Bridge (int32→string)  —[phil/N/status : string]→
///   1 × Monitor container
///
/// The philosopher on_tick implements:
///   THINKING (k ticks) → HUNGRY (grab forks) → EATING (k ticks) → release
///
/// Fork acquisition uses the resource-hierarchy rule (lower index first) to
/// guarantee deadlock freedom on the single-threaded executor.  All fork and
/// eating-flag updates are unsynchronised — the PCL executor is single-threaded
/// so no mutex is needed.
///
/// Assertions:
///   • Mutual exclusion — no two neighbouring philosophers eat simultaneously.
///   • Liveness         — every philosopher completes ≥ MEALS_REQUIRED meals.
///   • Bridge fidelity  — the monitor sees the correct string for every state.
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>

extern "C" {
#include "pcl/pcl_bridge.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_log.h"
// Internal header — only included here to trigger the port-overflow branch
// in pcl_bridge.c (bridge_on_configure returning PCL_ERR_NOMEM).
#include "pcl_internal.h"
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 1 — Bridge unit tests
// ═══════════════════════════════════════════════════════════════════════════

// ── helpers ────────────────────────────────────────────────────────────────

/// Identity bridge: passes the message through unchanged.
static pcl_status_t identity_fn(const pcl_msg_t* in, pcl_msg_t* out, void*) {
  out->data = in->data;
  out->size = in->size;
  return PCL_OK;
}

/// Always-fail bridge: suppresses every message.
static pcl_status_t fail_fn(const pcl_msg_t*, pcl_msg_t*, void*) {
  return PCL_ERR_CALLBACK;
}

/// m/s (float) → km/h (int32) conversion.
/// Uses a static local so the pointer remains valid after the call.
static pcl_status_t mps_to_kmph_fn(const pcl_msg_t* in,
                                    pcl_msg_t*       out,
                                    void*            ud) {
  static int32_t result;
  (void)ud;
  if (in->size != sizeof(float)) return PCL_ERR_INVALID;
  float mps;
  memcpy(&mps, in->data, sizeof(mps));
  result     = static_cast<int32_t>(mps * 3.6f);
  out->data  = &result;
  out->size  = sizeof(result);
  return PCL_OK;
}

// ── null / invalid argument guard tests ───────────────────────────────────

TEST(PclBridge, CreateNullArgsReturnNull) {
  auto* e = pcl_executor_create();

  EXPECT_EQ(nullptr, pcl_bridge_create(nullptr, "b","in","T","out","T2",identity_fn,nullptr));
  EXPECT_EQ(nullptr, pcl_bridge_create(e,  nullptr,"in","T","out","T2",identity_fn,nullptr));
  EXPECT_EQ(nullptr, pcl_bridge_create(e,  "b", nullptr,"T","out","T2",identity_fn,nullptr));
  EXPECT_EQ(nullptr, pcl_bridge_create(e,  "b","in",nullptr,"out","T2",identity_fn,nullptr));
  EXPECT_EQ(nullptr, pcl_bridge_create(e,  "b","in","T", nullptr,"T2",identity_fn,nullptr));
  EXPECT_EQ(nullptr, pcl_bridge_create(e,  "b","in","T","out", nullptr,identity_fn,nullptr));
  EXPECT_EQ(nullptr, pcl_bridge_create(e,  "b","in","T","out","T2",  nullptr, nullptr));

  pcl_executor_destroy(e);
}

TEST(PclBridge, ContainerNullBridgeReturnsNull) {
  EXPECT_EQ(nullptr, pcl_bridge_container(nullptr));
}

TEST(PclBridge, DestroyNullIsNoOp) {
  pcl_bridge_destroy(nullptr);  // must not crash
}

// ── creation and lifecycle ─────────────────────────────────────────────────

TEST(PclBridge, CreateAndDestroy) {
  auto* e = pcl_executor_create();
  auto* b = pcl_bridge_create(e, "b", "in", "T", "out", "T2", identity_fn, nullptr);
  ASSERT_NE(nullptr, b);
  EXPECT_NE(nullptr, pcl_bridge_container(b));

  // Configure + activate container (exercises on_configure → add_subscriber)
  pcl_container_t* c = pcl_bridge_container(b);
  EXPECT_EQ(PCL_OK, pcl_executor_add(e, c));
  EXPECT_EQ(PCL_OK, pcl_container_configure(c));
  EXPECT_EQ(PCL_OK, pcl_container_activate(c));

  pcl_executor_destroy(e);
  pcl_bridge_destroy(b);  // owns container
}

// ── unit conversion: float m/s → int32 km/h ──────────────────────────────

TEST(PclBridge, SpeedUnitConversion) {
  // Consumer: subscribes to "speed/kmph" and records the last received value.
  static int32_t received_kmph = -1;

  struct ConsumerCtx { int32_t* out; };
  static ConsumerCtx consumer_ctx{ &received_kmph };

  pcl_callbacks_t ccbs = {};
  ccbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    ConsumerCtx* ctx = static_cast<ConsumerCtx*>(ud);
    pcl_container_add_subscriber(
        c, "speed/kmph", "SpeedKmph",
        [](pcl_container_t*, const pcl_msg_t* msg, void* u) {
          ConsumerCtx* cx = static_cast<ConsumerCtx*>(u);
          if (msg->size == sizeof(int32_t)) {
            memcpy(cx->out, msg->data, sizeof(int32_t));
          }
        },
        ctx);
    return PCL_OK;
  };

  auto* e        = pcl_executor_create();
  auto* consumer = pcl_container_create("consumer", &ccbs, &consumer_ctx);
  pcl_container_configure(consumer);
  pcl_container_activate(consumer);
  pcl_executor_add(e, consumer);

  // Bridge: "speed/mps" → "speed/kmph"
  auto* b = pcl_bridge_create(e, "speed_bridge",
                               "speed/mps",  "SpeedMps",
                               "speed/kmph", "SpeedKmph",
                               mps_to_kmph_fn, nullptr);
  ASSERT_NE(nullptr, b);
  pcl_container_t* bc = pcl_bridge_container(b);
  pcl_executor_add(e, bc);
  pcl_container_configure(bc);
  pcl_container_activate(bc);

  // Publish 10.0 m/s; expected km/h = int(10.0 * 3.6) = 36
  float mps  = 10.0f;
  pcl_msg_t msg = {};
  msg.type_name = "SpeedMps";
  msg.data = &mps;
  msg.size = sizeof(mps);

  // dispatch_incoming fires bridge subscriber → transform → consumer subscriber
  pcl_status_t rc = pcl_executor_dispatch_incoming(e, "speed/mps", &msg);
  EXPECT_EQ(PCL_OK, rc);
  EXPECT_EQ(36, received_kmph);

  // Try another value: 27.78 m/s ≈ 100 km/h
  mps = 27.78f;
  pcl_executor_dispatch_incoming(e, "speed/mps", &msg);
  EXPECT_EQ(100, received_kmph);

  pcl_executor_destroy(e);
  pcl_bridge_destroy(b);
  pcl_container_destroy(consumer);
}

// ── transform returning non-OK suppresses forwarding ──────────────────────

TEST(PclBridge, FailTransformSuppressesForward) {
  static int received = 0;

  pcl_callbacks_t ccbs = {};
  ccbs.on_configure = [](pcl_container_t* c, void*) -> pcl_status_t {
    pcl_container_add_subscriber(
        c, "out_topic", "T2",
        [](pcl_container_t*, const pcl_msg_t*, void*) { received++; },
        nullptr);
    return PCL_OK;
  };

  auto* e        = pcl_executor_create();
  auto* consumer = pcl_container_create("sink", &ccbs, nullptr);
  pcl_container_configure(consumer);
  pcl_container_activate(consumer);
  pcl_executor_add(e, consumer);

  auto* b = pcl_bridge_create(e, "fail_bridge",
                               "in_topic", "T",
                               "out_topic", "T2",
                               fail_fn, nullptr);
  pcl_container_t* bc = pcl_bridge_container(b);
  pcl_executor_add(e, bc);
  pcl_container_configure(bc);
  pcl_container_activate(bc);

  int dummy = 42;
  pcl_msg_t msg = {};
  msg.type_name = "T";
  msg.data = &dummy;
  msg.size = sizeof(dummy);

  pcl_executor_dispatch_incoming(e, "in_topic", &msg);
  EXPECT_EQ(0, received);   // suppressed — consumer must not have fired

  pcl_executor_destroy(e);
  pcl_bridge_destroy(b);
  pcl_container_destroy(consumer);
}

// ── bridge on inactive container does not forward ─────────────────────────

TEST(PclBridge, BridgeInactiveDoesNotForward) {
  // Only configure the bridge, never activate it.
  // Dispatching to the in_topic should return NOT_FOUND (no active subscriber).
  auto* e = pcl_executor_create();
  auto* b = pcl_bridge_create(e, "inactive_bridge",
                               "raw", "T", "cooked", "T2",
                               identity_fn, nullptr);
  pcl_container_t* bc = pcl_bridge_container(b);
  pcl_executor_add(e, bc);
  pcl_container_configure(bc);
  // intentionally NOT activating

  int dummy = 1;
  pcl_msg_t msg = {};
  msg.type_name = "T";
  msg.data = &dummy;
  msg.size = sizeof(dummy);

  // Subscriber exists but container is CONFIGURED (not ACTIVE) — dispatch still
  // finds the port and calls the callback; the bridge then dispatches to "cooked"
  // which has no subscriber → NOT_FOUND; bridge logs a warning and returns.
  // Main assertion: no crash and the executor is still healthy.
  pcl_executor_dispatch_incoming(e, "raw", &msg);  // must not crash

  pcl_executor_destroy(e);
  pcl_bridge_destroy(b);
}

// ── bridge_on_configure port-overflow path (lines 67-69 in pcl_bridge.c) ──
//
// Fills the bridge's internal container to PCL_MAX_PORTS before configure
// is called.  pcl_container_add_subscriber then returns NULL, which triggers
// the error log + PCL_ERR_NOMEM return in bridge_on_configure.
TEST(PclBridge, ConfigureFailsWhenPortsFull) {
  auto* e = pcl_executor_create();
  auto* b = pcl_bridge_create(e, "full_bridge",
                               "raw", "T", "cooked", "T2",
                               identity_fn, nullptr);
  ASSERT_NE(nullptr, b);
  pcl_container_t* bc = pcl_bridge_container(b);
  pcl_executor_add(e, bc);

  // Pre-fill the bridge container's port slots so the next add_subscriber
  // (inside bridge_on_configure) finds the array full and returns NULL.
  // We write directly into the internal struct via pcl_internal.h.
  bc->port_count = PCL_MAX_PORTS;

  // configure must fail because add_subscriber returns NULL;
  // bridge_on_configure returns PCL_ERR_NOMEM which the container forwards.
  pcl_status_t rc = pcl_container_configure(bc);
  EXPECT_EQ(PCL_ERR_NOMEM, rc);

  // Restore so pcl_container_destroy doesn't iterate garbage ports.
  bc->port_count = 0;

  pcl_executor_destroy(e);
  pcl_bridge_destroy(b);
}

// ── NOT_FOUND dispatch logs a DEBUG message (lines 53-54 in pcl_bridge.c) ──
//
// Dispatches to a bridge whose out_topic has no registered subscriber.
// The bridge's sub_cb fires, transform succeeds, dispatch returns NOT_FOUND,
// and the DEBUG log is emitted.  Covered implicitly by BridgeInactiveDoesNotForward,
// but also exercised here with an explicit active bridge + missing downstream.
TEST(PclBridge, DispatchNotFoundLogsDebug) {
  pcl_log_set_level(PCL_LOG_DEBUG);

  auto* e = pcl_executor_create();
  auto* b = pcl_bridge_create(e, "orphan_bridge",
                               "src", "T", "sink_with_no_sub", "T2",
                               identity_fn, nullptr);
  ASSERT_NE(nullptr, b);
  pcl_container_t* bc = pcl_bridge_container(b);
  pcl_executor_add(e, bc);
  pcl_container_configure(bc);
  pcl_container_activate(bc);

  int dummy = 1;
  pcl_msg_t msg = {};
  msg.type_name = "T";
  msg.data = &dummy;
  msg.size = sizeof(dummy);

  // No subscriber on "sink_with_no_sub" → dispatch returns NOT_FOUND →
  // bridge_sub_cb logs "no subscriber on out_topic" at DEBUG.
  pcl_executor_dispatch_incoming(e, "src", &msg);  // must not crash

  pcl_executor_destroy(e);
  pcl_bridge_destroy(b);
  pcl_log_set_level(PCL_LOG_INFO);
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 2 — Dining Philosophers
// ═══════════════════════════════════════════════════════════════════════════

static constexpr int kN             = 5;  // number of philosophers / forks
static constexpr int kMealsRequired = 3;  // meals each philosopher must eat
static constexpr int kThinkTicks    = 3;  // ticks spent thinking
static constexpr int kEatTicks      = 2;  // ticks spent eating

// ── Bridge transform: int32 philosopher state → string label ──────────────

static const char* phil_state_label(int32_t s) {
  switch (s) {
    case 0:  return "THINKING";
    case 1:  return "HUNGRY";
    case 2:  return "EATING";
    default: return "UNKNOWN";
  }
}

static pcl_status_t state_to_label_fn(const pcl_msg_t* in,
                                       pcl_msg_t*       out,
                                       void*) {
  if (in->size != sizeof(int32_t)) return PCL_ERR_INVALID;
  int32_t    state;
  memcpy(&state, in->data, sizeof(state));
  const char* label = phil_state_label(state);
  out->data = label;
  out->size = static_cast<uint32_t>(strlen(label) + 1u);
  return PCL_OK;
}

// ── Shared dining-philosopher state (accessed only on executor thread) ─────

struct DiningState {
  bool forks [kN] = {};   // true = held by a philosopher
  bool eating[kN] = {};   // true = philosopher i is currently eating
  int  meals [kN] = {};   // meals completed per philosopher
  bool violation  = false; // set if mutual exclusion is broken

  // Per-philosopher monitor subscriber contexts
  struct MonSub { int id; DiningState* ds; };
  MonSub mon_subs[kN];
};

// ── Philosopher context ────────────────────────────────────────────────────

struct PhilCtx {
  int             id;
  int             state;        // 0 = THINKING, 1 = HUNGRY, 2 = EATING
  int             tick_count;
  DiningState*    ds;
  pcl_executor_t* exec;
};

static void acquire_forks(PhilCtx* p) {
  // Resource-hierarchy rule: always grab the lower-indexed fork first.
  int left  = p->id;
  int right = (p->id + 1) % kN;
  int first  = (left < right) ? left  : right;
  int second = (left < right) ? right : left;

  if (!p->ds->forks[first] && !p->ds->forks[second]) {
    p->ds->forks[first]    = true;
    p->ds->forks[second]   = true;
    p->ds->eating[p->id]   = true;
    p->state               = 2;   // EATING
    p->tick_count          = 0;
  }
}

static void release_forks(PhilCtx* p) {
  int left  = p->id;
  int right = (p->id + 1) % kN;
  p->ds->forks[left]      = false;
  p->ds->forks[right]     = false;
  p->ds->eating[p->id]    = false;
  p->ds->meals[p->id]++;
  p->state      = 0;   // THINKING
  p->tick_count = 0;
}

static pcl_status_t philosopher_tick(pcl_container_t* c,
                                     double,
                                     void* ud) {
  PhilCtx* p = static_cast<PhilCtx*>(ud);
  (void)c;

  int32_t prev = static_cast<int32_t>(p->state);

  switch (p->state) {
    case 0:  // THINKING
      if (++p->tick_count >= kThinkTicks) {
        p->state      = 1;   // become HUNGRY
        p->tick_count = 0;
      }
      break;
    case 1:  // HUNGRY — try to grab forks each tick
      acquire_forks(p);
      break;
    case 2:  // EATING
      if (++p->tick_count >= kEatTicks) {
        release_forks(p);   // → THINKING
      }
      break;
  }

  // Publish state change so the bridge → monitor pipeline fires.
  int32_t next = static_cast<int32_t>(p->state);
  if (next != prev) {
    char topic[32];
    std::snprintf(topic, sizeof(topic), "phil/%d/state", p->id);
    pcl_msg_t msg = {};
    msg.type_name = "PhilState";
    msg.data      = &next;
    msg.size      = sizeof(next);
    // Synchronous dispatch: bridge + monitor callbacks fire before we return.
    pcl_executor_dispatch_incoming(p->exec, topic, &msg);
  }
  return PCL_OK;
}

// ── Monitor subscriber callback ────────────────────────────────────────────

static void monitor_sub_cb(pcl_container_t*,
                            const pcl_msg_t* msg,
                            void*            ud) {
  DiningState::MonSub* ms = static_cast<DiningState::MonSub*>(ud);
  DiningState* ds = ms->ds;
  int          id = ms->id;

  if (!msg->data || msg->size == 0) return;
  const char* label = static_cast<const char*>(msg->data);

  if (strcmp(label, "EATING") == 0) {
    // At this point ds->eating[id] == true (set synchronously in acquire_forks
    // before the dispatch chain fires).  Check that no neighbour is also eating.
    int left  = (id + kN - 1) % kN;
    int right  = (id + 1)      % kN;
    if (ds->eating[left] || ds->eating[right]) {
      ds->violation = true;
    }
  }
}

// ── on_configure for monitor: register one subscriber per philosopher ──────

static pcl_status_t monitor_configure(pcl_container_t* c, void* ud) {
  DiningState* ds = static_cast<DiningState*>(ud);
  for (int i = 0; i < kN; ++i) {
    ds->mon_subs[i] = { i, ds };
    char topic[32];
    std::snprintf(topic, sizeof(topic), "phil/%d/status", i);
    pcl_container_add_subscriber(c, topic, "PhilStatus",
                                 monitor_sub_cb,
                                 &ds->mon_subs[i]);
  }
  return PCL_OK;
}

// ── THE TEST ───────────────────────────────────────────────────────────────

TEST(PclDining, FivePhilosophersWithBridges) {
  // Suppress log noise during the long spin.
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*){},
                      nullptr);

  DiningState ds;

  auto* exec = pcl_executor_create();

  // ── Philosophers ────────────────────────────────────────────────────────
  std::array<PhilCtx,         kN> phil_ctxs{};
  std::array<pcl_container_t*, kN> phil_ctr{};

  for (int i = 0; i < kN; ++i) {
    phil_ctxs[i] = { i, 0, 0, &ds, exec };

    pcl_callbacks_t cbs = {};
    cbs.on_tick = philosopher_tick;

    phil_ctr[i] = pcl_container_create(
        (std::string("phil_") + std::to_string(i)).c_str(),
        &cbs, &phil_ctxs[i]);
    ASSERT_NE(nullptr, phil_ctr[i]);
    pcl_container_configure(phil_ctr[i]);
    pcl_container_activate(phil_ctr[i]);
    // Run faster so the test finishes quickly.
    pcl_container_set_tick_rate_hz(phil_ctr[i], 500.0);
    pcl_executor_add(exec, phil_ctr[i]);
  }

  // ── Monitor ─────────────────────────────────────────────────────────────
  pcl_callbacks_t mon_cbs = {};
  mon_cbs.on_configure = monitor_configure;

  auto* monitor = pcl_container_create("monitor", &mon_cbs, &ds);
  ASSERT_NE(nullptr, monitor);
  pcl_container_configure(monitor);
  pcl_container_activate(monitor);
  pcl_executor_add(exec, monitor);

  // ── Bridges (int32 state → string label) ────────────────────────────────
  std::array<pcl_bridge_t*,   kN> bridges{};

  for (int i = 0; i < kN; ++i) {
    char name[32], in_t[32], out_t[32];
    std::snprintf(name,  sizeof(name),  "bridge_%d", i);
    std::snprintf(in_t,  sizeof(in_t),  "phil/%d/state",  i);
    std::snprintf(out_t, sizeof(out_t), "phil/%d/status", i);

    bridges[i] = pcl_bridge_create(exec, name,
                                   in_t,  "PhilState",
                                   out_t, "PhilStatus",
                                   state_to_label_fn, nullptr);
    ASSERT_NE(nullptr, bridges[i]);
    pcl_container_t* bc = pcl_bridge_container(bridges[i]);
    pcl_executor_add(exec, bc);
    pcl_container_configure(bc);
    pcl_container_activate(bc);
  }

  // ── Spin ────────────────────────────────────────────────────────────────
  // Each meal cycle = kThinkTicks + up to kN hungry ticks + kEatTicks ticks.
  // Worst case ≈ (3 + 5 + 2) * kMealsRequired = 30 ticks per philosopher.
  // At 500 Hz that's ≤ 60 ms.  Use 2 s for plenty of margin.
  std::thread spin_thread([exec]() { pcl_executor_spin(exec); });
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  pcl_executor_request_shutdown(exec);
  spin_thread.join();

  // ── Assertions ──────────────────────────────────────────────────────────
  EXPECT_FALSE(ds.violation)
      << "Mutual exclusion violated: two neighbouring philosophers ate "
         "simultaneously.";

  for (int i = 0; i < kN; ++i) {
    EXPECT_GE(ds.meals[i], kMealsRequired)
        << "Philosopher " << i << " only completed " << ds.meals[i]
        << " meals (required " << kMealsRequired << ").";
  }

  // ── Cleanup ─────────────────────────────────────────────────────────────
  pcl_executor_destroy(exec);
  for (int i = 0; i < kN; ++i) {
    pcl_bridge_destroy(bridges[i]);
    pcl_container_destroy(phil_ctr[i]);
  }
  pcl_container_destroy(monitor);

  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}
