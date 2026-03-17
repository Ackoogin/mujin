/// \file test_pcl_oom.cpp
/// \brief OOM-injection tests for PCL executor — covers the malloc-failure
///        guard branches in pcl_executor_post_incoming that cannot be reached
///        without deliberately making allocations fail.
///
/// Linked with -Wl,--wrap=malloc,--wrap=calloc so that a thread-local
/// countdown can steer exactly which heap allocation fails.
#include <gtest/gtest.h>

#include <stdlib.h>
#include <string.h>

extern "C" {
#include "pcl/pcl_executor.h"
#include "pcl/pcl_container.h"
#include "pcl/pcl_bridge.h"
#include "pcl/pcl_log.h"
}

// ── Malloc interposer ────────────────────────────────────────────────────
//
// g_oom_countdown (thread-local):
//   -1  → never fail (normal operation)
//    0  → fail the very next allocation
//    N  → skip N allocations, fail the (N+1)-th
//
// After a failure fires, the countdown resets to -1 automatically.

extern "C" void* __real_malloc(size_t size);
extern "C" void* __real_calloc(size_t nmemb, size_t size);

static thread_local int g_oom_countdown = -1;

extern "C" void* __wrap_malloc(size_t size) {
  if (g_oom_countdown >= 0) {
    if (g_oom_countdown == 0) {
      g_oom_countdown = -1;
      return nullptr;
    }
    --g_oom_countdown;
  }
  return __real_malloc(size);
}

extern "C" void* __wrap_calloc(size_t nmemb, size_t size) {
  if (g_oom_countdown >= 0) {
    if (g_oom_countdown == 0) {
      g_oom_countdown = -1;
      return nullptr;
    }
    --g_oom_countdown;
  }
  return __real_calloc(nmemb, size);
}

// Helper: silence logs during OOM tests so stderr isn't cluttered.
static void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {}, nullptr);
}
static void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

// ── pcl_executor_post_incoming allocation sequence ──────────────────────
//
// Inside post_incoming the allocations happen in this order:
//   #0  calloc(1, sizeof(pcl_pending_msg_t))       → if NULL → return PCL_ERR_NOMEM
//   #1  malloc(strlen(topic)+1)  [pcl_strdup_local] → if NULL → free+return PCL_ERR_NOMEM  (line 429)
//   #2  malloc(strlen(type)+1)   [pcl_strdup_local] → if NULL → free+return PCL_ERR_NOMEM  (line 429)
//   #3  malloc(msg->size)                           → if NULL → free+return PCL_ERR_NOMEM  (line 435-436)

// Helper: call fn with OOM triggered on the Nth allocation from entry.
// Storing the result BEFORE any EXPECT_* macro ensures no GTest-internal
// allocations consume the countdown between arming and the target call.
struct OomPostCtx { pcl_executor_t* e; const char* topic; pcl_msg_t* msg; };

static pcl_status_t do_post(void* ud) {
  auto* c = static_cast<OomPostCtx*>(ud);
  return pcl_executor_post_incoming(c->e, c->topic, c->msg);
}

// Allocation order before post_incoming: executor_create uses 1× calloc.
// Inside post_incoming: calloc(pending), malloc(topic), malloc(type_name), malloc(data).
// Countdown N = skip N allocations, fail the (N+1)-th.

// ── Fail allocation #0: calloc for pcl_pending_msg_t ─────────────────

TEST(PclOom, PostIncomingPendingAllocFails) {
  silence_logs();
  auto* e = pcl_executor_create();
  pcl_msg_t msg = {}; msg.type_name = "T";
  OomPostCtx ctx{e, "t", &msg};

  g_oom_countdown = 0;               // fail the very next alloc (pending calloc)
  pcl_status_t rc = do_post(&ctx);
  g_oom_countdown = -1;
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pcl_executor_destroy(e);
  restore_logs();
}

// ── Fail allocation #1: topic strdup → free_pending_msg path ─────────

TEST(PclOom, PostIncomingTopicStrdupFails) {
  silence_logs();
  auto* e = pcl_executor_create();
  pcl_msg_t msg = {}; msg.type_name = "T";
  OomPostCtx ctx{e, "t", &msg};

  g_oom_countdown = 2;                // skip executor+pending, fail malloc(topic)
  pcl_status_t rc = do_post(&ctx);
  g_oom_countdown = -1;
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pcl_executor_destroy(e);
  restore_logs();
}

// ── Fail allocation #2: type_name strdup → free_pending_msg path ─────

TEST(PclOom, PostIncomingTypeNameStrdupFails) {
  silence_logs();
  auto* e = pcl_executor_create();
  pcl_msg_t msg = {}; msg.type_name = "T";
  OomPostCtx ctx{e, "t", &msg};

  g_oom_countdown = 2;                // skip pending+topic, fail malloc(type_name) → free_pending_msg
  pcl_status_t rc = do_post(&ctx);
  g_oom_countdown = -1;
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pcl_executor_destroy(e);
  restore_logs();
}

// ── Fail allocation #3: data malloc → lines 467-469 ──────────────────

TEST(PclOom, PostIncomingDataMallocFails) {
  silence_logs();
  auto* e = pcl_executor_create();
  int payload = 42;
  pcl_msg_t msg = {}; msg.type_name = "T"; msg.data = &payload; msg.size = sizeof(payload);
  OomPostCtx ctx{e, "t", &msg};

  g_oom_countdown = 3;                // skip pending+topic+type_name, fail malloc(data)
  pcl_status_t rc = do_post(&ctx);
  g_oom_countdown = -1;
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pcl_executor_destroy(e);
  restore_logs();
}

// ── Bridge: container_create fails (pcl_bridge.c lines 108-109) ──────────
//
// pcl_bridge_create allocates the bridge struct (calloc #0), then calls
// pcl_container_create which does its own calloc (#1).  Failing #1 causes
// pcl_bridge_create to free the bridge struct and return NULL.

static pcl_status_t dummy_bridge_fn(const pcl_msg_t*, pcl_msg_t*, void*) {
  return PCL_OK;
}

struct OomBridgeCtx { pcl_executor_t* e; pcl_bridge_t* result; };

static pcl_status_t do_bridge_create(void* ud) {
  auto* c = static_cast<OomBridgeCtx*>(ud);
  c->result = pcl_bridge_create(c->e, "b", "in", "T", "out", "T2",
                                dummy_bridge_fn, nullptr);
  return PCL_OK;
}

TEST(PclOom, BridgeCreateContainerAllocFails) {
  silence_logs();
  auto* e = pcl_executor_create();
  OomBridgeCtx ctx{e, nullptr};

  // Alloc #0 = calloc for pcl_bridge_t (succeed)
  // Alloc #1 = calloc for pcl_container_t inside pcl_container_create (fail)
  g_oom_countdown = 1;
  do_bridge_create(&ctx);          // no GTest macro between arm and call
  g_oom_countdown = -1;
  EXPECT_EQ(nullptr, ctx.result);

  pcl_executor_destroy(e);
  restore_logs();
}

// ── post_response_cb: data malloc fails (lines 553-554) ──────────────────
//
// Inside pcl_executor_post_response_cb the allocation sequence is:
//   #0  calloc(1, sizeof(node))   → node created
//   #1  malloc(size)              → data buffer; if NULL: free(node) + ERR_NOMEM

TEST(PclOom, PostResponseCbDataMallocFails) {
  silence_logs();
  auto* e = pcl_executor_create();

  auto cb = [](const pcl_msg_t*, void*) {};
  int payload = 42;

  // Fail the 2nd allocation inside the call (skip node calloc, fail data malloc).
  g_oom_countdown = 1;
  pcl_status_t rc = pcl_executor_post_response_cb(e, cb, nullptr, &payload, sizeof(payload));
  g_oom_countdown = -1;
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pcl_executor_destroy(e);
  restore_logs();
}
