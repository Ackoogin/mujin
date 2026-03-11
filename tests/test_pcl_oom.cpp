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

// ── Fail allocation #0: calloc for pcl_pending_msg_t ─────────────────

TEST(PclOom, PostIncomingPendingAllocFails) {
  silence_logs();
  auto* e = pcl_executor_create();

  pcl_msg_t msg = {};
  msg.type_name = "T";

  g_oom_countdown = 0; // fail the very next alloc (calloc for pending_msg)
  EXPECT_EQ(pcl_executor_post_incoming(e, "t", &msg), PCL_ERR_NOMEM);
  g_oom_countdown = -1;

  pcl_executor_destroy(e);
  restore_logs();
}

// ── Fail allocation #1: topic strdup → line 429 ──────────────────────

TEST(PclOom, PostIncomingTopicStrdupFails) {
  silence_logs();
  auto* e = pcl_executor_create();

  pcl_msg_t msg = {};
  msg.type_name = "T";

  // Let calloc (#0) succeed; fail malloc #1 (topic strdup)
  g_oom_countdown = 1;
  EXPECT_EQ(pcl_executor_post_incoming(e, "t", &msg), PCL_ERR_NOMEM);
  g_oom_countdown = -1;

  pcl_executor_destroy(e);
  restore_logs();
}

// ── Fail allocation #2: type_name strdup → line 429 ─────────────────

TEST(PclOom, PostIncomingTypeNameStrdupFails) {
  silence_logs();
  auto* e = pcl_executor_create();

  pcl_msg_t msg = {};
  msg.type_name = "T";

  // Let calloc (#0) and topic strdup (#1) succeed; fail #2 (type_name strdup)
  g_oom_countdown = 2;
  EXPECT_EQ(pcl_executor_post_incoming(e, "t", &msg), PCL_ERR_NOMEM);
  g_oom_countdown = -1;

  pcl_executor_destroy(e);
  restore_logs();
}

// ── Fail allocation #3: data malloc → lines 435-436 ─────────────────

TEST(PclOom, PostIncomingDataMallocFails) {
  silence_logs();
  auto* e = pcl_executor_create();

  int payload = 42;
  pcl_msg_t msg = {};
  msg.type_name = "T";
  msg.data = &payload;
  msg.size = sizeof(payload);

  // Let calloc (#0), topic strdup (#1), type_name strdup (#2) succeed;
  // fail #3 (data malloc — lines 435-436)
  g_oom_countdown = 3;
  EXPECT_EQ(pcl_executor_post_incoming(e, "t", &msg), PCL_ERR_NOMEM);
  g_oom_countdown = -1;

  pcl_executor_destroy(e);
  restore_logs();
}
