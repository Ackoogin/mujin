/// \file test_pcl_log.cpp
/// \brief Tests for PCL logging — default handler, custom handler, level filtering.
#include <gtest/gtest.h>

#include <string>
#include <vector>

extern "C" {
#include "pcl/pcl_log.h"
#include "pcl/pcl_container.h"
}

// ── Test helpers ────────────────────────────────────────────────────────

struct LogEntry {
  pcl_log_level_t level;
  std::string container_name;
  std::string message;
};

static std::vector<LogEntry> g_captured;

static void capture_handler(pcl_log_level_t level,
                            const char*     container_name,
                            const char*     message,
                            void*           user_data) {
  (void)user_data;
  LogEntry entry;
  entry.level = level;
  entry.container_name = container_name ? container_name : "";
  entry.message = message ? message : "";
  g_captured.push_back(entry);
}

class PclLogTest : public ::testing::Test {
protected:
  void SetUp() override {
    g_captured.clear();
    pcl_log_set_handler(capture_handler, nullptr);
    pcl_log_set_level(PCL_LOG_DEBUG); // capture everything
  }

  void TearDown() override {
    pcl_log_set_handler(nullptr, nullptr); // revert to default
    pcl_log_set_level(PCL_LOG_INFO);
  }
};

// ── Tests ───────────────────────────────────────────────────────────────

TEST_F(PclLogTest, CustomHandlerReceivesMessages) {
  pcl_log(nullptr, PCL_LOG_INFO, "hello %s", "world");

  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].level, PCL_LOG_INFO);
  EXPECT_EQ(g_captured[0].message, "hello world");
  EXPECT_EQ(g_captured[0].container_name, "");
}

TEST_F(PclLogTest, ContainerNamePassedThrough) {
  auto* c = pcl_container_create("mynode", nullptr, nullptr);
  pcl_log(c, PCL_LOG_WARN, "test msg");

  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].container_name, "mynode");
  EXPECT_EQ(g_captured[0].level, PCL_LOG_WARN);

  pcl_container_destroy(c);
}

TEST_F(PclLogTest, LevelFiltering) {
  pcl_log_set_level(PCL_LOG_WARN);

  pcl_log(nullptr, PCL_LOG_DEBUG, "debug");
  pcl_log(nullptr, PCL_LOG_INFO,  "info");
  pcl_log(nullptr, PCL_LOG_WARN,  "warn");
  pcl_log(nullptr, PCL_LOG_ERROR, "error");

  // only WARN and above should be captured
  ASSERT_EQ(g_captured.size(), 2u);
  EXPECT_EQ(g_captured[0].level, PCL_LOG_WARN);
  EXPECT_EQ(g_captured[1].level, PCL_LOG_ERROR);
}

TEST_F(PclLogTest, FormatString) {
  pcl_log(nullptr, PCL_LOG_INFO, "count=%d pi=%.1f", 42, 3.14);

  ASSERT_EQ(g_captured.size(), 1u);
  EXPECT_EQ(g_captured[0].message, "count=42 pi=3.1");
}

TEST_F(PclLogTest, RevertToDefaultHandler) {
  pcl_log_set_handler(nullptr, nullptr);

  // should not crash — writes to stderr
  pcl_log(nullptr, PCL_LOG_INFO, "default handler test");
}
