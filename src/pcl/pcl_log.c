/// \file pcl_log.c
/// \brief PCL logging implementation.
#include "pcl/pcl_log.h"
#include "pcl/pcl_container.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ── Module state ────────────────────────────────────────────────────────

static pcl_log_handler_t g_handler   = NULL;
static void*             g_user_data = NULL;
static pcl_log_level_t   g_min_level = PCL_LOG_INFO;

// ── Default handler ─────────────────────────────────────────────────────

static const char* level_str(pcl_log_level_t level) {
  switch (level) {
    case PCL_LOG_DEBUG: return "DEBUG";
    case PCL_LOG_INFO:  return "INFO";
    case PCL_LOG_WARN:  return "WARN";
    case PCL_LOG_ERROR: return "ERROR";
    case PCL_LOG_FATAL: return "FATAL";
    default:            return "???";
  }
}

static void default_handler(pcl_log_level_t level,
                            const char*     container_name,
                            const char*     message,
                            void*           user_data) {
  (void)user_data;
  if (container_name && container_name[0] != '\0') {
    fprintf(stderr, "[%s] [%s] %s\n", level_str(level), container_name, message);
  } else {
    fprintf(stderr, "[%s] %s\n", level_str(level), message);
  }
}

// ── Public API ──────────────────────────────────────────────────────────

void pcl_log_set_handler(pcl_log_handler_t handler, void* user_data) {
  g_handler   = handler;
  g_user_data = user_data;
}

void pcl_log_set_level(pcl_log_level_t min_level) {
  g_min_level = min_level;
}

void pcl_log(const pcl_container_t* c,
             pcl_log_level_t        level,
             const char*            fmt, ...) {
  char buf[1024];
  const char* name;
  pcl_log_handler_t handler;
  va_list args;

  if (level < g_min_level) return;

  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  name = c ? pcl_container_name(c) : NULL;
  handler = g_handler ? g_handler : default_handler;
  handler(level, name, buf, g_user_data);
}
