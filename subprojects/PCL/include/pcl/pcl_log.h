/// \file pcl_log.h
/// \brief PYRAMID Container Library logging API.
///
/// Provides a printf-style logging function with pluggable handler.
/// Default handler writes to stderr.  A ROS2 adapter can redirect to
/// RCLCPP_INFO/WARN/etc. by installing a custom handler.
#ifndef PCL_LOG_H
#define PCL_LOG_H

#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Log handler callback signature.
/// \param level           Log severity.
/// \param container_name  Name of the container that logged (may be NULL
///                        for executor-level messages).
/// \param message         Formatted message string (null-terminated).
/// \param user_data       Opaque context set via pcl_log_set_handler().
typedef void (*pcl_log_handler_t)(pcl_log_level_t level,
                                  const char*     container_name,
                                  const char*     message,
                                  void*           user_data);

/// \brief Install a custom log handler.
///
/// Pass NULL to revert to the default stderr handler.
/// Not thread-safe -- call before pcl_executor_spin().
void pcl_log_set_handler(pcl_log_handler_t handler, void* user_data);

/// \brief Set the minimum log level.
///
/// Messages below this level are discarded.  Default: PCL_LOG_INFO.
void pcl_log_set_level(pcl_log_level_t min_level);

/// \brief Emit a log message, printf-style.
/// \param c      Container context (may be NULL for executor-level logs).
/// \param level  Severity level.
/// \param fmt    printf format string.
void pcl_log(const pcl_container_t* c,
             pcl_log_level_t        level,
             const char*            fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // PCL_LOG_H
