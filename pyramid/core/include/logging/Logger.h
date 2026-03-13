#pragma once

#include <string>
#include <vector>
#include <chrono> // For timestamps
#include <memory> // For std::shared_ptr if needed later
#include <mutex>  // For std::mutex
#include <functional> // For std::function
#include <cstdint> // For uint64_t
#include <atomic> // For std::atomic
#include <map> // For listener storage

namespace pyramid {
namespace core {
namespace logging {

/// Defines the severity level of a log message.
enum class LogLevel {
  Debug,
  Warning,  
  Info,
  Error
};

/// /// Represents a single log entry.
///
struct LogEntry {
  LogLevel level;
  std::chrono::system_clock::time_point timestamp;
  std::string message;
};

/// /// Type alias for a log listener callback function.
///
using LogListener = std::function<void(const LogEntry&)>;

/// /// Unique identifier for a subscribed listener.
///
using ListenerHandle = uint64_t;

/// /// The main logging class.
/// Provides functionality to log messages with different severity levels.
/// Allows subscribing to receive live log updates.
///
class Logger {
public:
  /// /// Default constructor.
  ///
  Logger();

  /// /// Default destructor.
  ///
  ~Logger();

  // Prevent copying and assignment
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;
  Logger(Logger&&) = delete;
  Logger& operator=(Logger&&) = delete;

  /// /// Logs a message with a specific severity level.
  /// \param level The severity level of the message.
  /// \param message The message content to log.
  ///
  virtual void log(LogLevel level, const std::string& message);

  /// /// Retrieves a copy of the entire log history in a thread-safe manner.
  /// \return A vector containing copies of log entries.
  ///
  std::vector<LogEntry> getHistory() const;

  /// /// Subscribes a listener to receive log entry notifications.
  /// \param listener The callback function to be invoked for each log entry.
  /// \return A unique handle for the subscription, used for unsubscribing.
  ///
  ListenerHandle subscribe(LogListener listener);

  /// /// Unsubscribes a listener using its handle.
  /// \param handle The unique handle returned by the subscribe method.
  /// \return True if the listener was found and unsubscribed, false otherwise.
  ///
  bool unsubscribe(ListenerHandle handle);

  // Add other methods later

private:
  std::vector<LogEntry> log_history_; // Storage for log entries
  mutable std::mutex history_mutex_; // Mutex to protect log_history_

  std::map<ListenerHandle, LogListener> listeners_; // Storage for listeners
  std::atomic<ListenerHandle> next_listener_handle_{0}; // Counter for unique handles
  mutable std::mutex listeners_mutex_; // Mutex to protect listeners_
  // Add other internal state later
};

} // namespace logging
} // namespace core
} // namespace pyramid



