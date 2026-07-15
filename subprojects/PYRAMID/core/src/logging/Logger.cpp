#include <logging/Logger.h>

#include <pcl/pcl_log.h>

#include <exception>
#include <mutex>
#include <vector>

namespace {

pcl_log_level_t toPclLogLevel(pyramid::core::logging::LogLevel level) {
  using pyramid::core::logging::LogLevel;

  switch (level) {
    case LogLevel::Debug:
      return PCL_LOG_DEBUG;
    case LogLevel::Info:
      return PCL_LOG_INFO;
    case LogLevel::Warning:
      return PCL_LOG_WARN;
    case LogLevel::Error:
      return PCL_LOG_ERROR;
  }

  return PCL_LOG_ERROR;
}

} // namespace

namespace pyramid {
namespace core {
namespace logging {

  Logger::Logger() {
    // Constructor implementation (if needed)
  }

  Logger::~Logger() {
    // Destructor implementation (if needed)
  }

  void Logger::log(LogLevel level, const std::string& message) {
    // 1. Create the entry
    LogEntry entry;
    entry.level = level;
    entry.message = message;
    entry.timestamp = std::chrono::system_clock::now();

    {
      std::lock_guard<std::mutex> lock(history_mutex_); // Lock the history mutex
      log_history_.push_back(entry);
    } // history_mutex_ released here

    pcl_log(nullptr, toPclLogLevel(level), "%s", message.c_str());

    // 3. Notify listeners (under listeners lock, but call outside lock)
    // Create a copy of listeners to avoid holding the lock while calling callbacks
    // This prevents deadlocks if a listener calls back into the logger.
    std::vector<LogListener> listeners_to_call;
    {
      std::lock_guard<std::mutex> lock(listeners_mutex_);
      listeners_to_call.reserve(listeners_.size());
      for(const auto& pair : listeners_) {
        listeners_to_call.push_back(pair.second);
      }
    } // listeners_mutex_ released here

    // Call listeners outside the lock
    for(const auto& listener : listeners_to_call) {
      try {
        listener(entry); // Call listener with the log entry
      } catch (const std::exception& e) {
        // Route listener failures through the same central logging sink.
        pcl_log(nullptr, PCL_LOG_ERROR,
                "Exception caught in log listener: %s", e.what());
      } catch (...) {
        pcl_log(nullptr, PCL_LOG_ERROR,
                "Unknown exception caught in log listener.");
      }
    }
  }

  std::vector<LogEntry> Logger::getHistory() const {
    std::lock_guard<std::mutex> lock(history_mutex_); // Lock the mutex
    // Return a copy of the history while the lock is held
    return log_history_;
  }

  ListenerHandle Logger::subscribe(LogListener listener) {
    std::lock_guard<std::mutex> lock(listeners_mutex_); // Lock listener access
    ListenerHandle handle = next_listener_handle_++; // Generate unique handle
    listeners_[handle] = listener; // Store listener
    return handle;
  }

  bool Logger::unsubscribe(ListenerHandle handle) {
    std::lock_guard<std::mutex> lock(listeners_mutex_); // Lock listener access
    size_t erased_count = listeners_.erase(handle); // Attempt to erase by handle
    return erased_count > 0;
  }

  // Implement other Logger methods here

} // namespace logging
} // namespace core
} // namespace pyramid


