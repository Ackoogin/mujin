/// \file spin_thread.hpp
/// \brief RAII helper that drives an Executor on a background thread.
///
/// Replaces the recurring "std::thread + atomic stop-flag + spin_once + yield"
/// pattern used by examples and tests.
#ifndef PCL_SPIN_THREAD_HPP
#define PCL_SPIN_THREAD_HPP

#include "pcl/executor.hpp"

#include <atomic>
#include <chrono>
#include <thread>

namespace pcl {

/// \brief Run Executor::spinOnce on a dedicated thread until destruction.
///
/// The destructor signals stop and joins the thread; spin_once uses the
/// supplied tick budget. Construct after the executor has been populated
/// with containers; destroy before the executor itself goes out of scope.
class SpinThread {
public:
  explicit SpinThread(Executor& executor,
                      std::chrono::milliseconds tick = std::chrono::milliseconds(10))
      : executor_(&executor),
        tick_ms_(static_cast<uint32_t>(tick.count())),
        thread_([this] { run(); }) {}

  ~SpinThread() { stop(); }

  SpinThread(const SpinThread&) = delete;
  SpinThread& operator=(const SpinThread&) = delete;
  SpinThread(SpinThread&&) = delete;
  SpinThread& operator=(SpinThread&&) = delete;

  /// \brief Request shutdown and join the thread. Idempotent.
  void stop() {
    if (stop_.exchange(true, std::memory_order_acq_rel)) return;
    if (thread_.joinable()) thread_.join();
  }

private:
  void run() {
    while (!stop_.load(std::memory_order_acquire)) {
      executor_->spinOnce(tick_ms_);
      std::this_thread::yield();
    }
  }

  Executor* executor_ = nullptr;
  uint32_t  tick_ms_ = 10;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace pcl

#endif  // PCL_SPIN_THREAD_HPP
