/// \file await.hpp
/// \brief Helpers for blocking on std::future while keeping an Executor
/// spinning.
///
/// Bindings expose async APIs returning std::future. Callers that don't have
/// their own spin thread can use pcl::await to drive a target executor until
/// the future resolves (or a timeout expires).
#ifndef PCL_AWAIT_HPP
#define PCL_AWAIT_HPP

#include "pcl/executor.hpp"

#include <chrono>
#include <future>
#include <optional>

namespace pcl {

/// \brief Spin \p executor in 10 ms slices until \p future is ready.
///
/// Returns true when the future resolves before the deadline. Returns false on
/// timeout; the future remains valid in that case.
template <class T>
bool await(Executor& executor,
           std::future<T>& future,
           std::chrono::milliseconds timeout =
               std::chrono::milliseconds(5000),
           std::chrono::milliseconds tick = std::chrono::milliseconds(10)) {
  if (!future.valid()) return false;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const auto tick_ms = static_cast<uint32_t>(tick.count());
  while (future.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    executor.spinOnce(tick_ms);
  }
  return true;
}

/// \brief Block on the future and return its value, spinning \p executor.
///
/// Returns std::nullopt on timeout.
template <class T>
std::optional<T> awaitValue(Executor& executor,
                            std::future<T> future,
                            std::chrono::milliseconds timeout =
                                std::chrono::milliseconds(5000)) {
  if (!await(executor, future, timeout)) return std::nullopt;
  return future.get();
}

}  // namespace pcl

#endif  // PCL_AWAIT_HPP
