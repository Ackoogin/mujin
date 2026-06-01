#pragma once

#include <atomic>
#include <memory>

namespace ame::neuro {

class CancelSource;

// C++17-compatible cooperative cancellation token.
// Cooperation is expected but not required: the Advisor always bounds wall-time
// via future::wait_for() regardless of whether the backend honours cancel.
class CancelToken {
public:
    CancelToken() = default;
    bool cancelled() const noexcept {
        return flag_ && flag_->load(std::memory_order_relaxed);
    }
private:
    friend class CancelSource;
    std::shared_ptr<std::atomic<bool>> flag_;
};

class CancelSource {
public:
    CancelSource()
        : flag_(std::make_shared<std::atomic<bool>>(false)) {}

    CancelToken token() const {
        CancelToken tok;
        tok.flag_ = flag_;
        return tok;
    }

    void request_cancel() noexcept {
        flag_->store(true, std::memory_order_relaxed);
    }

private:
    std::shared_ptr<std::atomic<bool>> flag_;
};

} // namespace ame::neuro
