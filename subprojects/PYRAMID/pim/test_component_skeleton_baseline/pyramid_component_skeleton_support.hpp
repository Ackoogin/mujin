// Auto-generated component skeleton support. Regenerated on every run.
#pragma once

#include <cassert>
#include <memory>
#include <utility>

namespace pyramid::component_skeleton {

template <typename T>
class HandlerSlot {
public:
  HandlerSlot(T& handler) : ptr_(&handler) {}

  HandlerSlot(std::unique_ptr<T> handler)
      : ptr_(handler.get()), owned_(std::move(handler)) {
    assert(ptr_ != nullptr);
  }

  T& get() {
    assert(ptr_ != nullptr);
    return *ptr_;
  }

private:
  T* ptr_;
  std::unique_ptr<T> owned_;
};

}  // namespace pyramid::component_skeleton
