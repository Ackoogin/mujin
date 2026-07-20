// Auto-generated component skeleton support. Regenerated on every run.
#pragma once

#include <cassert>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace pyramid::component_skeleton {

// Resolves a port's wire content type by port name. The skeleton
// calls it once per port when constructing the port facades, so a
// process can encode each port with its own codec. An empty
// resolver means "application/json for every port".
using ContentTypeResolver =
    std::function<std::string(const std::string&)>;

inline std::string resolveContentType(
    const ContentTypeResolver& codec_for, const char* port) {
  return codec_for ? codec_for(port) : std::string("application/json");
}

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
