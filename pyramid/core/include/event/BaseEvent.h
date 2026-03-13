#pragma once

#include <memory> // For std::shared_ptr

namespace pyramid {
namespace core {
  namespace event {

  /// \brief Base structure for all events managed by the EventBus
  /// Contains type information for safe casting implicitly via dynamic_cast.
  struct BaseEvent {
  /// \brief Virtual destructor to ensure proper cleanup of derived event types
  /// and enable dynamic_cast
  virtual ~BaseEvent() = default;

  // Prevent copying/moving base event slices
  BaseEvent(const BaseEvent&) = delete;
  BaseEvent& operator=(const BaseEvent&) = delete;
  BaseEvent(BaseEvent&&) = delete;
  BaseEvent& operator=(BaseEvent&&) = delete;

  protected:
  // Protected constructor so only derived classes can be instantiated
  BaseEvent() = default;
  };

  } // namespace event
} // namespace core
} // namespace pyramid 


