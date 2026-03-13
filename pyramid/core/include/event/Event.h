#pragma once

#include <event/BaseEvent.h>
#include <utility> // For std::forward/move

namespace pyramid {
namespace core {
  namespace event {

  /// \brief Templated event structure holding specific event data
  /// Inherits from BaseEvent to allow storage in heterogeneous containers.
  /// \tparam T The type of the data payload for this event
  template <typename T>
  struct Event : BaseEvent {
  /// \brief The actual data payload associated with the event
  T data;

  /// \brief Constructs an Event by copying the data payload
  /// \param event_data The data to copy
  explicit Event(const T& event_data) : BaseEvent(), data(event_data) {}

  /// \brief Constructs an Event by moving the data payload
  /// \param event_data The data to move
  explicit Event(T&& event_data) : BaseEvent(), data(std::move(event_data)) {}

  /// \brief Constructs an Event by forwarding arguments directly to the data payload's constructor
  /// This allows in-place construction of the data within the event.
  /// \tparam Args Types of arguments for T's constructor
  /// \param args Arguments to forward to T's constructor
  template <typename... Args>
  explicit Event(Args&&... args) : BaseEvent(), data(std::forward<Args>(args)...) {}
  
  // Default copy/move constructors/assignment operators are deleted because
  // BaseEvent deletes them. This is desirable as we typically handle events
  // via std::shared_ptr<BaseEvent>, avoiding slicing and managing lifetime explicitly.
  };

  } // namespace event
} // namespace core
} // namespace pyramid 


