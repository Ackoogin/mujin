#pragma once

#ifdef _WIN32
#define NOMINMAX // Prevent Windows.h from defining min/max macros
#endif

#include <event/BaseEvent.h>
#include <event/Event.h>
#include <memory>         // For std::shared_ptr
#include <functional>     // For std::function
#include <vector>
#include <unordered_map>
#include <typeindex>      // For std::type_index
#include <queue>
#include <mutex>          // For std::mutex, std::lock_guard
#include <atomic>         // For std::atomic_size_t
#include <limits>

namespace pyramid {
namespace core {
  namespace event {

  using SubscriptionHandle = size_t;
  const SubscriptionHandle INVALID_SUBSCRIPTION_HANDLE = std::numeric_limits<SubscriptionHandle>::max();

  /// \brief
  /// Manages event posting, subscription, and dispatching.
  /// Supports arbitrary event data types via template-based type erasure.
  /// Provides thread-safe operations for posting, subscribing, unsubscribing,
  /// and dispatching events.
  
  class EventBus {
  public:
  EventBus() = default;
  ~EventBus() = default;

  // Prevent copying and moving
  EventBus(const EventBus&) = delete;
  EventBus& operator=(const EventBus&) = delete;
  EventBus(EventBus&&) = delete;
  EventBus& operator=(EventBus&&) = delete;

  /// \brief
  /// Posts an event to the queue for later dispatch.
  /// This operation is thread-safe.
  
  /// \param event_ptrA shared pointer to the event object (must derive from BaseEvent).</param>
  void post(std::shared_ptr<BaseEvent> event_ptr);

  /// \brief
  /// Subscribes a listener function to a specific event data type.
  /// This operation is thread-safe.
  
  /// \tparam TThe data type of the event to listen for.</typeparam>
  /// \param listenerA function (or lambda) taking a 'const T&' as input.</param>
  /// \returnA handle for the subscription, used for unsubscribing. Returns INVALID_SUBSCRIPTION_HANDLE on failure.</returns>
  template <typename T>
  SubscriptionHandle subscribe(std::function<void(const T&)> listener);

  /// \brief
  /// Unsubscribes a listener using its subscription handle.
  /// This operation is thread-safe.
  
  /// \param handleThe handle returned by the subscribe method.</param>
  /// \returnTrue if the listener was found and unsubscribed, false otherwise.</returns>
  bool unsubscribe(SubscriptionHandle handle);

  /// \brief
  /// Dispatches all pending events in the queue to their respective subscribers.
  /// Events are dispatched in the order they were posted (FIFO).
  /// This operation is thread-safe.
  
  void dispatchPending();

  private:
  // Listener function wrapper that accepts BaseEvent shared_ptr
  using GenericListener = std::function<void(const std::shared_ptr<BaseEvent>&)>;
  struct ListenerInfo {
    GenericListener listener;
    SubscriptionHandle handle;
  };
  // Map from event type_index to a list of listeners for that type
  std::unordered_map<std::type_index, std::vector<ListenerInfo>> listeners_;
  std::mutex listeners_mutex_;

  // Queue for pending events
  std::queue<std::shared_ptr<BaseEvent>> event_queue_;
  std::mutex queue_mutex_;
  
  // Counter for generating unique subscription handles
  std::atomic_size_t next_handle_{0};
  };

  // --- Template Implementation --- 

  template <typename T>
  SubscriptionHandle EventBus::subscribe(std::function<void(const T&)> listener) {
  if (!listener) {
  return INVALID_SUBSCRIPTION_HANDLE; // Cannot subscribe a null listener
  }

  // Generate a unique handle for this subscription
  // Use fetch_add for atomic increment and get previous value
  // Check for potential overflow, although highly unlikely with size_t
  SubscriptionHandle handle = next_handle_.fetch_add(1);
  if (handle == INVALID_SUBSCRIPTION_HANDLE) {
   // Very unlikely overflow case or initial state confusion
   // Try incrementing again if possible or handle error
   // For now, just return invalid. A real implementation might loop or throw.
   return INVALID_SUBSCRIPTION_HANDLE;
  }

  // Create a wrapper lambda that performs the dynamic cast
  GenericListener generic_listener = 
  [listener](const std::shared_ptr<BaseEvent>& base_event_ptr) {
    // Attempt to cast the BaseEvent pointer to the specific Event<T> pointer
    if (auto specific_event_ptr = std::dynamic_pointer_cast<const Event<T>>(base_event_ptr)) {
    // If cast is successful, call the original listener with the data
    listener(specific_event_ptr->data);
    }
    // If cast fails, it means the event was not of type Event<T>,
    // so we simply don't call the listener.
  };

  std::type_index event_type_index(typeid(Event<T>));

  // Lock the listener map before modifying it
  std::lock_guard<std::mutex> lock(listeners_mutex_);
  listeners_[event_type_index].push_back({generic_listener, handle});

  return handle;
  }

  } // namespace event
} // namespace core
} // namespace pyramid 


