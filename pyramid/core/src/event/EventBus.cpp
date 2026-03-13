#include <event/EventBus.h>
#include <event/Event.h>      // Required for dynamic_pointer_cast in dispatch
#include <vector>
#include <algorithm> // For std::remove_if

namespace pyramid {
namespace core {
namespace event {

  void EventBus::post(std::shared_ptr<BaseEvent> event_ptr) {
    if (!event_ptr) return; // Don't post null events

    std::lock_guard<std::mutex> lock(queue_mutex_);
    event_queue_.push(std::move(event_ptr)); 
  }

  bool EventBus::unsubscribe(SubscriptionHandle handle) {
    if (handle == INVALID_SUBSCRIPTION_HANDLE) {
      return false;
    }

    bool found = false;
    std::lock_guard<std::mutex> lock(listeners_mutex_);

    // Iterate through all event types in the listener map
    for (auto& pair : listeners_) {
      std::vector<ListenerInfo>& listeners = pair.second;
      
      // Use erase-remove idiom to find and remove the listener by handle
      auto original_size = listeners.size();
      listeners.erase(
        std::remove_if(listeners.begin(), listeners.end(),
                       [handle](const ListenerInfo& info) {
                         return info.handle == handle;
                       }),
        listeners.end());

      // Check if an element was actually removed
      if (listeners.size() < original_size) {
        found = true;
        // Assuming handles are unique, we can stop searching once found
        break; 
      }
    }

    return found;
  }

  void EventBus::dispatchPending() {
    // Create a temporary queue to hold events while dispatching.
    // This minimizes the time the main queue mutex is held.
    std::queue<std::shared_ptr<BaseEvent>> current_queue;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      // Swap the member queue with our local empty queue
      std::swap(event_queue_, current_queue);
    }

    // Now process the local queue without holding the queue mutex.
    while (!current_queue.empty()) {
      std::shared_ptr<BaseEvent> event_ptr = current_queue.front();
      current_queue.pop();

      if (!event_ptr) continue; // Skip null events if any got in

      // Determine the type of the event
      std::type_index event_type_index(typeid(*event_ptr)); 
      
      // Make a copy of the listeners for this event type *before* iterating.
      // This prevents issues if a listener tries to subscribe/unsubscribe during dispatch.
      std::vector<ListenerInfo> listeners_copy;
      {
        std::lock_guard<std::mutex> lock(listeners_mutex_);
        const auto it = listeners_.find(event_type_index);
        if (it != listeners_.end()) {
          listeners_copy = it->second; // Copy the vector of listeners
        }
      }
      
      // Call the listeners from the copied list
      // This loop does not require locking the listeners_mutex_
      for (const auto& listener_info : listeners_copy) {
        // The generic listener lambda handles the dynamic_cast internally
        listener_info.listener(event_ptr); 
      }
    }
  }

} // namespace event
} // namespace core
} // namespace pyramid


