# PYRAMID Container Library (PCL) Design 

## Overview

The PYRAMID Container Library (PCL) is a standard, deployable component "container" designed for autonomous mission systems. Its primary objective is to encapsulate core business logic behind a consistent lifecycle while entirely decoupling the logic from external middleware (such as ROS2, DDS, or sockets). 

PCL allows components to be portable, reusable, and deterministic, ensuring that the exact same autonomy logic can be run in a highly resource-constrained embedded environment (e.g., bare-metal C/Ada), a high-fidelity desktop simulation, or a distributed system over a network.

## Core Architectural Principles

1. **Zero External Dependencies**: The core library (`pcl_core`) is written in strict C17/C89 with zero external dependencies—not even the Standard Template Library (STL).
2. **Deterministic, Single-Threaded Execution**: Each container has its own tick loop and state machine. The `pcl_executor` dispatches all incoming messages, service requests, and timer ticks sequentially on a single thread. This eliminates the need for complex internal mutexes and synchronization within the component's business logic.
3. **Pluggable Transports**: I/O is treated as a separate, pluggable concern. PCL provides an intra-process default transport out-of-the-box, but external systems (like a ROS2 node) can inject a custom `pcl_transport_t` adapter to bridge PCL ports to a distributed network.
4. **C ABI with C++ Ergonomics**: To guarantee maximum portability (including integration with Ada and Rust), the ABI boundary is purely C. However, a lightweight, header-only C++ wrapper (`pcl::Component` and `pcl::Executor`) is provided to offer RAII semantics, virtual method overrides, and modern C++ ergonomics without compromising the ABI.

## Component Lifecycle

PCL enforces a strict state machine based on the ROS2 lifecycle model, ensuring safe initialization and shutdown sequences.

*   `UNCONFIGURED`: The initial state. The container allocates memory but does not hold active resources.
*   `CONFIGURED`: The state where parameters are checked, and input/output ports (publishers, subscribers, services) are created. Ports *must* be created in this state to guarantee deterministic memory layouts.
*   `ACTIVE`: The operational state. The container's `on_tick()` method is called at a configured rate (e.g., 100 Hz), and it is allowed to publish messages.
*   `FINALIZED`: The terminal state indicating the component has been gracefully shut down and its resources can safely be freed by the host.

## Executor and Transport

### The Executor (`pcl_executor_t`)
The executor is responsible for driving one or more containers. It provides:
*   **Rate Limiting**: Ensures that each container's `on_tick()` is called according to its specific target Hz.
*   **Sequential Dispatch**: Guarantees that callbacks (`on_tick`, message reception, service handling) never execute concurrently for the same container.
*   **Graceful Shutdown**: Provides a timeout-based sequenced shutdown mapping (`ACTIVE` -> `CONFIGURED` -> `FINALIZED`) for all managed containers.
*   **Intra-Process Direct Dispatch**: If no external transport wrapper is provided, the executor handles zero-copy pointer handoffs directly between publishers and subscribers that reside within the same executor memory space.
*   **Queued Cross-Thread Ingress**: External I/O threads call `pcl_executor_post_incoming()` to enqueue work. The executor deep-copies the topic, type name, and payload bytes, then drains that queue on the executor thread during `spin()` / `spin_once()`.

### The Transport Adapter (`pcl_transport_t`)
An interface containing function pointers (`publish`, `serve`, `subscribe`, `shutdown`) that acts as a bridge. For instance, a ROS2 bridge application would initialize a PCL Executor and pass in a transport struct containing pointers to functions that ultimately call `rclcpp::Publisher::publish`.

### Threading and I/O Boundary
PCL draws a hard line between **business logic** and **transport I/O**:

*   All user callbacks (`on_configure`, `on_activate`, subscriber callbacks, service handlers, `on_tick`) run only on the executor thread.
*   External threads must **never** call component callbacks directly.
*   External threads may receive data from gRPC, DDS, ROS2, sockets, or device drivers, but they must hand that data to the executor via `pcl_executor_post_incoming()`.
*   `pcl_executor_post_incoming()` copies the message into an internal ingress queue, so producer threads can safely release or reuse their buffers immediately after posting.
*   The executor drains queued ingress before ticking containers, which creates a deterministic "apply external updates, then run logic" boundary.

This means a gRPC server thread does network work, deserializes enough to classify the message, and posts it into the executor queue. The container itself still processes the request on the single executor thread.

## Serialization and Service Patterns

PCL is designed to handle fully serialized byte buffers or raw struct pointers. The `pcl_msg_t` struct encapsulates a generalized type containing:
*   `data`: The payload pointer (which the container borrows).
*   `size`: The length of the payload.
*   `type_name`: A canonical string (e.g., `"WorldState"`) used by the transport adapter to determine how to deserialize the payload from the wire.

For same-thread direct dispatch, `pcl_msg_t` is borrowed for the duration of the call. For cross-thread ingress, `pcl_executor_post_incoming()` performs a deep copy before returning.

PCL components enforce the architectural service patterns defined by the system, handling distributed state mutations via asynchronous CRUD (Create, Read, Update, Delete) envelopes and requirements streaming.

### Ingress Sequence

```text
external I/O thread
  -> receive / deserialize / classify
  -> pcl_executor_post_incoming(executor, topic, msg)
  -> return to middleware immediately

executor thread
  -> drain ingress queue
  -> dispatch subscriber callback / service handler
  -> run on_tick()
```

For long-latency outbound calls such as gRPC clients, the recommended pattern is the inverse: the executor creates the request, an adapter-owned I/O thread waits for the network completion, and the completion is posted back into the executor as a new ingress event.

## Example Integration (C++)

```cpp
#include "pcl/component.hpp"
#include "pcl/executor.hpp"

class MySensor : public pcl::Component {
public:
    MySensor() : Component("my_sensor") {}

protected:
    pcl_status_t on_configure() override {
        // Read parameters
        double threshold = paramF64("alert_threshold", 50.0);
        
        // Define ports
        pub_alert_ = addPublisher("alerts", "AlertMsg");
        return PCL_OK;
    }

    pcl_status_t on_activate() override {
        setTickRateHz(10.0); // Run at 10 Hz
        return PCL_OK;
    }

    pcl_status_t on_tick(double dt) override {
        logInfo("Sensor ticking... dt=%f", dt);
        // ... business logic ...
        return PCL_OK;
    }

private:
    pcl_port_t* pub_alert_ = nullptr;
};

int main() {
    MySensor sensor;
    pcl::Executor exec;
    
    sensor.configure();
    sensor.activate();
    
    exec.add(sensor);
    exec.spin(); // Blocks and runs the tick loop
    
    return 0;
}
```

See also:

*   `examples/external_io_bridge_example.c` for a concrete producer-thread -> executor-queue -> subscriber flow.
*   `tests/test_pcl_executor.cpp` for a test that proves the external thread can reuse its source buffer immediately after posting.
*   `examples/ada/pcl_bindings.ads` and `examples/ada/pcl_sensor_demo.adb` for an Ada-facing wrapper and example over the same C ABI.
