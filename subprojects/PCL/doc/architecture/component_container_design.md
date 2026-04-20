# PCL Component Container — Developer Guide

## 1. Overview

The PYRAMID Container Library (PCL) provides a standard component container for autonomous mission systems. It encapsulates business logic behind a consistent lifecycle while entirely decoupling it from external middleware.

PCL uses a **hybrid architecture**: a pure-C ABI core for maximum portability (Ada, C, C++, Rust), with an optional header-only C++ wrapper for ergonomic authoring. Transport adapters (ROS2, DDS, sockets, shared memory) are pluggable — your component code never depends on middleware.

---

## 2. Design Principles

| # | Principle | Rationale |
|---|-----------|-----------|
| P1 | **Logic owns the thread** | Business logic runs on exactly one thread (the executor). No internal mutexes needed. |
| P2 | **I/O is injected** | Transport adapters (ROS2, DDS, sockets, shared-mem) are set via function pointers / vtable, not compiled in. |
| P3 | **C-ABI at the boundary** | All public symbols are `extern "C"` with opaque handles. C++ internals are hidden behind the ABI wall. |
| P4 | **Lifecycle is explicit** | Components follow a state machine: `UNCONFIGURED → CONFIGURED → ACTIVE → FINALIZED`. Compatible with ROS2 lifecycle but no rclcpp dependency. |
| P5 | **Zero-copy where possible** | Intra-process communication between containers in the same executor uses pointer handoff, not serialization. |

---

## 3. Architecture

```
┌---------------------------------------------------------┐
│                    Executor (single thread)              │
│                                                         │
│  ┌--------------┐  ┌--------------┐  ┌--------------┐  │
│  │  Container A  │  │  Container B  │  │  Container C  │  │
│  │ ┌----------┐ │  │ ┌----------┐ │  │ ┌----------┐ │  │
│  │ │  Logic   │ │  │ │  Logic   │ │  │ │  Logic   │ │  │
│  │ │ (user)   │ │  │ │ (user)   │ │  │ │ (user)   │ │  │
│  │ └----┬-----┘ │  │ └----┬-----┘ │  │ └----┬-----┘ │  │
│  │      │       │  │      │       │  │      │       │  │
│  │ ┌----▼-----┐ │  │ ┌----▼-----┐ │  │ ┌----▼-----┐ │  │
│  │ │ Port I/O │ │  │ │ Port I/O │ │  │ │ Port I/O │ │  │
│  │ └----------┘ │  │ └----------┘ │  │ └----------┘ │  │
│  └--------------┘  └--------------┘  └--------------┘  │
│                                                         │
│  ┌-------------------------------------------------┐    │
│  │         Transport Adapter (pluggable)            │    │
│  │   ROS2 / DDS / SharedMem / Socket / gRPC        │    │
│  └-------------------------------------------------┘    │
└---------------------------------------------------------┘
```

The library itself (`pcl_core`) is pure C17 with zero dependencies. A header-only C++ wrapper (`pcl/component.hpp`) provides RAII, virtual method overrides, and modern C++ ergonomics without compromising the ABI. Ada and C consumers use the raw C API directly.

---

## 4. Core C API

### 4.1 Opaque Handles

```c
/* pcl_container.h */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handles */
typedef struct pcl_executor_t  pcl_executor_t;
typedef struct pcl_container_t pcl_container_t;
typedef struct pcl_port_t      pcl_port_t;

/* Return codes */
typedef enum {
    PCL_OK             = 0,
    PCL_ERR_INVALID    = -1,
    PCL_ERR_STATE      = -2,   /* wrong lifecycle state for this operation */
    PCL_ERR_TIMEOUT    = -3,
    PCL_ERR_CALLBACK   = -4,   /* user callback returned error */
    PCL_ERR_NOMEM      = -5,
} pcl_status_t;

/* Lifecycle states */
typedef enum {
    PCL_STATE_UNCONFIGURED = 0,
    PCL_STATE_CONFIGURED   = 1,
    PCL_STATE_ACTIVE       = 2,
    PCL_STATE_FINALIZED    = 3,
} pcl_state_t;
```

### 4.2 Lifecycle Callbacks (User Implements)

```c
/* User-provided callbacks — all called on the executor thread */
typedef struct {
    pcl_status_t (*on_configure)(pcl_container_t* self, void* user_data);
    pcl_status_t (*on_activate)(pcl_container_t* self, void* user_data);
    pcl_status_t (*on_deactivate)(pcl_container_t* self, void* user_data);
    pcl_status_t (*on_cleanup)(pcl_container_t* self, void* user_data);
    pcl_status_t (*on_shutdown)(pcl_container_t* self, void* user_data);

    /* Periodic tick — called at configured rate while ACTIVE */
    pcl_status_t (*on_tick)(pcl_container_t* self, double dt_seconds,
                            void* user_data);
} pcl_callbacks_t;
```

### 4.3 Container Lifecycle

```c
/* Create / destroy */
pcl_container_t* pcl_container_create(const char* name,
                                       const pcl_callbacks_t* callbacks,
                                       void* user_data);
void pcl_container_destroy(pcl_container_t* c);

/* Lifecycle transitions */
pcl_status_t pcl_container_configure(pcl_container_t* c);
pcl_status_t pcl_container_activate(pcl_container_t* c);
pcl_status_t pcl_container_deactivate(pcl_container_t* c);
pcl_status_t pcl_container_cleanup(pcl_container_t* c);
pcl_status_t pcl_container_shutdown(pcl_container_t* c);

pcl_state_t  pcl_container_state(const pcl_container_t* c);
const char*  pcl_container_name(const pcl_container_t* c);

/* Parameters (key-value config) */
pcl_status_t pcl_container_set_param_str(pcl_container_t* c,
                                          const char* key, const char* value);
pcl_status_t pcl_container_set_param_f64(pcl_container_t* c,
                                          const char* key, double value);
pcl_status_t pcl_container_set_param_i64(pcl_container_t* c,
                                          const char* key, int64_t value);
pcl_status_t pcl_container_set_param_bool(pcl_container_t* c,
                                           const char* key, bool value);

const char*  pcl_container_get_param_str(const pcl_container_t* c,
                                          const char* key,
                                          const char* default_val);
double       pcl_container_get_param_f64(const pcl_container_t* c,
                                          const char* key, double default_val);
```

### 4.4 Port I/O (Service & Pub/Sub)

```c
/* Port types */
typedef enum {
    PCL_PORT_PUBLISHER   = 0,
    PCL_PORT_SUBSCRIBER  = 1,
    PCL_PORT_SERVICE     = 2,   /* request-reply server */
    PCL_PORT_CLIENT      = 3,   /* request-reply client */
} pcl_port_type_t;

/* Message buffer — user owns the data, container borrows it */
typedef struct {
    const void*  data;
    uint32_t     size;
    const char*  type_name;      /* e.g. "SensorReading", "StatusResponse" */
} pcl_msg_t;

/* Subscriber callback — called on executor thread */
typedef void (*pcl_sub_callback_t)(pcl_container_t* c,
                                    const pcl_msg_t* msg,
                                    void* user_data);

/* Service handler — called on executor thread, must fill response */
typedef pcl_status_t (*pcl_service_handler_t)(pcl_container_t* c,
                                               const pcl_msg_t* request,
                                               pcl_msg_t* response,
                                               void* user_data);

/* Create ports (must be called during on_configure) */
pcl_port_t* pcl_container_add_publisher(pcl_container_t* c,
                                         const char* topic,
                                         const char* type_name);
pcl_port_t* pcl_container_add_subscriber(pcl_container_t* c,
                                          const char* topic,
                                          const char* type_name,
                                          pcl_sub_callback_t cb,
                                          void* user_data);
pcl_port_t* pcl_container_add_service(pcl_container_t* c,
                                       const char* service_name,
                                       const char* type_name,
                                       pcl_service_handler_t handler,
                                       void* user_data);

/* Publish (from on_tick or service handler) */
pcl_status_t pcl_port_publish(pcl_port_t* port, const pcl_msg_t* msg);
```

### 4.5 Executor

```c
/* Executor — runs one or more containers on a single thread */
pcl_executor_t* pcl_executor_create(void);
void            pcl_executor_destroy(pcl_executor_t* e);

pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c);

/* Spin — blocks, runs all containers' ticks and I/O callbacks in round-robin */
pcl_status_t pcl_executor_spin(pcl_executor_t* e);

/* Spin once — process pending work, return immediately */
pcl_status_t pcl_executor_spin_once(pcl_executor_t* e, uint32_t timeout_ms);

/* Thread-safe ingress from external I/O threads (deep-copies payload) */
pcl_status_t pcl_executor_post_incoming(pcl_executor_t* e,
                                        const char* topic,
                                        const pcl_msg_t* msg);

/* Request shutdown (thread-safe, can be called from signal handler) */
void pcl_executor_request_shutdown(pcl_executor_t* e);

/* Logging — integrates with transport adapter (e.g. ROS2 RCLCPP_INFO) */
void pcl_log(pcl_container_t* c, int level, const char* fmt, ...);

#ifdef __cplusplus
}
#endif
```

---

## 5. C++ Wrapper

The header-only C++ wrapper (`pcl/component.hpp`) provides ergonomic authoring on top of the C ABI. The wrapper uses static trampoline functions to bridge virtual method calls to the C callback interface.

```cpp
// pcl/component.hpp — header-only C++ convenience wrapper
namespace pcl {

class Component {
public:
    Component(std::string_view name) {
        pcl_callbacks_t cbs = {};
        cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
            return static_cast<Component*>(ud)->on_configure();
        };
        cbs.on_activate = [](pcl_container_t* c, void* ud) -> pcl_status_t {
            return static_cast<Component*>(ud)->on_activate();
        };
        cbs.on_tick = [](pcl_container_t* c, double dt, void* ud) -> pcl_status_t {
            return static_cast<Component*>(ud)->on_tick(dt);
        };
        // ... other callbacks
        handle_ = pcl_container_create(name.data(), &cbs, this);
    }

    virtual ~Component() { pcl_container_destroy(handle_); }

    // Override these in your component
    virtual pcl_status_t on_configure() { return PCL_OK; }
    virtual pcl_status_t on_activate()  { return PCL_OK; }
    virtual pcl_status_t on_deactivate(){ return PCL_OK; }
    virtual pcl_status_t on_tick(double dt) { return PCL_OK; }
    // ...

    // Convenience helpers
    pcl_port_t* addPublisher(const char* topic, const char* type) {
        return pcl_container_add_publisher(handle_, topic, type);
    }
    double paramF64(const char* key, double def) {
        return pcl_container_get_param_f64(handle_, key, def);
    }
    void logInfo(const char* fmt, ...) { /* delegates to pcl_log */ }

    pcl_container_t* handle() { return handle_; }

private:
    pcl_container_t* handle_;
};

} // namespace pcl
```

---

## 6. Transport Adapter Layer

The container's ports produce/consume `pcl_msg_t` (opaque byte buffers + type name). A **transport adapter** connects these to the real middleware:

```
Container Port --pcl_msg_t--► Transport Adapter --► Wire
                                   │
                          ┌--------┼--------┐
                          ▼        ▼        ▼
                        ROS2    SharedMem  Socket
```

### 6.1 Adapter Interface

Implement a transport adapter by filling in four function pointers:

```c
typedef struct {
    /* Called when container publishes — adapter sends to wire */
    pcl_status_t (*publish)(void* adapter_ctx, const char* topic,
                            const pcl_msg_t* msg);

    /* Called to register a subscription on the adapter side */
    pcl_status_t (*subscribe)(void* adapter_ctx, const char* topic,
                               const char* type_name);

    /* Service routing */
    pcl_status_t (*serve)(void* adapter_ctx, const char* service_name,
                          const pcl_msg_t* request, pcl_msg_t* response);

    /* Cleanup */
    void (*shutdown)(void* adapter_ctx);

    void* adapter_ctx;
} pcl_transport_t;

pcl_status_t pcl_executor_set_transport(pcl_executor_t* e,
                                         const pcl_transport_t* transport);
```

### 6.2 ROS2 Adapter Example

```cpp
// ros2_adapter.cpp — wraps PCL containers as rclcpp_lifecycle::LifecycleNode
class Ros2Adapter {
public:
    Ros2Adapter(rclcpp::executors::SingleThreadedExecutor& ros_exec,
                pcl_executor_t* pcl_exec);

    // Wraps each pcl_container_t as a LifecycleNode with matching services/pubs
    void bridge(pcl_container_t* container);

    // Pumps pcl_executor_spin_once from a ROS2 timer callback
    void spin_integrated();
};
```

---

## 7. Lifecycle State Machine

PCL enforces a strict lifecycle compatible with ROS 2 managed nodes but with no rclcpp dependency:

```mermaid
stateDiagram-v2
    [*] --> UNCONFIGURED
    UNCONFIGURED --> CONFIGURED : configure() / on_configure
    CONFIGURED --> ACTIVE : activate() / on_activate
    ACTIVE --> CONFIGURED : deactivate() / on_deactivate
    CONFIGURED --> UNCONFIGURED : cleanup() / on_cleanup
    ACTIVE --> FINALIZED : shutdown() / on_shutdown
    CONFIGURED --> FINALIZED : shutdown() / on_shutdown
    UNCONFIGURED --> FINALIZED : shutdown() / on_shutdown
    FINALIZED --> [*]
```

**Key rules:**
- **Ports must be created during `on_configure`.** Dynamic port creation after configure is not supported.
- **`on_tick()` is only called while `ACTIVE`.** Each container has its own tick rate (set via parameter).
- **Shutdown is graceful** with a configurable timeout: `ACTIVE → on_deactivate → on_shutdown → FINALIZED`.

**Executor tick loop** (while `ACTIVE`):

```
while (!shutdown_requested) {
    t_now = clock();
    dt = t_now - t_prev;

    for each container in executor:
        if container.state == ACTIVE:
            drain_ingress_queue(container)          // posted by I/O threads
            process_incoming_messages(container)   // dispatch subscriber callbacks
            process_service_requests(container)     // dispatch service handlers
            container.callbacks.on_tick(container, dt, user_data)

    sleep_until(next_tick)
}
```

All callbacks execute on the **single executor thread** — no mutexes needed in user code. External transport threads never call user callbacks directly; they only enqueue messages via `pcl_executor_post_incoming()`.

---

## 8. Serialization

Ports exchange `pcl_msg_t` — a pointer, size, and type name string. PCL is **format-agnostic**: the `data` field is an opaque byte buffer. Each transport adapter knows how to route bytes on the wire.

For **intra-process** communication (no transport adapter set), messages are passed by pointer with zero-copy semantics.

For **cross-process** or **cross-network** communication, you can optionally use POD structs with layout descriptors to enable automatic serialization by the transport adapter:

```c
typedef struct {
    uint64_t version;
    double   value;
    bool     valid;
} SensorReading;

/* Layout descriptor for transport adapters */
static const pcl_field_desc_t SensorReading_fields[] = {
    { "version", PCL_TYPE_U64,  offsetof(SensorReading, version) },
    { "value",   PCL_TYPE_F64,  offsetof(SensorReading, value) },
    { "valid",   PCL_TYPE_BOOL, offsetof(SensorReading, valid) },
    { NULL, 0, 0 }
};
```

This keeps the core minimal while supporting zero-copy intra-process and automatic serialization for common message types.

---

## 9. Writing a Component (C++)

Here is a complete example of a PCL component using the C++ wrapper:

```cpp
#include "pcl/component.hpp"
#include "pcl/executor.hpp"

class TemperatureSensor : public pcl::Component {
public:
    TemperatureSensor() : Component("temp_sensor") {}

protected:
    pcl_status_t on_configure() override {
        threshold_ = paramF64("alert_threshold", 50.0);
        pub_alert_ = addPublisher("alerts", "AlertMsg");
        addSubscriber("config_updates", "ConfigMsg",
            [this](const pcl_msg_t* msg) { /* handle config update */ });
        return PCL_OK;
    }

    pcl_status_t on_activate() override {
        setTickRateHz(10.0);
        return PCL_OK;
    }

    pcl_status_t on_tick(double dt) override {
        logInfo("Sensor ticking... dt=%f", dt);
        // Read sensor, check threshold, publish alert if needed
        return PCL_OK;
    }

private:
    double threshold_ = 0.0;
    pcl_port_t* pub_alert_ = nullptr;
};

int main() {
    TemperatureSensor sensor;
    pcl::Executor exec;

    sensor.configure();
    sensor.activate();

    exec.add(sensor);
    exec.spin();  // Blocks and runs the tick loop

    return 0;
}
```

---

## 10. Writing a Component (C)

The same component in pure C, suitable for Ada interop or embedded targets:

```c
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"

typedef struct {
    pcl_port_t* pub_alert;
    double threshold;
} SensorData;

pcl_status_t sensor_on_configure(pcl_container_t* c, void* ud) {
    SensorData* d = (SensorData*)ud;
    d->threshold = pcl_container_get_param_f64(c, "alert_threshold", 50.0);
    d->pub_alert = pcl_container_add_publisher(c, "alerts", "AlertMsg");
    return PCL_OK;
}

pcl_status_t sensor_on_tick(pcl_container_t* c, double dt, void* ud) {
    SensorData* d = (SensorData*)ud;
    /* Read sensor, check threshold, publish alert if needed */
    return PCL_OK;
}

int main(void) {
    SensorData data = {0};
    pcl_callbacks_t cbs = {
        .on_configure = sensor_on_configure,
        .on_tick      = sensor_on_tick,
    };

    pcl_container_t* c = pcl_container_create("temp_sensor", &cbs, &data);
    pcl_executor_t* e = pcl_executor_create();

    pcl_container_configure(c);
    pcl_container_activate(c);
    pcl_executor_add(e, c);
    pcl_executor_spin(e);

    pcl_executor_destroy(e);
    pcl_container_destroy(c);
    return 0;
}
```

---

## 11. Writing a Component (Ada)

The pure-C ABI enables direct Ada bindings with no C++ dependency:

```ada
procedure On_Configure(Self : System.Address; User_Data : System.Address)
  return Interfaces.C.int
  with Convention => C;

Callbacks : aliased pcl_callbacks_t := (
  on_configure  => On_Configure'Access,
  on_activate   => On_Activate'Access,
  on_tick       => On_Tick'Access,
  others        => null
);
Container : pcl_container_t_Access :=
    pcl_container_create("sensor", Callbacks'Access, User_Data'Address);
```

See `subprojects/PYRAMID/examples/ada/pcl_bindings.ads` for the full Ada binding specification.

---

## 12. External I/O Integration

PCL draws a hard line between business logic and transport I/O. External threads (gRPC, sockets, device drivers) must never call component callbacks directly. Instead, they post messages into the executor's ingress queue:

```
external I/O thread
  -> receive / deserialize / classify
  -> pcl_executor_post_incoming(executor, topic, msg)
  -> return to middleware immediately

executor thread
  -> drain ingress queue
  -> dispatch subscriber callback / service handler
  -> run on_tick()
```

For long-latency outbound calls (e.g., gRPC clients), the recommended pattern is the inverse: the executor creates the request, an adapter-owned I/O thread waits for network completion, and the completion is posted back into the executor as a new ingress event.

See `subprojects/PCL/examples/external_io_bridge_example.c` for a concrete producer-thread to executor-queue to subscriber flow.

