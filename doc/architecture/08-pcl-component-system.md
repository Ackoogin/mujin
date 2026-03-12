# 08 — PCL Component System Architecture

This document provides Mermaid diagrams illustrating the PYRAMID Container Library (PCL)
component system: its layered architecture, communication patterns, and deployment
configurations with different communication bindings.

---

## 1. PCL Layer Stack

The PCL separates application logic from middleware through a clean layered design.
The pure-C core has zero external dependencies; language wrappers and transport
adapters are injected at composition time.

```mermaid
block-beta
  columns 1
  block:app["Application Layer"]
    columns 3
    WM["WorldModelComponent"]
    PL["PlannerComponent"]
    EX["ExecutorComponent"]
  end
  block:cpp["C++ Wrapper (component.hpp)"]
    columns 1
    COMP["pcl::Component — RAII, virtual methods, trampolines"]
  end
  block:capi["Pure C API (pcl_container.h / pcl_executor.h)"]
    columns 4
    CONT["Container"]
    EXEC["Executor"]
    PORT["Ports"]
    PARAM["Parameters"]
  end
  block:transport["Transport Adapter Layer (pcl_transport.h)"]
    columns 4
    NONE["None (intra-process)"]
    ROS2["ROS 2 / DDS"]
    GRPC["gRPC"]
    SHM["Shared Memory"]
  end

  app --> cpp
  cpp --> capi
  capi --> transport
```

---

## 2. Container & Executor Internal Architecture

Each container encapsulates one component's business logic behind a lifecycle state
machine. The executor drives one or more containers on a single thread, guaranteeing
deterministic callback ordering.

```mermaid
graph TB
  subgraph Executor["pcl_executor_t (single thread)"]
    direction TB
    IQ["Ingress Queue<br/>(thread-safe, deep-copy)"]
    DISPATCH["Message Dispatch"]
    TICK["Tick Loop"]

    IQ -->|"drain"| DISPATCH

    subgraph C1["Container A"]
      direction LR
      PUB1["pub: ~/state"]
      SUB1["sub: ~/cmd"]
      SVC1["svc: get_fact"]
      TICK1["on_tick()"]
    end

    subgraph C2["Container B"]
      direction LR
      PUB2["pub: ~/plan"]
      SUB2["sub: ~/state"]
      SVC2["svc: solve_goal"]
      TICK2["on_tick()"]
    end

    DISPATCH -->|"topic match"| SUB1
    DISPATCH -->|"topic match"| SUB2
    TICK --> TICK1
    TICK --> TICK2
  end

  EXT["External I/O Thread<br/>(sensor, middleware, gRPC)"]
  EXT -->|"pcl_executor_post_incoming()"| IQ

  PUB1 -->|"intra-process<br/>zero-copy"| SUB2

  style Executor fill:#f0f4ff,stroke:#3366cc
  style C1 fill:#e8f5e9,stroke:#2e7d32
  style C2 fill:#fff3e0,stroke:#e65100
```

---

## 3. Component Lifecycle State Machine

All containers follow a strict lifecycle compatible with ROS 2 managed nodes.
Ports are created during `on_configure`; ticking only occurs while `ACTIVE`.

```mermaid
stateDiagram-v2
  [*] --> UNCONFIGURED : create()

  UNCONFIGURED --> CONFIGURED : configure()
  CONFIGURED --> ACTIVE : activate()
  ACTIVE --> CONFIGURED : deactivate()
  CONFIGURED --> UNCONFIGURED : cleanup()

  UNCONFIGURED --> FINALIZED : shutdown()
  CONFIGURED --> FINALIZED : shutdown()
  ACTIVE --> FINALIZED : shutdown()

  FINALIZED --> [*] : destroy()

  note right of UNCONFIGURED
    No ports exist yet
  end note

  note right of CONFIGURED
    Ports created, params set
    on_tick NOT called
  end note

  note right of ACTIVE
    on_tick called at tick_rate_hz
    Publishing enabled
  end note
```

---

## 4. Port & Communication Model

Components communicate through typed ports. The transport adapter layer determines
whether messages stay in-process or cross a network boundary.

```mermaid
graph LR
  subgraph ComponentA["Component A (Publisher)"]
    CODE_A["Business Logic"]
    PUB["pcl_port_publish()"]
    CODE_A --> PUB
  end

  subgraph TransportLayer["Transport Decision"]
    direction TB
    CHECK{{"Transport<br/>adapter set?"}}
    INTRA["Intra-process<br/>direct dispatch<br/>(zero-copy)"]
    WIRE["Adapter publish()<br/>serialize → wire"]
    CHECK -->|"No"| INTRA
    CHECK -->|"Yes"| WIRE
  end

  subgraph ComponentB["Component B (Subscriber)"]
    SUB["Subscriber callback"]
    CODE_B["Business Logic"]
    SUB --> CODE_B
  end

  PUB --> CHECK
  INTRA --> SUB
  WIRE -->|"post_incoming()<br/>(deep copy)"| SUB

  style TransportLayer fill:#fff8e1,stroke:#f9a825
  style ComponentA fill:#e8f5e9,stroke:#2e7d32
  style ComponentB fill:#e3f2fd,stroke:#1565c0
```

---

## 5. Bridge Pattern

A bridge is a managed container that subscribes to one topic, applies a transform,
and dispatches the result to a different topic — enabling unit conversion, encoding
changes, and protocol translation without modifying either endpoint.

```mermaid
graph LR
  subgraph Sensor["Sensor Component"]
    PUB_S["pub: sensors/speed_mps<br/>type: SpeedMps"]
  end

  subgraph Bridge["Bridge Container<br/>'speed_conv'"]
    SUB_B["sub: sensors/speed_mps"]
    FN["transform_fn()<br/>m/s → km/h"]
    PUB_B["dispatch: hmi/speed_kmph<br/>type: SpeedKmph"]
    SUB_B --> FN --> PUB_B
  end

  subgraph HMI["HMI Component"]
    SUB_H["sub: hmi/speed_kmph"]
  end

  PUB_S --> SUB_B
  PUB_B --> SUB_H

  style Bridge fill:#f3e5f5,stroke:#7b1fa2
  style Sensor fill:#e8f5e9,stroke:#2e7d32
  style HMI fill:#e3f2fd,stroke:#1565c0
```

---

## 6. Deployment Example — Single Process, No Middleware

All three AME components share one executor. Communication is intra-process
zero-copy. No transport adapter is set. This is the simplest deployment, suitable
for testing, simulation, and embedded targets.

```mermaid
graph TB
  subgraph Process["Single Process"]
    subgraph Exec["pcl_executor_t"]
      direction TB

      subgraph WMC["WorldModelComponent"]
        WM_SVC["svc: get_fact, set_fact, query_state"]
        WM_PUB["pub: ~/state"]
        WM_TICK["on_tick: publish state if dirty"]
      end

      subgraph PC["PlannerComponent"]
        PC_WM["inprocess_wm_ = &wm"]
        PC_METHOD["solveGoal() → direct call"]
      end

      subgraph EC["ExecutorComponent"]
        EC_WM["inprocess_wm_ = &wm"]
        EC_TICK["on_tick: tickOnce()"]
        EC_FACTORY["BT::Factory"]
      end
    end
  end

  PC_WM -.->|"direct pointer<br/>(zero-copy)"| WMC
  EC_WM -.->|"direct pointer<br/>(zero-copy)"| WMC
  WM_PUB -->|"intra-process dispatch"| PC
  WM_PUB -->|"intra-process dispatch"| EC

  style Process fill:#f5f5f5,stroke:#424242
  style Exec fill:#e8eaf6,stroke:#283593
  style WMC fill:#e8f5e9,stroke:#2e7d32
  style PC fill:#fff3e0,stroke:#e65100
  style EC fill:#e3f2fd,stroke:#1565c0
```

---

## 7. Deployment Example — ROS 2 Transport Binding

Each component runs in its own executor (potentially in separate processes or nodes).
A ROS 2 transport adapter bridges PCL ports to DDS topics and services. The planner
uses `setQueryStateCallback()` instead of a direct pointer, receiving world state
snapshots over the wire.

```mermaid
graph TB
  subgraph Node1["Process / Node 1"]
    subgraph Exec1["pcl_executor_t"]
      WMC2["WorldModelComponent"]
    end
    TA1["ROS 2 Transport Adapter"]
    Exec1 --- TA1
  end

  subgraph Node2["Process / Node 2"]
    subgraph Exec2["pcl_executor_t"]
      PC2["PlannerComponent"]
    end
    TA2["ROS 2 Transport Adapter"]
    Exec2 --- TA2
  end

  subgraph Node3["Process / Node 3"]
    subgraph Exec3["pcl_executor_t"]
      EC2["ExecutorComponent"]
    end
    TA3["ROS 2 Transport Adapter"]
    Exec3 --- TA3
  end

  subgraph DDS["DDS Network (ROS 2 middleware)"]
    T_STATE["topic: ~/state<br/>WorldState"]
    T_PLAN["topic: ~/plan<br/>PlanResult"]
    S_FACT["service: get_fact"]
    S_GOAL["service: solve_goal"]
  end

  TA1 <-->|"rclcpp pub/sub"| T_STATE
  TA1 <-->|"rclcpp service"| S_FACT
  TA2 <-->|"rclcpp pub/sub"| T_STATE
  TA2 <-->|"rclcpp service"| S_GOAL
  TA2 <-->|"rclcpp pub/sub"| T_PLAN
  TA3 <-->|"rclcpp pub/sub"| T_PLAN
  TA3 <-->|"rclcpp pub/sub"| T_STATE

  style DDS fill:#fce4ec,stroke:#c62828
  style Node1 fill:#e8f5e9,stroke:#2e7d32
  style Node2 fill:#fff3e0,stroke:#e65100
  style Node3 fill:#e3f2fd,stroke:#1565c0
```

---

## 8. Deployment Example — Mixed Bindings with Bridges

A realistic deployment where some components are co-located (sharing an executor for
zero-copy speed) while others are distributed over different transports. Bridges
handle protocol translation at system boundaries.

```mermaid
graph TB
  subgraph Vehicle["Vehicle On-Board Computer"]
    subgraph ExecA["pcl_executor_t (autonomy)"]
      WMC3["WorldModelComponent"]
      EC3["ExecutorComponent"]
      BRIDGE_OUT["Bridge: WM state<br/>→ protobuf encoding"]
    end

    subgraph ExecS["pcl_executor_t (sensors)"]
      LIDAR["LidarComponent"]
      IMU["ImuComponent"]
      BRIDGE_UNIT["Bridge: m/s → km/h"]
    end
  end

  subgraph GCS["Ground Control Station"]
    subgraph ExecG["pcl_executor_t"]
      PLAN["PlannerComponent"]
      HMI["HMI / Dashboard"]
    end
    TAG["gRPC Transport Adapter"]
    ExecG --- TAG
  end

  WMC3 -->|"intra-process<br/>zero-copy"| EC3
  LIDAR -->|"intra-process"| BRIDGE_UNIT
  BRIDGE_UNIT -->|"intra-process"| WMC3
  IMU -->|"intra-process"| WMC3
  WMC3 -->|"intra-process"| BRIDGE_OUT

  BRIDGE_OUT <-->|"gRPC / protobuf<br/>over network"| TAG
  TAG <--> PLAN
  TAG <--> HMI

  style Vehicle fill:#e8f5e9,stroke:#2e7d32
  style GCS fill:#e3f2fd,stroke:#1565c0
  style ExecA fill:#f1f8e9,stroke:#558b2f
  style ExecS fill:#f9fbe7,stroke:#827717
  style ExecG fill:#e8eaf6,stroke:#283593
```

---

## 9. Deployment Example — Ada / C Cross-Language Binding

The pure-C ABI enables components written in different languages to share an executor.
An Ada sensor component links against the same `libpcl` as the C++ autonomy components.

```mermaid
graph TB
  subgraph Process["Single Process (mixed-language)"]
    subgraph Exec["pcl_executor_t"]
      direction TB

      subgraph Ada["Ada Components"]
        ADA_SENSOR["Ada_Sensor_Component<br/>(pcl_bindings.ads)"]
        ADA_SAFETY["Ada_Safety_Monitor<br/>(SPARK proven)"]
      end

      subgraph Cpp["C++ Components"]
        CPP_WM["WorldModelComponent"]
        CPP_PLAN["PlannerComponent"]
        CPP_EXEC["ExecutorComponent"]
      end
    end
  end

  ADA_SENSOR -->|"pub: ~/sensor_data<br/>(C ABI, zero-copy)"| CPP_WM
  ADA_SAFETY -->|"sub: ~/state<br/>(C ABI, zero-copy)"| CPP_WM
  CPP_WM -->|"intra-process"| CPP_PLAN
  CPP_PLAN -->|"intra-process"| CPP_EXEC

  subgraph ABI["Shared Interface"]
    CABI["Pure C ABI<br/>pcl_container_t / pcl_port_t / pcl_msg_t<br/>No C++ symbols cross boundary"]
  end

  Ada -.->|"links against"| ABI
  Cpp -.->|"links against"| ABI

  style Process fill:#f5f5f5,stroke:#424242
  style Ada fill:#fff3e0,stroke:#e65100
  style Cpp fill:#e3f2fd,stroke:#1565c0
  style ABI fill:#f3e5f5,stroke:#7b1fa2
```

---

## 10. Transport Adapter Vtable

The transport adapter is a simple vtable of four function pointers. Implementing a
new transport binding requires only filling in this struct.

```mermaid
classDiagram
  class pcl_transport_t {
    +void* adapter_ctx
    +publish(ctx, topic, msg) pcl_status_t
    +subscribe(ctx, topic, type_name) pcl_status_t
    +serve(ctx, service, request, response) pcl_status_t
    +shutdown(ctx) void
  }

  class IntraProcessAdapter {
    -executor* exec
    +publish() direct dispatch
    +subscribe() register callback
    +serve() direct call
  }

  class ROS2Adapter {
    -rclcpp::Node* node
    -map~string,Publisher~ pubs
    -map~string,Subscription~ subs
    +publish() rclcpp publish
    +subscribe() create subscription → post_incoming()
    +serve() rclcpp service server
  }

  class GrpcAdapter {
    -grpc::Channel* channel
    -serializer* codec
    +publish() serialize → stream
    +subscribe() async read → post_incoming()
    +serve() unary RPC handler
  }

  class SharedMemAdapter {
    -shm_region* region
    -sem_t* notify
    +publish() memcpy → signal
    +subscribe() wait → post_incoming()
    +serve() req/resp through shm
  }

  pcl_transport_t <|.. IntraProcessAdapter
  pcl_transport_t <|.. ROS2Adapter
  pcl_transport_t <|.. GrpcAdapter
  pcl_transport_t <|.. SharedMemAdapter
```

---

## 11. Message Flow Through the Stack

End-to-end message flow from a publisher in one component to a subscriber in another,
showing how the transport adapter layer intercepts when present.

```mermaid
sequenceDiagram
  participant App as Component A<br/>(publisher)
  participant Port as pcl_port_publish()
  participant Exec as Executor
  participant Transport as Transport Adapter
  participant Wire as Network / DDS / gRPC
  participant Exec2 as Remote Executor
  participant Sub as Component B<br/>(subscriber)

  App->>Port: pcl_port_publish(port, &msg)

  alt No transport adapter (intra-process)
    Port->>Exec: dispatch_incoming(topic, msg)
    Exec->>Sub: subscriber_callback(msg)
    Note over Exec,Sub: Zero-copy, same thread
  else Transport adapter set
    Port->>Transport: adapter->publish(ctx, topic, msg)
    Transport->>Wire: serialize & send
    Wire->>Transport: receive on I/O thread
    Transport->>Exec2: pcl_executor_post_incoming()<br/>(deep copy, thread-safe)
    Note over Transport,Exec2: Crosses thread boundary
    Exec2->>Sub: subscriber_callback(msg)<br/>on executor thread
  end
```
