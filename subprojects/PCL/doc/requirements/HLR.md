# PCL High-Level Requirements

Requirements for the PYRAMID Composition Library (PCL), the standard deployable component container framework for autonomous mission systems.

PCL encapsulates core business logic behind a consistent lifecycle while entirely decoupling logic from external middleware (ROS2, DDS, sockets, etc.), enabling the same autonomy logic to run in resource-constrained embedded environments (bare-metal C/Ada), desktop simulations, or distributed systems over a network.

## Subject Matter

The subject matter of PCL is the lifecycle management, deterministic execution, inter-component communication, and pluggable transport abstraction for PYRAMID components.

**Exclusions:**

- Domain-specific business logic (handled by components built on PCL).
- Serialization format choices (PCL is format-agnostic; payloads are opaque byte buffers).
- Network protocol details beyond the TCP socket reference transport.
- Middleware-specific bindings (ROS2, DDS adapters are separate concerns).

## Design Decisions

### D1 - Zero External Dependencies
The core library (`pcl_core`) is written in strict C17 with zero external dependencies -- not even the C++ STL.

**Why**: Guarantees portability to bare-metal embedded targets, Ada, and Rust through a stable C ABI.

### D2 - Deterministic Single-Threaded Execution
All user callbacks execute on a single executor thread. No internal mutexes are required within component business logic.

**Why**: Eliminates data races and non-deterministic behavior in safety-critical autonomy code.

### D3 - Pluggable Transport Adapters
I/O is a separate, pluggable concern. A C function-pointer vtable (`pcl_transport_t`) bridges PCL ports to external middleware.

**Why**: Decouples component logic from deployment topology and middleware choice.

### D4 - C ABI with C++ Ergonomics
The ABI boundary is purely C. Header-only C++ wrappers provide RAII, virtual methods, and modern ergonomics without compromising portability.

**Why**: Guarantees integration with Ada, Rust, and C++ while offering developer convenience.

### D5 - Hard I/O Thread Boundary
External I/O threads must never call component callbacks directly. They enqueue messages via `pcl_executor_post_incoming()`, which deep-copies the payload.

**Why**: Maintains the deterministic single-threaded execution guarantee while accepting data from concurrent producers.

### D6 - Lifecycle State Machine
Containers follow a strict state machine (UNCONFIGURED -> CONFIGURED -> ACTIVE -> FINALIZED) enforcing safe initialization, port creation, and shutdown sequences.

**Why**: Deterministic resource allocation and teardown, modelled on the ROS2 managed lifecycle.

### D7 - Bridge as First-Class Primitive
Bridges are managed containers that subscribe, transform, and re-dispatch on the same tick -- enabling unit conversion, encoding change, and protocol translation without custom glue code.

**Why**: Standardizes the common pattern of adapting between component interfaces.

### D8 - Runtime Composition via Plugins and Manifests
Transports and codecs are loadable at runtime through a versioned C plugin ABI, and a deployment manifest maps each endpoint to a named transport. Deployment topology is data, not code.

**Why**: The same component binary must compose with different middleware (sockets, shared memory, UDP, gRPC, ROS2) per deployment without relinking.

### D9 - Fail-Closed Compose-Time Validation
Every remote endpoint route is validated at compose time against the routed transport's declared interaction capabilities and QoS. Anything unprovable (missing capability, undeclared reliability, unknown peer, malformed manifest) is rejected before the system runs, with a precise diagnostic, and nothing is left partially installed.

**Why**: Configuration errors in safety-relevant deployments must surface at startup, never as silent misbehaviour in flight.

---

## Detailed Requirements

## Container Lifecycle

### PCL.001 - Lifecycle State Machine
The container shall implement a strict state machine with states: UNCONFIGURED (initial), CONFIGURED, ACTIVE, and FINALIZED (terminal).

**Rationale**: Deterministic initialization and shutdown sequences are essential for safety-critical systems.

### PCL.002 - Lifecycle Transitions
The container shall enforce valid transitions: UNCONFIGURED -> CONFIGURED (configure), CONFIGURED -> ACTIVE (activate), ACTIVE -> CONFIGURED (deactivate), CONFIGURED -> UNCONFIGURED (cleanup), and any-state -> FINALIZED (shutdown).

**Rationale**: Invalid transitions must be rejected to prevent undefined behavior.

### PCL.003 - Transition Callback Invocation
Each lifecycle transition shall invoke its corresponding user-supplied callback
when one is configured.

**Rationale**: Components need hooks to perform setup and teardown at each phase.

### PCL.004 - Callback Failure Aborts Transition
A lifecycle transition shall preserve the prior state when its callback reports
failure.

**Rationale**: Failed initialization must not leave the container in an inconsistent state.

### PCL.005 - Container Identity
Each container shall have a human-readable name set at creation time, queryable at any point.

**Rationale**: Diagnostics and logging require component identification.

## Port Management

### PCL.006 - Port Types
The container shall support four port types: publisher, subscriber, service (server), and client.

**Rationale**: Pub/sub and request/reply are the two fundamental communication patterns.

### PCL.007 - Port Creation During Configure Only
Port creation shall be permitted only while a container is being configured.

**Rationale**: Immutable port topology after configuration enables deterministic memory layout and prevents runtime resource leaks.

### PCL.008 - Port Capacity Limit
A container shall reject creation of more than 64 ports.

**Rationale**: Bounded resource usage is required for embedded targets with fixed memory budgets.

### PCL.009 - Publish Constraint
A container shall reject publishing unless it is active.

**Rationale**: Ensures consumers only receive messages from fully initialized components.

### PCL.010 - Subscriber Callback Dispatch
When a message arrives on a subscribed topic, the container's subscriber callback shall be invoked on the executor thread with the message payload.

**Rationale**: Components need to process incoming data.

### PCL.011 - Service Handler Dispatch
When a service request arrives at a server, the container's service handler shall be invoked on the executor thread. The handler populates the response synchronously within that invocation.

**Rationale**: Server-side request/reply handling for queries and commands.

### PCL.085 - Async Service Invocation
A service client shall receive an asynchronous response on the executor thread.

**Rationale**: Non-blocking service calls allow the client to continue processing while awaiting responses.

### PCL.086 - Deferred Service Response
A service handler shall be able to complete its response after the handler
invocation returns.

**Rationale**: Service handlers often need to aggregate data from multiple sources or wait for external events before responding.

### PCL.087 - Streaming Service Response
A service handler shall be able to produce a cancellable sequence of response
messages.

**Rationale**: Query services often need to return large result sets incrementally (like gRPC server streaming).

## Parameters

### PCL.012 - Typed Parameter Storage
The container shall support key-value parameters of types: string, double (f64), int64, and boolean.

**Rationale**: Components need configuration data without external config file dependencies.

### PCL.013 - Parameter Capacity Limit
A container shall reject creation of more than 128 parameters.

**Rationale**: Bounded resource usage for embedded targets.

### PCL.014 - Parameter Default Values
Parameter retrieval shall return a caller-supplied default when the key is not found or the stored type does not match the requested type.

**Rationale**: Safe fallback behavior prevents crashes from missing configuration.

## Tick Rate and Periodic Execution

### PCL.015 - Configurable Tick Rate
Each container shall have a configurable periodic execution rate.

**Rationale**: Components need periodic execution at domain-appropriate rates.

### PCL.016 - Tick Rate Validation
Non-positive periodic execution rates shall be rejected.

**Rationale**: Prevents infinite loops or undefined timer behavior.

### PCL.017 - Delta Time Reporting
Each periodic callback shall receive the elapsed wall-clock time since its
previous invocation.

**Rationale**: Components need accurate time deltas for rate-independent logic.

## Executor

### PCL.018 - Multi-Container Execution
The executor shall drive one or more containers on a single thread, ticking each at its configured rate.

**Rationale**: Multiple components share a single thread for deterministic, lock-free execution.

### PCL.019 - Spin and Spin-Once
Executor processing shall be available in blocking and single-pass modes.

**Rationale**: Blocking mode for production; single-pass mode for testing and integration.

### PCL.020 - Shutdown Request
The executor shall support an async-signal-safe, thread-safe shutdown request that stops the spin loop.

**Rationale**: External signal handlers and watchdog threads need to terminate the executor safely.

### PCL.021 - Graceful Shutdown
The executor shall provide a graceful shutdown that transitions each ACTIVE container through deactivate -> shutdown -> FINALIZED, with a configurable timeout.

**Rationale**: Orderly teardown prevents resource leaks and ensures cleanup callbacks fire.

### PCL.022 - Intra-Process Direct Dispatch
When no transport adapter is set, the executor shall route published messages directly to matching subscribers via zero-copy pointer handoff.

**Rationale**: Efficient same-process communication without serialization overhead.

### PCL.023 - Service Invocation
The executor shall support intra-process service invocation by name, dispatching to the registered handler across all managed containers.

**Rationale**: Enables direct service calls in testing and single-process deployments.

### PCL.024 - Container Add and Remove
The executor shall support adding and removing containers dynamically (before spin or between spin_once calls).

**Rationale**: Flexible composition of component graphs.

## Cross-Thread Ingress

### PCL.025 - Cross-Thread Message Posting
External I/O threads shall be able to enqueue incoming messages without
retaining ownership of their buffers.

**Rationale**: External producer threads must be able to post data without retaining ownership of buffers.

### PCL.026 - Ingress Queue Drain
Queued messages shall be drained and dispatched to subscriber callbacks on the executor thread during the next spin cycle.

**Rationale**: Maintains the single-threaded callback execution guarantee.

### PCL.027 - Async Response Delivery
Asynchronous transport responses shall be delivered on the executor thread.

**Rationale**: Non-blocking async service invocation requires a response delivery mechanism.

### PCL.057 - Cross-Thread Local Service Request Queuing
External threads shall be able to enqueue local service requests for later
execution on the executor thread.

**Rationale**: D5 forbids external threads from calling component callbacks directly. Without this function there is no thread-safe path for an external producer to trigger a local service handler -- callers were forced to use `pcl_executor_invoke_service()` which bypasses the threading model when called off the executor thread.

## Transport Adapter

### PCL.028 - Transport Adapter Interface
The transport adapter shall expose optional communication operations through a
stable C interface.

**Rationale**: Pluggable I/O without link-time coupling to any specific middleware.

### PCL.029 - Transport Wiring
Executor communication shall be selectable between a configured transport and
direct intra-process dispatch.

**Rationale**: Runtime selection of communication backend.

### PCL.030 - Dispatch Incoming From Transport
A transport already executing on the executor thread shall be able to dispatch
an incoming message synchronously.

**Rationale**: Transport adapters that run on the executor thread need a synchronous dispatch path.

### PCL.088 - Transport Client Service Invocation
Client-side service invocation shall use the selected transport asynchronously.

**Rationale**: Service invocation must be as pluggable as pub/sub to maintain transport abstraction (D3). Components should not be coupled to specific transport implementations when calling services.

### PCL.089 - Endpoint Routing Configuration
Each endpoint's local and remote delivery mode shall be configurable at
startup.

**Rationale**: Endpoint locality must be deployment configuration, not inferred from whether an interface is `provided` or `consumed`.

### PCL.090 - Named Peer Transport Registration
Remote endpoint traffic shall use the named transport selected by its route.

**Rationale**: Multi-executor mesh and bridge deployments require more than one remote peer per executor.

### PCL.091 - Remote Ingress Filtering
Subscriber and provided-service endpoints shall accept remote traffic only when their route configuration allows remote delivery, and when the source peer matches any configured peer allow-list.

**Rationale**: Remote exposure must be explicit so local-only endpoints are not accidentally reachable over the network.

## TCP Socket Transport

### PCL.031 - Server Mode
The socket transport shall support accepting remote peer connections.

**Rationale**: Cross-process PCL communication for distributed deployments.

### PCL.032 - Client Mode
The socket transport shall support establishing a connection to a remote peer.

**Rationale**: Client-side of cross-process communication.

### PCL.033 - Wire Protocol
Socket transport messages shall use a big-endian length-prefixed binary frame with distinct publish, service-request, and service-response message types.

**Rationale**: Minimal, unambiguous framing for reliable message delimitation.

### PCL.034 - Gateway Container
The server-mode transport shall provide a gateway container that receives service requests from the wire and dispatches them to registered service handlers.

**Rationale**: Service routing for remote clients.

### PCL.035 - Non-Blocking Send
Socket egress shall not perform blocking input or output on the executor thread.

**Rationale**: Protects the deterministic tick loop from network latency.

### PCL.036 - Async Remote Service Invocation
Remote client service invocation shall enqueue its request and deliver its response on the executor thread.

**Rationale**: Non-blocking remote service calls from component logic.

### PCL.092 - Remote Peer Identity
The reference socket transport shall associate inbound traffic with a configured peer identity and pass that identity into executor routing decisions for remote pub/sub and service dispatch.

**Rationale**: Per-peer routing and exposure control require the transport to preserve the logical source peer across ingress.

### PCL.096 - Robust Client Connect Semantics
Socket client connection management shall tolerate bounded startup delay and remote peer restarts.

**Rationale**: Production deployments cannot assume strict start order between peers, and peers may restart. Fail-fast semantics force every caller to implement retry plumbing, which is error-prone and often done incorrectly. Centralising the robust connect path in the transport keeps component logic focused on application behaviour while still detecting genuine unreachable peers within a bounded deadline.

### PCL.093 - Inter-Process Shared Memory Bus
The shared-memory transport shall support multiple local processes joining the same named bus through an OS shared-memory region containing participant mailboxes.

**Rationale**: A host-local deployment needs a pluggable transport with lower overhead than sockets while still preserving the PCL transport abstraction.

### PCL.094 - Shared Memory Publish Fan-Out
The shared-memory transport shall fan out published messages from one participant to the other participants on the same named bus and preserve the source participant identity for remote ingress filtering.

**Rationale**: A central-bus topology must deliver the same remote pub/sub semantics as other transports without devolving into point-to-point links.

### PCL.095 - Shared Memory Async Remote Service Invocation
The shared-memory transport shall support async remote service invocation across processes by routing requests to the unique advertised provider on the bus and delivering responses back to the caller on the executor thread.

**Rationale**: Request/reply traffic must be transport-pluggable just like pub/sub, including across process boundaries on the same host.

### PCL.098 - Shared Memory Atomic Fan-Out
A shared-memory publish shall enqueue its frame for every intended participant or for none of them.

**Rationale**: A congested participant must not cause silent partial delivery to the remaining participants.

### PCL.103 - Shared Memory Topic Backpressure
A shared-memory topic shall be configurable to wait for mailbox capacity for a bounded duration.

**Rationale**: Selected low-rate topics need a bounded way to tolerate temporary mailbox pressure.

### PCL.097 - UDP Datagram Transport
PCL shall provide a connectionless best-effort publish-subscribe transport.

**Rationale**: High-rate telemetry, sensor feeds, and state broadcasts often tolerate occasional loss but cannot afford TCP framing overhead or head-of-line blocking. A separate UDP transport keeps semantics explicit (pub/sub-only, best-effort) and prevents accidental use for reliability-sensitive RPC.

## Logging

### PCL.037 - Printf-Style Logging
PCL shall provide context-aware severity logging with formatted messages.

**Rationale**: Consistent logging API across all components.

### PCL.038 - Pluggable Log Handler
Configured logging shall be replaceable by a caller-supplied handler.

**Rationale**: Integration with ROS2 logging, file loggers, or custom sinks.

### PCL.039 - Log Level Filtering
Log messages below the configured minimum severity shall be discarded.

**Rationale**: Noise reduction in production; verbosity in development.

### PCL.040 - Log Levels
PCL shall support five log levels: DEBUG, INFO, WARN, ERROR, FATAL.

**Rationale**: Standard severity classification.

## Bridge

### PCL.041 - Bridge Creation
A bridge shall be created with an input topic/type, output topic/type, and a transform function. The bridge allocates an internal container owned by the bridge.

**Rationale**: Reusable topic-to-topic adaptation without custom container code.

### PCL.042 - Bridge Transform Dispatch
A successful bridge transformation shall publish the transformed message during the same executor cycle.

**Rationale**: Same-tick forwarding for latency-sensitive data pipelines.

### PCL.043 - Bridge Message Suppression
If the transform function returns a non-OK status, the message shall be silently dropped without dispatching to the output topic.

**Rationale**: Filtering and validation within the transform pipeline.

### PCL.044 - Bridge Null Safety
Bridge creation shall reject NULL arguments (executor, name, topics, types, transform function) and return NULL.

**Rationale**: Defensive programming against misuse.

## Robustness and Error Handling

### PCL.045 - Null Handle Safety
All public API functions shall handle NULL handles gracefully, returning appropriate error codes or safe defaults.

**Rationale**: Defensive programming prevents crashes from misuse.

### PCL.046 - Allocation Failure Handling
A failed internal allocation shall leave no partial resource ownership.

**Rationale**: Graceful degradation under memory pressure.

### PCL.047 - Status Codes
PCL shall use a consistent set of status codes: OK, ERR_INVALID, ERR_STATE, ERR_TIMEOUT, ERR_CALLBACK, ERR_NOMEM, ERR_NOT_FOUND, ERR_PORT_CLOSED.

**Rationale**: Uniform error reporting across the API.

## C++ Wrappers

### PCL.048 - Component Base Class
The C++ component wrapper shall manage component lifetime and provide type-safe component configuration.

**Rationale**: Ergonomic C++ interface without compromising the C ABI.

### PCL.049 - Executor Wrapper
The C++ executor wrapper shall manage executor lifetime and provide type-safe component execution.

**Rationale**: C++ developers expect RAII and type safety.

### PCL.056 - Public Route Configuration API
PCL shall expose a public C ABI for configuring endpoint locality and peer selection without editing transport internals manually.

**Rationale**: Local-vs-remote deployment choices should be expressible through stable runtime configuration APIs, not by rewriting transport glue.

## Codecs

### PCL.058 - Codec Abstraction
Payload codecs shall be pluggable by content type through a stable C interface.

**Rationale**: PCL treats payloads as opaque bytes (see Exclusions); codecs adapt typed values at the boundary without violating D1.

### PCL.059 - Codec Registry
PCL shall provide process-wide registration and ordered lookup of payload codecs by content type.

**Rationale**: A bridge process spanning several components must host their codec plugins side by side and select among them at dispatch time.

## Transport Capabilities and QoS

### PCL.060 - Transport Capability Declaration
Each transport shall declare the interaction capabilities it supports.

**Rationale**: Compose-time validation (D9) needs an authoritative statement of what a transport can carry.

### PCL.061 - Endpoint Capability Requirements
Each remote endpoint kind shall declare the transport capability it requires.

**Rationale**: Deterministic mapping from interface shape to required transport behaviour.

### PCL.062 - QoS Declaration and Floor Semantics
A transport shall satisfy an endpoint quality-of-service floor only when every declared offered value meets or exceeds it.

**Rationale**: Reliability must be proven by the transport, not assumed by the deployment.

### PCL.063 - Compose-Time Route Validation
Remote endpoint composition shall reject a route whose selected transport cannot prove the required capability and quality of service.

**Rationale**: D9 -- misconfiguration must be caught before traffic flows.

## Plugin ABI and Loader

### PCL.064 - Transport Plugin ABI
Transport plugins shall use a version-gated stable C interface.

**Rationale**: D8 -- a stable, versioned C contract lets independently built middleware adapters load into any PCL process.

### PCL.065 - Plugin Loader Fail-Closed Behaviour
Plugin loading shall leave nothing registered when plugin identity or compatibility cannot be validated.

**Rationale**: A half-loaded or ABI-incompatible plugin must never be reachable from live traffic.

### PCL.066 - Codec Plugin Batch Loading
A codec plugin batch shall retain every valid codec independently of invalid entries.

**Rationale**: Deployments enumerate codecs as data; one bad path must not abort the rest of the composition.

### PCL.067 - Safe Teardown-Then-Unload
Transport plugin code shall remain loaded until its transport resources and worker threads have been released.

**Rationale**: Unloading a library while its threads still execute in it is undefined behaviour; the safe order must be enforced centrally.

### PCL.068 - Reference Transport Plugins
The socket, UDP, and shared-memory transports shall each ship as loadable plugins that construct their transport from an opaque JSON configuration (role, addressing, and the owning executor), fail closed (NULL) on missing or invalid configuration, declare accurate capabilities and QoS, and export teardown (and, for the socket server, gateway access) symbols.

**Rationale**: D8 -- the reference transports must be composable purely through the plugin ABI.

## Manifest-Driven Endpoint Routing

### PCL.069 - Routing Manifest
PCL shall construct named transports and endpoint routes from a deployment routing manifest.

**Rationale**: D8 -- one manifest composes heterogeneous middleware per deployment.

### PCL.070 - Routing Atomicity and Fail-Closed Diagnostics
A failed routing manifest load shall leave executor routing state unchanged.

**Rationale**: D9 -- partial composition after a failed load is worse than no composition.

### PCL.078 - Manifest-Driven Remote Streaming Invocation
Remote streaming invocation shall use the named transport selected for that
endpoint.

**Rationale**: Streaming calls must honour the same deployment-selected
transport routing as other remote endpoint interactions.

### PCL.099 - Streaming Route Capability
A remote streaming-client route shall be rejected during composition when its
selected transport does not support streaming.

**Rationale**: An unsupported streaming route must fail before runtime traffic
can reach it.

### PCL.100 - Routed Transport Gateway Discovery
A caller shall be able to discover the optional gateway container exposed by a
named routed transport.

**Rationale**: A component that provides remote services needs an explicit way
to attach transport-owned ingress without making the manifest loader infer
component intent.

### PCL.077 - Manifest Exclusive Realization Groups
A routing manifest shall reject a configuration that selects endpoints from
both sides of a declared mutually exclusive group.

**Rationale**: Selecting both realizations of one logical interaction creates
duplicate delivery paths and must fail during composition.

## Transport Template and Conformance

### PCL.071 - Transport Template Scaffold
PCL shall provide a reusable worker-thread transport scaffold around blocking input and output hooks.

**Rationale**: Every concrete transport re-implements the same threading skeleton; centralizing it makes new adapters small and uniformly correct (D2, D5).

### PCL.072 - Transport Conformance Suite
Every in-tree transport adapter shall pass the reusable delivery conformance suite.

**Rationale**: A common behavioural bar keeps transport semantics interchangeable (D3).

### PCL.073 - APOS LVC Transport
PCL shall provide an APOS Local Virtual Channel transport that preserves the standard transport semantics.

**Rationale**: Bare-metal ASAAC/APOS targets are a primary deployment environment (D1, D3).

## Portable Allocator

### PCL.074 - Portable Allocator
All variable-length memory crossing the C interface shall use one portable allocation domain.

**Rationale**: Buffers cross DLL/EXE boundaries between different C runtimes (MSVC plugins vs GNAT/MinGW executables); a mismatched allocator pair corrupts the heap (D1, D4).

## Transport Threading Model

### PCL.075 - Transport Threading-Model Contract
Transport adapters shall isolate blocking input, blocking output, and foreign-thread activity from component callbacks on the executor thread.

**Rationale**: D2 requires all component business logic to run on the single executor thread; D5 requires a hard I/O thread boundary. Stating the contract once, at the vtable, lets third-party adapter authors implement it correctly without reading reference-transport internals.

### PCL.076 - Transport Threading-Model Conformance Suite
Every in-tree transport shall pass the reusable threading-model conformance suite.

**Rationale**: A common, mechanically-checked threading bar keeps the deterministic-execution guarantee (D2, D5) from silently regressing as new transports and plugins are added.

## Standalone Process Runtime

### PCL.079 - Standalone Component Process Runtime
PCL shall run one component as a standalone process using deployment-selected
codecs and transports.

**Rationale**: A standalone component process needs one reusable owner for
deployment composition, lifecycle sequencing, execution, and cleanup.

## Transport Flow Control and Monitoring

### PCL.080 - Remote Subscription Registration
Remote subscriber setup shall register each subscriber only with the transport
selected for that endpoint.

**Rationale**: A transport needs the selected topic and type to establish
remote delivery without making setup order affect the result.

### PCL.081 - Bounded Executor Ingress
Executor ingress queue storage shall be boundable by deployment configuration.

**Rationale**: Long-running processes need an explicit memory bound and an
observable backpressure result.

### PCL.082 - Shared-Memory Subscription Interest
The shared-memory transport shall enqueue published topics only for
participants that registered interest in those topics.

**Rationale**: Bounded participant mailboxes must not carry unrelated traffic.

### PCL.101 - Shared-Memory Service Backpressure
A shared-memory unary service request shall produce a terminal outcome within a
bounded interval when its provider mailbox is full.

**Rationale**: Short bursts may be retried, but congestion must not leave a
client call pending indefinitely.

### PCL.083 - Bounded Reliable-Socket Egress
Reliable-socket outbound frame storage shall not exceed 16 MiB.

**Rationale**: Bounded storage prevents sustained network congestion from
causing unbounded process memory growth.

### PCL.084 - UDP Received-Datagram Accounting
The UDP transport shall report the number of datagrams received from the
network.

**Rationale**: Operators need to distinguish an idle link from received traffic
that could not be decoded.

### PCL.102 - UDP Sequence-Gap Accounting
The UDP transport shall report inferred forward sequence gaps independently for
each traffic source.

**Rationale**: Per-source gap accounting provides lightweight loss evidence
without claiming acknowledgements or retransmission.

---

## Design Decision Traceability

| Requirement | Design Decision |
| :--- | :--- |
| `PCL.001` | `D6` |
| `PCL.002` | `D6` |
| `PCL.003` | `D6` |
| `PCL.004` | `D6` |
| `PCL.005` | `D6` |
| `PCL.006` | `D3` |
| `PCL.007` | `D6` |
| `PCL.008` | `D1` |
| `PCL.009` | `D6` |
| `PCL.010` | `D2` |
| `PCL.011` | `D2` |
| `PCL.085` | `D3`, `D5` |
| `PCL.086` | `D2`, `D5` |
| `PCL.087` | `D2`, `D5` |
| `PCL.012` | `D1` |
| `PCL.013` | `D1` |
| `PCL.014` | `D1` |
| `PCL.015` | `D2` |
| `PCL.016` | `D2` |
| `PCL.017` | `D2` |
| `PCL.018` | `D2` |
| `PCL.019` | `D2` |
| `PCL.020` | `D2`, `D5` |
| `PCL.021` | `D6` |
| `PCL.022` | `D3` |
| `PCL.023` | `D2` |
| `PCL.024` | `D2` |
| `PCL.025` | `D5` |
| `PCL.026` | `D2`, `D5` |
| `PCL.027` | `D5` |
| `PCL.057` | `D2`, `D5` |
| `PCL.028` | `D3` |
| `PCL.029` | `D3` |
| `PCL.030` | `D3` |
| `PCL.088` | `D3` |
| `PCL.089` | `D3` |
| `PCL.090` | `D3` |
| `PCL.091` | `D2`, `D3`, `D5` |
| `PCL.031` | `D3` |
| `PCL.032` | `D3` |
| `PCL.033` | `D3` |
| `PCL.034` | `D3` |
| `PCL.035` | `D2`, `D5` |
| `PCL.036` | `D3`, `D5` |
| `PCL.092` | `D3`, `D5` |
| `PCL.093` | `D3`, `D5` |
| `PCL.094` | `D3`, `D5` |
| `PCL.095` | `D3`, `D5` |
| `PCL.098` | `D3`, `D5` |
| `PCL.103` | `D3`, `D5` |
| `PCL.037` | `D1` |
| `PCL.038` | `D3` |
| `PCL.039` | `D1` |
| `PCL.040` | `D1` |
| `PCL.041` | `D7` |
| `PCL.042` | `D2`, `D7` |
| `PCL.043` | `D7` |
| `PCL.044` | `D7` |
| `PCL.045` | `D1` |
| `PCL.046` | `D1` |
| `PCL.047` | `D1` |
| `PCL.048` | `D4` |
| `PCL.049` | `D4` |
| `PCL.056` | `D3`, `D4` |
| `PCL.058` | `D1`, `D3`, `D8` |
| `PCL.059` | `D8` |
| `PCL.060` | `D8`, `D9` |
| `PCL.061` | `D9` |
| `PCL.062` | `D9` |
| `PCL.063` | `D9` |
| `PCL.064` | `D8` |
| `PCL.065` | `D8`, `D9` |
| `PCL.066` | `D8` |
| `PCL.067` | `D5`, `D8` |
| `PCL.068` | `D3`, `D8` |
| `PCL.069` | `D8` |
| `PCL.070` | `D9` |
| `PCL.077` | `D8`, `D9` |
| `PCL.078` | `D3`, `D8` |
| `PCL.099` | `D9` |
| `PCL.100` | `D3`, `D8` |
| `PCL.071` | `D2`, `D3`, `D5` |
| `PCL.072` | `D3` |
| `PCL.073` | `D1`, `D3` |
| `PCL.074` | `D1`, `D4` |
| `PCL.075` | `D2`, `D5` |
| `PCL.076` | `D2`, `D5` |
| `PCL.079` | `D1`, `D3`, `D6`, `D8`, `D9` |
| `PCL.080` | `D3` |
| `PCL.081` | `D1` |
| `PCL.082` | `D1`, `D3` |
| `PCL.101` | `D1`, `D3` |
| `PCL.083` | `D1`, `D5` |
| `PCL.084` | `D7` |
| `PCL.102` | `D7` |
