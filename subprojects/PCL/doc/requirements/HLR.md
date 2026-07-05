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
Each lifecycle transition shall invoke the corresponding user-supplied callback (`on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown`). Null callbacks shall be treated as no-ops returning success.

**Rationale**: Components need hooks to perform setup and teardown at each phase.

### PCL.004 - Callback Failure Aborts Transition
If a lifecycle callback returns a non-OK status, the transition shall be aborted and the container shall remain in its previous state.

**Rationale**: Failed initialization must not leave the container in an inconsistent state.

### PCL.005 - Container Identity
Each container shall have a human-readable name set at creation time, queryable at any point.

**Rationale**: Diagnostics and logging require component identification.

## Port Management

### PCL.006 - Port Types
The container shall support four port types: publisher, subscriber, service (server), and client.

**Rationale**: Pub/sub and request/reply are the two fundamental communication patterns.

### PCL.007 - Port Creation During Configure Only
Ports shall only be creatable during the `on_configure` callback. Attempts to create ports outside this phase shall fail.

**Rationale**: Immutable port topology after configuration enables deterministic memory layout and prevents runtime resource leaks.

### PCL.008 - Port Capacity Limit
The container shall support a fixed maximum number of ports (64). Exceeding this limit shall return an error.

**Rationale**: Bounded resource usage is required for embedded targets with fixed memory budgets.

### PCL.009 - Publish Constraint
Publishing shall only be permitted while the container is in the ACTIVE state. Attempts to publish on inactive containers shall return `PCL_ERR_PORT_CLOSED`.

**Rationale**: Ensures consumers only receive messages from fully initialized components.

### PCL.010 - Subscriber Callback Dispatch
When a message arrives on a subscribed topic, the container's subscriber callback shall be invoked on the executor thread with the message payload.

**Rationale**: Components need to process incoming data.

### PCL.011 - Service Handler Dispatch
When a service request arrives at a server, the container's service handler shall be invoked on the executor thread. The handler populates the response synchronously within that invocation.

**Rationale**: Server-side request/reply handling for queries and commands.

### PCL.011a - Async Service Invocation
Client-side service invocation may be asynchronous. The client enqueues a request via the transport; the response callback fires on the executor thread when the reply arrives.

**Rationale**: Non-blocking service calls allow the client to continue processing while awaiting responses.

### PCL.011b - Deferred Service Response
A service handler may defer its response by returning `PCL_PENDING` and saving the service context. The handler shall later call `pcl_service_respond()` to send the response. This enables handlers that need to perform async operations (e.g., call other services) before responding.

**Rationale**: Service handlers often need to aggregate data from multiple sources or wait for external events before responding.

### PCL.011c - Streaming Service Response
A streaming service handler may return `PCL_STREAMING` to begin a multi-message response stream. The handler shall use `pcl_stream_send()` to send messages, `pcl_stream_end()` to complete normally, or `pcl_stream_abort()` to terminate with an error. Clients may cancel mid-stream via `pcl_stream_cancel()`, and servers may poll cancellation status via `pcl_stream_is_cancelled()`.

**Rationale**: Query services often need to return large result sets incrementally (like gRPC server streaming).

## Parameters

### PCL.012 - Typed Parameter Storage
The container shall support key-value parameters of types: string, double (f64), int64, and boolean.

**Rationale**: Components need configuration data without external config file dependencies.

### PCL.013 - Parameter Capacity Limit
The container shall support a fixed maximum number of parameters (128). Exceeding this limit shall return an error.

**Rationale**: Bounded resource usage for embedded targets.

### PCL.014 - Parameter Default Values
Parameter retrieval shall return a caller-supplied default when the key is not found or the stored type does not match the requested type.

**Rationale**: Safe fallback behavior prevents crashes from missing configuration.

## Tick Rate and Periodic Execution

### PCL.015 - Configurable Tick Rate
Each container shall have a configurable tick rate in Hz (default 100 Hz). The `on_tick` callback shall be called at approximately this rate while the container is ACTIVE.

**Rationale**: Components need periodic execution at domain-appropriate rates.

### PCL.016 - Tick Rate Validation
Invalid tick rates (zero, negative) shall be rejected with `PCL_ERR_INVALID`.

**Rationale**: Prevents infinite loops or undefined timer behavior.

### PCL.017 - Delta Time Reporting
The `on_tick` callback shall receive the elapsed wall-clock time since the previous tick as a parameter.

**Rationale**: Components need accurate time deltas for rate-independent logic.

## Executor

### PCL.018 - Multi-Container Execution
The executor shall drive one or more containers on a single thread, ticking each at its configured rate.

**Rationale**: Multiple components share a single thread for deterministic, lock-free execution.

### PCL.019 - Spin and Spin-Once
The executor shall provide both blocking (`spin`) and single-pass (`spin_once`) execution modes.

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
The executor shall provide `pcl_executor_post_incoming()` for external I/O threads to enqueue messages safely. The function shall deep-copy the topic, type name, and payload before returning.

**Rationale**: External producer threads must be able to post data without retaining ownership of buffers.

### PCL.026 - Ingress Queue Drain
Queued messages shall be drained and dispatched to subscriber callbacks on the executor thread during the next spin cycle.

**Rationale**: Maintains the single-threaded callback execution guarantee.

### PCL.027 - Async Response Delivery
The executor shall support `pcl_executor_post_response_cb()` for delivering async service responses from transport threads back to the executor thread.

**Rationale**: Non-blocking async service invocation requires a response delivery mechanism.

### PCL.057 - Cross-Thread Local Service Request Queuing
The executor shall provide `pcl_executor_post_service_request()` for external threads to safely enqueue an intra-process service call. The function shall deep-copy the service name, type name, and request payload before returning. The service handler and response callback shall both execute on the executor thread during the next spin cycle.

**Rationale**: D5 forbids external threads from calling component callbacks directly. Without this function there is no thread-safe path for an external producer to trigger a local service handler -- callers were forced to use `pcl_executor_invoke_service()` which bypasses the threading model when called off the executor thread.

## Transport Adapter

### PCL.028 - Transport Adapter Interface
The transport adapter shall be a C struct of function pointers (`publish`, `serve`, `subscribe`, `shutdown`) with an opaque context pointer.

**Rationale**: Pluggable I/O without link-time coupling to any specific middleware.

### PCL.029 - Transport Wiring
The executor shall accept a transport adapter before spinning. Passing NULL shall revert to intra-process direct dispatch.

**Rationale**: Runtime selection of communication backend.

### PCL.030 - Dispatch Incoming From Transport
The transport adapter shall be able to dispatch incoming messages to subscriber callbacks via `pcl_executor_dispatch_incoming()` when already on the executor thread.

**Rationale**: Transport adapters that run on the executor thread need a synchronous dispatch path.

### PCL.030a - Transport Client Service Invocation
The transport adapter interface shall include an optional `invoke_async` function pointer for client-side service invocation. The executor shall provide `pcl_executor_invoke_async()` which routes requests through the configured transport.

**Rationale**: Service invocation must be as pluggable as pub/sub to maintain transport abstraction (D3). Components should not be coupled to specific transport implementations when calling services.

### PCL.030b - Endpoint Routing Configuration
PCL shall support explicit endpoint routing at startup for publisher, subscriber, provided-service, and consumed-service endpoints. Each endpoint route shall support local-only, remote-only, or local-plus-remote operation.

**Rationale**: Endpoint locality must be deployment configuration, not inferred from whether an interface is `provided` or `consumed`.

### PCL.030c - Named Peer Transport Registration
The executor shall support registration of multiple named peer transports and shall resolve remote endpoint traffic to a specific peer transport using the configured route.

**Rationale**: Multi-executor mesh and bridge deployments require more than one remote peer per executor.

### PCL.030d - Remote Ingress Filtering
Subscriber and provided-service endpoints shall accept remote traffic only when their route configuration allows remote delivery, and when the source peer matches any configured peer allow-list.

**Rationale**: Remote exposure must be explicit so local-only endpoints are not accidentally reachable over the network.

## TCP Socket Transport

### PCL.031 - Server Mode
The socket transport shall support server mode: listen on a TCP port, accept a client, and spawn recv/send threads.

**Rationale**: Cross-process PCL communication for distributed deployments.

### PCL.032 - Client Mode
The socket transport shall support client mode: connect to a server, spawn recv/send threads, and support async remote service invocation.

**Rationale**: Client-side of cross-process communication.

### PCL.033 - Wire Protocol
The socket transport shall use a framed binary protocol: `[4-byte length big-endian][payload]` with type bytes for PUBLISH (0x00), SVC_REQ (0x01), and SVC_RESP (0x02).

**Rationale**: Minimal, unambiguous framing for reliable message delimitation.

### PCL.034 - Gateway Container
The server-mode transport shall provide a gateway container that receives service requests from the wire and dispatches them to registered service handlers.

**Rationale**: Service routing for remote clients.

### PCL.035 - Non-Blocking Send
All socket writes shall be performed by a dedicated send thread via a mutex-protected FIFO queue. No blocking I/O shall occur on the executor thread.

**Rationale**: Protects the deterministic tick loop from network latency.

### PCL.036 - Async Remote Service Invocation
The client transport shall support `invoke_remote_async()` which enqueues a service request and fires a callback on the executor thread when the response arrives.

**Rationale**: Non-blocking remote service calls from component logic.

### PCL.036a - Remote Peer Identity
The reference socket transport shall associate inbound traffic with a configured peer identity and pass that identity into executor routing decisions for remote pub/sub and service dispatch.

**Rationale**: Per-peer routing and exposure control require the transport to preserve the logical source peer across ingress.

### PCL.036e - Robust Client Connect Semantics
The socket transport shall offer an extended client-create API that provides:
(a) bounded retry with exponential backoff for the initial connect,
(b) a connection-state callback exposing `CONNECTING`/`CONNECTED`/`DISCONNECTED` transitions,
(c) transparent auto-reconnect by the receive thread after a dropped connection, and
(d) TCP keepalive on all connected sockets for timely detection of silent peer death.
Host resolution shall use `getaddrinfo` (thread-safe, IPv6-capable).
The legacy single-shot client create shall remain available as a thin wrapper so existing call sites are unaffected.

**Rationale**: Production deployments cannot assume strict start order between peers, and peers may restart. Fail-fast semantics force every caller to implement retry plumbing, which is error-prone and often done incorrectly. Centralising the robust connect path in the transport keeps component logic focused on application behaviour while still detecting genuine unreachable peers within a bounded deadline.

### PCL.036b - Inter-Process Shared Memory Bus
The shared-memory transport shall support multiple local processes joining the same named bus through an OS shared-memory region containing participant mailboxes.

**Rationale**: A host-local deployment needs a pluggable transport with lower overhead than sockets while still preserving the PCL transport abstraction.

### PCL.036c - Shared Memory Publish Fan-Out
The shared-memory transport shall fan out published messages from one participant to the other participants on the same named bus and preserve the source participant identity for remote ingress filtering.

**Rationale**: A central-bus topology must deliver the same remote pub/sub semantics as other transports without devolving into point-to-point links.

### PCL.036d - Shared Memory Async Remote Service Invocation
The shared-memory transport shall support async remote service invocation across processes by routing requests to the unique advertised provider on the bus and delivering responses back to the caller on the executor thread.

**Rationale**: Request/reply traffic must be transport-pluggable just like pub/sub, including across process boundaries on the same host.

### PCL.036g - Shared Memory Atomic Fan-Out And Topic Backpressure
The shared-memory transport shall treat each published fan-out as one transaction: before writing a published frame, it shall verify that every target participant mailbox can accept the frame; if any target lacks capacity, no target shall receive that frame. The default publish path shall remain non-blocking and return an error on congestion. The transport shall also provide a per-topic configuration API allowing selected topics to wait for mailbox capacity up to a bounded timeout, so deployments can bind topics either to the generic non-blocking output path or to a topic-specific backpressure policy.

**Rationale**: Safety-relevant pub/sub traffic must not silently split fan-out delivery across a subset of peers. Optional per-topic backpressure allows reliable low-rate command/state topics to tolerate transient mailbox pressure without forcing high-rate telemetry topics to block component output paths. Any component that enables topic-specific blocking must review its output threading and deadline budget.

### PCL.036f - UDP Datagram Transport (Pub/Sub Only)
PCL shall ship a connectionless UDP datagram transport for best-effort publish/subscribe traffic.
Each datagram shall carry exactly one PUBLISH message serialised without a length prefix (UDP preserves message boundaries).
The transport shall deliberately not expose `invoke_async`, `respond`, `serve`, or `invoke_stream`: service RPC and streaming services remain exclusive to reliable transports.
Each transport instance shall bind a local UDP port and publish to a single configured remote peer; multi-peer fan-out is achieved by instantiating one transport per peer and registering each with `pcl_executor_register_transport()`.
Inbound datagrams shall be posted as remote ingress from the configured logical peer ID so existing per-endpoint peer allow-lists apply unchanged.

**Rationale**: High-rate telemetry, sensor feeds, and state broadcasts often tolerate occasional loss but cannot afford TCP framing overhead or head-of-line blocking. A separate UDP transport keeps semantics explicit (pub/sub-only, best-effort) and prevents accidental use for reliability-sensitive RPC.

## Logging

### PCL.037 - Printf-Style Logging
PCL shall provide a `pcl_log()` function with printf-style formatting, log level, and optional container context.

**Rationale**: Consistent logging API across all components.

### PCL.038 - Pluggable Log Handler
PCL shall support installing a custom log handler. Passing NULL shall revert to the default stderr handler.

**Rationale**: Integration with ROS2 logging, file loggers, or custom sinks.

### PCL.039 - Log Level Filtering
PCL shall support a minimum log level. Messages below this level shall be discarded.

**Rationale**: Noise reduction in production; verbosity in development.

### PCL.040 - Log Levels
PCL shall support five log levels: DEBUG, INFO, WARN, ERROR, FATAL.

**Rationale**: Standard severity classification.

## Bridge

### PCL.041 - Bridge Creation
A bridge shall be created with an input topic/type, output topic/type, and a transform function. The bridge allocates an internal container owned by the bridge.

**Rationale**: Reusable topic-to-topic adaptation without custom container code.

### PCL.042 - Bridge Transform Dispatch
When a message arrives on the bridge's input topic, the transform function shall be called. If it returns PCL_OK, the transformed message shall be dispatched immediately to the output topic on the same tick.

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
When internal memory allocation fails, the function shall return `PCL_ERR_NOMEM` and clean up any partially allocated resources.

**Rationale**: Graceful degradation under memory pressure.

### PCL.047 - Status Codes
PCL shall use a consistent set of status codes: OK, ERR_INVALID, ERR_STATE, ERR_TIMEOUT, ERR_CALLBACK, ERR_NOMEM, ERR_NOT_FOUND, ERR_PORT_CLOSED.

**Rationale**: Uniform error reporting across the API.

## C++ Wrappers

### PCL.048 - Component Base Class
The C++ wrapper shall provide a `pcl::Component` base class with virtual lifecycle methods, RAII destruction, parameter helpers, port creation helpers, and logging convenience methods.

**Rationale**: Ergonomic C++ interface without compromising the C ABI.

### PCL.049 - Executor Wrapper
The C++ wrapper shall provide a `pcl::Executor` class with RAII destruction, type-safe `add(Component&)`, and spin/shutdown methods.

**Rationale**: C++ developers expect RAII and type safety.

## Service Bindings (Proto/Generated)

### PCL.050 - Wire-Name Constants
Generated service bindings shall provide compile-time constants for all service wire names matching the proto definitions.

**Rationale**: Eliminates string literal duplication and typo risk.

### PCL.051 - Topic Constants
Generated service bindings shall provide compile-time constants for all standard topic names.

**Rationale**: Consistent topic naming across all language bindings.

### PCL.052 - Service Handler Base Class
Generated bindings shall provide a `ServiceHandler` base class with virtual methods for each service operation, with default stub implementations.

**Rationale**: Type-safe service dispatch with minimal boilerplate.

### PCL.053 - JSON Builder Functions
Generated bindings shall provide helper functions for constructing standard requirement and evidence JSON payloads.

**Rationale**: Consistent payload format across components.

### PCL.054 - Subscribe Wrappers
Generated bindings shall provide topic subscription helper functions that register PCL subscriber ports with correct topic and type names.

**Rationale**: Eliminates manual topic/type string wiring.

### PCL.055 - Dispatch Function
Generated bindings shall provide a dispatch function that routes service channel enum values to the appropriate handler method.

**Rationale**: Clean mapping from wire protocol to business logic.

### PCL.056 - Public Route Configuration API
PCL shall expose a public C ABI for configuring endpoint locality and peer selection without editing transport internals manually.

**Rationale**: Local-vs-remote deployment choices should be expressible through stable runtime configuration APIs, not by rewriting transport glue.

## Codecs

### PCL.058 - Codec Abstraction
PCL shall define a format-agnostic codec vtable (`pcl_codec_t`) carrying an ABI version, a content-type string, and `encode`/`decode`/`free_msg` function pointers, so payload serialization is pluggable without coupling `pcl_core` to any format.

**Rationale**: PCL treats payloads as opaque bytes (see Exclusions); codecs adapt typed values at the boundary without violating D1.

### PCL.059 - Codec Registry
PCL shall provide a codec registry mapping content-type strings to borrowed codec vtables: registration shall reject NULL arguments (`PCL_ERR_INVALID`), ABI-version mismatches and duplicate vtable pointers (`PCL_ERR_STATE`); multiple codecs may share a content type, with lookup returning the first registered and indexed iteration returning them in registration order; the registry shall expose count and clear operations and a lazily created, process-stable default registry.

**Rationale**: A bridge process spanning several components must host their codec plugins side by side and select among them at dispatch time.

## Transport Capabilities and QoS

### PCL.060 - Transport Capability Declaration
Each transport shall have an interaction-capability mask (pub/sub, unary RPC, streaming RPC, action RPC). A plugin may declare its mask explicitly via the optional caps symbol; when absent, the mask shall be conservatively derived from the non-NULL vtable slots. The action capability shall never be derived (it has no vtable slot). A NULL vtable or handle shall yield no capabilities (fail closed).

**Rationale**: Compose-time validation (D9) needs an authoritative statement of what a transport can carry.

### PCL.061 - Endpoint Capability Requirements
Each endpoint kind shall map to the transport capability it requires: publisher/subscriber to pub/sub, provided/consumed to unary RPC, stream-provided to streaming RPC. Unrecognised kinds shall impose no requirement.

**Rationale**: Deterministic mapping from interface shape to required transport behaviour.

### PCL.062 - QoS Declaration and Floor Semantics
Transports may declare an offered QoS (reliability: unspecified < best-effort < reliable); endpoints may declare a QoS floor. An offer shall satisfy a floor only when every declared dimension is greater than or equal to the floor; an undeclared (unspecified) offer shall fail any declared floor.

**Rationale**: Reliability must be proven by the transport, not assumed by the deployment.

### PCL.063 - Compose-Time Route Validation
The executor shall validate an endpoint route against the capability mask and offered QoS of each transport it routes to -- named peers or the default transport slot -- failing closed with `PCL_ERR_NOT_FOUND` for unknown/absent transports and `PCL_ERR_STATE` for missing capabilities or unmet QoS floors, and emitting a human-readable diagnostic naming the endpoint and the deficiency.

**Rationale**: D9 -- misconfiguration must be caught before traffic flows.

## Plugin ABI and Loader

### PCL.064 - Transport Plugin ABI
Transport plugins shall export a versioned ABI: a mandatory ABI-version symbol and entry-point symbol (returning a borrowed transport vtable for an opaque JSON configuration string), plus optional capability, QoS, and teardown symbols. The ABI version constant shall gate loading.

**Rationale**: D8 -- a stable, versioned C contract lets independently built middleware adapters load into any PCL process.

### PCL.065 - Plugin Loader Fail-Closed Behaviour
The plugin loader shall fail closed when a plugin cannot be trusted: missing file or missing mandatory symbols (`PCL_ERR_NOT_FOUND`), ABI-version mismatch or a NULL vtable/codec from the entry point (`PCL_ERR_STATE`). On any failure the library shall be unloaded and nothing registered. The loader shall also expose a portable raw open/symbol/unload wrapper over the platform dynamic loader.

**Rationale**: A half-loaded or ABI-incompatible plugin must never be reachable from live traffic.

### PCL.066 - Codec Plugin Batch Loading
The loader shall load codec plugins individually (registering the codec and returning an owned handle) and in batches from a path array, a path-list environment variable, or a manifest file (blank lines and `#` comments ignored). Batch loading shall skip entries that are missing or are not codec plugins, and successfully loaded plugins shall remain resident while their codecs are registered.

**Rationale**: Deployments enumerate codecs as data; one bad path must not abort the rest of the composition.

### PCL.067 - Safe Teardown-Then-Unload
`pcl_plugin_unload_transport()` shall invoke the plugin's teardown symbol (releasing live resources and stopping background threads) before unloading the library, and shall degrade to a plain unload when the symbol is absent. NULL handles shall be rejected with `PCL_ERR_INVALID`.

**Rationale**: Unloading a library while its threads still execute in it is undefined behaviour; the safe order must be enforced centrally.

### PCL.068 - Reference Transport Plugins
The socket, UDP, and shared-memory transports shall each ship as loadable plugins that construct their transport from an opaque JSON configuration (role, addressing, and the owning executor), fail closed (NULL) on missing or invalid configuration, declare accurate capabilities and QoS, and export teardown (and, for the socket server, gateway access) symbols.

**Rationale**: D8 -- the reference transports must be composable purely through the plugin ABI.

## Manifest-Driven Endpoint Routing

### PCL.069 - Routing Manifest
PCL shall load a line-based routing manifest with `transport <peer_id> <plugin_path> [config...]` and `route <endpoint> <kind> <peers>[ <reliability>]` directives (blank lines and `#` comments ignored): standing up each transport plugin, registering it as a named peer with its declared capabilities and QoS, installing each endpoint route, and validating every route at compose time. An empty or comment-only manifest shall succeed with an empty routing handle; the handle shall own the loaded plugins and report how many transports it loaded.

**Rationale**: D8 -- one manifest composes heterogeneous middleware per deployment.

### PCL.070 - Routing Atomicity and Fail-Closed Diagnostics
A failed manifest load shall leave nothing installed: transports already stood up shall be torn down (via safe teardown-then-unload), installed routes cleared, and a precise diagnostic written. The load shall fail closed on malformed lines, unknown directives or kinds or reliability tokens, oversized peer lists, unknown peers, duplicate transport peers, and routes duplicating an existing route; destroying a routing handle shall preserve unrelated transports (including an unrelated default transport) and free its executor transport slots for reuse.

**Rationale**: D9 -- partial composition after a failed load is worse than no composition.

## Transport Template and Conformance

### PCL.071 - Transport Template Scaffold
PCL shall provide a transport template implementing the standard transport threading model (dedicated send and receive worker threads, non-blocking vtable calls, cross-thread ingress via the executor) around engineer-supplied blocking I/O hooks (`send_blocking`, `recv_blocking`, optional `open`/`close`/`wake`). Creation shall fail closed on missing mandatory hooks or a failed `open`; send-hook failures shall drop the frame (logged) without wedging the send thread; hard receive failures shall log and retry until destroy; frames with unknown kinds or unmatched response sequence ids shall be ignored; after shutdown, publish and async invocation shall fail with `PCL_ERR_STATE` and pending entries shall be reclaimed; destroy shall drain queued frames, honour every peer alias the transport was known by, and clear the executor's default slot only when it points at this transport.

**Rationale**: Every concrete transport re-implements the same threading skeleton; centralizing it makes new adapters small and uniformly correct (D2, D5).

### PCL.072 - Transport Conformance Suite
PCL shall provide a reusable transport conformance suite (vtable shape, publish round trip, service round trip, publish ordering) that every transport adapter implementation shall pass.

**Rationale**: A common behavioural bar keeps transport semantics interchangeable (D3).

### PCL.073 - APOS LVC Transport
PCL shall provide a transport over APOS Local Virtual Channels, built on the transport template, that frames PCL messages onto raw APOS payloads with 16-bit string-length limits enforced at encode time and malformed or truncated inbound APOS messages dropped without disrupting the receive loop. Creation shall fail closed without an executor. The bundled APOS stub shall implement the blocking and non-blocking send/receive, multi-channel wait, timeout, delay-callback, and process-setup semantics of the APOS binding.

**Rationale**: Bare-metal ASAAC/APOS targets are a primary deployment environment (D1, D3).

## Portable Allocator

### PCL.074 - Portable Allocator
PCL shall provide `pcl_alloc`/`pcl_calloc`/`pcl_realloc`/`pcl_free` routing through a single CRT-independent process heap on Windows (plain malloc/free elsewhere), rejecting zero-size and arithmetically overflowing requests with NULL, zeroing calloc'd memory, preserving contents across realloc growth, treating realloc of NULL as alloc and realloc to zero as free, and accepting `pcl_free(NULL)` as a no-op. All variable-length fields crossing the C ABI shall be allocated and freed through this pair.

**Rationale**: Buffers cross DLL/EXE boundaries between different C runtimes (MSVC plugins vs GNAT/MinGW executables); a mismatched allocator pair corrupts the heap (D1, D4).

## Transport Threading Model

### PCL.075 - Transport Threading-Model Contract
Every transport adapter shall obey a uniform threading contract at the `pcl_transport_t` boundary:
(a) inbound wire traffic shall be deep-copied off the transport's own receive/worker threads and posted to executor-owned queues, so that subscriber callbacks, service handlers, stream callbacks, and async response callbacks execute only on the executor thread while it drains those queues, never inline on a transport thread;
(b) executor-facing egress entry points (`publish`, `invoke_async`, `invoke_stream`) shall not perform blocking I/O, remote-service waits, provider-discovery polling, or unbounded lock waits -- they may validate, copy the payload, register correlation state, enqueue work to a transport-owned worker, and return promptly;
(c) an `invoke_async` response callback shall never fire synchronously before `invoke_async` returns; it shall be delivered on a later executor spin;
(d) transport teardown shall wake and join any blocked worker threads before releasing resources the workers may still touch, and shall reclaim any queued-but-undelivered work.

**Rationale**: D2 requires all component business logic to run on the single executor thread; D5 requires a hard I/O thread boundary. Stating the contract once, at the vtable, lets third-party adapter authors implement it correctly without reading reference-transport internals.

### PCL.076 - Transport Threading-Model Conformance Suite
PCL shall provide a reusable threading-model conformance suite -- distinct from the delivery conformance suite (PCL.072) -- that proves the PCL.075 contract for a transport: (a) subscriber ingress runs on the executor/spin thread and not inline on a transport worker; (b) an async response callback runs on the executor thread and does not fire inline from `invoke_async`; (c) executor-facing egress returns promptly while the real send path is blocked; and (d) destroy wakes and joins a blocked send worker. Every in-tree transport (template, shared memory, UDP, socket) shall pass the applicable cases.

**Rationale**: A common, mechanically-checked threading bar keeps the deterministic-execution guarantee (D2, D5) from silently regressing as new transports and plugins are added.

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
| `PCL.011a` | `D3`, `D5` |
| `PCL.011b` | `D2`, `D5` |
| `PCL.011c` | `D2`, `D5` |
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
| `PCL.030a` | `D3` |
| `PCL.030b` | `D3` |
| `PCL.030c` | `D3` |
| `PCL.030d` | `D2`, `D3`, `D5` |
| `PCL.031` | `D3` |
| `PCL.032` | `D3` |
| `PCL.033` | `D3` |
| `PCL.034` | `D3` |
| `PCL.035` | `D2`, `D5` |
| `PCL.036` | `D3`, `D5` |
| `PCL.036a` | `D3`, `D5` |
| `PCL.036b` | `D3`, `D5` |
| `PCL.036c` | `D3`, `D5` |
| `PCL.036d` | `D3`, `D5` |
| `PCL.036g` | `D3`, `D5` |
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
| `PCL.050` | `D3` |
| `PCL.051` | `D3` |
| `PCL.052` | `D4` |
| `PCL.053` | `D3` |
| `PCL.054` | `D3` |
| `PCL.055` | `D3` |
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
| `PCL.071` | `D2`, `D3`, `D5` |
| `PCL.072` | `D3` |
| `PCL.073` | `D1`, `D3` |
| `PCL.074` | `D1`, `D4` |
| `PCL.075` | `D2`, `D5` |
| `PCL.076` | `D2`, `D5` |
