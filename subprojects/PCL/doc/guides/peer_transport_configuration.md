# Peer and Transport Configuration Guide

This guide describes how to configure local and remote routing for PCL endpoints at startup.

It reflects the current API shape in:

- `pcl_port_set_route(...)`
- `pcl_executor_set_endpoint_route(...)`
- `pcl_executor_register_transport(...)`
- `pcl_executor_set_transport(...)`

The key design rule is:

- `provided` and `consumed` describe interface role, not locality
- locality is chosen explicitly per endpoint at startup

## 1. Concepts

PCL now supports three routing modes for endpoints:

- `PCL_ROUTE_LOCAL`
  endpoint is reachable only within the local executor
- `PCL_ROUTE_REMOTE`
  endpoint is reachable only through one or more peer transports
- `PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE`
  endpoint is reachable both locally and remotely

There are two kinds of route attachment:

- concrete port route
  used for publisher, subscriber, provided-service, and stream-service ports created on a container
- executor endpoint route
  used for consumed/client endpoints where there is no concrete local server port object

## 2. What Gets Configured Where

Use `pcl_port_set_route(...)` for endpoints created with:

- `pcl_container_add_publisher(...)`
- `pcl_container_add_subscriber(...)`
- `pcl_container_add_service(...)`
- `pcl_container_add_stream_service(...)`

Use `pcl_executor_set_endpoint_route(...)` for:

- consumed unary services invoked via `pcl_executor_invoke_async(...)`

Use `pcl_executor_register_transport(...)` for:

- registering a transport instance under a logical peer name such as `"planner"` or `"bridge_b"`

Use `pcl_executor_set_transport(...)` only for:

- a default legacy transport
- a default remote path when a remote endpoint route does not name a peer explicitly

## 3. Peer Identity Model

Each remote peer is identified by a short logical name, for example:

- `"planner"`
- `"executor_b"`
- `"northbound_bridge"`

That name is the join key between:

- your deployment config
- `pcl_executor_register_transport(...)`
- any route peer list
- socket transport peer identity via `pcl_socket_transport_set_peer_id(...)`

Recommended rule:

- choose stable logical peer IDs that describe the remote executor role, not the host or port

For example, prefer:

- `"mission_bridge"`

instead of:

- `"10.10.4.22:9001"`

## 4. Startup Configuration Pattern

The intended startup flow is:

1. create the executor
2. create and initialize any transport instances
3. register each remote transport under a peer ID
4. configure each container
5. create ports during `on_configure`
6. apply route config to each concrete port
7. apply executor-level route config for consumed endpoints
8. activate containers
9. add containers to the executor
10. spin

In code terms, the route decision is made before traffic starts flowing.

## 5. Route Semantics by Endpoint Kind

### Publisher

Configured on the publisher port with `pcl_port_set_route(...)`.

- local route:
  published message is dispatched only to matching subscribers in the same executor
- remote route:
  published message is sent only to the configured peer transport(s)
- local+remote route:
  published message is delivered locally and also sent to configured peers

### Subscriber

Configured on the subscriber port with `pcl_port_set_route(...)`.

- local route:
  accepts only local executor delivery
- remote route:
  accepts only remote ingress from allowed peers
- local+remote route:
  accepts both local and remote ingress

If a remote subscriber lists peers, only those peers may deliver to it.

### Provided Service

Configured on the service port with `pcl_port_set_route(...)`.

- local route:
  callable only by local containers in the same executor
- remote route:
  callable only from remote peers allowed by the route
- local+remote route:
  callable from both local and remote callers

This is the normal way to expose `*.provided` locally, remotely, or both.

### Consumed Service

Configured on the executor with `pcl_executor_set_endpoint_route(...)`.

- local route:
  `pcl_executor_invoke_async(...)` resolves the service locally
- remote route:
  `pcl_executor_invoke_async(...)` routes the call through the named peer transport

For consumed unary services, use exactly one remote peer in v1.

Important current rule:

- consumed endpoints do not accept `local+remote`
- choose one explicit route per consumed service

This avoids ambiguous fallback behavior.

### Streaming Provided Service

Configured on the stream service port with `pcl_port_set_route(...)` the same way as unary provided services.

## 6. Validation Rules

PCL currently enforces these checks:

- route mode must not be `PCL_ROUTE_NONE`
- peer count must not exceed `PCL_MAX_ENDPOINT_PEERS`
- peer IDs may only be supplied when `PCL_ROUTE_REMOTE` is set
- subscriber/provided port peer lists must not contain null peer IDs
- consumed endpoint routes must not use `PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE`

Practical meaning:

- local-only routes should use `peer_ids = NULL` and `peer_count = 0`
- remote-only consumed routes should name exactly one peer
- local+remote is for concrete ports, not consumed unary client routes

## 7. Concrete Examples

### Example A: Local `provided`, Remote `consumed`

One container exposes a local service and calls another service on peer `"planner"`.

```c
static pcl_status_t on_configure(pcl_container_t* self, void* ud) {
  pcl_port_t* service = pcl_container_add_service(
      self, "local.status", "StatusReq", handle_status, ud);
  if (!service) return PCL_ERR_NOMEM;

  return pcl_port_set_route(service, PCL_ROUTE_LOCAL, NULL, 0);
}

void configure_executor_routes(pcl_executor_t* exec) {
  const char* peers[] = {"planner"};
  pcl_endpoint_route_t route = {
      .endpoint_name = "planner.solve",
      .endpoint_kind = PCL_ENDPOINT_CONSUMED,
      .route_mode = PCL_ROUTE_REMOTE,
      .peer_ids = peers,
      .peer_count = 1,
  };
  pcl_executor_set_endpoint_route(exec, &route);
}
```

### Example B: Remote `provided`, Local `consumed`

The same executor exposes a service to remote peer `"bridge_a"` but keeps its own consumed dependency local.

```c
static pcl_status_t on_configure(pcl_container_t* self, void* ud) {
  const char* peers[] = {"bridge_a"};
  pcl_port_t* service = pcl_container_add_service(
      self, "track.update", "TrackReq", handle_track_update, ud);
  if (!service) return PCL_ERR_NOMEM;

  return pcl_port_set_route(service, PCL_ROUTE_REMOTE, peers, 1);
}

void configure_executor_routes(pcl_executor_t* exec) {
  pcl_endpoint_route_t route = {
      .endpoint_name = "local.lookup",
      .endpoint_kind = PCL_ENDPOINT_CONSUMED,
      .route_mode = PCL_ROUTE_LOCAL,
      .peer_ids = NULL,
      .peer_count = 0,
  };
  pcl_executor_set_endpoint_route(exec, &route);
}
```

### Example C: Local + Remote Publisher Fanout

One publisher sends the same topic to local subscribers and peer `"telemetry"`.

```c
static pcl_status_t on_configure(pcl_container_t* self, void* ud) {
  const char* peers[] = {"telemetry"};
  pcl_port_t* pub = pcl_container_add_publisher(
      self, "state/topic", "StateMsg");
  if (!pub) return PCL_ERR_NOMEM;

  return pcl_port_set_route(pub, PCL_ROUTE_LOCAL | PCL_ROUTE_REMOTE, peers, 1);
}
```

### Example D: Remote Subscriber Allow-List

Only peer `"bridge_b"` may inject remote traffic into the subscriber.

```c
static pcl_status_t on_configure(pcl_container_t* self, void* ud) {
  const char* peers[] = {"bridge_b"};
  pcl_port_t* sub = pcl_container_add_subscriber(
      self, "intel/topic", "IntelMsg", on_intel, ud);
  if (!sub) return PCL_ERR_NOMEM;

  return pcl_port_set_route(sub, PCL_ROUTE_REMOTE, peers, 1);
}
```

## 8. Socket Transport Peer Setup

The socket transport itself is still one connection per transport object.

So for multiple peers, create one socket transport per peer and register each one separately.

Example:

```c
pcl_socket_transport_t* to_planner =
    pcl_socket_transport_create_client("127.0.0.1", 7001, exec);
pcl_socket_transport_t* to_bridge =
    pcl_socket_transport_create_client("127.0.0.1", 7002, exec);

pcl_socket_transport_set_peer_id(to_planner, "planner");
pcl_socket_transport_set_peer_id(to_bridge, "bridge_b");

pcl_executor_register_transport(
    exec, "planner", pcl_socket_transport_get_transport(to_planner));
pcl_executor_register_transport(
    exec, "bridge_b", pcl_socket_transport_get_transport(to_bridge));
```

For server mode, set the same logical peer ID that your route tables expect for the connected peer:

```c
pcl_socket_transport_t* server =
    pcl_socket_transport_create_server(7001, exec);
pcl_socket_transport_set_peer_id(server, "planner_client");
```

That peer ID is what PCL uses for remote ingress filtering.

### 8.1. Robust Client Connect (retry, state callback, auto-reconnect)

The legacy `pcl_socket_transport_create_client(...)` is a **single-shot**
connect: if the server is not yet listening, it returns NULL and the
caller has no transport.  For production deployments where start order
is not guaranteed and peers may restart, use
`pcl_socket_transport_create_client_ex(...)` with a
`pcl_socket_client_opts_t`:

| Field | Meaning |
|-------|---------|
| `connect_timeout_ms` | Total deadline for the initial connect. `0` = unlimited (if `max_retries` > 0) or single-shot (if `max_retries` == 0). |
| `max_retries` | Maximum retry attempts on refused/unreachable. `0` = no retry. |
| `auto_reconnect` | Non-zero = recv_thread transparently reconnects after a drop using the same backoff policy. |
| `state_cb` | Optional `pcl_socket_state_cb_t` fired on every state transition. |
| `state_cb_data` | Opaque pointer passed to `state_cb`. |

Connection retries use exponential backoff starting at 100 ms and
doubling up to a 2 s cap.  TCP keepalive is enabled on every connected
socket so silent peer death is detected within ~8 s on Linux.

Example: planner client that survives planner restarts.

```c
static void on_planner_state(pcl_socket_state_t state, void* ud) {
  (void)ud;
  switch (state) {
    case PCL_SOCKET_STATE_CONNECTING:    log_info("planner: connecting");    break;
    case PCL_SOCKET_STATE_CONNECTED:     log_info("planner: up");            break;
    case PCL_SOCKET_STATE_DISCONNECTED:  log_warn("planner: link dropped");  break;
  }
}

pcl_socket_client_opts_t opts = {
    .connect_timeout_ms = 30000,  // up to 30 s for the planner to boot
    .max_retries        = 1000,
    .auto_reconnect     = 1,      // keep reconnecting after drops
    .state_cb           = on_planner_state,
    .state_cb_data      = NULL,
};

pcl_socket_transport_t* to_planner =
    pcl_socket_transport_create_client_ex("127.0.0.1", 7001, exec, &opts);
if (!to_planner) {
  // Only returned as NULL if initial connect exhausted all retries.
  fatal("planner unreachable");
}
pcl_socket_transport_set_peer_id(to_planner, "planner");
```

### 8.2. Semantics During Disconnect

While the client is reconnecting (`state == PCL_SOCKET_STATE_DISCONNECTED`
or `PCL_SOCKET_STATE_CONNECTING`):

- **Publish** frames are dequeued by `send_thread` and silently dropped
  (pub/sub best-effort semantics; the wire is simply unavailable).
- **Async service calls** remain on the pending list but will not receive
  a response — application code should apply its own timeouts.
- **Inbound messages** cannot arrive until the link is re-established.

Applications that need stronger guarantees should gate calls on
`pcl_socket_transport_get_state(...)` or act on the state callback.

### 8.3. Zero-Change Upgrade Path

`pcl_socket_transport_create_client(host, port, exec)` is preserved as a
thin wrapper around `create_client_ex(..., NULL)` and retains the exact
legacy behaviour: single attempt, fail-fast on refused, no reconnect,
no callbacks.  Existing call sites need no changes.

## 8.4. UDP Datagram Transport

For high-rate telemetry and state broadcasts where occasional loss is
acceptable and the framing overhead of TCP is not, PCL also ships a
UDP datagram transport in `pcl/pcl_transport_udp.h`.  It implements the
same `pcl_transport_t` vtable but deliberately exposes only pub/sub —
`invoke_async`, `respond`, `serve`, and `invoke_stream` are all NULL,
so the executor returns `PCL_ERR_NOT_FOUND` for unsupported RPC calls
rather than silently accepting unreliable service traffic.

Wire format (single datagram — UDP preserves boundaries, so no
length prefix is needed):

```
[1:type=PUBLISH=0x00][2:topic_len][topic]
[2:type_len][type_name][4:data_len][data]
```

Each `pcl_udp_transport_t` instance binds one local UDP port and
publishes to one configured remote peer.  For multi-peer fan-out,
instantiate one UDP transport per peer and register each separately
with `pcl_executor_register_transport(...)`.

Example — telemetry sender and receiver on the same host:

```c
/* Receiver */
pcl_udp_transport_t* rx =
    pcl_udp_transport_create(9100, "127.0.0.1", 9200, rx_exec);
pcl_udp_transport_set_peer_id(rx, "sensor_head");
pcl_executor_register_transport(
    rx_exec, "sensor_head", pcl_udp_transport_get_transport(rx));

/* Sender (on another process) */
pcl_udp_transport_t* tx =
    pcl_udp_transport_create(9200, "127.0.0.1", 9100, tx_exec);
pcl_udp_transport_set_peer_id(tx, "collector");
pcl_executor_set_transport(tx_exec, pcl_udp_transport_get_transport(tx));
```

### Constraints

- Pub/sub only — use the TCP transport for service RPC and streaming.
- Payload cap of 1400 bytes per published message (below the standard
  Ethernet MTU so IP fragmentation is rare); oversized publishes return
  `PCL_ERR_NOMEM`.
- Best-effort delivery — application code must tolerate drops, or wrap
  the channel with its own sequencing/retransmit layer.
- IPv4 only in v1; IPv6 support is a straightforward extension.

## 9. Bridge and Chained Topologies

PCL does not do automatic multi-hop forwarding.

For a chain like:

- Executor A <-> Bridge B <-> Executor C

the bridge behavior must be explicit:

- a container in B consumes one endpoint
- its business logic republishes or reinvokes another endpoint

Recommended pattern:

- treat each bridge side as an ordinary routed endpoint
- keep forwarding logic inside a normal container callback or service handler
- register transports in B for both peers

Example bridge layout:

- peer `"left"` registered on B
- peer `"right"` registered on B
- subscriber on `bridge.in` accepts remote from `"left"`
- publisher on `bridge.out` routes remote to `"right"`

This keeps chain behavior visible and deterministic.

## 10. Suggested Static Config Shape

PCL does not currently ship a config file parser for routing, but the intended deployment model is a small startup table like:

```c
typedef struct {
  const char* peer_id;
  const char* host;
  uint16_t    port;
  bool        is_server;
} peer_cfg_t;

typedef struct {
  const char*         endpoint_name;
  pcl_endpoint_kind_t endpoint_kind;
  uint32_t            route_mode;
  const char* const*  peer_ids;
  uint32_t            peer_count;
} endpoint_cfg_t;
```

Recommended separation:

- peer table
  declares available remote links
- endpoint table
  declares locality and peer selection

This keeps deployment decisions out of business logic.

## 11. Recommended Naming and Conventions

- use stable peer IDs that represent remote roles
- keep service and topic names identical across local and remote deployments
- prefer exact per-endpoint routing over broad wildcard policy in v1
- default unconfigured concrete ports to local behavior
- default unconfigured consumed routes to legacy executor transport behavior only when intentionally using `pcl_executor_set_transport(...)`

## 12. Troubleshooting

If a remote call returns `PCL_ERR_NOT_FOUND`, check:

- the consumed endpoint route names the correct peer
- that peer was registered with `pcl_executor_register_transport(...)`
- the transport supports the required vtable entry such as `invoke_async`

If remote ingress is dropped, check:

- the subscriber or provided service includes `PCL_ROUTE_REMOTE`
- the source peer ID matches the allow-list
- the socket transport peer ID was set with `pcl_socket_transport_set_peer_id(...)`

If a publisher does not fan out remotely, check:

- the publisher route includes `PCL_ROUTE_REMOTE`
- any named peers exist in the executor transport registry
- for unnamed remote fallback, a default transport was set with `pcl_executor_set_transport(...)`

## 13. Current Limitations

- consumed unary service routes are explicit local or explicit remote, not local+remote
- the reference socket transport is still one connection per transport object (auto-reconnect refreshes the same peer endpoint; multi-client fan-in still requires one server transport per peer)
- multi-hop routing is not automatic
- route configuration is API-driven today, not file-driven in core PCL
- the UDP transport is pub/sub only — service RPC (`invoke_async`, `respond`) and streaming services are not supported over UDP; use the socket (TCP) transport for those

Those are deliberate v1 constraints to keep routing behavior explicit and testable.
