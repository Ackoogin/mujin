# Transport Capability Model — Supporting Heterogeneous Middleware

Status: proposed (planning). Raised 2026-06-26.

## Problem

PYRAMID runs components against the generic PCL runtime through runtime-loaded
**transport plugins** (one `.so` per middleware). Different middleware expose
**different interaction primitives**, and a plugin must be honest about which it
supports:

| Middleware | Native primitives |
|------------|-------------------|
| gRPC       | **RPC only** — unary + server-streaming (no native pub/sub) |
| ROS2       | **Mixed** — topics (pub/sub), services (unary RPC), actions (long-running goal/feedback/result/cancel) |
| DDS        | **Pub/sub first** — topics; RPC only via the optional DDS-RPC layer |
| MQTT       | **Pub/sub only** — topics, QoS 0/1/2 |
| UDP        | **Pub/sub only**, best-effort/lossy |
| TCP socket | **Mixed** (framed) — pub/sub + unary RPC (our impl) |
| shared mem | **Mixed** — pub/sub + unary + streaming RPC (our impl) |

A component's *contract* (the `.proto` data model) declares endpoints that each
need a specific primitive: a topic needs pub/sub, a service needs unary RPC, a
streaming service needs streaming RPC. Today the match between "what the contract
needs" and "what the transport provides" is **implicit** — it shows up only as a
`NULL` function pointer in the transport vtable and a runtime `PCL_ERR_NOT_FOUND`.
That is fragile: a misconfiguration surfaces late, as a confusing per-call error,
instead of early, as a clear "transport X cannot carry endpoint Y" failure.

This document plans a **capability model** so that:

1. Every transport plugin **declares** the primitives it supports.
2. The framework **validates**, at compose/configure time, that each routed
   endpoint's required primitive is supported by its transport — and **fails
   closed with a precise diagnostic** otherwise.
3. Deployments can **mix middleware per endpoint** (e.g. gRPC for services, DDS
   for telemetry topics) on one component.
4. Capability *gaps* are handled by an explicit policy (fail-closed by default;
   opt-in adapters only), never by silent emulation.

See also
[`subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
(plugin directionality, both-ways core contract) and the PCL transport ABI in
`subprojects/PCL/include/pcl/pcl_transport.h`.

## Where capability already lives

PCL already names the primitives — the model formalises what exists rather than
inventing new concepts.

- **Endpoint / port kinds** (`pcl_types.h`): `PUBLISHER`, `SUBSCRIBER`,
  `SERVICE`/`PROVIDED`, `CLIENT`/`CONSUMED`, `STREAM_SERVICE`/`STREAM_PROVIDED`.
- **Transport vtable** (`pcl_transport.h`): `publish`, `subscribe` (pub/sub);
  `serve`, `invoke_async`, `respond` (unary RPC); `invoke_stream`, `stream_send`,
  `stream_end`, `stream_cancel` (streaming RPC). Any pointer may be `NULL` =
  "not supported".

So a capability is just a **named bundle of vtable slots that an endpoint kind
requires**. The model gives that bundle a name, a declaration mechanism, and a
validation point.

## Capability taxonomy

Define a small, stable capability set. Each maps to (a) the endpoint kinds that
require it and (b) the vtable slots that implement it, in each direction.

| Capability | Endpoint kinds | Provider-side (ingress) | Consumer-side (egress) |
|------------|----------------|-------------------------|------------------------|
| `PUBSUB`        | PUBLISHER / SUBSCRIBER         | `subscribe` (+ deliver to executor) | `publish` |
| `RPC_UNARY`     | PROVIDED / CONSUMED (service)  | `serve` / server ingress + `respond` | `invoke_async` |
| `RPC_STREAM`    | STREAM_PROVIDED / streaming consume | server stream + `stream_send`/`stream_end` | `invoke_stream` (+ `stream_cancel`) |
| `RPC_ACTION`*   | (ROS2 actions; future)        | goal accept + feedback stream + result + cancel | goal send + feedback recv + cancel |

`*` `RPC_ACTION` is a composite (a long-running goal with a feedback stream, a
terminal result, and cancellation). It can be **modelled on top of** `RPC_STREAM`
+ `stream_cancel` rather than as a new vtable surface; flagged separately because
only ROS2 (and DDS-RPC with extensions) provides it natively.

Orthogonal **delivery-semantics / QoS** dimension (not a capability gate, but part
of the profile and validated where a contract demands it):

- reliability: `BEST_EFFORT` (UDP, MQTT-QoS0) vs `RELIABLE` (TCP, gRPC, DDS-reliable);
- ordering, durability/retain, multicast/fan-out, max message size.

A contract endpoint may carry a **minimum QoS requirement** (e.g. a command
service must be `RELIABLE`); routing it over a `BEST_EFFORT`-only transport is a
validation error, not a silent downgrade.

## Endpoint → required-capability mapping

Derived mechanically from the contract by the generator:

| Contract construct | Endpoint kind | Required capability |
|--------------------|---------------|---------------------|
| topic (proto topic option) | PUBLISHER / SUBSCRIBER | `PUBSUB` |
| unary `rpc`        | PROVIDED / CONSUMED | `RPC_UNARY` |
| server-streaming `rpc` | STREAM_PROVIDED / consume | `RPC_STREAM` |
| (future) action    | action provided / consumed | `RPC_ACTION` |

## Transport capability profiles (matrix)

Authoritative declaration lives **in each plugin** (see next section); this matrix
is the design intent and the test oracle.

| Transport | `PUBSUB` | `RPC_UNARY` | `RPC_STREAM` | `RPC_ACTION` | Reliability | Codec coupling |
|-----------|:---:|:---:|:---:|:---:|---|---|
| gRPC      | ✗ | ✓ | ✓ | ⟂ (via stream) | RELIABLE | **coupled** (protobuf) |
| ROS2      | ✓ | ✓ | ✓ | ✓ (native) | configurable QoS | **coupled** (ROS2 IDL / `application/ros2`) |
| DDS       | ✓ | ⟂ (DDS-RPC) | ⟂ | ✗ | configurable QoS | decoupled (CDR/codec) |
| MQTT      | ✓ | ✗ | ✗ | ✗ | QoS 0/1/2 | decoupled |
| UDP       | ✓ | ✗ | ✗ | ✗ | BEST_EFFORT | decoupled |
| TCP socket| ✓ | ✓ | ✗ (todo) | ✗ | RELIABLE | decoupled |
| shared mem| ✓ | ✓ | ✓ | ✗ | RELIABLE (intra-host) | decoupled |

✓ native · ⟂ available via an explicit adapter/extension · ✗ unsupported.

This matrix makes the heterogeneity explicit: **gRPC is RPC-only, UDP/MQTT/DDS are
pub/sub-first, ROS2/socket/shm are mixed.** A component whose contract has both
topics and services cannot be carried end-to-end by gRPC *or* by UDP alone — it
needs ROS2/socket/shm, or per-endpoint routing across two transports.

## Capability declaration & negotiation

**Today:** capability is inferred from `NULL` vtable slots — implicit and
un-introspectable before a call.

**Proposed:** make it explicit and queryable, without breaking the existing ABI.

- Add an optional exported symbol to the transport plugin ABI:
  `uint32_t pcl_transport_plugin_caps(const char* config_json)` returning a
  bitmask of `PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM |
  PCL_CAP_RPC_ACTION` plus QoS flags. `config_json` is honoured because a plugin's
  capability can depend on mode (e.g. the gRPC plugin in `mode:client` advertises
  `RPC_UNARY|RPC_STREAM` egress; in `mode:server` it advertises the ingress side).
- The loader exposes the mask via `pcl_plugin_load_transport` (out-param) or a
  `pcl_transport_caps(handle)` accessor.
- **Back-compat:** if the symbol is absent, the loader *derives* the mask from the
  non-NULL vtable slots (so existing plugins keep working). Declaration is the
  source of truth; derivation is the fallback.

This keeps the vtable unchanged (NULL still means "not implemented") while giving
the framework a cheap, pre-call capability picture for validation and diagnostics.

## Compose-time validation (fail closed, early, precise)

A new validation pass runs when transports are bound to an executor (at configure,
before `spin`). For each routed endpoint:

1. Resolve the endpoint's required capability (from its kind) and any QoS floor.
2. Resolve the transport(s) it is routed to (default transport or named peers).
3. Assert each routed transport's declared mask satisfies the requirement.
4. On mismatch, fail with a **precise** message:
   `endpoint "object_of_interest.create_requirement" needs RPC_UNARY but peer
   "telemetry" (udp) provides {PUBSUB}` — not a late per-call `PCL_ERR_NOT_FOUND`.

This is the transport analogue of the codec registry's existing **fail-closed**
rule. It turns "wrong middleware for this contract" into a startup error a
deployment engineer can act on.

## Per-endpoint routing & mixed-middleware deployments

PCL already supports this substrate — the model leans on it:

- `pcl_executor_register_transport(e, peer_id, vtable)` registers **named peer
  transports**; `pcl_executor_set_endpoint_route` binds an endpoint to one or more
  `peer_ids` with `LOCAL`/`REMOTE` modes.
- So a single component can route its **services over gRPC** and its **telemetry
  topics over DDS/MQTT** by loading two transport plugins and routing each
  endpoint to the appropriate peer.
- Validation (above) runs per route, so each endpoint is checked against the
  transport it actually uses — not a single global transport.

The generator/manifest should make this declarative: a deployment manifest maps
`endpoint -> peer_id`, and `peer_id -> {plugin .so, config_json}`.

## Coupled vs decoupled codecs

Capability is about *primitives*; the wire **format** is a second axis.

- **Coupled** middleware own their serialization: gRPC ⇒ protobuf, ROS2 ⇒ ROS2
  IDL. Their coupled plugins carry a content type (`application/grpc`,
  `application/ros2`) and the codec is either pass-through (bytes already in the
  native format) or, preferably, marshals the frozen `pyramid_<T>_c` struct into
  the native wire format inside the plugin (see the gRPC shim-retirement plan in
  `doc/todo/PYRAMID/TODO.md`).
- **Decoupled** middleware (socket/shm/udp/DDS/MQTT) carry opaque bytes; the wire
  format is a separately selected **codec plugin** (`application/json`,
  `application/flatbuffers`, `application/protobuf`).

The capability model and the codec model compose: a route is
`(endpoint) -> (transport plugin) [+ (codec plugin) if decoupled]`, and both the
transport-capability check and the codec fail-closed check apply.

## Capability-gap policy

When a contract needs a primitive the chosen transport lacks:

- **Default: fail closed.** Do not silently emulate. (e.g. do not fake RPC over a
  UDP topic, or pub/sub over a gRPC stream, behind the user's back.)
- **Opt-in adapters**, where they make engineering sense, are **explicit and
  named** so the wire behaviour is visible and auditable:
  - `PUBSUB over RPC_STREAM` — a server-streaming subscription (a subscriber opens
    a long-lived stream; publishes are stream frames). Lets gRPC carry telemetry.
  - `RPC_UNARY over PUBSUB` — request/reply with correlation IDs over two topics.
    Lets a pub/sub-only bus carry services (at the cost of correlation/timeout
    handling).
  An adapter advertises the *derived* capability in its mask, so validation passes
  only because the adapter is explicitly present.

## ROS2 specifics

ROS2 is the canonical **mixed** middleware and the reference for the model:

- **Topics → `PUBSUB`.** The coupled ROS2 plugin already wires `publish` /
  `subscribe` to `RclcppRuntimeAdapter`.
- **Services → `RPC_UNARY`.** Provided side advertised via the plugin-private
  `advertise_unary` symbol; the **consumed/client side (`invoke_async`) is still a
  gap** (tracked in `doc/todo/PYRAMID/TODO.md`) — closing it is required for the
  both-ways core contract.
- **Stream services → `RPC_STREAM`** via `advertise_stream`.
- **Actions → `RPC_ACTION`.** Not yet mapped. Model an action as goal (unary-ish
  request) + feedback (`RPC_STREAM`) + result (terminal) + cancel
  (`stream_cancel`). Decide whether to expose actions as a first-class contract
  construct or compose them from stream + cancel.

ROS2's QoS (reliability/durability/history) is the prime example of the QoS
dimension: the profile must carry it, and command-style endpoints should pin
`RELIABLE`.

## Generator & manifest responsibilities

- **Generator** emits, per endpoint, its kind and required capability + QoS floor
  (already implicit in the facade; make it explicit metadata the validator reads).
- **Manifest** maps endpoints → peer_ids → `{plugin, config_json}`, enabling
  mixed-middleware deployments and feeding the validation pass.
- **Plugins** export `pcl_transport_plugin_caps` (with NULL-slot derivation as
  fallback).

## Staged implementation plan

1. **Declare capabilities.** ✅ **Done** — `PCL_CAP_*` flags +
   `pcl_transport_caps_from_vtable` in [`pcl_capabilities.h`](../../../subprojects/PCL/include/pcl/pcl_capabilities.h);
   optional `pcl_transport_plugin_caps` symbol (`PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL`)
   in `pcl_plugin.h`; loader accessor `pcl_plugin_transport_caps()` returns the
   declared mask or derives it from non-NULL vtable slots when the symbol is
   absent. Covered by `test_pcl_capabilities`. Pure introspection — no behaviour
   change.
2. **Audit existing plugins** against the matrix; add the caps symbol to gRPC,
   ROS2, socket, shm, UDP and assert the matrix in tests. ✅ **Done** — all five
   declare caps (gRPC `RPC_UNARY|RPC_STREAM`, ROS2
   `PUBSUB|RPC_UNARY|RPC_STREAM`, socket `PUBSUB|RPC_UNARY`, shm
   `PUBSUB|RPC_UNARY|RPC_STREAM`, udp `PUBSUB`), asserted in
   `test_pcl_capabilities` / the coupled-plugin load tests. The coupled vtables
   carry fail-closed stubs, so their explicit declarations correct what
   derivation would mis-report.
3. **Compose-time validation** pass: endpoint required-capability vs routed
   transport mask, fail closed with precise diagnostics. ✅ **Done** —
   `pcl_endpoint_required_caps(kind)` maps endpoint kind → required cap;
   `pcl_transport_caps_supports(have, required)`; `pcl_executor_validate_endpoint_route()`
   checks each remote peer's recorded caps and fails closed (`PCL_ERR_NOT_FOUND`
   for a missing peer transport, `PCL_ERR_STATE` for a missing capability) with a
   precise `diag` string. Caps are recorded per transport: `set_transport`/
   `register_transport` derive from the vtable, and new `*_caps` variants accept
   the plugin's **declared** mask (authoritative for coupled plugins whose vtables
   carry fail-closed stubs). Positive + negative tests in `test_pcl_capabilities`.
   *Not yet wired into an automatic compose step* — it is a callable pass that the
   manifest-routing layer (stage 5) and generated bring-up will invoke.
4. **QoS floor** carried on endpoints + validated (start with reliability).
5. **Manifest-driven per-endpoint routing** across multiple transport plugins;
   one mixed-middleware e2e (e.g. services/gRPC + topics/udp on one component).
6. **Close ROS2 consumed (`invoke_async`)** so ROS2 satisfies the both-ways core.
7. **Adapters (opt-in)** for `PUBSUB over RPC_STREAM` and/or `RPC_UNARY over
   PUBSUB`, each advertising the derived capability, behind explicit config.
8. **Actions** (`RPC_ACTION`) for ROS2 if/when required.

## Open decisions

Decisions are tracked centrally in the WIP doc's decisions table
([`transport_plugins_wip.md`](transport_plugins_wip.md) §3) — the capability-model
ones are D2 (ROS2 consumed scope), D3 (`RPC_ACTION` first-class vs composed),
D4 (QoS placement), D5 (adapters in v1), and D6 (unify the `mode`/`role` config
convention across plugins).
