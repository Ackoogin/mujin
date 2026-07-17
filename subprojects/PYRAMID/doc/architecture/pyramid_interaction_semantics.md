# PYRAMID Interaction Semantics

This page defines how interaction patterns — publish, subscribe, unary RPC,
server-streaming RPC, and (reserved) actions — are expressed **inside the
`.proto` contract**, so that every transport projection (PCL, gRPC, ROS2, …)
derives the full interaction surface from one source of truth.

It is the reference for the contract convention. The parser
(`pim/proto_parser.py`), the generator (`pim/cpp/`, `pim/ada/`), and the
transport projections ([generated_bindings.md](generated_bindings.md),
[ros2_transport_semantics.md](ros2_transport_semantics.md)) implement it.
Developer-facing usage of the pub/sub surface and the interaction facade is in
the [pub/sub & interaction facade guide](../guides/pubsub_interaction_guide.md).

## Design Principle

The `.proto` contract is the single source of truth for **payload meaning and
interaction pattern**. A transport projects the contract; it never redefines
the pattern and never carries pattern information the contract lacks.

Concretely, given only the `.proto`, the generator can answer for every
operation: *is this publish, subscribe, unary, server-stream, client-stream,
bidi, or action — and on what topic, with what QoS?* Topics are never read
from out-of-band lookup tables (the one frozen legacy exception is scoped
below).

## Pattern Resolution — Two Layers

Pattern is resolved in two layers. The signature layer keeps the common case
decoration-free; the annotation layer is authoritative and removes ambiguity.

### Layer 1 — signature convention (the default)

The signature shape sets the **inferred** pattern:

| request | response | inferred pattern |
|---------|----------|------------------|
| `T` (non-empty) | `U` (non-empty) | unary RPC |
| `T` | `stream U` | server-streaming RPC |
| `stream T` | `U` | client-streaming RPC |
| `stream T` | `stream U` | bidi RPC |
| `T` | `Empty` | **publish** |
| `Empty` | `stream T` | **subscribe** |

In addition, `classify_port_service()` recognises the two **port-grammar**
service shapes at service scope (see below), which resolves the
`Empty → stream T` shape unambiguously for grammar-conforming services.

### Layer 2 — method option (the source of truth)

The proto custom option `pyramid.options.pyramid_op`
(`proto/pyramid/options/pyramid.options.proto`, shipped as part of the
contract package) is the authoritative layer. It carries what a bare
signature cannot: the **topic wire-name** and **QoS**.

```proto
extend google.protobuf.MethodOptions {
  pyramid.options.Interaction pyramid_op = 50001;
}

message Interaction {
  Pattern pattern = 1;   // overrides Layer-1 inference; INFER defers to signature
  string  topic   = 2;   // wire name for PUBLISH/SUBSCRIBE, e.g. "agra.ma_action.request"
  Qos     qos     = 3;   // reliability / durability / history depth
}

enum Pattern { INFER = 0; UNARY = 1; SERVER_STREAM = 2; CLIENT_STREAM = 3;
               BIDI = 4; PUBLISH = 5; SUBSCRIBE = 6; ACTION = 7; }
```

Resolution rule: `pattern == INFER` (or no option) ⇒ use Layer 1; any other
value ⇒ use the option and ignore the inference.

Per-method signature inference is ambiguous in exactly two places, so for
**free-form (hand-written) services** an explicit option is required there:

- `rpc Foo(T) returns (Empty)` — a publish, or a void unary command.
  Annotate `UNARY` or `PUBLISH`.
- `rpc Sub(Empty) returns (stream T)` — a subscribe, or a no-argument
  server-streaming RPC. Not ambiguous inside the port grammar (the sole-rpc
  `Read(Empty) → stream T` service is the canonical Information port); for
  free-form services, annotate.

The field number 50001 sits in the internal-use extension range
(50000–99999) so it cannot collide with a globally-registered extension.

## Canonical Pattern Matrix

| Pattern | request | response | option needed? | topic? |
|---------|---------|----------|----------------|--------|
| Unary RPC | `T` | `U` | no | no |
| Void command | `T` | `Empty` | **yes** (`UNARY`) | no |
| Server-stream RPC | `T` | `stream U` | no | no |
| Client-stream RPC | `stream T` | `U` | no | no |
| Bidi RPC | `stream T` | `stream U` | no | no |
| Publish | `T` | `Empty` | `PUBLISH` + `topic` | yes |
| Subscribe | `Empty` | `stream T` | `SUBSCRIBE` + `topic` | yes |
| Action | goal | feedback + result | **yes** (`ACTION`) | n/a (reserved) |

## The Port Grammar

Generated (MBSE) contract trees follow a strict **port grammar**: every
service is one of two shapes, classified at service scope by
`classify_port_service()`:

- **Request port** — exactly `Create` / `Read` / `Update` / `Cancel` with
  fixed signatures (`Read(Query) → stream T`, `Ack` responses, `Cancel(Identifier)`).
- **Information port** — a single `Read(Empty) → stream T` rpc.

Within the grammar, patterns and topics are **derivable mechanically**, and
the MBSE proto generator (`pim/mbse/proto_generator.py`) stamps the Layer-2
options onto the generated services — topic-bearing methods are never
hand-authored in these trees. Topic names follow:

```
<project>.<interface_snake>.<role>      role ∈ { request, entity, information }
```

A Request port realized as pub/sub uses the **correlated pair**: a
`.request` topic (consumer → provider, the service's `_Request` wrapper
message) and a `.entity` topic (provider → consumer, status
transitions). Correlation is by `Entity.id` in the payload. An Information
port maps to a single `.information` topic. Semantics, legs, and the
provider/consumer API are in the
[pub/sub & interaction facade guide](../guides/pubsub_interaction_guide.md).

## Topic Modelling Rules

- **Topic identity is the wire name, not the method.** The `topic` option is
  the identity that a publish method and a subscribe method (in different
  components / packages) share. Two methods with the same `topic` are the two
  ends of one topic. The method name remains the binding/codegen handle.
- **Array topics use streaming element types, not repeated wrappers.** An
  array topic is `stream ElementType` in the `SUBSCRIBE` shape, so the
  element type stays the canonical data-model message and the generator emits
  the vector payload helper.

## Pattern Stamp = Default Realization

For grammar-conforming ports, the RPC and pub/sub projections of a port are
**declared, interchangeable realizations of one interaction**, selected per
directed leg at compose time:

- `binding_manifest.json` carries an `interactions` section grouping each
  leg's service endpoints with its projected topic endpoint (and per-command
  pub/sub projectability);
- PCL routing manifests enforce one-realization-per-leg via `exclusive`
  groups (fail closed at `pcl_transport_routing_load()`);
- the `pattern` stamp serves as the **default realization** for the leg.

See the [guide](../guides/pubsub_interaction_guide.md) §5 for choosing the
realization at deployment, and
[generic_contract_layout.md](generic_contract_layout.md) for the manifest
schema.

## QoS

Contract QoS (`Interaction.qos`) is **intent** — a floor (e.g. a command
topic pins `RELIABLE`). Each transport plugin declares **capability** — a
ceiling. Compose-time validation reconciles the two and fails closed on a
mismatch (see
[transport_codec_plugin_system.md](transport_codec_plugin_system.md)).
QoS reliability floors are emitted into binding manifests and the generated
C++ topic constants.

## Transport Projection

Each projection consumes the resolved pattern uniformly:

| Pattern | PCL | gRPC | ROS2 |
|---------|-----|------|------|
| Unary / void | service request/response | unary rpc | typed `.srv` pending; envelope `srv/PclService` today |
| Server-stream | streamed service | server-streaming rpc | open service + `frames` topic (envelope) |
| Client-stream / bidi | (future) | native streaming rpc | (future) |
| Publish / subscribe | port pub/sub on topic wire-name | n/a (or wrapped) | typed `pyramid_msgs` message on mapped topic (default); envelope fallback for array topics |
| Action | (future) | n/a | reserved (`ACTION`); unimplemented |

### Direct (non-PCL) consumer compatibility

An endpoint that knows nothing about PCL can use the contract as-is, to a
transport-dependent degree:

- **gRPC — works as-is.** The annotation is a plain proto3
  `MethodOptions` extension; `protoc` and the gRPC plugins ignore options
  they do not consume, so stubs generate normally. Because pub/sub is
  modelled *as rpc*, a stock gRPC client gets a directly usable surface: a
  `PUBLISH` method is an ordinary unary call, a `SUBSCRIBE` method an
  ordinary server-streaming call. The only requirement is that
  `pyramid/options/pyramid.options.proto` is on the `protoc` import path
  (the same dependency shape as `google/api/annotations.proto`).
- **ROS2 — typed topics work as-is; services do not yet.** The contract
  projects to native ROS2 IDL: `pim/ros2_idl_codegen.py` emits the
  `pyramid_msgs` ament package (`.msg`/`.srv`) and
  `pim/ros2_marshal_codegen.py` emits the `domain_model` ↔ `pyramid_msgs`
  marshalling and `rclcpp` wire codec. The default live wire for topics is
  the typed `pyramid_msgs` message (a plain `rclcpp` node interoperates with
  no PYRAMID framing). Array topics and unary/stream **service framing**
  still use the generic envelope (`PclEnvelope` / `PclService` /
  `PclOpenStream`), so a non-PCL node cannot consume PYRAMID services
  natively yet. `.action` is not generated. Details in
  [ros2_transport_semantics.md](ros2_transport_semantics.md).

## Legacy Topic Side-Table (frozen)

The Tactical Objects standard topics predate contract-derived topics and live
in a JSON side-table (`pim/topic_metadata/tactical_objects_topics.json` via
`pim/standard_topics.py`). This is **frozen legacy compatibility**:

- The generators consult it only for the legacy
  `pyramid.components.<component>.services.<provided|consumed>` layout, and
  only when the contract itself yields no topics
  (`TopicSpecResolver.topics_for_proto`). Contract-derived topics always win.
- Do not add topics to it and do not use it for new contract trees.

Full deletion remains available: stamp the legacy Tactical Objects contract
with `PUBLISH`/`SUBSCRIBE` options carrying the existing wire names, switch
the generator to contract-derived topics for that tree, and prove the
generated bindings unchanged by diff.

## Status

| Item | State |
|------|-------|
| Layer-1 signature inference (RPC streaming) | implemented in `proto_parser.py` |
| Port-grammar classifier (Layer-1, service scope) | implemented (`classify_port_service`) |
| Layer-2 method option (`pyramid.options.pyramid_op`) | implemented; parsed into `ProtoRpc.pattern/topic/qos`; MBSE generator stamps options mechanically |
| Contract-derived topics + QoS floors | implemented (manifests + generated C++ topic constants) |
| RPC ↔ pub/sub interchangeability (interaction facade) | implemented for grammar-conforming ports ([guide](../guides/pubsub_interaction_guide.md)) |
| Correlated pair over real transports | proven cross-process over SHM and UDP with contract-derived routing/QoS |
| gRPC as-is consumption | compatible by design |
| ROS2 native IDL projection | typed topic wire is the live default; service framing and array topics remain envelope; `.action` not generated |
| Legacy side-table | frozen compat, scoped to the legacy layout; deletion deferred |
| Actions (`ACTION` pattern) | reserved, unimplemented |

The executed plans behind this work (`pubsub_contract_generation_plan.md`,
`agra_pubsub_shm_udp_proving_plan.md`,
`rpc_pubsub_interchangeability_plan.md`) were retired in the 2026-07-10 doc
review; their design intents are summarised in
[`doc/plans/PYRAMID/README.md`](../../../../doc/plans/PYRAMID/README.md)
and their full text is in git history.
