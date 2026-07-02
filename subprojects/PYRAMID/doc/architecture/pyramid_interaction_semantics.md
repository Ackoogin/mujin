# PYRAMID Interaction Semantics

This page defines how interaction patterns — publish, subscribe, unary RPC,
server-streaming RPC, and (future) actions — are expressed **inside the
`.proto` contract**, so that every transport projection (PCL, gRPC, ROS2, …)
can derive the full interaction surface from one source of truth.

It is the design reference for the contract convention; the parser
(`pim/proto_parser.py`), the generator (`pim/cpp_codegen.py`,
`pim/ada_codegen.py`), and the transport projections
([generated_bindings.md](generated_bindings.md),
[ros2_transport_semantics.md](ros2_transport_semantics.md)) implement it.

> **Update (2026-07).** Two developments postdate the original proposal and
> change how it is applied:
>
> 1. **The MBSE-generated contract tree** (`pim/test/`, emitted by
>    `pim/mbse/proto_generator.py`) has a strict *port grammar*: every
>    service is either a Request port (`Create`/`Read`/`Update`/`Cancel`,
>    fixed shapes) or an Information port (`Read(Empty) → stream T`, sole
>    rpc). Within that grammar the patterns this doc annotates by hand are
>    **derivable mechanically**, and the Layer-2 options below are stamped
>    by the MBSE generator rather than authored. The signature-ambiguity
>    rules in this doc apply to *free-form* (hand-written) services only.
> 2. **The transport/codec plugin system** landed
>    ([transport_codec_plugin_system.md](transport_codec_plugin_system.md)):
>    capability-model validation (`PUBSUB`/`RPC_UNARY`/…), per-endpoint
>    routing manifests, and plugin QoS profiles. Contract QoS (below) is
>    *intent*; plugin QoS is *capability*; compose-time validation
>    reconciles them.
>
> The current plan of record for pub/sub generation — including topic
> naming, correlated request/requirement topic pairs, and the migration of
> this doc's proposals onto the new tree — is
> [doc/plans/PYRAMID/pubsub_contract_generation_plan.md](../../../../doc/plans/PYRAMID/pubsub_contract_generation_plan.md).

## The Problem

Today the contract carries only **half** of the interaction surface:

- **RPC is in the contract.** Each `rpc` is parsed in `proto_parser.py` and
  classified purely by signature: the `stream` keyword on the request/response
  sets `client_streaming` / `server_streaming`. Unary vs server-streaming is
  therefore already a *signature convention* derived from `.proto`.
- **Pub/sub is not in the contract.** Topics live in a hand-maintained
  side-table loaded by `pim/standard_topics.py`. The module itself is now
  data-driven (the Tactical Objects topic set lives in
  `pim/topic_metadata/tactical_objects_topics.json`, matched by declared
  `package_match` metadata rather than hardcoded package branching), but the
  generator still reads pub/sub topics from that metadata file, never from the
  `.proto`.

The consequence: transport projections faithfully reproduce whatever the
contract says, but the contract is **silent on topics**. So the heterogeneous
middleware mapping (the value of the ROS2 work) is driven for pub/sub by a
JSON lookup table that must be edited per component, out of band from the
contract it is supposed to project.

```mermaid
flowchart LR
  subgraph Today["Today (split source of truth)"]
    Proto1[".proto<br/>(RPC only)"] --> Gen1["generate_bindings.py"]
    Side["standard_topics.py<br/>(pub/sub side-table)"] --> Gen1
    Gen1 --> Out1["bindings + transport projections"]
  end
  subgraph Target["Target (single source of truth)"]
    Proto2[".proto<br/>(RPC + pub/sub + actions)"] --> Gen2["generate_bindings.py"]
    Gen2 --> Out2["bindings + transport projections"]
  end
```

## Design Principle

The `.proto` contract is the single source of truth for **payload meaning and
interaction pattern**. A transport projects the contract; it never redefines
the pattern and never carries pattern information the contract lacks.

Concretely, given only the `.proto`, the generator must be able to answer for
every operation: *is this publish, subscribe, unary, server-stream,
client-stream, bidi, or action — and on what topic, with what QoS?*

## Two-Layer Convention

Pattern is resolved in two layers. The signature layer keeps the common case
decoration-free; the annotation layer is authoritative and removes ambiguity.

### Layer 1 — signature convention (the default)

The signature shape sets the **inferred** pattern. This generalizes the rule
already in use for streaming and extends it to topics:

| request | response | inferred pattern |
|---------|----------|------------------|
| `T` (non-empty) | `U` (non-empty) | unary RPC |
| `T` | `stream U` | server-streaming RPC |
| `stream T` | `U` | client-streaming RPC |
| `stream T` | `stream U` | bidi RPC |
| `T` | `Empty` | **publish** |
| `Empty` | `stream T` | **subscribe** |

This is the `operation(type) -> Empty == publish` convention, made symmetric
with a `subscribe` shape. `google.protobuf.Empty` is already imported by the
service protos, so no new dependency is required for the convention itself.

### Layer 2 — method option (the source of truth)

Pure *per-method* signature inference is ambiguous in exactly two places,
both live in the current contract:

- `rpc Foo(T) returns (Empty)` — a **publish**, or a **void unary command**
  (e.g. a delete that does not ack)? Identical shapes.
- `rpc Sub(Empty) returns (stream T)` — a **subscribe**, or a
  **server-streaming RPC** that happens to take no argument? Identical shapes.

(In the MBSE port grammar the second case is *not* ambiguous: a service whose
sole rpc is `Read(Empty) → stream T` is the canonical Information port, and
the classifier resolves it at service scope. The ambiguity — and the
option-required rule below — applies to free-form services.)

So a proto custom option is the authoritative layer. It is the proto-idiomatic
mechanism (same pattern as `google.api.http` and gRPC's own method options),
it is machine-readable, and — critically — it carries the transport metadata
that a bare signature cannot: the **topic wire-name** and **QoS**.

```proto
// pyramid/options/pyramid.options.proto
syntax = "proto3";
package pyramid.options;
import "google/protobuf/descriptor.proto";

extend google.protobuf.MethodOptions {
  pyramid.options.Interaction pyramid_op = 50001;  // provisional field number
}

message Interaction {
  Pattern pattern = 1;   // overrides Layer-1 inference; INFER defers to signature
  string  topic   = 2;   // wire name for PUBLISH/SUBSCRIBE, e.g. "standard.entity_matches"
  Qos     qos     = 3;   // reliability / durability / history depth
}

enum Pattern {
  INFER         = 0;   // resolve from signature (Layer 1)
  UNARY         = 1;
  SERVER_STREAM = 2;
  CLIENT_STREAM = 3;
  BIDI          = 4;
  PUBLISH       = 5;
  SUBSCRIBE     = 6;
  ACTION        = 7;   // ROS2 goal / feedback / result (see below)
}

message Qos {
  Reliability reliability = 1;   // RELIABLE | BEST_EFFORT
  Durability  durability  = 2;   // VOLATILE | TRANSIENT_LOCAL
  uint32      depth       = 3;    // history depth; 0 = transport default
  enum Reliability { RELIABILITY_DEFAULT = 0; RELIABLE = 1; BEST_EFFORT = 2; }
  enum Durability  { DURABILITY_DEFAULT = 0; VOLATILE = 1; TRANSIENT_LOCAL = 2; }
}
```

Resolution rule: `pattern == INFER` (or no option) ⇒ use Layer 1; any other
value ⇒ use the option and ignore the inference. Where Layer 1 is ambiguous
(`-> Empty`, or `Empty -> stream`), the generator **requires** an explicit
option and fails the build otherwise, so an ambiguous shape can never silently
resolve the wrong way.

## Canonical Pattern Matrix

| Pattern | request | response | option needed? | topic? | notes |
|---------|---------|----------|----------------|--------|-------|
| Unary RPC | `T` | `U` | no | no | existing path |
| Void command | `T` | `Empty` | **yes** (`UNARY`) | no | else inferred as PUBLISH |
| Server-stream RPC | `T` | `stream U` | no | no | existing path |
| Client-stream RPC | `stream T` | `U` | no | no | |
| Bidi RPC | `stream T` | `stream U` | no | no | |
| Publish | `T` | `Empty` | recommended (`PUBLISH` + `topic`) | yes | topic name from option |
| Subscribe | `Empty` | `stream T` | recommended (`SUBSCRIBE` + `topic`) | yes | topic name from option |
| Action | goal | feedback + result | **yes** (`ACTION`) | n/a | ROS2 action mapping |

Worked examples (illustrative, legacy/free-form style — `Tactical_Objects_Topics`
is not an existing service; in the MBSE tree, topic-bearing methods are never
hand-authored, the options are stamped onto the port services at generation
time):

```proto
service Object_Of_Interest_Service {
  // Unary — inferred, no option.
  rpc CreateRequirement(tactical.ObjectInterestRequirement)
      returns (base.Identifier);

  // Void command — Empty response is ambiguous, so annotate.
  rpc DeleteRequirement(base.Identifier) returns (google.protobuf.Empty) {
    option (pyramid.options.pyramid_op) = { pattern: UNARY };
  }
}

service Tactical_Objects_Topics {
  // Publish — topic name and QoS come from the contract, not standard_topics.py.
  rpc PublishObjectEvidence(tactical.ObjectDetail) returns (google.protobuf.Empty) {
    option (pyramid.options.pyramid_op) = {
      pattern: PUBLISH
      topic: "standard.object_evidence"
      qos: { reliability: RELIABLE durability: VOLATILE depth: 10 }
    };
  }

  // Subscribe — array payload via stream of the element type.
  rpc SubscribeEntityMatches(google.protobuf.Empty)
      returns (stream tactical.ObjectMatch) {
    option (pyramid.options.pyramid_op) = {
      pattern: SUBSCRIBE
      topic: "standard.entity_matches"
    };
  }
}
```

## Topic Modelling Notes

Pub/sub is many-to-many and decoupled, while an `rpc` lives inside a `service`
and reads as point-to-point. Two rules keep the contract honest:

- **Topic identity is the wire name, not the method.** The `topic` option is the
  identity that `PublishX` and `SubscribeX` (in different components / packages)
  share. Two methods with the same `topic` are the two ends of one topic. The
  method name remains the binding/codegen handle.
- **Array topics use streaming element types, not repeated wrappers.** Today
  `standard.entity_matches` is `std::vector<ObjectMatch>` via `is_array` in the
  topic spec. Under this convention an array topic is `stream ElementType` in
  the `SUBSCRIBE` shape, so the element type stays the canonical data-model
  message and the generator keeps emitting the vector payload helper.

## Migration: Retire `standard_topics.py` (legacy tree only)

This migration applies to the **legacy hand-written tree** (the Tactical
Objects contract). The MBSE-generated tree never uses the side-table: its
topics are stamped as options at generation time (see the plan of record).
Note also that the side-table's substring `package_match` is known to leak
into other trees (`"tactical_objects"` matches
`pim_osprey.tactical_objects`, stamping legacy wire names into new-tree
bindings) — scoping the side-table to the legacy layout is part of the
plan's Phase 2, independent of full retirement.

Once topics are expressed in the contract, the Python side-table becomes
redundant and is removed. Migration in order:

1. **Add the options proto** (`pyramid/options/pyramid.options.proto`) and parse
   method options. The current rpc regex in `proto_parser.py` stops at the
   `returns(...)` clause and discards the trailing `{ option … }` body — extend
   it (or add a second pass over the method block) to capture the option, and
   add `pattern`, `topic`, and `qos` fields to `ProtoRpc`.
2. **Author topic methods** for each entry currently in
   `TACTICAL_OBJECTS_TOPIC_SPECS`, with `PUBLISH` / `SUBSCRIBE` and the existing
   wire names, preserving direction (`*_SUBSCRIBE_TOPICS` vs
   `*_PUBLISH_TOPICS`).
3. **Switch the generator** to derive `(sub_topics, pub_topics)` from parsed
   methods instead of `topics_for_service()`; keep the emitted
   `subscribe*` / `publish*` / `kTopic*` surface byte-for-byte so existing
   components and tests are unaffected.
4. **Delete `standard_topics.py`** and its imports in `cpp_codegen.py` /
   `ada_codegen.py` once the generated output matches the pre-migration
   snapshot.
5. **Regenerate and diff.** The generated bindings for the proving path
   (Tactical Objects) must be unchanged; that diff is the migration's
   correctness proof.

## Transport Projection Hooks

Each projection consumes the resolved pattern uniformly:

| Pattern | PCL | gRPC | ROS2 |
|---------|-----|------|------|
| Unary / void | service request/response | unary rpc | `srv/PclService` |
| Server-stream | streamed service | server-streaming rpc | open service + `frames` topic |
| Client-stream / bidi | (future) | native streaming rpc | (future) |
| Publish / subscribe | port pub/sub on topic wire-name | n/a (or wrapped) | `msg/PclEnvelope` on mapped topic |
| Action | (future) | n/a | ROS2 action (currently unimplemented) |

This closes two gaps called out in
[ros2_transport_semantics.md](ros2_transport_semantics.md): **QoS** now has a
contract home (`Interaction.qos`), and **actions** have a reserved pattern
(`ACTION`) so the projection has something concrete to map when the first
production user arrives.

## Direct (Non-PCL) Consumer Compatibility

A first-class requirement: an endpoint that knows nothing about PCL must be
able to use the contract **as-is** — a stock gRPC client against the `.proto`,
or a native ROS2 node against generated `.msg`/`.srv`/`.action`. The convention
must not break that. The two transports sit at opposite ends of "as-is".

### gRPC — transparent, works as-is

The annotation is plain proto3 and does **not** disturb gRPC codegen:

- A custom `MethodOptions` extension is metadata on the method descriptor.
  `protoc` and the gRPC plugins ignore options they do not consume, so
  `protoc --grpc_out` produces the normal service stub whether or not
  `pyramid_op` is present. The option is recoverable via descriptor reflection
  but changes nothing about the generated interface.
- Because pub/sub is modelled **as rpc**, a stock gRPC client gets a directly
  usable surface with no PCL knowledge: a `PUBLISH` method is an ordinary unary
  call, a `SUBSCRIBE` method is an ordinary server-streaming call. This is the
  main reason to model topics as rpc rather than invent a "topic" primitive
  (proto has none): the gRPC projection is native by construction.
- One real constraint: to **compile** a service `.proto` that *uses* the
  option, `protoc` needs the extension definition on its import path. So
  `pyramid/options/pyramid.options.proto` ships as part of the delivered
  contract package — the same dependency shape as `google/api/annotations.proto`.
  A consumer that wants only stubs can strip options instead, but shipping the
  one small file is cleaner. Keep the field number in the internal-use range
  (50000–99999; `50001` is fine) so it can never collide with a
  globally-registered extension.

Net: gRPC is the easy direction — the contract is already "as-is" usable, and
the annotation is invisible to anyone who does not look for it.

### ROS2 — needs a native-IDL projection (generated, but not yet on the live wire)

This is the harder direction. The native-IDL projection now **exists as
generated artifacts**: `pim/ros2_idl_codegen.py` emits real `.msg`/`.srv`
(the `pyramid_msgs` ament package) and `pim/ros2_marshal_codegen.py` emits the
`domain_model` ↔ `pyramid_msgs` marshalling + `rclcpp` wire codec, round-trip
verified. What the **live transport** carries, however, is still the generic
envelope — `msg/PclEnvelope`, `srv/PclService`, `srv/PclOpenStream` — with
**opaque payload bytes** and a `content_type`. A non-PCL ROS2 node therefore
still cannot use the contract directly today: `ros2 topic echo` shows a blob,
not fields.

So there are two ROS2 modes, and direct interop requires the second:

| Mode | Wire type | Codec | Who can consume |
|------|-----------|-------|-----------------|
| **Envelope** (current live wire) | `PclEnvelope` bytes | JSON / FB / protobuf, selectable | PCL ↔ PCL over ROS2 |
| **Native IDL** (generated; wire switch pending) | generated `.msg`/`.srv`/`.action` | ROS2 CDR (fixed) | any ROS2 node, no PCL |

Native IDL mode is a new **proto → ROS2 IDL projection**, analogous to the
existing proto → FlatBuffers projection, emitting real interface definitions
from the same contract:

| Contract element | ROS2 native artifact |
|------------------|----------------------|
| proto `message` | `.msg` (typed fields) |
| `PUBLISH` / `SUBSCRIBE` | a topic carrying the native `.msg`, at the mapped name, with QoS from `Interaction.qos` |
| `UNARY` / void | `.srv` |
| `SERVER_STREAM` | `.action` (or a typed frames topic) — *not* the envelope open+frames trick, which is PCL-internal |
| `ACTION` | `.action` |

Design points this raises:

- **Type-system mapping is not 1:1.** proto3 `oneof` (→ discriminator + union
  fields), `map` (→ key/value array), `optional`/wrapper types, well-known
  types, and bounded vs unbounded arrays all need documented rules, and the
  generator should fail the build on an unmappable construct rather than emit a
  lossy `.msg`. This is the same class of problem already handled for
  FlatBuffers.
- **The two modes cannot share a topic.** Native `.msg` and `PclEnvelope` are
  different wire types; a native node and a PCL node only interoperate if both
  use native IDL on that interface. "Expose natively" is therefore a per-
  interface decision. Carry it in the annotation — e.g. a transport profile or
  `native: true` on `Interaction` (or a generator backend selection) — so the
  contract records which interfaces are externally native vs PCL-internal.
- **The PCL side needs a bridge** for native interfaces: native `.msg` ↔
  `pcl_msg_t` with a CDR/ros2-native codec, with business logic still handed
  onto the PCL executor per the existing threading rule. The envelope support
  layer stays for PCL-to-PCL links; native IDL is an additional, parallel
  surface, not a replacement.

In short: the annotation is what *lets* both projections exist from one
contract — gRPC reads it as ordinary rpc, ROS2-native reads `topic`/`qos`/
`pattern` to emit idiomatic `.msg`/`.srv`/`.action`. The work item is the ROS2
native-IDL backend; gRPC already meets the "as-is" bar.

## Status

| Item | State |
|------|-------|
| Convention defined (this doc) | proposed; superseded in part by the port-grammar plan (see Update note at top) |
| Streaming inference (Layer 1, RPC subset) | already implemented in `proto_parser.py` |
| Topic inference (Layer 1, pub/sub) | not implemented; for the MBSE tree, planned as a port-grammar classifier rather than per-method inference |
| Method option (Layer 2) | not implemented; planned as MBSE-stamped (machine-written), not hand-authored |
| `standard_topics.py` retirement | metadata data-driven (JSON side-table); known cross-tree substring leak; scoping + retirement planned (plan Phase 2/5) |
| QoS / action projection | reserved; QoS reconciliation with plugin QoS profiles defined in the plan |
| gRPC direct (as-is) consumption | compatible by design; annotation is transparent to `protoc`/gRPC |
| ROS2 native IDL (`.msg`/`.srv`) projection | generated + round-trip verified (`pyramid_msgs`, `pyramid_ros2_codec.hpp`); live wire still envelope-only, `.action` not generated |

This doc remains the reference for the *convention*; sequencing and the
concrete implementation now live in
[doc/plans/PYRAMID/pubsub_contract_generation_plan.md](../../../../doc/plans/PYRAMID/pubsub_contract_generation_plan.md).
No generator or contract behavior changes until the plan's phases run and the
Tactical Objects binding snapshot is shown unchanged.
