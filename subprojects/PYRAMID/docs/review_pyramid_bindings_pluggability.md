# PYRAMID Generated Bindings — Pluggability & Consistency Review

Reviewer scope: how PYRAMID-generated bindings let a consuming component
swap codecs and transports with **only configuration / startup wiring**, no
bespoke per-codec or per-transport branching in business logic.

Branch: `claude/review-pyramid-bindings-1hYVp`
Date: 2026-04-19

---

## 1. Summary verdict

The pluggability **architecture** in PYRAMID is sound:

- A registry-based code generator with a clean `CodecBackend` ABC.
- A transport-agnostic `ServiceHandler` interface generated from `.proto`.
- A generated `codec_dispatch` layer that selects codec by `content_type`.
- A consistent PCL-executor handoff rule ("transport threads do wire work
  only; business logic runs on the executor").

The **realisation** is partial: the design intent is "components use the
generated bindings + a content-type config; transport selection is just
startup wiring", but in practice the only live consumer
(`StandardBridge`) re-implements the dispatch layer by hand, the
transports have three different activation shapes, and several
asymmetries make truly drop-in codec swap incomplete.

In short: the **intent is right and visible**, but the proving-ground
component currently bypasses the very layer that would make pluggability
zero-code, so the promise isn't yet exercised end-to-end.

---

## 2. What "generated bindings" actually means here

Driven from `.proto` by `pim/generate_bindings.py` via a registry
(`pim/codec_backends.py`), PYRAMID emits, per language:

| Layer | C++ output | Ada output |
|-------|------------|------------|
| Types | `examples/cpp/generated/pyramid_data_model_*_types.hpp` | `pyramid-data_model-*-types.ads` |
| JSON codec | `examples/json/cpp/pyramid_data_model_*_json_codec.hpp` (and a legacy non-suffixed `examples/cpp/generated/pyramid_data_model_*_codec.hpp`) | `pyramid-data_model-*-types_codec.ads` |
| FlatBuffers codec | `examples/cpp/generated/flatbuffers/cpp/pyramid_data_model_*_flatbuffers_codec.hpp` | shim |
| Protobuf codec | `examples/protobuf/cpp/pyramid_data_model_*_protobuf_codec.hpp` | shim |
| Service binding | `examples/cpp/generated/pyramid_services_tactical_objects_provided.hpp` (typed `ServiceHandler`, `dispatch(...)`, wire-name constants) | `pyramid-services-tactical_objects-provided.ads` |
| gRPC transport | `examples/grpc/cpp/pyramid_components_*_grpc_transport.{hpp,cpp}` (`ServerHost::start(address)`) | C-ABI shim specs |
| ROS2 transport | `examples/ros2/cpp/pyramid_components_*_ros2_transport.{hpp,cpp}` (`ServiceBinder::bind()`) | endpoint-constant specs |
| Codec dispatch | `examples/dispatch/cpp/pyramid_data_model_*_codec_dispatch.hpp` | `*-codec_dispatch.ads` |

The contract is well-stated in `docs/service_binding_codegen.md:177-188`:

> Codec selection: by `content_type`; the generated dispatch layer decodes
> bytes to proto-native types; handlers operate only on typed values.
> Transport selection: by generated transport binding; handlers must not
> branch on transport choice.

---

## 3. Pluggability mechanics, in order of strength

### 3.1 Service handler abstraction — STRONG

`pyramid_services_tactical_objects_provided.hpp:78-102` defines a single
typed `ServiceHandler` ABC (`handleCreateRequirement`, `handleReadMatch`,
…). All three transports re-use it:

- PCL: `dispatch(handler, channel, …, content_type, …)` at line 193-199.
- gRPC: `MatchingObjectsServiceImpl` etc. (`pyramid_components_tactical_objects_services_provided_grpc_transport.cpp:92-239`)
  forward to the same handler via the executor.
- ROS2: `ServiceBinder::bind()` (`…_ros2_transport.cpp:23-30`) wires
  ingress to `pcl_executor_post_service_request`, which lands on the same
  handler.

Net: a component that implements `ServiceHandler` is genuinely reusable
across transports. This is the cleanest part of the design.

### 3.2 Backend registry — STRONG (generator-side)

`pim/codec_backends.py` exposes an `ABC` + `register()` pattern; each
backend module under `pim/backends/` self-registers. Adding a new codec
is "subclass + register"; the orchestrator does not need to be edited.

Caveat: this is generator-time pluggability only. Runtime cannot acquire
new codecs; they must be regenerated and recompiled.

### 3.3 Codec dispatch layer — INTENT GOOD, USAGE INCONSISTENT

The generator emits a clean per-package dispatcher
(`examples/dispatch/cpp/pyramid_data_model_common_codec_dispatch.hpp`).
For each typed message it produces:

```cpp
inline std::string serialize(const …::GeodeticPosition& msg,
                             const char* content_type);
```

…that does the routing. This is exactly the seam business logic should
sit on top of. Two important properties of the current emission:

- Codecs are gated by `#if defined(CODEC_FLATBUFFERS)` /
  `#if defined(CODEC_PROTOBUF)` (lines 21-27, 52-59, …) — runtime
  selection by `content_type` is real, but the available set is fixed at
  build time.
- `deserializeXxx(...)` only implements JSON; non-JSON paths
  unconditionally throw with the comment "FlatBuffers and Protobuf
  deserialize to their own types; conversion to the common type requires
  a mapping layer" (lines 70-74, 99-103, …).

So the dispatch is **symmetric for serialise, asymmetric for
deserialise**. A consumer that "just changes `content_type`" gets full
encode but partial decode.

### 3.4 PCL port-level codec selection — STRONG (when used)

A port is created with a `content_type`; PCL routes the message and the
generated `dispatch(handler, channel, …, content_type, …)` picks the
codec. Threaded all the way through `StandardBridge::on_configure`
(`StandardBridge.cpp:376-413`) the codec is set once via a constructor
parameter and propagated to every `addService` / `addPublisher` /
`addSubscriber` call. CLI flag `--content-type` in
`tactical_objects_main.cpp:139-143` flows that down.

This is the closest the codebase gets to "no bespoke code, just
configuration". Switching the live tactical-objects app between JSON,
FlatBuffers, and Protobuf is genuinely a CLI argument.

### 3.5 Transport selection — MIXED

Transports are **not interchangeable at the call site**. Each presents a
different activation API:

| Transport | Startup call (provider) |
|-----------|--------------------------|
| PCL socket | `pcl_socket_transport_create_server(port, exec)` then `pcl_executor_set_transport(...)` (`tactical_objects_main.cpp:186-195`) |
| gRPC | `provided::grpc_transport::buildServer(address, exec)` returning a `ServerHost` (`…_grpc_transport.hpp:29-30`) |
| ROS2 | construct a `pyramid::transport::ros2::Adapter` (e.g. `RclcppRuntimeAdapter(node)`), then `ServiceBinder(adapter, exec).bind()` (`…_ros2_transport.hpp:11-20`) |
| PCL shared-memory | central named-bus (foundation only at PCL layer; tactical projection still TBD per `docs/service_schema_tactical_objects.md:86`) |

These shapes are not unifiable today: ServerHost vs adapter+binder vs
socket factory. So transport swap is "additional setup" (per the user's
intent), but each transport is its own bespoke setup snippet rather than
a single pluggable factory. There is no `Transport` trait / abstract
factory shared across them.

---

## 4. Inconsistencies and gaps

### 4.1 The proving-ground component bypasses the dispatch layer

`StandardBridge.cpp` is the live tactical-objects consumer of the
generated bindings. It:

1. Includes the generated `*_provided.hpp`, `*_consumed.hpp`, and the
   per-codec headers directly (`StandardBridge.cpp:3-7`).
2. Re-defines `kJsonContentType` / `kFlatBuffersContentType` /
   `kProtobufContentType` and helper predicates `is_json_content_type`
   etc. (lines 36-50).
3. Hand-rolls per-codec branches in `encode_match_array` (lines 289-306),
   `encode_evidence_requirement` (lines 308-317), `decode_object_evidence`
   (lines 319-337) — the exact pattern the generated `codec_dispatch`
   exists to remove.
4. Never #includes any `*_codec_dispatch.hpp`.

Effect: the dispatch layer's whole reason for being — letting components
sit on `serialize(msg, content_type)` and forget the codec list — is
unrealised in the only running consumer. Any new codec requires editing
StandardBridge in addition to regenerating bindings.

A second symptom: JSON's "encode a vector of `ObjectMatch`" path
(line 297-305) builds the JSON array inline (`"[" + toJson(match) + …
"]"`) because the JSON codec exposes per-message `toJson`, while
FlatBuffers/Protobuf expose `toBinary(vector<…>)` directly. The
collection-encoding surface is **asymmetric across codecs**, which is
why the dispatch layer can't easily cover it today.

### 4.2 Two parallel JSON output locations

- Old: `examples/cpp/generated/pyramid_data_model_common_codec.hpp` (no
  `_json_` suffix). Used by `StandardBridge.cpp` via `tactical_codec::toJson`.
- New: `examples/json/cpp/pyramid_data_model_common_json_codec.hpp`
  (standardised under `examples/<backend>/cpp/`). Referenced by the
  generated dispatch layer.

The dispatch layer therefore won't compile against the headers the
StandardBridge currently consumes without alignment — partly explaining
why the consumer hasn't migrated. The `near-term standardisation` list
in `docs/service_binding_codegen.md:228-241` flags this (item 3:
"reducing remaining Ada filename/package compatibility glue").

### 4.3 FlatBuffers / Protobuf output layouts also differ

- FlatBuffers C++: `examples/cpp/generated/flatbuffers/cpp/`.
- Protobuf C++:    `examples/protobuf/cpp/`.
- gRPC C++:        `examples/grpc/cpp/`.
- ROS2 C++:        `examples/ros2/cpp/`.

There is no single rule like "every backend lives at
`examples/<backend>/<lang>/`". Build glue and `#include` hygiene are
backend-specific. This is purely a layout issue, but it is a real
friction point when a new backend needs to be added.

### 4.4 Compile-time #ifdef vs runtime content-type

`pyramid_data_model_common_codec_dispatch.hpp:21-27,52-59` brackets
non-JSON codecs in `#if defined(CODEC_FLATBUFFERS)` /
`#if defined(CODEC_PROTOBUF)`. At runtime the same file does
`std::strcmp(content_type, kFlatBuffers)`. Consequences:

- A binary built without `CODEC_FLATBUFFERS` will compile happily but
  throw `std::runtime_error("unsupported codec: …")` if a peer sets the
  port type to `application/flatbuffers`.
- There is no introspection ("which codecs does this binary support?"),
  no graceful degradation, no negotiation. The mismatch is silent until
  first message.

### 4.5 Asymmetric serialise / deserialise in the dispatch layer

For every type the generator emits:

```cpp
std::string serialize(...);                       // routes JSON/FB/Protobuf
T deserializeXxx(const void*, size_t,             // JSON only; throws otherwise
                 const char* content_type);
```

(see `pyramid_data_model_common_codec_dispatch.hpp:65-74, 94-103, …`).
A bidirectional consumer therefore can't be built on top of the
dispatch layer alone — exactly the situation that pushed StandardBridge
back into hand-rolled branches.

### 4.6 gRPC transport hard-binds protobuf

`pyramid_services_tactical_objects_grpc_dispatch.hpp:14` declares
`kProtobufContentType` as the only request type, and the gRPC server
shim at `…_grpc_transport.cpp:70` sets `request.type_name =
grpc_detail::kProtobufContentType` unconditionally. This is
semantically reasonable (gRPC is protobuf-native) but it means
"transport" and "codec" are not orthogonal axes — gRPC pins the codec.
Worth documenting explicitly so consumers don't expect
`grpc + flatbuffers`.

### 4.7 Ada story has a stated short-cut

`docs/service_binding_codegen.md:189-203` is candid:

> JSON is expected to remain native at the Ada layer. FlatBuffers and
> Protobuf may use generated C/C++ shims internally. … This is a
> short-term implementation choice, not a change to the public contract.

So Ada *callers* see a typed surface, but Ada is not really pluggable at
the codec implementation level today. This is openly tracked, not a
hidden defect.

### 4.8 Application-level transport selection is bespoke

`tactical_objects_main.cpp:186-200` hard-codes the socket transport.
There is no factory like:

```cpp
auto transport = pyramid::transport::create("grpc:127.0.0.1:50111", exec);
transport->bind(server_handler);
```

Switching the app from socket to gRPC or ROS2 today requires editing
`main.cpp` to call a different builder (`buildServer` vs
`pcl_socket_transport_create_server` vs `Adapter` + `ServiceBinder`).
Each transport has its own lifetime object (`ServerHost`,
`pcl_socket_transport_t*`, `Adapter` + `ServiceBinder`), with no common
RAII type.

---

## 5. Cross-language conformance signal

`docs/service_schema_tactical_objects.md:149-166` reports the
`tobj_master_conformance_e2e` matrix as currently passing for:

- socket + JSON
- socket + FlatBuffers
- gRPC + Protobuf

…with `socket + Protobuf`, shared-memory tactical projection, and
ROS2/Ada combinations not yet in the master matrix. The matrix shape
matches the issues above: gRPC is only proven in its native pairing
(protobuf), and codec orthogonality is exercised under the socket
transport, not across transports.

---

## 6. Concrete recommendations

In rough priority order, with the user's stated bar in mind ("additional
setup like starting a gRPC server is fine; bespoke code is not"):

1. **Migrate `StandardBridge` onto the generated `codec_dispatch`
   layer.** Replace `is_json_content_type` / `is_flatbuffers_content_type`
   / `is_protobuf_content_type` and the hand-rolled `encode_*` /
   `decode_*` helpers with `codec_dispatch::serialize(msg, content_type)`
   and `codec_dispatch::deserializeXxx(...)`. This will immediately
   surface gaps 4.1, 4.2, 4.5 as compile errors and force them shut.

2. **Make the dispatch layer fully bidirectional.** Either generate a
   shared deserialise that returns the JSON-codec type and lets the
   FB/Protobuf paths convert at the boundary, or generate codec-specific
   typed deserialisers and have the dispatcher own the conversion.

3. **Decide on a single codec output convention.** Recommend
   `examples/<backend>/<lang>/pyramid_<package>_<backend>_codec.{hpp,cpp}`
   for all four codec backends; align the legacy
   `examples/cpp/generated/pyramid_data_model_*_codec.hpp` JSON path with
   the new `examples/json/cpp/...` path so includes are deterministic.

4. **Replace `#if defined(CODEC_*)` in the dispatcher with a small
   registry table.** Even a generated `kAvailableCodecs[]` plus a
   `bool supports(content_type)` helper would let consumers query
   available codecs at startup instead of failing on first message.

5. **Introduce a transport factory abstraction.** A minimal
   `pyramid::transport::ProvidedHost` interface with
   `start(executor)` / `shutdown()`, with concrete
   `SocketProvidedHost`, `GrpcProvidedHost`, `Ros2ProvidedHost`
   implementations, would let `tactical_objects_main.cpp` look like:

   ```cpp
   auto host = pyramid::transport::makeProvidedHost(spec);  // "socket://...", "grpc://..."
   host->start(remote_exec);
   ```

   The transport-specific *setup* (open ports, attach to an `rclcpp`
   node, etc.) is still configuration; the *call shape* in main becomes
   identical across transports.

6. **Document that gRPC pins the codec to protobuf.** This is fine, but
   today the matrix table can be misread to suggest `grpc + json` is
   intended.

7. **Address Ada non-JSON paths separately.** The current shim policy is
   honest, but it should be a tracked debt with an owner, not just a
   policy paragraph.

---

## 7. Bottom line

The generator's *abstractions* — backend registry, transport-agnostic
`ServiceHandler`, `content_type`-driven dispatch — are the right ones
for the user's stated goal. Where the codebase falls short of "just
binding use and configuration" today is mostly **realisation**:

- the dispatch layer exists but isn't used by the live consumer;
- transports lack a common activation interface;
- codec selection is symmetric on encode but not on decode;
- compile-time and runtime selection are not reconciled.

None of these are architectural dead-ends. Item 1 (StandardBridge
migrating onto the generated dispatch) is the single change that would
make the rest of the gaps either trivial or unavoidable to fix.
