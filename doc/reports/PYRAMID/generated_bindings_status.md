# Generated Bindings Status

## Purpose

This page is the current implementation and conformance status for PYRAMID's
generated binding system.

Use [generated_bindings.md](../../../subprojects/PYRAMID/doc/architecture/generated_bindings.md) for architecture and usage.
Use this page to answer:

- which Tactical Objects paths are currently proven
- which codec and transport combinations are tested
- what remains incomplete
- where to place new regression coverage

## Contract Rules

The current implementation is judged against these rules:

1. `.proto` is the downstream contract source of truth.
2. Component logic sees generated, proto-native types.
3. Codec choice does not change handler signatures.
4. Transport choice does not change handler signatures.
5. Generated adapters may translate delivery behavior, but not contract
   meaning.
6. Transport threads hand off to the PCL executor before business logic runs.

## Tactical Objects V1 Status

Tactical Objects is the current proving-ground component.

### Service bindings

| Capability | Status | Notes |
|------------|--------|-------|
| C++ provided/consumed bindings | active | provided and consumed facades have one typed service surface; generated content-type branches only mention codecs selected at binding generation time, and selected ROS2 startup binding hooks are emitted on the same facade |
| Ada provided/consumed bindings | active with compatibility shim | top-level generated service packages expose one typed `Invoke_*` procedure surface; generated `content_type` dispatch delegates to optional gRPC transports when that backend is generated, while `_Json` C ABI calls stay private; ROS2 endpoint constants are emitted on the same facade only when selected |
| Generated content-type metadata | active | `supportsContentType`, `supportedContentTypes`; emitted entries are selected by generator backend inputs, not include-path probing |
| Generated topic encode/decode | active | JSON, FlatBuffers, Protobuf |
| Standalone data-model dispatch | removed | superseded by generated service bindings |

### Codec backends

| Codec | Content type | PCL status | Notes |
|-------|--------------|------------|-------|
| JSON | `application/json` | active | default path |
| FlatBuffers | `application/flatbuffers` | active | C++ native; Ada typed API may use shim |
| Protobuf | `application/protobuf` | active | covered by generated binding and app E2E tests |

### Transport projections

| Transport | Status | Notes |
|-----------|--------|-------|
| PCL | active baseline | production proving path |
| gRPC | active optional projection | C++ transport plus Ada/C++ interop proof |
| ROS2 | active optional projection | generated top-level `bindRos2(...)` hooks/constants plus runtime adapter proof; no service-specific sidecar facade |
| PCL shared-memory bus | foundation active | Tactical Objects-specific projection remains incomplete |

## Current Proof Matrix

| Area | Test signal |
|------|-------------|
| Generated C++ service bindings | `test_pcl_proto_bindings` |
| Service-binding codec dispatch and pub/sub | `test_codec_dispatch_e2e` |
| Tactical Objects in-process E2E | `TacticalObjectsE2E.*` |
| C++ standalone bridge JSON/FlatBuffers | `tobj_cpp_bridge_client_e2e`, `tobj_cpp_bridge_client_flatbuffers_e2e` |
| Real app C++ JSON/FlatBuffers/Protobuf | `tobj_cpp_app_client_e2e`, `tobj_cpp_app_client_flatbuffers_e2e`, `tobj_cpp_app_client_protobuf_e2e` |
| Ada generated binding round-trip | `ada_generated_bindings_roundtrip` |
| Ada/C++ gRPC single service facade runtime | `tobj_ada_grpc_cpp_interop_e2e`, `ada_grpc_cpp_interop_e2e.gpr` |
| Ada socket active-find JSON/FlatBuffers | `tobj_ada_active_find_e2e`, `tobj_ada_active_find_flatbuffers_e2e` |
| Real app Ada active-find JSON/FlatBuffers | `tobj_ada_active_find_app_e2e`, `tobj_ada_active_find_app_flatbuffers_e2e` |
| ROS2 semantic projection | `test_ros2_transport_semantics`, `tobj_ros2_facade_e2e` |
| PCL shared-memory bus | `test_pcl_shared_memory_transport` |
| Binding generator dependency hygiene | `pyramid_binding_generation_dependencies` |

Focused v1 regression command:

```bat
ctest --test-dir build -C Release -R "(ProtoBindings|CodecDispatch|TacticalObjectsE2E|tobj_cpp_bridge|tobj_cpp_app)" --output-on-failure
```

## Tactical Objects Standard Interface

The real app path is the production-path signal.

`tactical_objects_app` hosts `StandardBridge`, which exposes the generated
Tactical Objects provided interface on the remote socket-facing executor. The
app supports:

- `--content-type application/json`
- `--content-type application/flatbuffers`
- `--content-type application/protobuf`

Current standard topics:

| Topic | Payload | Status |
|-------|---------|--------|
| `standard.entity_matches` | `ObjectMatch[]` | generated helpers active |
| `standard.object_evidence` | `ObjectDetail` | generated helpers active |
| `standard.evidence_requirements` | `ObjectEvidenceRequirement` | generated helpers active |

## Review Checklist

Use this table in reviews:

| Question | Expected answer |
|----------|-----------------|
| Does component code use generated types? | yes |
| Does component code avoid codec-specific handler signatures? | yes |
| Does component code avoid transport-specific handler signatures? | yes |
| Is content type configured once at startup/port creation? | yes |
| Are topic payloads encoded/decoded through generated helpers? | yes |
| Are service requests dispatched through generated `dispatch(...)`? | yes |
| Are generated service calls made through typed `invoke*` helpers rather than raw executor calls? | yes |
| Are `_Json`/`grpc_*` shim APIs absent from normal component code and examples? | yes |
| Are transport threads handing off to PCL executor-owned logic? | yes |
| Are stale standalone dispatch artifacts avoided? | yes |

## Current Gaps

The following are still not v1-complete:

- Tactical Objects projection over the PCL shared-memory bus
- Ada ROS2 runtime beyond generated top-level endpoint constants
- ROS2 action mapping for long-running / feedback-style workflows
- fully Ada-native gRPC runtime without generated C/C++ shim support
- applying the same proof level to other PYRAMID components

## Facade Closure Plan

The remaining facade work is narrower than the original binding cleanup. The
C++ `StandardBridge` no longer owns codec-specific topic encode/decode, but
the component boundary is not fully closed until raw executor, gRPC C ABI, and
JSON shim details disappear from ordinary component code and copied examples.

1. Generate consumed-side typed invokes. **Done.**
   Extend the C++ service generator so consumed facades expose the same
   `invoke*` and `decode*Response` helpers as provided facades. For Tactical
   Objects, this must include `cons::invokeCreateRequirement(...)` for
   `object_solution_evidence.create_requirement`.

2. Migrate `StandardBridge`. **Done.**
   Replace the active-find consumed call that currently builds `pcl_msg_t` and
   calls `pcl_executor_invoke_service(...)` with the generated consumed
   `invoke*` helper. Keep only domain mapping, content-type selection, and
   startup port wiring in the bridge.

3. Make generated dependencies explicit in both language facades. **Done.**
   `generate_bindings.py --backends ...` is the source of truth for C++ and
   Ada service facade dependencies. C++ service bindings no longer use
   `__has_include` to opportunistically enable FlatBuffers or Protobuf.
   JSON-only generation emits no FlatBuffers/Protobuf service constants,
   branches, aliases, or transport directories; mixed generations emit only
   the selected codec branches.

4. Add ROS2 to the selected service binding surface. **Done.**
   ROS2 generation emits top-level C++ `bindRos2(...)` startup hooks and Ada
   endpoint constants only when the `ros2` backend is requested. The stale
   component-level ROS2 transport files were removed, and the generated
   service facades still do not expose ROS2-specific business-call APIs.

5. Route Ada gRPC through the single typed service surface. **Done.**
   Keep `_Json` imports as generated implementation details, but do not add a
   parallel public `Channel`-based invoke API to the top-level service package.
   The existing generated `Invoke_*` procedure remains the only component-facing
   call shape. When `Content_Type => Grpc_Content_Type` is selected and the
   gRPC backend was requested at generation time, it delegates to the generated
   transport adapter, which owns C string handling, JSON/protobuf conversion,
   shim invocation, response decoding, and error reporting.

6. Move interop tests to the typed surface. **Done.**
   The primary Ada/C++ gRPC E2E calls the top-level generated service procedure
   (`Pyramid.Services.Tactical_Objects.Provided.Invoke_Create_Requirement`)
   with `Content_Type => Grpc_Content_Type`, a typed request, and the normal
   response callback. Dynamic DLL loading, channel setup, and raw `_Json` /
   `grpc_*` symbol lookup are confined to startup wiring and generated
   transport implementation details.

7. Add hygiene tests. **Done.**
   Add static checks that production Tactical Objects component code does not
   include codec-specific service headers, call `_Json`/`grpc_*` shim symbols,
   or call raw `pcl_executor_*` service APIs for generated services. Allowlist
   generated transport code and explicit compatibility harnesses. Add generator
   checks proving C++ and Ada facades only emit selected codec/transport
   dependencies.

8. Update examples and docs.
   Make typed generated facades the copied examples. Mark shim-level APIs as
   compatibility-only wherever they remain visible for ABI or test reasons.

Completion criteria:

- `StandardBridge.cpp` uses generated provided and consumed facades for all
  generated Tactical Objects services and topics.
- C++ provided and consumed facades have symmetric typed invoke/response
  helpers.
- C++ generated service facades do not expose FlatBuffers/Protobuf constants,
  includes, branches, or support metadata unless those backends were requested.
- Ada generated service packages expose a single typed service call surface;
  optional gRPC support is selected by `Content_Type` and generated only when
  the gRPC backend is requested.
- ROS2 generated output is selected at binding generation time: C++ top-level
  service facades expose startup `bindRos2(...)` hooks, Ada top-level service
  facades expose endpoint constants, and neither exposes ROS2-specific
  business-call APIs.
- Ada gRPC examples/tests exercise that top-level typed service procedure,
  with raw C ABI handling hidden in generated transport bodies.
- Raw `_Json`, `grpc_*`, and generated-service `pcl_executor_*` calls are
  limited to generated adapters and explicit compatibility tests.

## Where New Tests Belong

Place tests at the lowest layer that matches the behavior:

- generated service facade and codec selection: `test_pcl_proto_bindings`
- pub/sub codec dispatch: `test_codec_dispatch_e2e`
- real Tactical Objects behavior: `tactical_objects_app` E2E tests
- gRPC and ROS2 semantics: transport-specific tests, with normal calls made
  through the generated service facade rather than parallel transport APIs
- shared-memory bus behavior: PCL transport tests until Tactical projection
  exists

`standalone_bridge` remains useful compatibility coverage, but it is not the
production-path substitute for `tactical_objects_app`.
