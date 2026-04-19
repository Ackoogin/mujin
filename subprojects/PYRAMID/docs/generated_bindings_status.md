# Generated Bindings Status

## Purpose

This page is the current implementation and conformance status for PYRAMID's
generated binding system.

Use [generated_bindings.md](generated_bindings.md) for architecture and usage.
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
| C++ provided/consumed bindings | active | service `dispatch`, typed `invoke*`, topic helpers |
| Ada provided/consumed bindings | active | typed public surface |
| Generated content-type metadata | active | `supportsContentType`, `supportedContentTypes` |
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
| ROS2 | active runtime binding | generated projection plus runtime adapter proof |
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
| Ada socket active-find JSON/FlatBuffers | `tobj_ada_active_find_e2e`, `tobj_ada_active_find_flatbuffers_e2e` |
| Real app Ada active-find JSON/FlatBuffers | `tobj_ada_active_find_app_e2e`, `tobj_ada_active_find_app_flatbuffers_e2e` |
| ROS2 semantic projection | `test_ros2_transport_semantics`, `test_rclcpp_runtime_adapter` |
| PCL shared-memory bus | `test_pcl_shared_memory_transport` |

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
| Are transport threads handing off to PCL executor-owned logic? | yes |
| Are stale standalone dispatch artifacts avoided? | yes |

## Current Gaps

The following are still not v1-complete:

- Tactical Objects projection over the PCL shared-memory bus
- Ada ROS2 runtime beyond generated endpoint constants/specs
- ROS2 action mapping for long-running / feedback-style workflows
- fully Ada-native gRPC runtime without generated C/C++ shim support
- applying the same proof level to other PYRAMID components

## Where New Tests Belong

Place tests at the lowest layer that matches the behavior:

- generated service facade and codec selection: `test_pcl_proto_bindings`
- pub/sub codec dispatch: `test_codec_dispatch_e2e`
- real Tactical Objects behavior: `tactical_objects_app` E2E tests
- gRPC and ROS2 semantics: transport-specific tests
- shared-memory bus behavior: PCL transport tests until Tactical projection
  exists

`standalone_bridge` remains useful compatibility coverage, but it is not the
production-path substitute for `tactical_objects_app`.
