# PCL/PYRAMID Arbitrary Proto Binding Gap Analysis

## Purpose

This report assesses what is needed for the PCL/PYRAMID binding system to
support arbitrary `.proto` contracts, including schemas that do not follow the
current `pyramid.data_model.*` plus `pyramid.components.*.services.*` layout.

The current PCL runtime is already mostly schema-neutral: it routes opaque
`pcl_msg_t` buffers, service names, content types, and codec plugin calls. The
main gaps are in the PYRAMID binding generator, backend projections, build glue,
and productization assumptions around generated artifact names.

## Current State

The binding pipeline has a strong generic foundation:

- `subprojects/PYRAMID/pim/proto_parser.py` parses package names, imports,
  top-level messages, enums, nested messages/enums, oneofs, services, unary RPCs,
  and streaming RPC markers from any `.proto` tree.
- `ProtoTypeIndex` resolves message and enum names across parsed files.
- Generated C++ and Ada type/codecs mostly derive namespaces and file names from
  proto packages.
- PCL itself does not inspect proto fields; it only sees generic buffers,
  service/topic names, content types, and transport callbacks.
- The codec plugin ABI uses `content_type` plus `schema_id`, which is compatible
  with arbitrary schema families if generated schema IDs are generalized.

However, generation orchestration is not arbitrary-proto ready. A simple proto
like this parses successfully but currently produces no C++ JSON binding output:

```proto
syntax = "proto3";
package example.telemetry;

message Pose {
  double x = 1;
  double y = 2;
}

service Telemetry {
  rpc GetPose(Pose) returns (Pose);
}
```

The reason is structural filtering, not parser failure: non-`pyramid.data_model`
messages are not treated as type modules, and services are only generated when
their package contains `.services.`.

## Main Gaps

| Area | Current behavior | Gap for arbitrary proto |
|------|------------------|-------------------------|
| Type discovery | `generate_bindings.py` treats only `pyramid.data_model*` packages as data-model type files. | Any proto file with messages/enums that are referenced by generated services must be eligible for type, JSON codec, C-ABI, and backend generation. |
| Service discovery | Service facades are generated only for packages containing `.services.`. | Any proto service should be projectable onto PCL, with direction/role supplied by config or a default policy. |
| Namespace model | `pyramid.data_model*` is flattened/re-exported into `pyramid::domain_model`; service packages use PYRAMID-specific legacy aliases. | Generated namespaces need a neutral contract namespace strategy that does not assume PYRAMID roots. |
| Build globs | CMake consumes `pyramid_data_model_*`, `pyramid_components_*`, and `pyramid_services_*` artifact patterns. | CMake needs generated manifests or neutral glob patterns that compile all generated artifacts independent of package prefix. |
| Protobuf build | CMake runs `protoc` over `${PYRAMID_PROTO_DIR}/pyramid/data_model/*.proto`; gRPC scans `${PYRAMID_PROTO_DIR}/pyramid/components/*.proto`. | Protobuf and gRPC codegen must use the complete proto input set or generator-provided service/type manifests. |
| Protobuf include paths | `protobuf_backend.py` anchors generated `.pb.h` includes on the path segment `pyramid` when present. | Include path derivation must be relative to the configured proto import root, not a package brand. |
| Topic generation | `standard_topics.py` currently contains Tactical Objects-specific standard topics. | Topic support needs explicit source metadata or generic derivation rules; arbitrary proto services should not inherit domain-specific topics. |
| Service role model | `provided`/`consumed` is inferred from package names. | Arbitrary service packages need explicit role mapping, or generated facades should be role-neutral by default. |
| Test coverage | Existing tests validate Tactical Objects and Sensor Data Interpretation generated paths. | Add fixture protos for non-PYRAMID packages, mixed package roots, same-name messages across packages, and services in normal protobuf packages. |

## Detailed Findings

### 1. Parser Is Generic Enough For A First Increment

`proto_parser.py` already parses arbitrary package names, messages, enums,
services, and RPC signatures. The first barrier is not parsing; it is downstream
selection and generation policy.

This means the initial arbitrary-proto increment can avoid replacing the parser.
The safer first step is to preserve `ProtoFile` and `ProtoTypeIndex`, then
replace PYRAMID-specific discovery filters with a neutral contract model.

### 2. Generator Orchestration Filters Out Arbitrary Contracts

`generate_bindings.py` currently routes:

- type generation through `_discover_data_model_files()`, which selects only
  packages starting with `pyramid.data_model`;
- service-local wrapper generation through `_discover_service_message_files()`,
  which selects only packages containing `.services.`;
- service facade generation through the same `.services.` package check.

For arbitrary proto support, generation should instead classify files by
contents and references:

- `type_files`: files containing top-level messages/enums;
- `service_files`: files containing services;
- `wrapper_files`: service files that also declare messages/enums;
- `external_files`: imported files that should be left to external generated
  code rather than translated into PCL bindings.

The classification should be configurable, but the default should work for
ordinary protobuf layouts.

### 3. Generated C++ Still Assumes A PYRAMID Contract Shape

`cpp_codegen.py` has generic namespace helpers, but several key decisions remain
PYRAMID-specific:

- `_DATA_MODEL_PROTO_ROOT = "pyramid.data_model"`;
- `_DATA_MODEL_TYPES_NS = "pyramid::domain_model"`;
- umbrella header name `pyramid_data_model_types.hpp`;
- legacy service namespace/file prefix derivation that skips package segments
  such as `pyramid`, `components`, `data_model`, `base`, and `services`;
- service role inference from `provided` in the package name.

These assumptions are useful compatibility shims for current PYRAMID generated
code, but they should become one selectable naming policy, not the only policy.

### 4. Build Integration Is More Coupled Than The Generator

The CMake integration is currently the largest productization blocker. It globs
and creates targets from hardcoded generated artifact patterns:

- `pyramid_data_model_*_codec.cpp`;
- `pyramid_components_*_codec.cpp`;
- `pyramid_data_model_*_cabi_marshal.cpp`;
- `pyramid_components_*_cabi_marshal.cpp`;
- `flatbuffers/cpp/pyramid_*.fbs`;
- `flatbuffers/cpp/pyramid_services_*_flatbuffers_codec.cpp`;
- codec plugin names matching `pyramid_services_*_json_codec_plugin`;
- protobuf input path `${PYRAMID_PROTO_DIR}/pyramid/data_model/*.proto`;
- gRPC service path `${PYRAMID_PROTO_DIR}/pyramid/components/*.proto`.

Even if the generator emitted arbitrary package bindings, the current build
would not reliably compile, link, or package them. The build should consume a
generator manifest rather than reverse-engineering artifact roles from
PYRAMID-specific filenames.

### 5. Protobuf Backend Needs Import-Root Awareness

The protobuf backend comments correctly state that `protoc` derives `.pb.h`
paths from the proto file path relative to the import root. The implementation
currently anchors on the `pyramid` path segment when present.

For arbitrary proto trees, the generator needs to know the proto import root and
derive includes with:

```text
relative_path(proto_file, proto_import_root).replace(".proto", ".pb.h")
```

This same root should be shared by CMake `protoc` invocation, the protobuf
backend, and any generated gRPC transport projection.

### 6. Topics Are Not Generic Yet

The current generated topic helper path is intentionally Tactical
Objects-specific. `standard_topics.py` maps package names containing
`tactical_objects` to hardcoded standard topic names and payload types.

Arbitrary proto support should not infer topics from domain names. Options:

- no generated topics unless the proto uses explicit topic annotations;
- sidecar YAML/JSON manifest mapping topics to message types and directions;
- convention-based topic generation from selected RPC/message names, enabled
  only by configuration.

The safest default is "services only, no topics" for plain protobuf input.

## Target Architecture

The binding generator should build a neutral contract model before emitting any
language or backend code:

```text
proto tree
  -> parsed ProtoFile list
  -> BindingContract manifest
     - type modules
     - service modules
     - endpoint roles
     - topic declarations
     - backend options
     - compatibility naming policy
  -> C++/Ada/backends/CMake manifest
```

The compatibility naming policy can preserve current PYRAMID outputs. A neutral
policy can support arbitrary packages without requiring them to pretend to be
PYRAMID `data_model` or `components` packages.

## Recommended Work Plan

### Phase 1: Make JSON C++ Work For Plain Protos

1. Add a generator option such as `--contract-layout pyramid|generic`.
2. In generic mode, classify every file with messages/enums as a type file.
3. In generic mode, classify every file with services as a service file.
4. Generate C++ type headers/codecs for generic type files.
5. Generate role-neutral C++ service facades for generic services.
6. Add tests using `example.telemetry` or similar non-PYRAMID protos.

Definition of done: a plain proto package with `message Pose` and `service
Telemetry` produces compiling C++ JSON bindings and can dispatch through a PCL
service handler.

### Phase 2: Add A Generated Artifact Manifest

Emit a machine-readable manifest, for example:

```json
{
  "types": ["example_telemetry_types.hpp"],
  "json_codecs": ["example_telemetry_codec.cpp"],
  "cabi": ["example_telemetry_cabi.cpp"],
  "services": ["example_telemetry_services.cpp"],
  "plugins": [
    {
      "name": "example_telemetry_json_codec_plugin",
      "content_type": "application/json",
      "source": "example_telemetry_json_codec_plugin.cpp"
    }
  ]
}
```

CMake should consume this manifest instead of package-prefix globs.

### Phase 3: Generalize Protobuf And FlatBuffers

1. Make protobuf include path derivation relative to the proto import root.
2. Run `protoc` over manifest-listed proto files, not `pyramid/data_model`.
3. Generate FlatBuffers schemas for generic type/service closures.
4. Compile generic codec plugins using manifest entries.

Definition of done: the same non-PYRAMID fixture supports JSON, FlatBuffers,
and Protobuf content types.

### Phase 4: Generalize Transports

1. Make gRPC proto selection manifest-driven.
2. Make generated ROS2 naming independent of PYRAMID package roots, while
   preserving current `/pyramid/...` names under the compatibility policy.
3. Keep transport projections role-neutral where possible; map provider/client
   roles at binding time.

### Phase 5: Ada Parity

After the C++ path is stable, apply the same contract model to Ada:

- generic package derivation;
- no required `Pyramid.*` root;
- no hardcoded Tactical Objects topic assumptions;
- generic service and codec tests.

## Risks

- **Compatibility risk:** current Tactical Objects and Sensor Data
  Interpretation generated names are used by tests and app code. Preserve them
  with a `pyramid` layout policy until callers migrate.
- **Name collision risk:** arbitrary proto trees can contain repeated short
  names across packages. Generated code must keep package-qualified names as the
  canonical form and avoid broad flattening.
- **Protobuf semantic gap:** the in-tree parser is enough for the current proto
  subset but not a full protobuf compiler. Complex arbitrary protos may require
  either `protoc` descriptor-set input or a stronger parser.
- **Build graph churn:** manifest-driven builds require CMake refactoring, but
  they are the cleanest way to remove filename coupling.

## Bottom Line

PCL can already carry arbitrary proto-derived payloads because it is
schema-neutral. PYRAMID bindings cannot yet generate arbitrary proto contracts
because the generator and build system classify contracts through PYRAMID
package conventions. The practical path is to add a neutral binding-contract
manifest, keep current PYRAMID naming as a compatibility policy, and make CMake
consume generated manifests instead of hardcoded `pyramid_*` artifact patterns.
