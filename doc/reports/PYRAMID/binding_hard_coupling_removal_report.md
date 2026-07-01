# PYRAMID Binding Hard-Coupling Removal Report

> **Status: historical.** The removal plan defined here was implemented by the
> generic contract layout work. See
> [`generic_contract_layout.md`](../../../subprojects/PYRAMID/doc/architecture/generic_contract_layout.md)
> for the delivered design and
> [`generic_binding_implementation_progress.md`](generic_binding_implementation_progress.md)
> for the implementation record and the couplings still open. Keep this
> document as analysis context, not as a work plan.

## Purpose

This report identifies hard coupling in the PCL/PYRAMID binding system and
defines a removal plan. The target is zero hard coupling in reusable binding
infrastructure: no generator, backend, build, plugin, or transport code should
require a contract to use PYRAMID package roots, Tactical Objects package names,
or `data_model/components/services` directory structure.

Compatibility with existing PYRAMID generated names can remain, but only as an
explicit compatibility policy selected by configuration.

## Definition Of Zero Hard Coupling

The binding system reaches zero hard coupling when all of the following are
true:

- Any valid supported `.proto` package root can be used.
- Message/type generation is based on parsed proto contents, not package prefix.
- Service generation is based on parsed services, not `.services.` package
  segments.
- Endpoint role is explicit metadata or a role-neutral default, not inferred
  from `provided` or `consumed` package names.
- Topic generation is explicit metadata or disabled by default, not inferred
  from domain names.
- Build targets are created from a generated manifest, not filename globs with
  `pyramid_*`, `pyramid_data_model_*`, `pyramid_components_*`, or
  `pyramid_services_*` assumptions.
- Protobuf and gRPC build steps use the configured proto import roots and
  manifest-listed files, not fixed `pyramid/data_model` or
  `pyramid/components` paths.
- Existing PYRAMID/Tactical Objects outputs are produced by a compatibility
  naming policy layered on top of the generic model.

## Hard Coupling Inventory

| Coupling | Location | Current assumption | Removal direction |
|----------|----------|--------------------|-------------------|
| Data-model package root | `pim/generate_bindings.py` | Only `pyramid.data_model*` files are type modules. | Classify type modules by messages/enums and reachability. |
| Service package marker | `pim/generate_bindings.py` | Only packages containing `.services.` generate service facades. | Classify service modules by parsed `service` declarations. |
| Component-local wrapper marker | `pim/generate_bindings.py` | Wrapper messages are only in `.services.` packages. | Treat any service file with local messages/enums as wrapper-capable. |
| Domain namespace root | `pim/cpp_codegen.py` | Types live under `pyramid::domain_model`. | Derive native namespace from package or configured root namespace. |
| Umbrella type header | `pim/cpp_codegen.py` | Umbrella is `pyramid_data_model_types.hpp`. | Emit layout-specific umbrella names through a naming policy. |
| Legacy service namespace | `pim/cpp_codegen.py`, backend code | Service file prefixes collapse PYRAMID segments into `pyramid_services_*`. | Move to generated naming policy; generic default preserves package path. |
| Role inference | `pim/cpp_codegen.py`, `pim/ada_codegen.py` | `provided` in package means provider side; otherwise consumed/provider defaults. | Add endpoint role metadata; default to role-neutral service facade. |
| Standard topics | `pim/standard_topics.py` | Package containing `tactical_objects` gets hardcoded topics. | Replace with explicit topic manifest; default to no topics. |
| Protobuf include root | `pim/backends/protobuf_backend.py` | `.pb.h` include path anchors on the path segment `pyramid` when present. | Derive from configured proto import root. |
| FlatBuffers service grouping | `pim/backends/flatbuffers_backend.py` | Service grouping depends on `.services.` and PYRAMID segment skipping. | Group by service module identity from the neutral contract model. |
| ROS2 service detection | `pim/backends/ros2_backend.py` | Service packages are identified by `.services.` package segment. | Generate from manifest service entries. |
| CMake generated globs | `subprojects/PYRAMID/CMakeLists.txt` | Compiles hardcoded `pyramid_data_model_*`, `pyramid_components_*`, `pyramid_services_*` patterns. | Consume generator manifest artifact roles. |
| Protobuf codegen path | `subprojects/PYRAMID/CMakeLists.txt` | Runs `protoc` over `pyramid/data_model/*.proto`. | Run `protoc` over manifest-listed proto files. |
| gRPC codegen path | `subprojects/PYRAMID/CMakeLists.txt` | Runs gRPC codegen over `pyramid/components/*.proto`. | Run gRPC codegen over manifest-listed service protos. |
| Plugin target naming | `subprojects/PYRAMID/CMakeLists.txt` | Plugin source names must match `pyramid_services_*_codec_plugin.cpp`. | Use manifest plugin names and source paths. |
| Checked-in protobuf support | `src/protobuf_support/*` | Tactical Objects-specific protobuf codec support is part of active path. | Move to generated or manifest-selected support; keep tactical shim as compatibility only. |

## Removal Architecture

Introduce a neutral `BindingContract` as the only object downstream generators
consume:

```text
BindingContract
  proto_import_roots
  proto_files
  type_modules
  service_modules
  endpoint_roles
  topic_bindings
  codec_backends
  transport_backends
  naming_policy
```

The parser remains responsible for extracting protobuf structure. The contract
builder is responsible for classification. Language generators and backend
generators should not inspect package strings to determine whether something is
a data model, service, provider, consumer, or topic.

## Naming Policies

Two naming policies should exist:

| Policy | Purpose |
|--------|---------|
| `pyramid_compat` | Preserve current generated names, namespaces, service constants, and output file names for existing apps and tests. |
| `generic` | Derive names directly from proto package, service, and message identities without reserved PYRAMID segments. |

Compatibility policy is the migration bridge. It is not allowed to leak into
the generic contract model.

## Manifest-Driven Build

The generator should write a manifest such as:

```json
{
  "layout": "generic",
  "proto_import_roots": ["subprojects/PYRAMID/proto"],
  "type_sources": ["example_telemetry_codec.cpp"],
  "marshal_sources": ["example_telemetry_cabi_marshal.cpp"],
  "service_sources": ["example_telemetry_services.cpp"],
  "flatbuffers_schemas": ["flatbuffers/cpp/example_telemetry.fbs"],
  "codec_plugins": [
    {
      "target": "example_telemetry_json_codec",
      "source": "example_telemetry_json_codec_plugin.cpp",
      "content_type": "application/json"
    }
  ],
  "protobuf_protos": ["telemetry.proto"],
  "grpc_service_protos": ["telemetry.proto"]
}
```

CMake should read this manifest and create targets from declared roles. This
removes all generated filename and package-prefix coupling from build logic.

## Migration Plan

### Step 1: Add Coupling Tests

Add generator-only tests that fail today and define the intended behavior:

- `example.telemetry` with message and service in the same package;
- `vendor.api.telemetry.v1` with imported message packages;
- duplicate message short names across different packages;
- service package without `.services.`;
- package names containing no `pyramid`, `data_model`, `components`,
  `provided`, or `consumed`;
- Tactical Objects fixture under `pyramid_compat` to prove no regression.

### Step 2: Build The Neutral Contract Model

Create a contract-builder module that produces `BindingContract` from
`ProtoFile` values. It should:

- classify types by message/enum declarations;
- classify services by service declarations;
- compute service type closure from request/response fields;
- assign role-neutral endpoints unless metadata says otherwise;
- attach no topics unless metadata declares them;
- produce stable schema IDs from fully qualified proto names.

### Step 3: Isolate Compatibility Naming

Move all of the following behind the `pyramid_compat` naming policy:

- `pyramid.data_model` recognition;
- `pyramid::domain_model`;
- `pyramid_data_model_types.hpp`;
- `pyramid_services_*` collapsed service names;
- PYRAMID segment skipping;
- `provided`/`consumed` package role inference.

Generic mode should have no special cases for those strings.

### Step 4: Replace Topic Coupling

Delete package-name inference from `standard_topics.py`. Replace it with a
topic declaration source:

```yaml
topics:
  - name: standard.entity_matches
    type: pyramid.data_model.tactical.ObjectMatch
    repeated: true
    direction: subscribe
    services:
      - pyramid.components.tactical_objects.services.provided
```

Plain arbitrary protos should generate no topic helpers unless this metadata is
present.

### Step 5: Make CMake Manifest-Driven

Refactor `subprojects/PYRAMID/CMakeLists.txt` so it consumes generated manifest
roles instead of globbing `pyramid_*` patterns. Keep old globbing only as a
temporary fallback for delivered legacy binding trees.

### Step 6: Generalize Protobuf, FlatBuffers, gRPC, And ROS2

Each backend should consume service/type modules from `BindingContract`.
Backend-specific code should not decide what a component or data model is by
package spelling.

### Step 7: Remove Tactical Objects From Shared Paths

Tactical Objects should remain a consumer of the generic binding system, not an
implicit rule provider. Its remaining special behavior should live in:

- its proto files;
- a Tactical Objects topic metadata file;
- application adapter code;
- compatibility tests.

No shared generator or build path should contain Tactical Objects-specific
branching.

## Acceptance Criteria

The hard-coupling removal is complete when:

- a non-PYRAMID proto fixture generates C++ JSON bindings, compiles, and
  dispatches through PCL;
- the same fixture builds with FlatBuffers and Protobuf enabled;
- CMake creates codec and marshal targets from the generated manifest;
- gRPC and ROS2 generation do not scan fixed `pyramid/...` directories;
- Tactical Objects tests still pass under `pyramid_compat`;
- a repository-wide search for these strings in reusable generator/build
  infrastructure finds only compatibility-policy code, tests, or docs:
  `pyramid.data_model`, `pyramid.components`, `tactical_objects`,
  `pyramid_data_model_`, `pyramid_components_`, `pyramid_services_`.

## Non-Goals

- Removing the PYRAMID brand from existing public generated APIs in one step.
- Removing Tactical Objects application code.
- Replacing PCL's opaque message model.
- Implementing every protobuf language feature before the generic subset works.

## Bottom Line

The reusable binding infrastructure should treat PYRAMID as one contract layout,
not as the contract model. The immediate architectural fix is to route every
generator and build decision through a neutral `BindingContract` plus an
explicit naming policy. Once that exists, hardcoded PYRAMID and Tactical Objects
assumptions can be confined to compatibility mode and then retired module by
module.
