# Generic Contract Layout And Naming Policies

## Purpose

The binding generator originally assumed every contract used PYRAMID package
conventions — data-model types under `pyramid.data_model.*`, services under
`pyramid.components.*.services.*`, and a `pyramid::domain_model` /
`Pyramid.Data_Model.*` naming surface. This page documents the additions that
let the **same generator** also emit bindings for **arbitrary `.proto`
contracts** that do not follow those conventions, while keeping existing PYRAMID
output byte-for-byte identical.

Two layouts now exist, selected by `--contract-layout`:

| Layout | Meaning |
|--------|---------|
| `pyramid` (default) | PYRAMID compatibility. Reproduces the existing generated names, namespaces, file names, and topic behaviour exactly. |
| `generic` | Derives every name from the proto package / service / message identity. No reserved `pyramid` / `data_model` / `components` / `services` segments, no domain topics. |

Backward compatibility is a hard constraint: `pyramid` stays the default and its
output is unchanged.

## The Neutral Binding Contract

`pim/binding_contract.py` introduces a neutral model that downstream generators
consume instead of inspecting package strings:

- `build_contract(proto_files, layout)` classifies parsed `ProtoFile`s **by
  content, not by package name**:
  - `type_modules` — files that declare top-level messages/enums;
  - `service_modules` — files that declare `service`s;
  - `wrapper_modules` — service files that also declare messages/enums (the
    "services + refined request/response types in one file" shape);
  - `service_type_closures` — the request/response type closure per service;
  - `schema_ids` — stable IDs derived from fully-qualified proto names.

This means a service is recognised because the file declares a `service`, never
because the package contains `.services.`; a type module is recognised because
it declares messages/enums, never because the package starts with
`pyramid.data_model`.

### Target generic shape

A representative generic contract is: **one or two namespaces**; some files
carry general message types; other files carry **services plus refined message
types with the service declared flat in the main namespace** (not in a
`.services.` sub-package). `GenericNamingPolicy` homes services and refined types
flat in that namespace, e.g. package `example.telemetry` →
`example::telemetry` (C++) / `Example.Telemetry` (Ada).

## Naming Policies

`pim/binding_contract.py` defines a `NamingPolicy` protocol with two
implementations:

| Policy | Role |
|--------|------|
| `PyramidCompatNamingPolicy` | Preserves the current surface: `pyramid::domain_model`, `pyramid_data_model_types.hpp`, `pyramid_services_*` file prefixes, `provided`/`consumed` role inference, `Pyramid.Data_Model.*` Ada packages. |
| `GenericNamingPolicy` | Derives names from the proto package/service/message identity. Umbrella header: none. Service facades are **role-neutral** (no provided/consumed inference). |

C++ generators (`cpp_codegen.py`) accept an injected `naming_policy` (defaulting
to the pyramid-compat policy so existing call sites are unchanged). Ada type and
codec package names were already package-derived, so the generic Ada path reuses
them directly.

## What Generic Layout Produces

| Language / backend | Generic support |
|--------------------|-----------------|
| C++ JSON types + codecs | Yes — package-derived namespaces/headers |
| C++ role-neutral service facade | Yes — `ServiceHandler` + `dispatch(...)` per service |
| C++ FlatBuffers | Yes — policy-derived schemas/codecs, per-package type headers |
| C++ Protobuf | Yes — includes derived relative to the proto import root |
| C++ gRPC | Yes — import-root-relative `.pb.h`/`.grpc.pb.h`, policy-derived names |
| C++ ROS2 | Yes — service detection by parsed `service` decls, policy-derived names |
| Ada types + codecs | Yes — `Example.Telemetry.Types` / `..._Codec` |
| Ada role-neutral service facade | Yes — `AdaGenericServiceGenerator`, GNAT-compiled |

The Ada service facade uses a **separate** generator
(`AdaGenericServiceGenerator`) because the pyramid `AdaServiceGenerator` filters
RPCs to CRUD names only (`OP_PREFIXES` = Create/Read/Update/Delete) and drops
arbitrary RPCs. The generic facade emits per-RPC wire-name constants plus one
access-to-function handler per RPC in a `Service_Handlers` record, mirroring the
C++ `ServiceHandler`, and depends only on the generated types package.

## The Artifact Manifest

Every generation run (both layouts) writes `binding_manifest.json` into the
output directory. It records generated artifacts **by role** so the build can
consume declared roles instead of `pyramid_*` filename globs:

```json
{
  "layout": "pyramid" | "generic",
  "proto_import_roots": ["subprojects/PYRAMID/proto"],
  "proto_files": ["pyramid/data_model/....proto", ...],
  "types": ["..._types.hpp", ...],
  "json_codecs": ["..._codec.cpp", "..._codec.hpp", ...],
  "cabi": ["..._cabi.h", "..._cabi_marshal.cpp", ...],
  "services": ["..._provided.cpp", ...],
  "codec_plugins": [ {"target": "...", "source": "...", "content_type": "application/json"} ],
  "flatbuffers_schemas": ["flatbuffers/cpp/....fbs", ...],
  "protobuf_protos": [...],
  "grpc_service_protos": [...]
}
```

Paths are relative to the output directory (portable); `proto_files` are
relative to the proto import root. Roles not produced for the current
backend/language selection are empty.

## Manifest-Driven CMake

`subprojects/PYRAMID/CMakeLists.txt` gained an opt-in cache option:

```
PYRAMID_BINDING_SOURCE_MODE = glob (default) | manifest
```

- `glob` — legacy behaviour: build targets are populated from
  `pyramid_*` filename globs. Byte-identical to the pre-manifest build.
- `manifest` — the FlatBuffers schema list and the combined data-model + service
  JSON codec source list are selected from `binding_manifest.json` (read after
  the generator writes it during configure). The manifest's `json_codecs` role
  already carries both data-model and per-component codecs, so the separate
  `pyramid_components_*_codec.cpp` append is skipped to avoid double counting.

The selection logic lives in unit-tested CMake modules under
`subprojects/PYRAMID/cmake/`:

- `pyramid_manifest.cmake` — `pyramid_manifest_sources(...)`,
  `pyramid_manifest_plugins(...)`, `pyramid_manifest_field(...)` (uses
  `string(JSON ...)`);
- `pyramid_binding_sources.cmake` — `pyramid_binding_sources(OUT ROLE ... GLOBS ...)`
  choosing manifest vs glob, with glob fallback for absent roles.

Both are exercised in CI via `cmake -P` scripts under `cmake/tests/`, driven by
`tests/test_manifest_cmake_helper.py`. Manifest mode has been verified with a
real configure **and build**: `libpyramid_generated_codecs.a` links from
manifest-selected sources.

Staged remainder: the per-data-model-module C-ABI marshal loop still parses
module identity from `pyramid_data_model_*_cabi_marshal.cpp` filenames, and
codec-plugin targets and protoc/gRPC proto input paths remain on globs/fixed
paths. The default stays `glob` until those are also manifest-driven.

## Topic Metadata (data-driven)

Standard-topic generation is no longer domain-branched in code. `standard_topics.py`
loads topic metadata from an explicit JSON file and resolves topics purely from
it:

- `pim/topic_metadata/tactical_objects_topics.json` holds the Tactical Objects
  standard topics (wire names, `pyramid.data_model.tactical.*` payload types,
  provided/consumed subscribe/publish sets). This file **is** the pyramid-compat
  topic set — it contains the only domain data.
- The module default is that compat metadata (so pyramid output is unchanged);
  `EMPTY_METADATA` yields no topics for any package.
- Generic layout emits **no topics** (its service path does not call topic code,
  and empty metadata returns `({}, {})` even for a `tactical_objects` package).

Note: pub/sub topic generation is only *partially* realised (mostly wire-name
constants and some subscribe/publish scaffolding in the pyramid facades). The
5a change **relocated** that existing partial support into data and removed the
domain branching; it did not complete or extend pub/sub codegen. The JSON file
is the seam a fuller, explicit topic-declaration feature would build on.

## Backward-Compatibility Guarantee

With the default `pyramid` layout and `glob` CMake mode, the generator produces
**byte-identical** C++ (JSON/protobuf/flatbuffers/gRPC/ROS2) and Ada output to
the pre-refactor generator; only the new `binding_manifest.json` is added. This
is regression-locked by the Python test suite (`diff -qr` proofs) so PYRAMID and
Tactical Objects builds and tests are unaffected.

## Usage

```bash
# PYRAMID contracts (unchanged default)
python pim/generate_bindings.py proto/pyramid out/ --languages cpp,ada

# Arbitrary proto contract, all C++ backends
python pim/generate_bindings.py my_proto/ out/ \
    --languages cpp --backends json,flatbuffers,protobuf,grpc,ros2 \
    --contract-layout generic

# Arbitrary proto contract, Ada types + codecs + role-neutral service facade
python pim/generate_bindings.py my_proto/ out/ \
    --languages ada --backends json --contract-layout generic
```

To build with manifest-driven source selection:

```
cmake --preset all-off -DPYRAMID_BINDING_SOURCE_MODE=manifest
cmake --build build-all-off --target pyramid_generated_codecs
```

## Key Files

| Area | Files |
|------|-------|
| Neutral contract + naming policies | `subprojects/PYRAMID/pim/binding_contract.py` |
| Generator entry point + `--contract-layout` + manifest | `subprojects/PYRAMID/pim/generate_bindings.py` |
| C++ emitters (policy-aware) | `subprojects/PYRAMID/pim/cpp_codegen.py`, `pim/backends/*` |
| Ada emitters + generic service facade | `subprojects/PYRAMID/pim/ada_codegen.py` (`AdaGenericServiceGenerator`) |
| Topic metadata | `subprojects/PYRAMID/pim/standard_topics.py`, `pim/topic_metadata/tactical_objects_topics.json` |
| Manifest-driven CMake helpers | `subprojects/PYRAMID/cmake/pyramid_manifest.cmake`, `cmake/pyramid_binding_sources.cmake`, `cmake/tests/` |
| Tests | `subprojects/PYRAMID/tests/test_generic_*.py`, `test_binding_manifest.py`, `test_topic_metadata.py`, `test_manifest_cmake_helper.py`, `test_generic_ada.py` |
| Implementation progress log | `doc/reports/PYRAMID/generic_binding_implementation_progress.md` |
