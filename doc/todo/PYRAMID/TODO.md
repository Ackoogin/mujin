# PYRAMID — Tracked Follow-ups

## Protobuf codec plugin (generic transport coverage)

Status: open. Raised 2026-06-26 from the binding-performance regression check
([`doc/reports/PYRAMID/binding_performance_report_2026-04-28.md`](../../reports/PYRAMID/binding_performance_report_2026-04-28.md),
Linux Refresh section).

### Problem

The plugin-era generated C++ facade resolves codecs **only** through the codec
registry and **fails closed** with no static fallback. The registry is populated
by loading codec **plugins** (`.so`). Today only **json** and **flatbuffers**
codec plugins exist (`pyramid_codec_json_*`, `pyramid_codec_flatbuffers_*`).

There is **no `application/protobuf` codec plugin**. Consequences:

- Protobuf over the **generic transports** (local / shmem / socket), which go
  through the facade + registry, has no codec to resolve → encode/decode produce
  nothing.
- Protobuf is still fully functional via the **direct** codec
  (`*_protobuf_codec.hpp`, used by the codec microbenchmark) and via the **gRPC
  coupled transport** (`pyramid_grpc_transport`), which serialises protobuf
  directly rather than through the generic registry.

This is a **coverage gap**, not a performance regression — the C-ABI marshalling
boundary added no measurable overhead (see the report).

### Done so far

- `test_binding_performance` now **skips** the protobuf transport rows honestly
  (`GTEST_SKIP` + a `(skipped)` summary entry) when no `application/protobuf`
  codec is registered, instead of masking a hard `0/N` `EXPECT_EQ` failure.
  See `skipIfNoProtobufCodec` in
  [`subprojects/PYRAMID/tests/test_binding_performance.cpp`](../../../subprojects/PYRAMID/tests/test_binding_performance.cpp).

### Options to close

1. **Add an `application/protobuf` codec plugin** per component (mirror the json /
   flatbuffers plugin generation: `pyramid_codec_protobuf_<component>`), so the
   generic transports can carry protobuf via the registry like the other codecs.
   This is the real fix and restores the Windows-baseline protobuf transport rows.
2. If protobuf is intended to be **gRPC-coupled only** (not a standalone wire
   codec for socket/shmem), document that decision and keep the benchmark skip as
   the permanent, intended behaviour.

## Ada facade strictly codec-source-free (committed bindings = dist)

Status: done. Raised 2026-06-26. Closed 2026-06-26.

The committed `bindings/ada/generated` tree was trimmed to Ada `.ads/.adb` only
(removed stale C++ `.cpp/.hpp/.fbs` cruft). To make the committed tree match a
**plugin-only Ada dist**, the in-tree codec sources (json `*-types_codec.adb`,
`flatbuffers/ada/*`) should also drop out — but the Ada facade still compiles and
references them: post-W4 it fails closed via `Require_Codec`, yet `ada_codegen.py`
still emits `with …Types_Codec` clauses and the `From_Json` /
`Flatbuffers_Codec.From_Binary_*` fallback branches (dead when a codec plugin is
registered). C++ is already registry-only (no compiled-in codec fallback).

Closed by making `ada_codegen.py` emit registry-only service facades, routing
array schemas through the C-ABI codec registry, and removing the committed
json/flatbuffers Ada codec sources from the dist tree.

### Related direction

Fits the broader goal of making proto→binding→**plugin** a separate,
CI/CD-controllable generation step and dropping committed C++ bindings in favour
of build-time generation (see
[`doc/plans/PYRAMID/plugin_binding_v1_plan.md`](../../plans/PYRAMID/plugin_binding_v1_plan.md)).
