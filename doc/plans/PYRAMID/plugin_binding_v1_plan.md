# PYRAMID Plugin Binding — v1 Plan

Status: 2026-06-26 — **v1 COMPLETE** (W1–W6 done; criteria 1–4 met; 592/592
green). Branch `feat/transport-codec-plugin-strategy-a`.
Builds on [`plugin_binding_end_state_review.md`](./plugin_binding_end_state_review.md)
(current state + diagrams) and [`plugin_only_binding_handover.md`](./plugin_only_binding_handover.md).

## v1 definition (acceptance criteria)

1. **Clients link only core static libs.** A component/client links the
   framework (`pcl_core`) and its generated contract (typed facade +
   native↔C-struct marshalling). It links **no** codec, **no** transport, **no**
   wire deps (JSON / FlatBuffers / gRPC). Everything else is a plugin.
2. **Plugins handle everything else**, loaded at runtime as `.so`:
   transport (**socket, shm**) and codec (**json, flatbuffers**), plus the
   coupled **gRPC** target (transport+codec under one content_type).
3. **Generic config pass-through.** A client switch flows, as an opaque
   `config_json` string, through the loader into the plugin's entry point —
   uniform for **both** transport and codec plugins.
4. **Fail-closed, both languages.** With no plugin, encode/decode/transport
   fail closed. Holds for C++ **and** Ada (incl. scalar aliases).

```
   CLIENT (links: pcl_core + generated contract/marshalling ONLY)
        │  native types in / out; config switches
        ▼
   GENERATED FACADE ── marshals native <─> pyramid_<T>_c ──┐
        │  config_json (opaque) ───────────────────────────┼──► passed to plugins
        ▼                                                   ▼
   CODEC PLUGIN .so            TRANSPORT PLUGIN .so   (socket | shm | … )
   (json | flatbuffers)        composed at runtime
        └───────────── coupled TARGET plugin (gRPC): codec + transport ─────────┘
```

## Current state (done)

- ✅ Clean typed interface (C++ `ConsumedService`/`ProvidedService`; Ada facade) —
  native types, marshalling hidden. `dbc40fa`.
- ✅ C-ABI per-message struct boundary; **one cross-language codec `.so`** serves
  C++ and Ada (`test_ada_cpp_codec_roundtrip`). `dbc40fa`, `c76405a`.
- ✅ C++ plugin-only / fail-closed + default-plugin auto-load. `53b2cc3`.
- ✅ **Marshalling split** (`pyramid_generated_marshal`): plugin-only clients link
  marshalling only — no wire codec, no FlatBuffers. `c18c232`.
- ✅ Transport plugin ABI + **socket transport plugin** + decoupled codec×transport
  composition test. `d0d3149`.
- ✅ Transport config pass-through (`config_json` on `pcl_transport_plugin_entry`
  / `pcl_plugin_load_transport`).
- ✅ gRPC coupled target plugin — implemented, **gated** behind `PYRAMID_ENABLE_GRPC`,
  now **built + runtime-verified on Linux** (`2c9ce2c`). gRPC v1.72 + protobuf 30 +
  abseil + BoringSSL fetch and build from source here (the old "restricted network"
  caveat no longer holds). `test_grpc_transport_smoke` (real unary round-trip),
  `BindingPerformanceTest.Grpc_Tcp` (200/200 unary calls), and
  `test_grpc_coupled_plugin_load` (coupled `.so` exposes transport + codec under
  `application/grpc`) all pass. Required fixing protobuf `.pb.h` includes
  (short package-path → dotted protoc names) broken by the proto rename — a latent
  gap since the JSON/FB/Ada paths never touch `.pb.h`.

### v1 progress (this branch)

- ✅ **W2** Codec config pass-through: `pcl_codec_plugin_entry(config_json)`,
  `PCL_CODEC_ABI_VERSION=2`, `pcl_plugin_load_codec(path, config_json, …)`; config
  exposed via `codec_ctx`. Generator + all C++/Ada callers updated. Stub plugin
  records config; loader test asserts pass-through.
- ✅ **W1** Clients link only core libs: `tactical_objects_test_client`, **both Ada
  clients**, and `tactical_objects_app` (server) all load transport via
  `pcl_plugin_load_transport` (socket + shm) — verified `nm` shows zero static
  transport-create symbols. `stage_plugin_deploy.sh` link set trimmed.
- ✅ **W3** shm transport plugin (`pcl_transport_shared_memory_plugin`), config
  pass-through + fail-closed loader tests; proven end-to-end via
  `pyramid_bridge_e2e` (app on shm bus ↔ Ada bridge). udp plugin skipped
  (optional); gRPC runtime verify still network-gated.
- ✅ **W5** Default-plugin manifest: `pcl_codec_registry_load_plugins_from_manifest`,
  apps honor `--codec-manifest` / `PCL_CODEC_MANIFEST`; deploy script emits
  `codec_manifest.txt` + `transport_manifest.txt`; `tobj_cpp_app_client_manifest_e2e`
  runs app+client with no `--codec-plugin`.
- ✅ **W6** Per-module marshalling: `pyramid_marshal_<module>.a` per data-model
  module (+ aggregate `pyramid_generated_marshal` preserved for Ada/C++); deploy
  stages only each component's module closure (tactical_objects → common+tactical;
  autonomy_backend → common+autonomy). Verified a deployed client links/runs
  against its closure alone.
- ✅ **W4** Ada strict codec fail-closed + scalar-alias marshalling. Scalar
  aliases now cross the C-ABI codec plugin (string → `pyramid_str_t`, numeric →
  scalar pointer; `<Alias>_Pointers` packages). A `Require_Codec` gate at every
  encode/decode choke point raises `Program_Error` when no codec plugin is
  registered — the Ada facade now fails closed like C++ (no static fallback
  reachable without a plugin). Ada clients, the Ada bridge main, and the
  generated-bindings test load codecs from `PYRAMID_CODEC_PLUGINS`;
  `ada_generated_bindings_roundtrip` adds a negative assertion proving the
  facade raises without a registered codec.
- Full suite **592/592** green (`build-flatbuffers-only`, GNAT) after
  W1/W2/W3/W4/W5/W6 — **v1 acceptance criteria 1–4 all met**.

## Remaining work to reach v1

### W1 — Client links only core libs (drop static transport)
- Migrate `tactical_objects_test_client` (and the Ada clients / app mains) to
  **load the transport plugin** (`pcl_plugin_load_transport(socket_plugin, cfg,
  …)` + `executor.setTransport(vtable)`) instead of `pcl_socket_transport_create_*`.
- Drop `pcl_transport_socket` from the client link. End state client link set:
  `pcl_core` + `pyramid_generated_marshal` (+ the generated contract `.cpp`).
- Update the e2e harness to pass the transport plugin path to the client; update
  `stage_plugin_deploy.sh` `LINK_LIBS` to the final minimal set.

### W2 — Codec config pass-through (ABI bump)
- `pcl_codec_plugin_entry(const char* config_json)` (was `(void)`); bump
  `PCL_CODEC_ABI_VERSION`.
- `pcl_plugin_load_codec(path, const char* config_json, registry, &handle)`.
- Thread `config_json` into `codec_ctx` so `encode`/`decode` can honor it.
- Update: codec-plugin generator (`cpp_codegen.py` `_write_codec_plugin_impl`),
  every `load_codec` caller (default-load helper, apps, tests), and the Ada
  `Pcl_Plugins` binding + Ada loader.
- A client `--codec-option k=v` (or `--codec-config '{…}'`) reaches the codec.

### W3 — Plugin set: shm transport (+ keep socket); gRPC verified
- Wrap `pcl_transport_shared_memory` as a transport plugin (mirror
  `pcl_transport_socket_plugin.c`; read mode/params from `config_json`).
- (Optional) udp transport plugin for completeness.
- Runtime-verify the gRPC coupled target in a `PYRAMID_ENABLE_GRPC` build where
  the gRPC deps can be fetched (blocked in this sandbox — needs network).

### W4 — Ada strictly plugin-only (incl. aliases) — ✅ DONE
- ✅ Marshal scalar aliases across the Ada plugin boundary as the C++ plugin
  expects (string → `pyramid_str_t`, numeric → scalar pointer): alias branches
  in `Try_Cabi_Registry_Encode/Decode` + `<Alias>_Pointers` packages.
  (`pim/ada_codegen.py` `_collect_alias_schema_bindings`.)
- ✅ Fail closed like C++: a generated `Require_Codec (Content_Type)` gate at
  each encode/decode choke point raises `Program_Error` when no codec plugin is
  registered for the content type (gRPC content type exempt — it carries its
  own transport+codec). The static `To_Json`/`Flatbuffers_Codec` branches remain
  only as unreachable-without-a-plugin fallbacks behind the gate.
- ✅ Rewired consumers that relied on the old static path to load codecs from
  `PYRAMID_CODEC_PLUGINS` (`pcl_codec_registry_load_plugins_from_env`, newly
  bound in `pcl_plugins.ads`): both Ada clients, the Ada bridge main, and the
  generated-bindings test. `ada_generated_bindings_roundtrip` now asserts the
  facade raises without a registered codec, and decodes the (now JSON-encoded)
  `Identifier` responses via the codec rather than assuming raw bytes.

### W5 — Deployment / default-plugin load
- Config-driven default-plugin load (manifest in `<prefix>/lib/pcl/plugins/`)
  so apps run without explicit `--codec-plugin`/`--transport-plugin`.
- `stage_plugin_deploy.sh`: stage codec **and** transport plugins per component,
  the minimal core link set, and a manifest the runtime auto-loads.

### W6 — Per-module marshalling (polish; churn isolation)
Rationale: the data model and generated artifacts are version-controlled, so a
change in one data-model module must **not** churn the shipped artifacts of
components that don't use it. Today `pyramid_generated_marshal` is one static lib
(one `.o` per module — `common`, `base`, `tactical`, `autonomy`, `sensors`,
`sensorproducts`, `radar`); the linker already trims a client binary to its
module closure, so *binaries* are minimal, but the single lib (and a deployment
that ships it whole) re-versions whenever **any** module changes.

Polish (do per **data-model module**, NOT per service component — components
share `common`/`base`, so per-component would duplicate shared modules):
- Build a marshalling lib per module (`pyramid_marshal_common.a`, `_base.a`,
  `_tactical.a`, …); each component/client links only its module closure.
- `stage_plugin_deploy.sh`: stage only the closure modules per component so a
  `tactical_objects` deployment changes only when `tactical`/`common`/`base`
  change — not when `autonomy`/`sensors`/… change.
- Net effect: unrelated data-model edits produce no diff in an unrelated
  component's deployment dir / version-controlled artifacts.

(Not required for v1 correctness — link-time trimming already yields minimal
binaries — but it isolates version-control churn across the modular data model.)

## Sequencing (completed)

1. ✅ **W2** (config pass-through) — enables W1/W3 to pass plugin config cleanly.
2. ✅ **W1** (client → transport plugin) — proves "core libs only" end-to-end.
3. ✅ **W3** (shm plugin; gRPC still network-gated for runtime verify).
4. ✅ **W4** (Ada alias fail-closed).
5. ✅ **W5** (default-plugin manifest + deployment polish).
6. ✅ **W6** (per-module marshalling, churn isolation).

`build-flatbuffers-only` stayed green per step (592/592). **gRPC is now built and
runtime-verified on Linux** (`2c9ce2c`) — configure with `-DPYRAMID_ENABLE_GRPC=ON
-DPYRAMID_ENABLE_PROTOBUF=ON` (or the `all-on` preset); the deps fetch and build
from source. Follow-up direction (not blocking v1): make the proto→binding→plugin
generation a **separate, CI/CD-controllable step** and drop the committed C++
protobuf bindings in favour of build-time generation.
