# PYRAMID Plugin Binding — v1 Plan

Status: 2026-06-25. Branch `feat/transport-codec-plugin-strategy-a`.
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
- ◑ gRPC coupled target plugin — implemented, **gated** behind `PYRAMID_ENABLE_GRPC`,
  not runtime-verified (restricted network here).
- Full suite **587/587** green (`build-flatbuffers-only`, GNAT).

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

### W4 — Ada strictly plugin-only (incl. aliases)
- Marshal scalar aliases across the Ada plugin boundary as the C++ plugin
  expects (string → `pyramid_str_t`, numeric → scalar pointer): alias branches
  in `Try_Cabi_Registry_Encode/Decode` + alias pointer packages.
- Remove the Ada facade's static no-plugin path → fail closed like C++.

### W5 — Deployment / default-plugin load
- Config-driven default-plugin load (manifest in `<prefix>/lib/pcl/plugins/`)
  so apps run without explicit `--codec-plugin`/`--transport-plugin`.
- `stage_plugin_deploy.sh`: stage codec **and** transport plugins per component,
  the minimal core link set, and a manifest the runtime auto-loads.

## Suggested sequencing

1. **W2** (config pass-through) — enables W1/W3 to pass plugin config cleanly.
2. **W1** (client → transport plugin) — proves "core libs only" end-to-end.
3. **W3** (shm plugin; gRPC verify where possible).
4. **W4** (Ada alias fail-closed).
5. **W5** (default-plugin manifest + deployment polish).

Keep `build-flatbuffers-only` green per step; gRPC runtime verification is gated
on a network-capable build environment.
