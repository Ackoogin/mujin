# PYRAMID Plugin Binding — End-State Review

Status: 2026-06-25. Branch `feat/transport-codec-plugin-strategy-a`.
Companion to [`plugin_only_binding_handover.md`](./plugin_only_binding_handover.md)
and [`plugin_only_binding_progress.md`](./plugin_only_binding_progress.md).

This document is a **review snapshot**: what the runtime plugin binding looks
like now, what is finished, and the one remaining purity gap. Verified state:
**`build-flatbuffers-only` → 587/587 CTest green** (GNAT/Ada present).

Commits on the branch:

| Commit | Content |
|--------|---------|
| `dbc40fa` | Clean typed interface (C++) + Ada C-ABI parity + cross-language proof |
| `d0d3149` | Transport plugin + decoupled codec×transport composition |
| `53b2cc3` | Plugin-only flip — C++ fail-closed + default-plugin auto-load |
| `c76405a` | Ada uses the same C++ codec plugin; separate Ada plugin deleted |

---

## 1. The agreed end shape

The component is compiled **once** against the typed contract. It links **no**
concrete codec/transport bodies for dispatch. Everything below the clean,
language-native interface is hidden.

```
   ┌────────────────────────────────────────────────────────────┐
   │  COMPONENT / CLIENT CODE  (C++ or Ada)                       │
   │                                                              │
   │   consumer.objectOfInterestCreateRequirementAsync(req)       │   <- native types
   │   consumer.subscribeEntityMatches([](auto& matches){...})    │      (std::vector<ObjectMatch>,
   │                                                              │       Identifier, Ada records …)
   └───────────────────────────┬──────────────────────────────────┘
                               │  clean native types / methods only
   ┌───────────────────────────▼──────────────────────────────────┐
   │  GENERATED SERVICE FACADE (contract)                         │
   │   • typed ConsumedService / ProvidedService (C++)            │
   │   • typed Ada facade packages                                │
   │   • marshals native  <─>  frozen per-message C struct        │   <- to_c / from_c
   │   • selects codec from the runtime registry (by content_type │      (hidden)
   │     + schema_id); fail-closed when none                      │
   └───────────────────────────┬──────────────────────────────────┘
                               │  whole pyramid_<Type>_c struct (one cast)
   ┌───────────────────────────▼───────────┐   ┌────────────────────────┐
   │  CODEC PLUGIN  (.so, loaded at runtime)│   │ TRANSPORT PLUGIN (.so)  │
   │   encode(C struct) -> bytes            │   │  publish/serve/...      │
   │   decode(bytes) -> C struct            │   │  (socket, … )           │
   │   JSON / FlatBuffers / …               │   │  registry-selected      │
   └────────────────────────────────────────┘   └────────────────────────┘
```

Key property: **all marshalling, codec selection, and plugin binding live
below the generated interface line.** The client never sees `pcl_msg_t`,
`toJson`, a content-type string branch, or a plugin handle in its call path.

---

## 2. The cross-language plugin (one `.so` serves both languages)

The value crossing the plugin boundary is a **frozen per-message C struct**
(`pyramid_<Type>_c`: `pyramid_str_t` for strings, `pyramid_slice_t` for repeated,
`int32` enums, `uint8` bools, `has_x` flags, nested inline). Because that layout
is language-neutral, **one C++-built codec plugin serves C++ and Ada**.

```
        C++ client                         Ada client
            │ native C++ types                  │ native Ada records
            ▼                                    ▼
   C++ facade marshals               Ada facade marshals
   native <─> pyramid_<T>_c          native <─> pyramid_<T>_c   (To_C / From_C)
            │                                    │
            └───────────────┬────────────────────┘
                            ▼
            ┌───────────────────────────────────────┐
            │  pyramid_codec_json_tactical_objects.so │   ONE plugin, both langs
            │  (C++-built; consumes pyramid_<T>_c)    │
            └───────────────────────────────────────┘
```

Proof in the suite: `test_ada_cpp_codec_roundtrip` loads the **C++-built**
`pyramid_codec_json_tactical_objects` `.so` from Ada and round-trips a message
Ada-native → `To_C` → plugin → `From_C` → Ada-native. **There is no
Ada-specific codec plugin** — the hand-written one was deleted (`c76405a`).

---

## 3. Decoupled codec × transport, and coupled targets

Codec and transport are independent plugins composed at runtime; a "target"
plugin (gRPC/ROS2) may supply both under one content_type.

```
   registry-selected CODEC  ×  registry-selected TRANSPORT      (decoupled)
        json / flatbuffers          socket / udp / shm / intra

   ── or ──

   coupled TARGET plugin  ──>  registers BOTH a transport vtable
   (gRPC, ROS2)                and a codec under one content_type
```

- `pcl_transport_socket_plugin.c` wraps the socket transport as a loadable
  transport plugin (`pcl_transport_plugin_entry`).
- `TransportCodecPluginComposition.FlatBuffersCodecPluginOverSocketTransportPluginPubSub`
  proves a pub/sub round-trip with the codec from a codec plugin **and** the
  transport from a transport plugin (no statically chosen transport).
- The coupled **gRPC** target plugin (`pyramid_grpc_coupled_plugin.cpp`) is
  implemented and **gated behind `PYRAMID_ENABLE_GRPC`**; it was not
  runtime-verified here because this environment cannot fetch the gRPC deps
  (restricted network). `build-flatbuffers-only` does not build it and stays
  green.

---

## 4. What the client actually writes (clean interface)

```cpp
// C++ — tactical_objects_test_client.cpp (zero codec/wire knowledge)
TacticalObjectsTestClient client{executor, "application/json"};
auto fut = client.createRequirementAsync(makeRequirement());   // -> future<Result<Identifier>>
// subscribe delivers decoded native types:
consumer_.subscribeEntityMatches(
    [this](const std::vector<ObjectMatch>& m){ onEntityMatches(m); });
```

```ada
--  Ada — active-find client: native records in/out; marshalling hidden
Response := Provided.Invoke_Object_Of_Interest_Create_Requirement (Exec, Request);
```

The only "plugin" the client touches is **deployment wiring** in `main()` /
test harness (load a `.so` path, or rely on the default-plugin auto-load) — not
the call path.

---

## 5. Plugin-only / fail-closed status

```
                         encode/decode dispatch
   ┌──────────────┬───────────────────────────┬──────────────────────────┐
   │              │ message types              │ scalar aliases (Identifier)│
   ├──────────────┼───────────────────────────┼──────────────────────────┤
   │ C++ facade   │ registry plugin ONLY       │ registry plugin ONLY      │
   │              │ (fail-closed, no static)   │ (pyramid_str_t)           │
   ├──────────────┼───────────────────────────┼──────────────────────────┤
   │ Ada facade   │ registry plugin (C struct) │ facade static path        │  <- RESIDUAL
   │              │ → same C++ .so             │ (hidden; not plugin)      │
   └──────────────┴───────────────────────────┴──────────────────────────┘
```

- **C++:** strictly plugin-only and fail-closed. No `is_*_content_type` static
  fallback remains in the generated facade. With no codec registered, encode/
  decode fail closed (`PluginOnly.FacadeFailsClosedWithoutCodec`). A
  default-plugin auto-load (`pcl_codec_registry_load_plugins_from_paths` /
  `_from_env`, a gtest global environment, and app `main()`s) keeps everything
  runnable.
- **Ada:** message types are plugin-only via the shared C++ `.so`. The raw
  fallback that used to hand a native Ada record to the plugin was removed (it
  corrupted controlled types — the `finalize/adjust` crash). **Residual:**
  scalar aliases (e.g. `Identifier`) are still encoded/decoded by a
  *facade-internal* path rather than crossing to the plugin as `pyramid_str_t`,
  and the Ada facade keeps a static path for the no-plugin case. This is hidden
  from the client (clean interface intact) but is not strictly fail-closed
  plugin-only like C++.

---

## 6. Remaining work (the one gap)

To make Ada **strictly** plugin-only/fail-closed like C++:

1. Marshal scalar aliases across the Ada plugin boundary as the C++ plugin
   expects — string aliases as `pyramid_str_t`, numeric/enum aliases by scalar
   pointer — by adding alias branches to `Try_Cabi_Registry_Encode/Decode`
   (alias pointer packages + inline `Interfaces.C.Strings` marshalling).
2. Remove the Ada facade's static no-plugin path so it fails closed.

Everything else in the handover (items 1–6) is complete.

---

## 7. Where things live

| Concern | File(s) |
|---------|---------|
| C-ABI struct + C++ marshalling (generator) | `subprojects/PYRAMID/pim/cabi_codegen.py` |
| Ada C-ABI struct + marshalling (generator) | `subprojects/PYRAMID/pim/ada_cabi_codegen.py` |
| C++ facade generator (typed iface, plugin-only) | `subprojects/PYRAMID/pim/cpp_codegen.py` |
| Ada facade generator | `subprojects/PYRAMID/pim/ada_codegen.py` |
| Codec registry (many codecs/content_type) | `subprojects/PCL/src/pcl_codec_registry.c` |
| Plugin loader + default-load helpers | `subprojects/PCL/src/pcl_plugin_loader.c` |
| Socket transport plugin | `subprojects/PCL/src/pcl_transport_socket_plugin.c` |
| Coupled gRPC target (gated) | `subprojects/PYRAMID/plugins/pyramid_grpc_coupled_plugin.cpp` |
| Cross-language proof test | `subprojects/PYRAMID/tests/ada/test_ada_cpp_codec_roundtrip.adb` |
| Fail-closed test | `subprojects/PYRAMID/tests/test_plugin_only_fail_closed.cpp` |
| Transport×codec composition test | `subprojects/PYRAMID/tests/test_transport_codec_plugin_composition.cpp` |

---

## 8. Build / verify

```sh
cmake --preset flatbuffers-only
cmake --build --preset flatbuffers-only-release --target pyramid_ada_all --parallel "$(nproc)"
cmake --build --preset flatbuffers-only-release --parallel "$(nproc)"
ctest --test-dir build-flatbuffers-only -C Release --output-on-failure   # 587/587
```
