# Handover: Plugin-Only Binding Model (Option A — marshal to a C ABI)

Status: 2026-06-24. Branch `feat/transport-codec-plugin-strategy-a`.
Companion to
[`transport_codec_plugin_transition.md`](./transport_codec_plugin_transition.md)
and the ABI findings in
[`../../reports/PYRAMID/codec_plugin_abi_findings.md`](../../reports/PYRAMID/codec_plugin_abi_findings.md).

This note records **what is committed**, **why the current shape is interim**, and
the **remaining work to reach the agreed end state**: a *plugin-only* binding
model where the component is built once against the contract, all codec/transport
implementation lives in plugins, and **Ada and C++ converge on a C ABI** for the
typed value crossing the plugin boundary (via **Option A — marshalling**).

## End state (agreed)

1. **Contract = typed interface only.** The generated `xxx.provided` /
   `xxx.consumed` (typed handler base + `publish/subscribe/invoke/dispatch`
   declarations + data-model types) is all the component compiles against. The
   component links **no** concrete codec/transport bodies.
2. **Plugin-only.** Codec/transport come *only* from loaded plugins. There is **no
   static `is_*_content_type` fallback**. With no plugin loaded the operation
   fails closed.
3. **Composition axes.** codec plugin × transport plugin for decoupled combos
   (json/flatbuffers/protobuf × socket/udp/shm/intra); **coupled "target"
   plugins** (gRPC, ROS2) supply codec **and** transport together.
4. **C-ABI typed value (Option A).** The value crossing `encode`/`decode` is a
   frozen **C struct** per contract type, so one plugin serves both languages.
   Native types stay ergonomic (`std::string`, `Unbounded_String`, vectors); the
   generator emits **marshalling** (`to_c` / `from_c`) at the boundary. One copy
   per call; no breaking change to handler/business-logic APIs.

## What is committed so far (foundation — keep)

These remain correct and are the substrate for the end state:

- **C ABI + registry + loader (Phases 1–3):** `pcl/pcl_codec.h`,
  `pcl/pcl_plugin.h`, `pcl/pcl_codec_registry.{h,c}`, `pcl/pcl_plugin_loader.{h,c}`.
  Fail-closed, ABI-versioned. Tests: `PclCodecRegistry`, `PclPluginLoader`.
- **Ada C-ABI bindings:** `subprojects/PCL/bindings/ada/pcl_plugins.ads`, with the
  cross-language proof `tests/ada/test_pcl_plugin_loader.*` (`ada_plugin_loader_abi`).
- **ABI findings report** — the language-neutral control ABI and the (now being
  superseded) language-specific typed boundary.

## What is committed but INTERIM (will be reworked)

These currently implement the *hybrid* "registry-first with **static fallback**"
and the *Strategy A* per-language typed boundary. They must move to plugin-only +
C-ABI:

- `pim/cpp_codegen.py` — emits the registry-first prologue **and** retains the
  static `if/else` codec path; emits per-(contract,codec) C++ codec plugins that
  take the **C++ native type** as `void*`.
- `pim/ada_codegen.py` — same registry-first/static-fallback wiring in the Ada
  facades; hand-written Ada JSON codec plugin
  (`bindings/ada/pyramid-services-tactical_objects-json_codec_plugin.*`) takes the
  **Ada record** as `value`.
- The regenerated facades, the C++ `CodecPluginSwap` / `tobj_cpp_app_client_*_plugin_e2e`
  tests, the Ada `test_ada_json_codec_roundtrip`, and the `--codec-plugin` example
  wiring. All pass today, but on the **language-specific** boundary.

> The interim hybrid is intentionally additive so the tree stays green; it is a
> stepping stone, not the destination.

## Remaining work to reach the end state (Option A)

### 1. Define the C-ABI data model (the frozen value layout)
- For each contract message type, generate a C struct in a stable header, e.g.
  `pyramid_ObjectDetail_c`, with: `pcl_str_t {const char*; uint32_t}` for strings,
  `pcl_slice_t {const void*; uint32_t}` for repeated fields, enums as `int32_t`,
  optionals as `has_x` flag + value, nested types inline. Version it
  (`PYRAMID_DATAMODEL_ABI_VERSION`).
- **Ownership/free contract for `decode`** (plugin → component, the hard part):
  variable-length fields are plugin-allocated. Define either (a) a plugin-owned
  result freed by a generated `pyramid_<Type>_c_free`, or (b) a caller-supplied
  allocator passed through `codec_ctx`. Recommend (a): symmetric with the existing
  `pcl_codec_t.free_msg` ownership model. Encode (component → plugin) can **borrow**
  (ptr+len into the native value) — read-only, no allocation.

### 2. Generate marshalling (Option A) on both sides
- C++ generator: `to_c(const ObjectDetail&, pyramid_ObjectDetail_c*)` /
  `from_c(const pyramid_ObjectDetail_c*, ObjectDetail&)`. Strings/vectors borrow on
  `to_c`; allocate + own on `from_c`.
- Ada generator: same as Convention-C marshalling between the Ada record and the C
  struct (Ada record need not be Convention C if marshalled; that's the Option A
  choice).
- Keep native types unchanged — business logic and handlers are untouched.

### 3. Rework the codec plugins to the C struct
- `encode(ctx, schema_id, const void* value, ...)` where `value` is now
  `const pyramid_<Type>_c*` (not the native type). `decode` fills a
  `pyramid_<Type>_c*` and the component `from_c`s it.
- One codec plugin per (contract, codec) now serves **all** languages. Prove it by
  loading the *same* `.so` from both the C++ and Ada round-trip tests.

### 4. Make the facade plugin-only (remove static fallback)
- Drop the `is_*_content_type` static `if/else` from `cpp_codegen.py` /
  `ada_codegen.py`. The facade marshals native→C-struct, calls the registry codec,
  and on no-codec returns a fail-closed status. The component stops linking
  `pyramid_generated_codecs` / the Ada codec bodies.
- Update tests/examples: zero-plugin operation no longer encodes, so every
  component/test must load a (default) plugin. Add a config-driven default-plugin
  load (e.g. a manifest in `<prefix>/lib/pcl/plugins/` per the plan's Build System
  Changes) so apps remain runnable.

### 5. Transport + coupled-target plugins
- Compose the registry-selected codec with the registry-selected **transport**
  (transport plugin ABI + loader already exist; wrap an existing transport, e.g.
  socket, as the first transport plugin).
- Add a **coupled target plugin** (gRPC or ROS2) that registers both a transport
  vtable and a codec under the target's content_type — proving the "target informs
  both" path.

### 6. Generalize + clean up
- Roll the C-ABI marshalling across all contracts; delete the interim static
  fallback and the per-language typed codec plugins; refresh the committed
  generated snapshots; update `CodecPluginSwap` and the e2e suites to the
  plugin-only/default-plugin model.

## Suggested sequencing
Phase 1 (tactical, C++): items 1–4 on one contract → swap-without-rebuild green.
Phase 2: load the *same* C-ABI plugin from Ada (item 3 cross-language proof).
Phase 3: item 5 (transport + gRPC/ROS2 target). Phase 4: item 6 (generalize).

## Verification notes for the next session
- Build/verify in `build-flatbuffers-only` (json+flatbuffers, socket transport,
  GNAT available). Socket e2e need working loopback (the Codex sandbox blocks it;
  this host does not — run them here).
- Full pre-existing suite is **578/578** on this host; keep it green per phase
  until item 6 deliberately flips components to plugin-only.
