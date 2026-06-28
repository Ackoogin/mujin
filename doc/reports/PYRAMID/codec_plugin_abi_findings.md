# Codec/Transport Plugin ABI Findings (incl. Ada)

Status: 2026-06-24. Companion to the plugin-system architecture reference
[`subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md).
Records what was implemented for Strategy A and — the focus of this note — the
**ABI behaviour of the runtime codec plugins across the C, C++ and Ada
boundaries**, backed by tests that are checked in.

## What shipped

Strategy A ("per-type codec exports + runtime codec registry") is implemented
and verified end to end:

| Layer | Artefact |
|-------|----------|
| Codec ABI | `pcl/pcl_codec.h` — `pcl_codec_t` vtable, `PCL_CODEC_ABI_VERSION`, `pcl_codec_plugin_entry` symbol contract, plus a `free_msg` buffer-ownership hook added beyond the original plan sketch |
| Transport plugin ABI | `pcl/pcl_plugin.h` — `pcl_transport_abi_version` / `pcl_transport_plugin_entry`, `PCL_TRANSPORT_ABI_VERSION` |
| Runtime registry | `pcl/pcl_codec_registry.{h,c}` — register by `content_type`, fail-closed on ABI mismatch / duplicate / unknown |
| Loader | `pcl/pcl_plugin_loader.{h,c}` — `dlopen`/`LoadLibrary`, ABI check, fail-closed |
| Codec plugins | per-`(contract, codec)` C++ MODULE DLLs emitted by `cpp_codegen.py` with typed `encode/decode` dispatched by `schema_id` |
| Facade wiring | generated C++ facade consults the registry first, falls back to the static `if/else` path when no codec is registered |
| Ada bindings | `subprojects/PCL/bindings/ada/pcl_plugins.ads` — Convention-C bindings for the registry/loader/codec vtable |

Verified by, among others: `PclCodecRegistry` (8), `PclPluginLoader` (5),
`CodecPluginSwap` (3), the converted `tobj_cpp_app_client_{json,flatbuffers}_plugin_e2e`,
and the Ada `ada_plugin_loader_abi` cross-language test. Full suite green
(578/578 in the loopback-capable environment).

## The central question: what crosses the plugin boundary safely?

The plugin boundary has **two distinct ABIs**, and they have different
portability:

### 1. The control ABI is language-neutral

`pcl_codec_t`, `pcl_msg_t`, `pcl_status_t`, the registry handle and the loader
are plain C with stable, `Convention C` layouts. Any language with a C FFI can
drive them. This is proven from Ada: `test_pcl_plugin_loader.adb` creates a C
registry, `dlopen`s a **C** codec plugin (`pcl_codec_stub_plugin`) through
`pcl_plugin_load_codec`, fetches the vtable from the registry, and calls
`encode`/`decode`/`free_msg` through the C function pointers — all from an Ada
main. The 13-byte marker round-trips and the missing-plugin case returns
`PCL_ERR_NOT_FOUND`.

**Implication:** the registry + loader + vtable are a single shared mechanism;
they do not need a per-language re-implementation.

### 2. The *typed value* boundary is language-specific

`encode(ctx, schema_id, const void* value, ...)` and
`decode(ctx, schema_id, msg, void* out_value)` move a **native in-memory value**
of the type named by `schema_id`. In Strategy A the plugin casts that `void*`
to a concrete generated type (`pyramid::domain_model::ObjectDetail` in C++,
`Pyramid.Data_Model.Tactical.Object_Detail` in Ada). Those layouts are **not**
the same across languages: a C++ `std::string`/`std::optional`/`std::vector`
member has nothing in common with an Ada `Unbounded_String`/record layout.

**Implication — the headline ABI rule:** *a codec plugin is matched to the
language of the component it serves.* A C++ component loads C++ codec plugins; an
Ada component loads Ada codec plugins. The wire format (JSON/FlatBuffers/…) is
interoperable across languages because it is just bytes; the **codec plugin
binary** that produces those bytes from a typed value is not. This is not a
limitation of the design so much as an inherent property of Strategy A's
"keep full typing, move the codec body into a DLL" trade-off (see the plan's
Strategy A vs B/C table). Strategy B/C (reflective layout / canonical tree)
would make a single plugin contract-agnostic *and* language-agnostic, at the
cost of a frozen layout or a lowering copy.

## Ada-specific findings (the "ABI issues?" investigation)

Building an Ada codec plugin that the C loader can `dlopen` raised three
concrete questions; all were settled experimentally before any production code
was written.

1. **Can a GNAT-built shared library be `dlopen`'d and self-initialise?**
   Yes. Build it as a GNAT *encapsulated standalone library*:
   ```
   for Library_Kind use "dynamic";
   for Library_Standalone use "encapsulated";
   for Library_Auto_Init use "true";
   for Library_Interface use (... "Pcl_Plugins", "Pcl_Bindings", ...);
   ```
   and export the entry point with `pragma Export (C, Plugin_Entry,
   "pcl_codec_plugin_entry");`. `Library_Auto_Init` installs the elaboration
   (`adainit`) as a library constructor, so the loader does not need to know the
   library is Ada. The exported symbol appears as
   `pcl_codec_plugin_entry@@<lib>` (default-versioned); `dlsym` by the plain name
   resolves it.

2. **Two GNAT runtimes in one process — does the typed value survive?**
   Yes, for plain-data records. An encapsulated SAL bundles its own copy of the
   GNAT runtime *code*, so an Ada main loading an Ada plugin has two runtimes.
   A spike round-tripped a record containing an `Unbounded_String`
   (`(Id => "obj-42", Quality => 0.875)`): the main built the record, the plugin
   read it by address, serialised, and decoded back into a main-owned record;
   the value came back intact. This works because Ada heap allocation bottoms
   out at the process-wide libc `malloc`, so `Unbounded_String` storage created
   by one runtime can be read and freed by the other. **Caveat:** this reasoning
   covers plain data (which generated PYRAMID types are). Types with custom
   `Controlled` finalisation, task components, or secondary-stack-resident state
   could still be hazardous across the dual-runtime boundary and were not
   exercised; if such types ever enter a contract, prefer a *standard* (non-
   encapsulated) SAL loaded only by an Ada main so a single runtime owns
   everything.

3. **Linking.** The loader needs `-ldl` on Linux (`${CMAKE_DL_LIBS}`, already
   added to `pcl_core`); the Ada test/app gpr projects must add `-ldl` to their
   linker switches. `chars_ptr` ↔ `System.Address` is done with
   `Ada.Unchecked_Conversion` (both are address-width); `Interfaces.C.Strings.
   New_String` yields a libc-`malloc` buffer appropriate for `Out_Msg.Data`,
   released by `Interfaces.C.Strings.Free` in `free_msg`.

## Recommendation

- Keep the control ABI (registry/loader/vtable) as the single shared,
  versioned C contract; bind it once per language (done for C/C++ and Ada).
- Treat codec plugins as **language-matched** artefacts. The generator emits a
  C++ plugin and an Ada plugin per `(contract, codec)`; deployment loads the one
  matching the component's language. Wire formats remain cross-language on the
  wire.
- For Ada plugins, use encapsulated standalone libraries with `Library_Auto_Init`
  and restrict the typed boundary to plain-data generated records (the current
  contract shape). Revisit with a standard SAL or Strategy B/C if controlled
  types or a single contract-agnostic codec binary become requirements.
