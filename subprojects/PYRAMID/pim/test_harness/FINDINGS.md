# New PIM proto — plugin-system viability (record)

**Question:** can components from the new proto set (`subprojects/PYRAMID/pim/test/`)
communicate through the PCL/plugin runtime?

**Verdict: yes — verified.** Generation succeeds (48 protos → 203 messages, 13
enums, 42 services). The generator defects the duplicate-heavy PIM tree
surfaced (non-literal constants, missing service-local wrapper messages,
duplicate short names / flat-namespace collisions, short-name type resolution,
and a batch of Ada codec/service regressions) were fixed in `cpp_codegen.py`,
`cabi_codegen.py`, `ada_codegen.py`, and `ada_cabi_codegen.py`, with the old
proto tree confirmed regression-free throughout.

Verified end states:

- **C++/C-ABI plugin path**: `build_comms_test.sh`/`.bat` green (provider +
  client over PCL with the JSON codec plugin); every new-proto codec +
  cabi_marshal compiles.
- **No-relink plugin use**: `plugin_load_test.cpp` proves a codec-free client
  binary can load a prebuilt codec `.so`/`.dll` at runtime via
  `pcl_plugin_load_codec()`, with `config_json` threaded through the loader.
  Green on Linux and Windows.
- **Ada**: every generated Ada unit — data model, codecs, C-ABI, and service
  bindings — object-compiles with GNAT for both old and new proto trees
  (new PIM proto: 112 `.adb` → 244 objects, 0 failures; old proto: 0 failures).
- **Windows parity**: `.bat` equivalents drive MSVC via `_msvc_env.bat`; the
  full runtime plugin build (16 codec + 3 transport plugin DLLs) is green.
- **Other backends**: protobuf compiles end-to-end against real `protoc`
  output; flatbuffers schemas + codec headers validate via in-repo `flatc`;
  grpc/ros2 generate and target Component-NS (full compile needs grpc++/rclcpp,
  absent in the harness environment).

Reproduce: `./viability_check.sh` (or `.bat`); comms test:
`./build_comms_test.sh`; no-relink demo: `./build_plugin_load_test.sh`.

## Decisions taken (still binding on the generator)

1. **Wrapper messages → Component-NS.** Service-local oneof wrappers
   (`SPRRequirement_Service_Request`, `Empty`, …) are homed in their
   component's own namespace (`pyramid::components::…::services::provided`),
   matching the protobuf/flatbuffers/grpc/ros2 backends.
2. **Duplicate type names → de-flatten, don't rename.** Keep the proto's
   package structure; the C++ binding no longer flattens everything into one
   `domain_model`. Names shared across packages live in their sub-namespace
   (e.g. `domain_model::pim_osprey::sensor_products::SPRRequest`); the umbrella
   header flat-re-exports only uniquely-named types.
3. **C-ABI symbols → qualify all uniformly.** Every generated C struct is
   package-qualified (`pyramid_data_model_common_Ack_c`); the Ada generator
   mirrors the same scheme. Type resolution is FQN-aware throughout the
   codec/C-ABI generators — never first-match short-name lookup.

## Remaining follow-ups

- **FlatBuffers `.cpp` JSON-bridge include/call derivation** still assumes
  single-segment data-model packages; nested packages (e.g.
  `common_pim_components.authorisation`) need the same full-namespace
  treatment applied to the header path.
- The hardcoded topic catalog concern noted here was since resolved:
  `standard_topics.py` is now data-driven
  (`pim/topic_metadata/tactical_objects_topics.json`).

## Files

| File | Purpose |
|------|---------|
| `viability_check.sh` / `.bat` | Regenerate + syntax-check a facade (g++ / MSVC) |
| `components_comms_test.cpp` | Provider + client over PCL with the JSON codec plugin **compiled in** (static) |
| `build_comms_test.sh` / `.bat` | Generates bindings, compiles the sensor_products closure + plugin against pcl_core, runs the static comms test |
| `plugin_load_test.cpp` | Provider + client that load a **prebuilt** codec plugin at runtime (no relink) and pass config through the loader |
| `build_plugin_load_test.sh` / `.bat` | Builds the codec plugin as a standalone shared lib + the codec-free test, runs the no-relink demonstration |
| `_msvc_env.bat` | Helper: enter a VS x64 dev environment (vswhere/vcvars) for the `.bat` scripts |
