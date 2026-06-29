# New PIM proto — plugin-system viability check (handover doc)

**Question:** can components from the new proto set (`subprojects/PYRAMID/pim/test/`)
communicate through the PCL/plugin runtime?

**Verdict so far:** generation succeeds (48 protos → 203 messages, 13 enums, 42
services). Four defects found. **A, B, C, and D are fixed and
compile-verified for the C++/C-ABI plugin path.** Ada C-ABI symbols are now
package-qualified; broader GNAT validation is blocked by pre-existing native
Ada type-generator issues outside the C-ABI generator.

Reproduce: `./viability_check.sh`. Comms test: `./build_comms_test.sh` (green).

---

## Decisions taken (for handover)

1. **Wrapper messages → Component-NS.** Service-local oneof wrappers
   (`SPRRequirement_Service_Request`, `Empty`, …) are homed in their component's
   own namespace (`pyramid::components::…::services::provided`), matching the
   protobuf/flatbuffers/grpc/ros2 backends. (User-confirmed.)
2. **Duplicate type names → de-flatten, don't rename.** Keep the proto's package
   structure; the C++ binding stops flattening everything into one
   `domain_model`. Names shared across packages live in their sub-namespace
   (`domain_model::pim_osprey::sensor_products::SPRRequest`). (User-confirmed.)
3. **C-ABI symbols → qualify all uniformly.** Every generated C struct is
   package-qualified (`pyramid_data_model_common_Ack_c`). This is a cross-language
   ABI change; the Ada generator (`ada_cabi_codegen.py`) MUST be updated to the
   same scheme and re-validated with GNAT — **not yet done.** (User-confirmed.)

---

## Defect A — `Ack` non-literal — FIXED & verified

`Ack` gained a string `identifier`, so `constexpr Ack kAckOk{...}` is ill-formed.
Fix in `cpp_codegen.py`: emit constants of non-literal structs as `inline const`
(literal structs keep `constexpr`). Old + new compile.

## Defect B — wrapper messages never generated — FIXED & verified (Component-NS)

Routed service-proto wrapper messages (+ synthesized `Empty`) through the
existing types/JSON-codec/CABI pipeline, emitted in the component namespace.
Wrapper **types, JSON codec, CABI marshal, facade (.hpp/.cpp), and the JSON
codec plugin all compile.** Old proto unaffected.

Changed:
- `generate_bindings.py`: `_discover_service_message_files` + route through
  `CppTypesGenerator.write_file` / `CppDataModelCodecGenerator` /
  `CabiTypesGenerator.write_file` / `CabiMarshalGenerator.write_file`.
- `cpp_codegen.py`: facade classifies RPC/topic types by FQN — data-model →
  `using <subns>::T`; wrappers/`Empty` → local (include component types header).
- `_find_proto_root`: also accept a dir containing the `pyramid/` package root
  (the new `pim/test` layout), so the data-model index is discovered and the
  per-component JSON codec plugins are generated (8 for the new tree).

## Defect C — duplicate names + flat namespace + unqualified C-ABI — FIXED & verified (C++)

~22 short message names are reused across data-model packages (and some genuinely
differ, e.g. `RADAR_SensorProducts`).

- **Namespace ambiguity** (`error: reference to 'SPRRequest' is ambiguous`):
  the umbrella now flat-re-exports only uniquely-named types (+ their constants);
  duplicated names stay in their sub-namespace. CABI marshal + codec-plugin
  native references were switched from flat `domain_model::X` to the package
  sub-namespace. **Verified compile; old proto unaffected.**
- **C-ABI symbol collision** (`redefinition of pyramid_SPRRequirement_c`):
  `_c_struct_name(name, package)` now emits `<package_stem>_<name>_c`, threaded
  through every site in `cabi_codegen.py` and the inline sites in `cpp_codegen.py`
  (`_c_struct_for_type`). **Verified:** wrapper + data-model CABI marshals,
  facade, JSON codec plugin, and the OLD proto all compile (0 errors).

## Defect D — duplicate names break short-name TYPE RESOLUTION — FIXED & verified

`ProtoTypeIndex.resolve_message(short)` and base-field inlining resolve types by
**short name**, returning the *first* package that defines it. For duplicated
names this picks the wrong definition. Two observed failures when compiling the
`pim_osprey.sensor_products` data-model codec/cabi:

- JSON codec: `RADAR_SensorProducts` (osprey, `base = Sensor_Products`) is
  generated calling `fromJson(..., GenericComponent*)` — i.e. the *pim* duplicate's
  base — so no overload matches.
- CABI marshal (`pim_osprey.tactical_objects`): a `TrackControlCmd_Record` field
  frees `pyramid_data_model_pim_external_systems_TrackControlCmd_Record_c` instead
  of the local `pim_osprey_tactical_objects_...` one; and base-inlined member
  names (`tk_ctrl_cmd`, `tp_command`) don't match the emitted C struct members.

**Fix direction:** make type resolution FQN-aware throughout the codec/cabi
generators — resolve a field/base type to its *declaring* package using the
proto FQN (and the current package for bare same-package refs), never a
first-match short-name lookup. This touches `ProtoTypeIndex.resolve_message`
usage, `_inline_base_fields`, and the codec base-field emission. The namespace
work in Defect C already qualifies *names*; Defect D is about resolving the
*right definition* before naming it.

## Resolution — Defect D and Ada C-ABI qualification

### Defect D

Fixed in `cpp_codegen.py` and `cabi_codegen.py`.

- Added package-aware proto type resolution in the generators: bare names first
  resolve as `current_package.Type`, then only fall back when the short name is
  unique in the index. Duplicated short names no longer resolve to an arbitrary
  package.
- Base-field inlining now resolves the base message using the referencing
  message's package, then rewrites inlined base fields to FQNs relative to the
  base message's declaring package. This prevents inherited fields from being
  interpreted in the child message's package.
- JSON codec generation now qualifies foreign `toJson`/`fromJson` calls and
  includes codec headers for foreign message/enum fields introduced by base
  inlining.
- Contract codec plugin schema collection now includes service-local wrapper
  messages from the current service file, while avoiding link-time dependencies
  on unrelated consumed/provided wrapper objects not compiled by the harness.
- Enum string converters are emitted inline in codec headers so foreign enum
  conversion does not require linking every transitive codec implementation.

Verification:

- `./viability_check.sh /tmp/pim_generated_after_d` passed.
- `./build_comms_test.sh` passed: generated bindings, compiled 17 sources, ran
  the provider/client plugin comms test, and reported `PASS (0 failure(s))`.

### Ada C-ABI qualification

Fixed in `ada_cabi_codegen.py`.

- Ada CABI mirror record names now derive from the same package-qualified C ABI
  struct symbol as `cabi_codegen._c_struct_name`, e.g.
  `pyramid_data_model_common_Ack_c` maps to
  `Pyramid_Data_Model_Common_Ack_C`.
- Imported C free routines now use the package-qualified symbol, e.g.
  `Free_Pyramid_Data_Model_Common_Ack` imports
  `pyramid_data_model_common_Ack_c_free`.
- Empty CABI mirror records now receive a one-byte `Padding` component so GNAT
  accepts the Ada record declaration.

GNAT validation:

- Ada generation passed with
  `python3 subprojects/PYRAMID/pim/generate_bindings.py subprojects/PYRAMID/pim/test /tmp/pim_ada_generated_fqn2 --languages ada --backends json`.
- `gnatmake -c -I/tmp/pim_ada_generated_fqn2 /tmp/pim_ada_generated_fqn2/pyramid-data_model-base-cabi.adb`
  passed.
- Full GNAT validation of duplicate-heavy packages is still blocked outside the
  allowed edit scope by pre-existing `ada_codegen.py` output issues:
  empty native records in generated `*-types.ads` files and package segments
  named `Generic`, which is an Ada reserved word. Sample failing commands:
  `gnatmake -c .../pyramid-data_model-common-cabi.adb` stops in
  `pyramid-data_model-common-types.ads`, and
  `gnatmake -c .../pyramid-data_model-pim_osprey-sensor_products-cabi.adb`
  additionally reports reserved-word `Generic` package references.

## Remaining work checklist

- [x] Defect D: FQN-aware type/base resolution in codec + cabi generators.
- [x] Ada: apply the package-qualified C-ABI symbol scheme in
      `ada_cabi_codegen.py` (Decision 3).
- [x] Re-run `build_comms_test.sh` to green once D is fixed.
- [ ] Fix native Ada type-generator GNAT blockers in `ada_codegen.py`, then
      re-run full Ada CABI validation.
- [ ] Validate protobuf/flatbuffers/grpc/ros2 backends end-to-end (they already
      target Component-NS; not compile-checked here).

## Files

| File | Purpose |
|------|---------|
| `viability_check.sh` | Regenerate + syntax-check a facade |
| `components_comms_test.cpp` | Provider + client over PCL with the JSON codec plugin registered; passes once D is fixed |
| `build_comms_test.sh` | Generates bindings, compiles the sensor_products closure + plugin against libpcl_core, runs the test |
