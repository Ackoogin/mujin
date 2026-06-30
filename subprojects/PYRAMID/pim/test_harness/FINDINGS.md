# New PIM proto — plugin-system viability check (handover doc)

**Question:** can components from the new proto set (`subprojects/PYRAMID/pim/test/`)
communicate through the PCL/plugin runtime?

**Verdict:** generation succeeds (48 protos → 203 messages, 13 enums, 42
services). Four generator defects (A–D) are **fixed and verified** for the
C++/C-ABI plugin path. The plugin comms test passes (`build_comms_test.sh` →
`PASS (0 failure(s))`); every new-proto codec + cabi_marshal compiles; the old
proto has no regression.

**Ada:** C-ABI symbols are package-qualified, and generated native/C-ABI Ada
data-model units compile with GNAT for **both** the old proto tree and the new
duplicate-heavy PIM tree. Independently re-verified: the only remaining GNAT
stop is `gnatcoll.ads not found` (a missing external library in this env) on the
`*_types_codec` units — **old and new proto hit it identically**, so it is an
environment dependency, not a regression or generator bug. The earlier
"pre-existing ada_codegen.py issues" attribution was wrong: those were
regressions from this work (reserved `generic` segment, empty records,
array-naming, FQN resolution), now fixed in `ada_codegen.py` +
`ada_cabi_codegen.py`.

**Update (full compile completed):** the gnatcoll/pcl Ada specs are in-repo, so
*every* generated Ada unit — data model, codecs, C-ABI, and service bindings
(incl. the wrapper messages) — now object-compiles with GNAT for both the old
and new proto (0 failures). The other backends are validated to the extent the
local toolchain allows: protobuf compiles end-to-end against real `protoc`
output, flatbuffers schemas+codec headers validate via in-repo `flatc`,
grpc/ros2 generate and target Component-NS. See the "Remaining work checklist"
and "Ada full-compile fixes" sections below.

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
   ABI change; the Ada generator (`ada_cabi_codegen.py`) now mirrors the same
   scheme and has been re-validated with GNAT for generated data-model CABI/native
   units. (User-confirmed.)

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

### Ada C-ABI qualification and GNAT diagnosis

Fixed in `ada_cabi_codegen.py` and the Ada data-model portions of
`ada_codegen.py`.

- Ada CABI mirror record names now derive from the same package-qualified C ABI
  struct symbol as `cabi_codegen._c_struct_name`, e.g.
  `pyramid_data_model_common_Ack_c` maps to
  `Pyramid_Data_Model_Common_Ack_C`.
- Imported C free routines now use the package-qualified symbol, e.g.
  `Free_Pyramid_Data_Model_Common_Ack` imports
  `pyramid_data_model_common_Ack_c_free`.
- Empty CABI mirror records now receive a one-byte `Padding` component so GNAT
  accepts the Ada record declaration.
- Native Ada package segments are escaped when they collide with Ada reserved
  words; e.g. proto package segment `generic` emits Ada package segment
  `Generic_Pkg`.
- Empty native Ada records now receive a `Padding : Boolean := False;`
  component so GNAT accepts empty proto messages.
- Repeated-field native Ada array types are keyed by repeated element type, not
  by field name. This fixes collisions such as two `element` fields with
  different element types in `pyramid.data_model.generic_pim.generic`.
- Native Ada and Ada CABI type/selector resolution now carry proto package
  context. Cross-package fields and inlined base fields are emitted with the
  declaring Ada package, preventing duplicate short names such as
  `Generic_Component` from becoming ambiguous.
- Ada native and CABI specs now add `with` clauses for packages referenced by
  generated FQN field/member types, including types reached through inlined base
  fields. Google well-known types remain excluded because they are handled as
  scalar aliases in this generator path.

Corrected diagnosis:

- The earlier note that GNAT failures were "pre-existing `ada_codegen.py`
  issues" was incorrect. Old-proto data-model native/CABI Ada still compiles
  cleanly after this work, so these were regressions or newly surfaced
  new-proto shapes from the package-qualified/FQN work and the duplicate-heavy
  PIM test proto.
- Full "compile every generated `.adb`" validation is blocked in this local
  environment by missing external Ada dependencies, not by the fixed generator
  data-model units: JSON codecs require `gnatcoll.ads`; generated service stubs
  require `pcl_plugins.ads` and `pcl_bindings.ads`.

GNAT validation:

- Old proto generation passed:
  `python3 subprojects/PYRAMID/pim/generate_bindings.py subprojects/PYRAMID/proto /tmp/pyramid_old_ada_final --languages ada --backends json`.
- Old proto GNAT data-model validation passed for every generated non-codec,
  non-service `.adb`:
  `for f in /tmp/pyramid_old_ada_final/*.adb; do case "$f" in *types_codec.adb|*services*) continue;; esac; gnatmake -c -I/tmp/pyramid_old_ada_final "$f" || exit 1; done`.
- New proto generation passed:
  `python3 subprojects/PYRAMID/pim/generate_bindings.py subprojects/PYRAMID/pim/test /tmp/pyramid_new_ada_fixed10 --languages ada --backends json`.
- New proto GNAT data-model validation passed for every generated non-codec,
  non-service `.adb`, including duplicate-heavy Osprey/Seaspray packages:
  `for f in /tmp/pyramid_new_ada_fixed10/*.adb; do case "$f" in *types_codec.adb|*services*) continue;; esac; gnatmake -c -I/tmp/pyramid_new_ada_fixed10 "$f" || exit 1; done`.
- Full generated-Ada compile boundary was checked and stops on missing external
  libraries in this environment:
  `gnatmake -c -I/tmp/pyramid_old_ada_final /tmp/pyramid_old_ada_final/pyramid-data_model-common-types_codec.adb`
  and the same new-proto codec command both report `file "gnatcoll.ads" not
  found`; compiling an old generated service reports `file "pcl_plugins.ads" not
  found` and `file "pcl_bindings.ads" not found`.

## Remaining work checklist

- [x] Defect D: FQN-aware type/base resolution in codec + cabi generators.
- [x] Ada: apply the package-qualified C-ABI symbol scheme in
      `ada_cabi_codegen.py` (Decision 3).
- [x] Ada: fix native/CABI GNAT regressions surfaced by duplicate-heavy PIM
      packages and confirm old-proto data-model Ada still builds.
- [x] Re-run `build_comms_test.sh` to green once D is fixed.
- [x] **Literal compile of every generated Ada `.adb` including codecs and
      services.** GNATCOLL JSON (`core/external/gnatcoll-core`) + PCL Ada specs
      (`subprojects/PCL/bindings/ada`) are in-repo; GNAT 10.5 builds them. A
      `gprbuild -c` against `gnatcoll_json.gpr` object-compiles **every** generated
      unit: new PIM proto = 112 `.adb` → 244 objects, 0 failures; old proto = 18
      `.adb`, 0 failures (no regression). See "Ada full-compile fixes" below.
- [x] **Validate protobuf/flatbuffers/grpc/ros2 backends.** All four generate
      cleanly (629 files) and home wrapper messages in Component-NS.
      - **protobuf**: 45 generated codec headers compile against real `protoc`
        `.pb.h` output + libprotobuf — end-to-end green.
      - **flatbuffers**: 53 `.fbs` schemas validate with in-repo `flatc`; 53 C++
        codec headers compile against the flatc-generated headers + C++ data
        model. Fixed Defect-C in this backend (it referenced the dead flat
        `pyramid::domain_model::X` umbrella for duplicate/wrapper types; now uses
        the declaring sub-/Component-NS via `cpp_codegen._native_namespace_for_type`)
        and removed a dead hardcoded tactical block. Remaining follow-up: the
        `.cpp` JSON-bridge include/call derivation still assumes single-segment
        data-model packages and needs the same full-namespace treatment for the
        new nested packages (e.g. `common_pim_components.authorisation`).
      - **grpc / ros2**: generate + target Component-NS correctly; full compile
        needs grpc++ / rclcpp + ros2 IDL gen (absent in this env, same class as
        the earlier gnatcoll/pcl block).

## Ada full-compile fixes (new-proto codecs + services)

Codecs and service bindings had never been compiled before (blocked by the
missing gnatcoll/pcl specs), so a batch of generator defects surfaced and were
fixed in `ada_codegen.py` (+ `generate_bindings.py`):

- **Codec type collisions** — a message/enum whose Ada name equals an enclosing
  package segment (`Authorisation`) or a predefined type (`Boolean`) resolved to
  the wrong entity. `AdaDataModelCodecGenerator` now fully-qualifies own-package
  types with their Types package.
- **Codec foreign dispatch** — foreign `To_Json`/`From_Json` were resolved by
  short name (wrong package for duplicates like `Generic_Component`). Now resolved
  FQN-aware by the field's declaring package; `with`-deps now include oneof
  variant field codecs (was the missing-`with`/`Append`-ambiguity cause).
- **Codec field naming** — codec mirrored the types generator's component-name
  rule (qualify cross-package field types so the `Val_` shadow prefix matches).
- **Service `with Google.Protobuf.Types`** — google packages now skipped in the
  service type-package collector.
- **Service C-ABI symbols** — service bindings now reference the package-qualified
  C-ABI record + free names (Decision 3) and unique pointer-conversion packages
  (duplicate short names like `Sensor_Products` no longer collide).
- **Ada Defect B (wrapper messages)** — `_generate_json_ada` now routes the
  service-local oneof wrapper messages (+ synthesized `Empty`, incl. Empty-only
  services) through native types/codec/C-ABI in their Component-NS, mirroring the
  C++ path; the service binding withs them and binds their marshalling.
- **`Ack` default aggregate** — `(Success => True)` → `(Success => True, others => <>)`
  now that `Ack` carries an `Identifier`.
- **Standard topics** — payload types are resolved against the actual data model;
  topics whose payload is absent in this proto set (e.g. `ObjectEvidenceRequirement`)
  are dropped, and present payloads' packages are withed. (The hardcoded
  `standard_topics.py` catalog is a domain-coupling smell to resolve later.)

## Files

| File | Purpose |
|------|---------|
| `viability_check.sh` | Regenerate + syntax-check a facade |
| `components_comms_test.cpp` | Provider + client over PCL with the JSON codec plugin registered; passes once D is fixed |
| `build_comms_test.sh` | Generates bindings, compiles the sensor_products closure + plugin against libpcl_core, runs the test |
