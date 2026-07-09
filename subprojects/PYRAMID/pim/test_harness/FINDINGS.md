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

- **CLOSED: FlatBuffers `.cpp` JSON-bridge include/call derivation** for
  nested data-model packages is resolved. Verified 2026-07-04: the bridge
  derives includes and calls via `_cpp_type_namespace_for_type`, generated
  output for `common_pim_components.authorisation` carries the full-namespace
  include and calls, and
  `subprojects/PYRAMID/tests/test_generic_flatbuffers_protobuf.py::test_pim_flatbuffers_json_bridge_uses_full_namespace_for_nested_data_model`
  locks the nested-package fixture evidence. The `pim/test` FlatBuffers compile
  gate was re-run for item B2 in `doc/todo/PYRAMID/TODO.md`.
- **Ada FlatBuffers codec skips the reserved-word package rename**
  (found 2026-07-04): generated `flatbuffers/ada/*-flatbuffers_codec` units
  reference `Pyramid.Data_Model.Generic_Pim.Generic.Types` (illegal Ada;
  the unit is correctly emitted as `Generic_Pkg.Types`). Tracked as item B3
  in `doc/todo/PYRAMID/TODO.md`.
- The hardcoded topic catalog concern noted here was since resolved:
  `standard_topics.py` is now data-driven
  (`pim/topic_metadata/tactical_objects_topics.json`).
- **RESOLVED: the RPC/pub-sub seam duality this file's Phase C section
  below flags** (every `Create/Read/Update/Cancel` rpc generating both an
  RPC seam and a pub/sub seam with nothing reconciling a manifest routing
  both) is resolved by
  `doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md` (Phases 0-5,
  2026-07-09): a transaction-shaped C++ facade makes the two seams
  interchangeable realizations of one interaction, selected per leg at
  compose time via a manifest `exclusive` group, fail-closed against
  dual-routing. See `agra_seam_interchange_test.cpp`/
  `build_agra_seam_interchange_test.sh` (Phase 5's terminal cross-process
  proof, this same directory) and that plan's own evidence ledger for
  full detail.

## Files

| File | Purpose |
|------|---------|
| `viability_check.sh` / `.bat` | Regenerate + syntax-check a facade (g++ / MSVC) |
| `components_comms_test.cpp` | Provider + client over PCL with the JSON codec plugin **compiled in** (static) |
| `build_comms_test.sh` / `.bat` | Generates bindings, compiles the sensor_products closure + plugin against pcl_core, runs the static comms test |
| `plugin_load_test.cpp` | Provider + client that load a **prebuilt** codec plugin at runtime (no relink) and pass config through the loader |
| `build_plugin_load_test.sh` / `.bat` | Builds the codec plugin as a standalone shared lib + the codec-free test, runs the no-relink demonstration |
| `contract_routing_validation.cpp` / `build_contract_routing_test.sh` / `.bat` | Compose-time-only: contract-derived endpoint requirements validated against a NULL-vtable stub plugin's declared caps/QoS |
| `routed_egress_shm_test.cpp` / `build_routed_egress_test.sh` / `.bat` | **Data-plane, real transport**: proves generated pub/sub helpers cross a real `libpcl_transport_shared_memory_plugin.so` bus loaded via `pcl_transport_routing_load`, same-process then cross-process (see `doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md` Phase A) |
| `agra_shm_comms_test.cpp` / `build_agra_shm_comms_test.sh` / `.bat` | A-GRA example contract's correlated request/requirement pair, cross-process over real SHM, JSON + FlatBuffers (Phase C) |
| `agra_udp_proof_test.cpp` / `build_agra_udp_proof_test.sh` / `.bat` | UDP fail-closed negative gate + the BEST_EFFORT information topic over real UDP datagrams, cross-process (Phase D) |
| `agra_mixed_route_test.cpp` / `build_agra_mixed_route_test.sh` / `.bat` | **Plan-terminal**: MA/C2 each load two transports (real SHM + real UDP) from one manifest, full worked-example sequence, both codecs (Phase E) |
| `_msvc_env.bat` | Helper: enter a VS x64 dev environment (vswhere/vcvars) for the `.bat` scripts |

## A-GRA example contract over real transports (2026-07-08)

`doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md` phases A-E are
executed and green (see that plan's §6 evidence ledger for full detail):
the routed-egress seam (container port -> executor route -> plugin egress
-> remote executor ingress -> subscriber callback) is proven over a real
SHM transport loaded from a routing manifest (Phase A); a new
A-GRA-vocabulary example contract exists at `pim/agra_example/` (Phase B);
its correlated request/requirement pair round-trips cross-process over
real SHM (Phase C); its information topic round-trips over real UDP
datagrams, with the RELIABLE/BEST_EFFORT QoS-floor mismatch failing closed
(Phase D); and MissionAutonomy/C2Station each load both transports from
one manifest for the complete worked-example sequence (Phase E,
plan-terminal). All the harnesses above except `contract_routing_validation`
now move real bytes over real transports, not just compose-time
validation.
