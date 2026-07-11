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
  both) is resolved by the interaction-facade work (the retired
  `rpc_pubsub_interchangeability_plan.md`, Phases 0-5, 2026-07-09 — design
  intent in `doc/plans/PYRAMID/README.md`, full text in git history): a
  transaction-shaped C++ facade makes the two seams interchangeable
  realizations of one interaction, selected per leg at compose time via a
  manifest `exclusive` group, fail-closed against dual-routing. See
  `agra_seam_interchange_test.cpp`/`build_agra_seam_interchange_test.sh`
  (the terminal cross-process proof, this same directory) and the
  [pub/sub & interaction facade guide](../../doc/guides/pubsub_interaction_guide.md).

## Files

| File | Purpose |
|------|---------|
| `viability_check.sh` / `.bat` | Regenerate + syntax-check a facade (g++ / MSVC) |
| `components_comms_test.cpp` | Provider + client over PCL with the JSON codec plugin **compiled in** (static) |
| `build_comms_test.sh` / `.bat` | Generates bindings, compiles the sensor_products closure + plugin against pcl_core, runs the static comms test |
| `plugin_load_test.cpp` | Provider + client that load a **prebuilt** codec plugin at runtime (no relink) and pass config through the loader |
| `build_plugin_load_test.sh` / `.bat` | Builds the codec plugin as a standalone shared lib + the codec-free test, runs the no-relink demonstration |
| `contract_routing_validation.cpp` / `build_contract_routing_test.sh` / `.bat` | Compose-time-only: contract-derived endpoint requirements validated against a NULL-vtable stub plugin's declared caps/QoS |
| `routed_egress_shm_test.cpp` / `build_routed_egress_test.sh` / `.bat` | **Data-plane, real transport**: proves generated pub/sub helpers cross a real `libpcl_transport_shared_memory_plugin.so` bus loaded via `pcl_transport_routing_load`, same-process then cross-process (proving-plan Phase A) |
| `agra_shm_comms_test.cpp` / `build_agra_shm_comms_test.sh` / `.bat` | A-GRA example contract's correlated request/requirement pair, cross-process over real SHM, JSON + FlatBuffers (Phase C) |
| `agra_udp_proof_test.cpp` / `build_agra_udp_proof_test.sh` / `.bat` | UDP fail-closed negative gate + the BEST_EFFORT information topic over real UDP datagrams, cross-process (Phase D) |
| `agra_mixed_route_test.cpp` / `build_agra_mixed_route_test.sh` / `.bat` | **Plan-terminal**: MA/C2 each load two transports (real SHM + real UDP) from one manifest, full worked-example sequence, both codecs (Phase E) |
| `_msvc_env.bat` | Helper: enter a VS x64 dev environment (vswhere/vcvars) for the `.bat` scripts |

## A-GRA example contract over real transports (2026-07-08)

The SHM/UDP proving plan (the retired
`agra_pubsub_shm_udp_proving_plan.md` — design intent in
`doc/plans/PYRAMID/README.md`, full text in git history) phases A-E are
executed and green:
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

## LA-CAL integration rung 1 — Phase 0 recon/pinning (2026-07-11)

Plan: `doc/plans/PYRAMID/la_cal_integration_plan.md`. Phase 0 is the one
phase with no code deliverable; everything after it keys off these answers.
No local Sleet run was needed to close the gate — the `owp` grammar is
pinned from the **normative spec cross-checked against Sleet's reference
implementation**, which is more reproducible than a captured session and is
the authoritative source a capture would only sample.

**Sources.** OMSC-SPC-013 RevB (`open-arsenal/oms`,
`docs_markdown_unofficial/20_OMSC-SPC-013_...md`, tables 5.1-1…5.1-7, §6.1)
and Sleet `sleet-types/src/owp.rs` (Apache-2.0). They agree on every point.

**Pinned grammar** → `lacal/owp_grammar.md` (the frozen contract Phase 1
implements against): subprotocol token `owp`; one op per WS text frame;
space/tab delimiter with consecutive-collapse, CR/LF not delimiters (JSON
bodies survive); `INIT`/`INFO` carry JSON, `SUB
<sid> <message_name> <topic> [group]`, `PUB <topic> <msg>`,
`MSG <sid> <msg>`, `UNSUB <sid>`, `+OK` (verbose, first `+OK` precedes
`INFO`), `-ERR <name> [details]`. Identity (System/Service UUIDs, server_id,
version) comes from `INFO`.

**Exit-gate decisions:**

- **D2 (QoS) resolved → BEST_EFFORT default.** OMSC-SPC-013 does not address
  QoS/reliability/buffering; the plugin declares BEST_EFFORT, with an
  explicit `"declare_reliability":"reliable"` deploy override the only path
  to a RELIABLE claim (UDP-downgrade convention).
- **Vocabulary decision made → kit-native UCI on the real-Sleet path.**
  Sleet validates every PUB/MSG body against the UCI XSD
  (`schema_version="002.5.0"`, `UCI_MessageDefinitions_v2_5_0.xsd`) and
  requires `message_name` to be a UCI global element. The hand-authored
  `agra_example` (`MA_*`) vocabulary is a UCI-2.3 A-GRA extension not in the
  OMS-2.5 UCI set, so it cannot go through Sleet. **Phases 3–4 (real Sleet)
  adopt the kit-native UCI message set; Phases 1–2 are unaffected** — they
  run against an in-process mock server we control, which does not
  schema-validate. This is the plan's Risk-table fallback, now confirmed as
  the required branch.

**Licensing.** Sleet and the starter-kit repos are Apache-2.0; the OMS/UCI
document repos are public U.S. Federal Government works. No upstream code is
copied into this repo — `owp_grammar.md` is an original pinning note citing
sources.

**Exit gate: met.** Frame grammar + schema/validation behaviour documented;
D2 resolved; demo-vocabulary decision made. Phase 1 unblocked.

## LA-CAL integration rung 1 — Phase 1 owp client core (2026-07-11)

A PCL-free C++17 client core lands: `subprojects/PYRAMID/src/owp/`
(`owp_frame.{hpp,cpp}` table-driven serialize/parse, `owp_client.{hpp,cpp}`
websocketpp/asio client — one worker thread, INIT→INFO fail-closed with a
bounded timeout naming url+service_id (D3), identity from INFO (D6),
verbose `+OK`-before-`INFO` ordering, `-ERR` propagation, worker-thread MSG
dispatch, explicit close/join, no auto-reconnect in v1). Built as static lib
`pyramid_owp` behind a new `PYRAMID_ENABLE_OWP` option (default ON;
websocketpp+asio fetched under `AME_FOXGLOVE OR PYRAMID_ENABLE_OWP`). Tests
in `tests/test_owp_client.cpp` (googletest, in-process websocketpp mock
server), CTest-registered under the `owp.` prefix and excluded from the
PCL codec-plugin auto-linkage loop to keep the "no PCL dependency" boundary.

**Verification (mine).** `cmake --preset default` + build; `ctest -R owp`
→ **6/6 green** on Linux (g++ 11.4). The socket path could not run in the
implementing sandbox (loopback bind denied there), so verification here
surfaced and closed three defects that only appear when the mock actually
runs: (1) client handlers were registered *after* `get_connection()`, so the
connection was created handler-less and never signalled open — reordered
before `get_connection()`; (2) the open handler rejected any connection whose
echoed subprotocol wasn't exactly `owp`, failing valid opens where the echo
is empty (a genuine decline is already caught by `fail_handler`) — relaxed to
reject only a non-empty mismatch; (3) the hand-rolled INFO JSON parser held
its text by `const&` while `objectField()` built a sub-parser over a
temporary substring — dangling reference (UB) on the nested `uuids` object —
fixed by storing the buffer by value. Golden-string frame tests match the
`owp_grammar.md` examples byte-for-byte.

**Exit gate: met.** Mock-server unit suite green; grammar round-trips the
pinned golden frames. (Byte-for-byte against *live* Sleet frames is a
Phase-3 item — no live capture exists yet; the golden strings are the
spec-pinned stand-in.) Phase 2 (LA-CAL transport plugin) unblocked.

## LA-CAL integration rung 1 — Phase 2 transport plugin (2026-07-11)

The PUBSUB-only `pyramid_lacal_transport_plugin` now wraps `pyramid_owp`
behind the PCL transport ABI. Entry performs the bounded `INIT` → `INFO`
handshake and fails closed on an unreachable broker, timeout, or broker
`-ERR`; INFO supplies the exported System and Service identities. The
adapter declares `PCL_CAP_PUBSUB` and BEST_EFFORT by default, guards
`application/oms-json`, queues `PUB` without blocking, maps subscriber ports
to `SUB`, and posts `MSG` ingress through the executor's deep-copy remote
queue. Teardown closes the WebSocket and joins its worker before `dlclose`.

Phase 2 also closed a pre-existing PCL setup gap exposed by a real broker:
the transport ABI documented `subscribe` as being called once per subscriber
port, but the executor never invoked it. `pcl_executor_add` now subscribes
matching ports through already-registered named/default transports, and late
named-transport registration subscribes matching existing ports. This keeps
manifest-driven LA-CAL composition functional without test-only calls into
the transport vtable.

**Verification (Linux, `build-flatbuffers-only`).** The OWP client suite is
6/6 green; the LA-CAL plugin suite is 6/6 green (ABI/caps/QoS, unanswered and
rejected INIT, content-type guard, INFO identity, broker round-trip, and
RELIABLE-over-BEST_EFFORT manifest rollback); the shared PCL executor suite is
35/35 green; and transport-routing is 31/31 green. The broker round-trip uses
two real executors and requires no direct test-side `subscribe` call.

**Exit status:** Linux exit gate met. Windows-specific exports and socket
guards are present, but the Windows compile leg has not been run in this Linux
workspace and remains a CI/toolchain verification item. Phase 0 confirmed
that Sleet validates against UCI 2.5, so Phase 4's kit-native OMS JSON codec
prerequisite moves ahead of the real-Sleet Phase 3 harness.
