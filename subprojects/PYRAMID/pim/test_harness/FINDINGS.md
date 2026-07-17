# New PIM proto — plugin-system viability (record)

## LA-CAL integration rung 1 — Windows compile/test leg closed (2026-07-12)

The Windows compile leg flagged as outstanding since Phase 2 (this file,
"Phase 2 transport plugin") is now run, on native MSVC (VS 2022, `default`
preset, `Release`), against a working tree with no other uncommitted
changes. Two environment/code gaps were found and fixed or documented; all
mock-broker test suites are green.

**Root cause of "targets don't exist": `PYRAMID_BUILD_TESTS` cached OFF.**
The pre-existing `build/` tree had `PYRAMID_BUILD_TESTS:BOOL=OFF` cached
(set in some earlier session, likely for a proto-agnostic plugin-only
build), which silences the entire `tests/` subdirectory — not just the
LA-CAL targets. `PYRAMID_ENABLE_OWP=ON` alone was not sufficient. Re-running
`cmake --preset default -DPYRAMID_BUILD_TESTS=ON` restored `test_owp_client`,
`test_lacal_transport_plugin`, `test_oms_json_codec_uci`, `lacal_e2e_test`,
and `lacal_seam_test` to the generated project set. No project-code change
needed; noted here because the stale `test_tobj_*` etc. `.vcxproj` files
left over from a prior configure made the build tree *look* like tests were
already wired when they weren't — trust `cmake --build --target <name>`,
not file listings, to check whether a target is live.

**Real bug found and fixed: MSVC never took the websocketpp C++11 path.**
`pyramid_owp` (and, transitively, `pyramid_lacal_transport_plugin`) failed
to compile with `C1083: Cannot open include file: 'boost/version.hpp'`.
Cause: websocketpp's `common/random.hpp` (and `common/type_traits.hpp`)
gate their C++11 `<random>`/`<type_traits>` path on `__cplusplus >=
201103L`, but MSVC reports `__cplusplus` as `199711L` unless `/Zc:__cplusplus`
is passed project-wide — which this repo does not do — so it fell through
to `#include <boost/version.hpp>`, which isn't a dependency here. This is
the exact issue `ame_foxglove` already works around
(`subprojects/AME/src/CMakeLists.txt` / `foxglove_bridge.cpp`) via the
`_WEBSOCKETPP_CPP11_STL_` define, but the define was never carried over to
`pyramid_owp` when it was split out as a PCL-free library. Fixed by adding
`target_compile_definitions(pyramid_owp PUBLIC _WEBSOCKETPP_CPP11_STL_)` to
`subprojects/PYRAMID/src/owp/CMakeLists.txt` (propagates to
`pyramid_lacal_transport_plugin` and `test_owp_client` via the link graph)
plus the same define directly on `test_lacal_transport_plugin`
(`subprojects/PYRAMID/tests/CMakeLists.txt`), which embeds its own
in-process websocketpp mock broker and does not link `pyramid_owp`.

**Verification (Windows, native `build/`, MSVC, Release).** All five
owp/lacal/codec targets and their harness executables now build clean:
`pyramid_owp`, `pyramid_lacal_transport_plugin`, `pyramid_codec_oms_json_uci`,
`test_owp_client`, `test_lacal_transport_plugin`, `test_oms_json_codec_uci`,
`lacal_e2e_test`, `lacal_seam_test`. Running the mock-broker suites directly:

```
test_owp_client.exe            6/6  PASSED
test_lacal_transport_plugin.exe 7/7 PASSED  (incl. FailsClosedWhenInitUnanswered,
                                              FailsClosedWhenInitRejected,
                                              RejectsMismatchedContentType,
                                              ReliableFloorOverBestEffortFailsClosed,
                                              RpcEndpointOverPubsubPeerFailsClosed)
test_oms_json_codec_uci.exe    9/9  PASSED
```

**Sleet-dependent `.sh` harnesses run correctly under Git Bash on Windows
and SKIP cleanly.** `build_lacal_e2e_test.sh`, `build_lacal_seam_test.sh`,
and `build_lacal_interop_test.sh` were run with no `SLEET_URL`/`SLEET_BIN`
set. Each printed its documented `SKIP: ...` message and exited 0 —
including `build_lacal_e2e_test.sh`, which does a **from-scratch** `cmake -S
... -B build-flatbuffers-only -DPYRAMID_ENABLE_OWP=ON` configure and MSVC
build of the whole owp/lacal chain before reaching the SKIP check, so this
also re-proves the fix above against a second, independently-configured
build tree. This confirms the harness *convention* (bash driver + SKIP
discipline), not just the binaries, is portable to Windows/Git Bash. Actual
Sleet E2E delivery is still unverified on Windows — Sleet itself only runs
via the Linux/podman starter kit — and remains out of scope here, per the
plan's risk table.

**New Windows gap found, not fixed: the generated-facade path
(`build_lacal_generated_seam_test.sh`, Phase 5 step 1 / successor plan's
generated-P1 seam) does not run on a stock Windows checkout.** Two
independent blockers:

1. `python3` on `PATH` resolves to the Microsoft Store app-execution-alias
   stub, not a real interpreter, on this machine (`py` launcher works fine,
   `python3 --version` from a real install would too) — a local
   environment/PATH quirk, not a repo bug, but worth calling out since the
   script hardcodes `python3`.
2. More fundamentally: `pim/cpp/oms_json_codec_gen.py`'s `_WireNames`
   sidecar lookup reads `pim/uci_p1_seam/wire_names.json`, which is a
   **git symlink** (mode `120000`) to
   `../uci_generated/uci_2_5_0/wire_names.json`, along with two sibling
   `.proto` symlinks in the same tree. `core.symlinks` was `false` in this
   checkout, so git materialized these as plain-text files containing the
   literal link-target path — `json.loads()` on that text fails with
   `Expecting value: line 1 column 1 (char 0)`. Setting
   `git config core.symlinks true` and re-checking out the three affected
   paths (`wire_names.json`, `pyramid.data_model.uci.proto`,
   `pyramid.options.proto`) then failed with `Permission denied` creating
   the symlinks — this Windows account has neither Developer Mode nor
   `SeCreateSymbolicLinkPrivilege`, so real symlinks can't be created at
   all, even with the git setting corrected. Reverted (`core.symlinks
   false`, re-checked out the placeholder files) to leave the tree as
   found. **Net: Phase 5 step 1's generated-facade regeneration and the
   Phase-6-successor's generated-P1 seam (`kitty_hawk_pcl_consumer_plan.md`)
   cannot be regenerated on a Windows checkout without either (a) an
   elevated/Developer-Mode account plus `core.symlinks=true` set *before*
   the initial clone, or (b) replacing these three symlinks with real
   files/copies.** This is scoped narrowly — exactly 3 symlinks exist in
   the whole repo, all here — but is a genuine, previously-undocumented
   Windows portability gap.

**Files changed:** `subprojects/PYRAMID/src/owp/CMakeLists.txt`,
`subprojects/PYRAMID/tests/CMakeLists.txt` (both additive
`target_compile_definitions`, no behaviour change on Linux).

## Phase 3 exit: generated P1 seam + Kitty Hawk consumer, both live-verified over real Sleet (2026-07-12)

`pim/uci_p1_seam/` is the new overlay that puts the PIM port grammar
(Request/Entity command ports, four PUBLISH-pattern information ports)
on top of the checked-in xsd2proto-converted P1 tree (`pim/uci_generated/uci_2_5_0/`)
instead of the hand-authored `uci_seam_example`. `lacal_generated_seam_test`
now generates from this overlay and exercises the real `ActionCommandMT`/
`ActionCommandStatusMT` shape (composed `MessageType base`, `ID_Type`
composition, real `oneof` arms) end to end. The frozen hand codec
(`pyramid_oms_json_codec_uci.cpp`) is untouched beyond a header comment
marking it the permanent byte-equivalence golden fixture — no new messages
go there; new UCI messages ride the generated (xsd2proto + `oms_json_codec_gen.py`)
path from here on.

**Both live proofs pass against the persistent `external/ams-gra` Kitty Hawk
stack (podman), confirmed with a from-clean rebuild (no debug instrumentation,
byte-identical `pcl_executor.c` to HEAD):**

```
$ SLEET_URL=ws://... build_lacal_generated_seam_test.sh
PASS: generated UCI facade LA-CAL seam over Sleet

$ KITTYHAWK_SLEET_URL=ws://... build_kittyhawk_consumer_test.sh
PositionReport: PASS (11 valid samples)
ObservationMeasurementReport: PASS (3 valid samples)
ServiceStatus: PASS (rf-fm-demod=1 ir-search-and-track=1)
SignalReport: PASS (16 decoded samples)
PASS: Kitty Hawk generated-P1 consumer over Sleet
```

Getting there surfaced three real, previously-latent bugs — none specific
to this overlay, all fixed at the shared-tooling level:

1. **`wire_names.json` root-mapping only worked by accident before.** The
   generated P1 tree suffixes root message *types* with `MT` while the XSD
   *element* name stays bare (`ActionCommand` the element → `ActionCommandMT`
   the type — see `pim/uci_generated/README.md`). The C++ OMS-JSON codec
   generator (`pim/cpp/oms_json_codec_gen.py`) already resolved the
   element name correctly from `wire_names.json`'s `roots` map — the actual
   bug was that `pim/uci_p1_seam/`'s own `wire_names.json` symlink pointed
   one directory level too deep (`.../pyramid/data_model/wire_names.json`,
   which doesn't exist) instead of the tree root
   (`.../uci_generated/uci_2_5_0/wire_names.json`); a second file reopening
   the checked-in `pyramid.data_model.uci` package to add the three PIM
   port-grammar helper types (`Ack`/`Identifier`/`Query` — not XSD content)
   also silently clobbered that package's generated C++ types header, since
   the emitter keys one header per package name, not per source file — fixed
   by giving those three helpers their own `pyramid.data_model.uci_port_grammar`
   package instead of reopening the checked-in one.
2. **The codec generator never learned the single-variant "Information"
   wrapper shape.** `oms_json_codec_gen.py`'s wrapper-unwrap detection
   handled the two-variant Request/Entity command wrapper
   (`oneof { CommandType a=1; StatusType b=2; }`) but not the simpler
   one-variant information wrapper this overlay's four PUBLISH-pattern
   ports use (`X_Service_Information { oneof payload { XMT x=1; } }`) —
   `decode()`'s dispatch fell through to `PCL_ERR_NOT_FOUND` for every
   `*_Service_Information` id with no exception and no log, so the Kitty
   Hawk consumer's four subscriptions registered fine but silently received
   nothing. Generalized `_resolve_wrappers()` to accept single-variant
   wrappers too (same emission path, one arm instead of two); confirmed via
   `pim/test_oms_json_gen.py` (16 tests) that the seam-example golden output
   and the ActionCommand P1 path are both unaffected.
3. **The LA-CAL transport plugin's wire-message-name mapping was also
   command-wrapper-specific.** `owp_message_name()` in
   `pyramid_lacal_transport_plugin.cpp` had hardcoded
   `ActionCommand_Service_Request`/`_Entity` → bare-root-name mappings
   for the OWP `SUB` frame's message name, with no equivalent for
   `*_Service_Information`; without it Sleet rejected the `SUB` with
   `unknown message name PositionReport_Service_Information` etc. Added a
   general `_Service_Information` suffix-strip rule (a clean 1:1 mapping,
   unlike the irregular Request/Entity pair).

Local git-ignored registration: `external/ams-gra/sleet/services.d.local/kittyhawk-consumer.toml`
(mirrors the sibling `ame-sniffer.toml`/`seam-ma.toml`/`seam-c2.toml`
registrations already in that directory). Sleet only scans
`services.d.local` at container startup, so a registration change needs
`podman-compose up -d --force-recreate sleet`, which per the bring-up guide's
known gotcha requires an explicit `podman restart squall-rf-oms-adapter
squall-ir-oms-adapter` afterward or those two adapters stay wedged.

Regression bar held throughout: `pytest subprojects/PYRAMID/tests` (41
passed), `unittest pim/test_proto_parser.py` (3 passed),
`pim/test_oms_json_gen.py` (16 passed), and the default `pyramid` contract
layout's generated output confirmed byte-identical to pre-change (compared
via `tests/generation_baseline.py` against a clean worktree at the prior
commit).

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
| `agra_shm_comms_test.cpp` / `build_agra_shm_comms_test.sh` / `.bat` | A-GRA example contract's correlated request/entity pair, cross-process over real SHM, JSON + FlatBuffers (Phase C) |
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
its correlated request/entity pair round-trips cross-process over
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

## LA-CAL integration rung 1 — Phase 4 codec prerequisite (2026-07-11)

The real-Sleet path now has a deliberately small UCI 2.5 codec subset rather
than attempting to hand-model the full 100k-line schema. The selected messages
have concrete starter-kit roles: `SignalReport` is published by the independent
`rf-fm-demod` Skill and consumed by PYRAMID; `PositionReport` is consumed by
that Skill and can be published by PYRAMID. Frozen, bounded C structs live in
`include/pyramid/oms_json_types.h`, and
`libpyramid_codec_oms_json_uci.so` exposes codec ABI v2 with content type
`application/oms-json`.

The codec reproduces the upstream RF Skill's accepted minimal/enriched JSON
shape, validates RFC-4122 UUID text, extracts the RF
`Parametrics.Frequency.FrequencyAverage` field while tolerating unrelated
optional UCI fields, and implements the OMS JSON `NaN`/`Infinity` string
mapping. Fixed-size C fields avoid cross-DLL ownership for decoded values;
encoded buffers use `pcl_alloc`/`pcl_free`.

**Pinned independent sources.** The LA-CAL harness is commit
`9ea4cd2ebb5b6ed13e050391020bacfb1388ac60`; its UCI schema pin is
`73a286fc4b881d9f328dbd64148525606a92c33a`. The RF Skill fixture/builder is
tag `v2026.06.01`, commit
`af13bd2926b15253e795920c91320733a29927ea`. Fixture provenance is recorded
beside the JSON files under `tests/fixtures/lacal/`.

**Verification (Linux).** `test_oms_json_codec_uci` is 6/6 green: plugin
ABI/content type; foreign SignalReport golden encode and enriched decode;
foreign PositionReport golden encode and round trip; OMS special floating
point tokens; invalid UUID and unknown-schema negatives.

**Progress:** the codec/golden-fixture prerequisite is complete. Phase 4 is
not yet at its exit gate because live independently-authored peer traffic still
depends on the Phase 3 Sleet harness. Phase 3 is now unblocked.

## LA-CAL integration rung 1 — Phase 3 handover (2026-07-11)

The cross-process `lacal_e2e_test` and coordinator script are implemented.
They compose the real LA-CAL transport and OMS JSON codec through routing
manifests, provide publisher/subscriber modes, register two scratch services,
and skip cleanly when Sleet is absent or unreachable. The binary builds in
`build-all-off`; both skip paths were exercised successfully.

A local container using pinned Sleet tag `v2026.06.01` (commit
`e38f61d8ce0d75c8508434a52f2ed77c69cf6a3b`) accepted both INIT
registrations. Real schema validation first showed that UCI position latitude
and longitude are radians; the ABI, codec, tests, and E2E payload were updated
accordingly. The subsequent publisher frame reached Sleet but was rejected
because `AltitudeReference="WGS84"` is not in the pinned schema enum. Sleet
reports the allowed values as `ALTITUDE_BAROMETRIC`, `WGS_HAE`, `MSL`, and
`AGL`.

**Handover:** change the E2E payload and callback expectation to `WGS_HAE`,
then rerun the real-Sleet delivery and fail-closed negatives. Do not mark
Phase 3 complete until the subscriber receives the frame. The upstream RF
Skill PositionReport golden fixture also contains `WGS84`, which conflicts
with the pinned Sleet schema; preserve its provenance and decide separately
whether to retain it as an upstream compatibility fixture or add a
schema-valid fixture. The radians/ABI edits and new Phase 3 files in this
checkpoint have not received a final regression run.

## LA-CAL integration rung 1 — Phase 3 exit (2026-07-11)

**Phase 3 exit gate met.** The `WGS84` → `WGS_HAE` correction in
`lacal_e2e_test.cpp` (publisher payload + subscriber expectation) makes the
publisher frame schema-valid against the pinned UCI 2.5 XSD. Rerun against the
pinned Sleet container (`registry.gitlab.com/open-arsenal/ams-gra/hello-world-sk/infra/sleet:v2026.06.01`,
schema loaded: 722 elements / 4612 types) produced:

- **`PASS: cross-process PCL -> Sleet -> PCL PositionReport delivery`** — both
  scratch services INIT-registered (`pyramid-publisher`, `pyramid-subscriber`),
  Sleet accepted and routed the schema-valid frame, and the subscriber decoded
  the typed `PositionReport` (lat/lon radians, altitude, `WGS_HAE` reference).
- **Dead-Sleet SKIP:** unreachable `SLEET_URL` → `SKIP: Sleet is unreachable`.
- **Absent-config SKIP:** no env → `SKIP: set SLEET_URL, or set SLEET_BIN…`.
- **Fail-closed negatives** (mock-server unit tests, CTest-registered
  whenever `PYRAMID_ENABLE_OWP` is on — no live Sleet needed):
  unanswered INIT and `-ERR`-rejected INIT both return NULL from the plugin
  entry (`FailsClosedWhenInitUnanswered`, `FailsClosedWhenInitRejected`); a
  RELIABLE-floored route over a BEST_EFFORT declaration fails routing closed
  (`ReliableFloorOverBestEffortFailsClosed`). `owp.*` suite 12/12 green.

**Fixture decision.** The upstream RF-Skill `position_report.json` golden
fixture retains `WGS84`: `test_oms_json_codec_uci` exercises codec round-trip
fidelity, not Sleet schema validation, so the upstream provenance value is kept
without conflicting with the real-Sleet path. `OmsJsonUciCodec` suite 6/6 green.

The schema-valid enum on the wire is `WGS_HAE`; the codec passes
`AltitudeReference` through opaquely, so the same codec serves both the
upstream fixture and the Sleet path.

The captured PASS output and Sleet routing log are committed at
`lacal/sleet_e2e_run.log` for auditability (the run is manually launched, not
part of the default CTest suite, since it needs a live Sleet).

## LA-CAL integration rung 1 — Phase 4 exit (2026-07-11)

**Phase 4 exit gate met — both interop directions green against the
independently-authored peer.** The AMS GRA `la-cal-harness` (its own async
`OWPClient` + XSD-derived `OMSJsonGenerator`, installed editable under Python
3.12) drives `lacal_interop_driver.py` as the foreign peer opposite a PCL
`lacal_e2e_test` process, joined only by the pinned Sleet container:

- **Direction A (we consume foreign):** the harness publishes an XSD-derived
  OMS JSON `PositionReport`; Sleet `+OK`-accepts it as schema-valid; the PCL
  subscriber typed-decodes it → `position-ok`.
  `PASS A: harness -> Sleet -> PCL PositionReport`.
- **Direction B (foreign consumes us):** the PCL publisher publishes; the
  harness OWP client subscribes, receives the `MSG`, and validates the OMS JSON
  body against the expected `PositionReport` shape and values.
  `PASS B: PCL -> Sleet -> harness PositionReport`.
- **SKIP-safe:** the coordinator SKIPs on unreachable `SLEET_URL`, absent
  `LACAL_HARNESS_PYTHON`, or absent `UCI_XSD_PATH`; a reachable-but-rejecting
  Sleet is a FAIL.

**Shape agreement is the load-bearing result — with a precise scope.** The
harness's XSD-derived `generate_minimal("PositionReport")` produces the same
global-element envelope (`PositionReport → SecurityInformation / MessageHeader /
MessageData → InertialState.Position`) our hand-written codec derives from the
same XSD, differing only by `$type` JSON annotations (stripped; not UCI
content). The driver then **pins** the `SecurityInformation`, header identity,
and `InertialState.Position` subtrees to fixed values for a deterministic
cross-process comparison — so those pinned subtrees are authored by the driver,
not proven to be independently generator-derived. What is independently
established: (a) the harness's own OWP client + generator round-trip a
schema-valid PositionReport that our typed bindings decode, and (b) our codec's
output is decoded and value-checked by the foreign harness client, with Sleet
validating both against the UCI XSD. That is genuine interop over "loopback";
full field-by-field two-codec convergence on every nested element is *not*
claimed.

The verbose-mode handshake is used on both driver legs: the subscriber's `SUB`
is `+OK`-acked before it signals readiness (BEST_EFFORT has no broker-side
retention, so the peer's `PUB` must not race an inactive subscription), and the
publisher's `PUB` draws a `+OK`/`-ERR` so a schema rejection surfaces loudly.

Captured PASS output committed at `lacal/interop_run.log`. Codec round-trip
suite (`OmsJsonUciCodec`, 6/6) and `owp.*` (12/12) remain green; the Phase 4
codec prerequisite was recorded earlier.

## LA-CAL integration rung 1 — Phase 5 progress: RPC-impossible negative (2026-07-11)

The static half of the Phase 5 exit gate is in place ahead of the positive
seam-over-Sleet leg. `owp.LacalPlugin.RpcEndpointOverPubsubPeerFailsClosed`
routes a PROVIDED endpoint (requires `PCL_CAP_RPC_UNARY`) over the LA-CAL
transport (advertises `PCL_CAP_PUBSUB` only): `pcl_transport_routing_load` fails
closed with `PCL_ERR_STATE` and a `requires`-caps diagnostic, and no peer
transport is registered. This confirms the capability row now recorded in
`transport_codec_plugin_system.md` (LA-CAL: PUBSUB ✓, RPC_UNARY ✗) — the seam's
Request legs can be realized pub/sub over this peer but never RPC.
`owp.*` suite now 13/13.

**Next (Phase 5 positive):** the correlated request/entity interaction over
real Sleet using the UCI `ActionCommand` / `ActionCommandStatus` pair
(correlation key `CommandID.UUID`), which the pinned schema validates — the
vocabulary confirmed feasible by generating both messages from the XSD.

## LA-CAL integration rung 1 — Phase 5 exit (2026-07-11)

**Phase 5 exit gate met — the rpc-impossible / pubsub-works matrix is
demonstrated over the real broker.** Both halves:

- **pubsub-works (positive):** `lacal_seam_test` runs a correlated
  request/entity interaction over real Sleet with the UCI
  `ActionCommand` / `ActionCommandStatus` pair, **both legs realized pub/sub**
  over the LA-CAL (owp/asb) transport. `seam-c2` publishes an `ActionCommand`
  (`CommandState=NEW`) on the request topic and subscribes the entity
  topic; `seam-ma` subscribes the request topic and, on receipt, publishes two
  correlated `ActionCommandStatus` transitions (`RECEIVED`, `ACCEPTED`) keyed by
  the request's `CommandID.UUID`; `seam-c2` collects both by matching that key
  → `seam-ok`. `PASS: LA-CAL Phase 5 request/entity seam over Sleet (both
  legs pub/sub)`. Sleet validated every `ActionCommand`/`ActionCommandStatus`
  payload against the pinned UCI 2.5 schema. Captured at `lacal/seam_run.log`;
  SKIP-safe on absent/unreachable Sleet.
- **rpc-impossible (negative):** `owp.LacalPlugin.RpcEndpointOverPubsubPeerFailsClosed`
  (recorded above), plus the capability-matrix row in
  `transport_codec_plugin_system.md`.

**Codec extension:** `pyramid_oms_json_codec_uci` gained `ActionCommand` and
`ActionCommandStatus` (frozen C structs + OMS JSON map, hand-written like
PositionReport/SignalReport). `OmsJsonUciCodec` now 8/8, `owp.*` 13/13.

**Scope honesty — what the positive proves and doesn't.** `lacal_seam_test`
demonstrates the request/entity *interaction semantics* (correlated
request → entity transitions, both legs pub/sub, over the real broker,
UCI-validated) using the PCL publish/subscribe primitives directly — it is
**not** the generated `MaactionRequestPortProvider/Client` interaction facade
from `agra_seam_interchange_test.cpp`. Driving that generated facade over LA-CAL
would additionally require OMS-JSON codecs operating on the *generated* seam
structs (the seam facade currently routes proto3-JSON / FlatBuffers), which is
the `oms_json_backend.py` generator work deferred to rung 3 (§7). The facade's
realization-agnostic component-code claim is already proven over SHM in
`agra_seam_interchange_test`; this leg proves the same *interaction pattern*
rides the LA-CAL/Sleet transport with a schema-valid UCI vocabulary. Wiring the
generated facade end-to-end over LA-CAL — **from a proto contract through
generated bindings, with Ada codec compatibility** — is the active follow-on
(directed 2026-07-11); until it lands, the "pubsub-works over the real broker"
matrix cell is demonstrated but the *generated-facade* form of Phase 5 step 1
is not yet complete.

**Known harness limitation (readiness race).** `lacal_seam_test` and Phase 4
direction A signal readiness with fixed sleeps (200 ms/300 ms) rather than a
confirmed subscription: the C++ `owp` client's `subscribe()` queues `SUB` and
ignores the `+OK`, so a slow broker could drop a best-effort message before the
subscription registers. This is shared with the pre-existing `lacal_e2e_test`
and is deterministic on localhost TCP; a non-flaky fix needs the `owp` client to
surface SUB acknowledgement (Phase 4 direction B already uses the Python
client's verbose `+OK`). Listed as a follow-on, not a blocker.

## LA-CAL rung 1 — Phase 5 generated-facade path: proto -> OMS JSON (2026-07-11)

Directed follow-on to the hand-wired seam: prove the **generated** Request-port
facade over LA-CAL/Sleet, driven from a proto contract with Ada compatibility,
where the OMS JSON is **generated** (not hand-written). Large code work was
delegated to Codex; Claude specced, integrated, and ran the real-Sleet leg
(Codex has no Docker/Sleet access).

**Done and verified:**
- **`oms_json` generator backend** (`pim/backends/oms_json_backend.py` +
  `pim/cpp/oms_json_codec_gen.py` + `pim/ada/oms_json_codec_gen.py`, wired in
  `pim/generate_bindings.py`): `--backends oms_json` emits C++ **and** Ada
  OMS-JSON codecs. Scope is schema-valid UCI only (not arbitrary proto).
- **`pim/uci_seam_example/`**: a UCI-XSD-faithful Request-port contract
  (`ActionCommand`/`ActionCommandStatus`, `Cancel(Identifier)`,
  `.request`/`.entity` topic legs) that generates the interaction facade
  classes `ActioncommandRequestPort{Client,Provider,Handler}`.
- **Byte-equivalence**: the generated C++ OMS-JSON codec produces JSON
  byte-identical to the hand-written `pyramid_oms_json_codec_uci.cpp` for
  `ActionCommand`/`ActionCommandStatus` (Sleet already accepts that output).
  The generated codec also unwraps the `ActionCommand_Service_Request/_Entity`
  oneof wrappers to the bare UCI root JSON on encode and re-wraps on decode.
- **Ada**: the generated Ada OMS-JSON codec object-compiles (GNAT 10.5).
- **e2e harness** (`lacal_generated_seam_test.cpp` + `build_lacal_generated_seam_test.sh`):
  drives the generated facade Provider/Client (not raw primitives), both legs
  pub/sub over LA-CAL, generated OMS-JSON codec, SKIP-safe. Against real Sleet
  both services register and **Sleet raises no schema rejection** — the
  generated OMS JSON validates against the UCI 2.5 XSD. Fixes applied while
  integrating: `declare_reliability` for the RELIABLE contract floor over the
  BEST_EFFORT transport; both legs realized pubsub on the provider; codec loaded
  into `pcl_codec_registry_default()` (the registry the facade resolves from);
  and a pipefail-safe `libpcl_core.a` discovery in the build script.

**Verified completion (2026-07-11).** The generated facade now completes the
full correlated round trip through the pinned Sleet container:
`provider sawCreate=1`, `consumer submit.accepted=1`, and two ordered states
`RECEIVED`, `ACCEPTED` (`PASS: generated UCI facade LA-CAL seam over Sleet`).
The harness remains SKIP-safe and is intentionally not a CTest because CI does
not provide Sleet.

The final boundary split is explicit. Generated topic ports declare their
schema identity; the LA-CAL adapter maps the generated
`ActionCommand_Service_Request` and `_Entity` wrappers to the UCI root
message names `ActionCommand` and `ActionCommandStatus` for OWP `SUB`, while
ingress remains tagged `application/oms-json` for codec lookup. Thus the
oneof wrapper is unwrapped at the transport/facade boundary, not in the codec.

Harness hardening found during the live run: `PYRAMID_BUILD_DIR` now selects
both the canonical CMake `libpcl_core.a` and the matching LA-CAL plugin; the
facade's generated topic ports explicitly receive their remote `asb` route;
and the witness populates required UCI envelope fields and allows queued OWP
status publications to drain before provider teardown.

## Phase 6 progress (2026-07-11) — Kitty Hawk stack bring-up

Unlike Phases 3–5 (a single pinned `infra/sleet` container run ad hoc),
Phase 6 needs the full AMS GRA Hello World Kitty Hawk simulation: Supercell
(DIS ground truth + JSBSim), Squall (RF/IR sensor sim), the
`rf-fm-demod`/`ir-search-and-track` Skills, Sleet, and optionally
Graupel/Worldview. Per the user's request this is now a **persistent local
checkout** rather than a one-off spike: `getting-started` plus all eight
constituent projects cloned at tag `v2026.06.01` under `external/ams-gra/`
(git-ignored — see `/external/` in `.gitignore`), symlinked into
`getting-started/` so its `include:`-based `compose.yaml` resolves against
independent sibling clones. Exact recreation steps, image-pull gotchas
(`viz/worldview` has no `:latest` tag, only `v2026.06.01`), the Sleet
`services.d` local-registration override needed for any service beyond the
kit's own baked-in ones, and known podman-3.4.4 quirks are written up in
`subprojects/PYRAMID/doc/guides/ams_gra_starter_kit_bringup.md`.

Brought up with `podman` + `podman-compose` (installed user-side via
`pip install --user podman-compose`, no system package needed) — Docker was
available on this host but `podman`/`podman-compose` were installed instead
per the user's preference. All components except Prometheus/Grafana came up
clean (those two need `host-gateway` support in `extra_hosts`, which podman
3.4.4 predates — a benign, unrelated gap in the observability side-stack
only).

**Live traffic confirmed** against the running stack with a new read-only
OWP sniffer (`pim/test_harness/lacal/kittyhawk_owp_sniff.py`, registered as
scratch service `ame-sniffer` via the `services.d` override) subscribed to
`mission.position-report`, `mission.signal-report`,
`mission.observation-measurement-report`, and `mission.service-status`:

- `PositionReport` — Supercell's Eagle-1 ownship kinematics (WGS84
  lat/lon/alt + NED velocity), ~1 Hz.
- `ObservationMeasurementReport` — `ir-search-and-track`'s LOS Az/El
  detections referenced to the latest cached ownship position, several Hz.
- `ServiceStatus` — `NORMAL` from both `rf-fm-demod` and
  `ir-search-and-track` shortly after startup.
- `SignalReport` (`rf-fm-demod`) was **not** observed in the initial ~40s
  capture window. This is consistent with the scenario's own design (the
  Kitty Hawk pentagon flight pattern is built so the ownship genuinely gains
  and loses RF lock on the FM tower) rather than a fault — `energy_threshold`
  gating on real (simulated) RF energy, not a fixed timer.

One operational gotcha worth recording: restarting the `sleet` container
alone (e.g. after editing its `services.d` mount) leaves
`squall-rf-oms-adapter`/`squall-ir-oms-adapter` stuck emitting
`owp publish error: Operation canceled` instead of reconnecting — they need
an explicit `podman restart`. `rf-fm-demod-skill`/`ir-search-and-track-skill`
both recover from a Sleet bounce unattended (their own bounded-retry loop
per `docs/contracts.md`), so this looks specific to the Squall OMS-adapter
binaries' OWP client, not a scenario-wide issue.

**Scope note (2026-07-11):** the original Phase 6 payoff (WorldModel fact
grounding + truth-vs-perceived against DIS) was AME-facing; AME now lives in
a separate repo, so that scope is out of this plan. The PYRAMID-only
remainder — extending the OMS JSON UCI codec to cover
`ObservationMeasurementReport` and a PCL-only consumer harness against this
live stack — is tracked in
`doc/plans/PYRAMID/kitty_hawk_pcl_consumer_plan.md`, not here.
