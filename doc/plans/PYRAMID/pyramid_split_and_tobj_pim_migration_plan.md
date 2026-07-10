# PYRAMID Split & Tactical Objects PIM Migration Plan

**Scope:** (1) A plan to split `subprojects/PYRAMID` into the reusable
**binding/plugin capability** and the **examples/apps** that consume it;
(2) a gap analysis for moving Tactical Objects from the legacy hand-written
contract (`proto/`) onto the **new-style MBSE contract tree**
(`pim/test/`, PIM Osprey), whose port grammar realises every request-shaped
service as `Create` / `Read` / `Update` / `Cancel`.

**Date:** 2026-07-06
**Related:**
[`README.md`](README.md) (design-intent summary of the executed plans, incl. `pubsub_contract_generation_plan.md`),
[`standard_alignment.md`](../../../subprojects/PYRAMID/doc/architecture/tactical_objects/standard_alignment.md) (legacy design reference),
[`generic_contract_layout.md`](../../../subprojects/PYRAMID/doc/architecture/generic_contract_layout.md),
[`pyramid_interaction_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md),
[`doc/todo/PYRAMID/TODO.md`](../../todo/PYRAMID/TODO.md) (single tracker; E5 lands inside Part 2 here).

**Status:** proposed — not yet scheduled. Open decisions are listed at the
end of each part; nothing below changes generator output or build behaviour
until the phases run. **The original open decisions (D1.1, D1.3, D2.1–D2.6)
were resolved on 2026-07-07** — resolutions are recorded inline at each
decision and consolidated in the "Decision resolutions" section below. The
MBSE-model gap analysis (§2.6) subsequently surfaced one further open
decision, **D2.7** (the `circle_area` narrowing), tracked in §2.5.

## Decision resolutions (2026-07-07)

| ID | Resolution | Note |
|----|-----------|------|
| D1.1 | **`PYRAMID_COMPONENTS`** | As recommended |
| D1.3 | **Promote `pim/test/` to `PYRAMID_COMPONENTS/contracts/pim/`**, leaving a reduced synthetic fixture behind for generator tests | Departs from the "keep under capability" recommendation — commits to the split now rather than deferring it |
| D2.1 | **Re-enable `pMatchingObjects` for Osprey** | As recommended |
| D2.2 | **Hybrid: the Request port carries the criteria; the information port emits details tagged with the matching requirement id for tracing** | Refines the recommendation — correlation-by-requirement-id, not pure flat-topic + client-side ID filtering |
| D2.3 | **Add `policy` to the common `ObjectDetailRequest`/`Requirement`** | As recommended — active-find is a common capability |
| D2.4 | **Client-supplied UUIDs everywhere**; `Ack.identifier` echoes | As recommended |
| D2.5 | **`pTPTrackControl` / `pTPRadarTrackData` out of scope** | As recommended — stamp kinds when radar integration picks them up |
| D2.6 | **Frozen compat** for the legacy contract until first external consumer confirms migration, then retire | As recommended |

---

# Part 1 — Split PYRAMID into capability vs consumers

## 1.1 Why

`subprojects/PYRAMID` currently mixes two things with different lifecycles
and different future homes:

- **The capability**: the binding generator (`pim/`), the runtime core
  (`core/`), the codec/transport plugin system (`plugins/`, generated codec
  plugin targets), the manifest-driven CMake helpers (`cmake/`), the ROS2
  transport adapters (`ros2/`, `ros2_msgs/`), and SDK packaging
  (`sdk_template/`, `scripts/package_sdk.*`). This is proto-agnostic by
  design (the whole generic-layout workstream) and is the thing that would
  "later become its own repository" per the subproject README.
- **The consumers**: Tactical Objects (runtime + component + app),
  `pyramid_bridge`, the hand-written C++/Ada examples, the legacy contract
  tree (`proto/`) those consumers implement, and their E2E/conformance
  tests.

The build already knows this seam exists but expresses it as a single
boolean: `PYRAMID_BUILD_TESTS` gates `tactical_objects/`, `pyramid_bridge/`
and `tests/` because they "are wired to the default proto/ component set …
and cannot configure against an alternative `PYRAMID_PROTO_DIR`"
(`subprojects/PYRAMID/CMakeLists.txt:847-860`). The singleton
`PYRAMID_PROTO_DIR` / `PYRAMID_CPP_BINDINGS_DIR` design means **one build
tree = one contract**, which both blocks clean layering and blocks Part 2's
migration (which needs legacy and PIM bindings side by side).

## 1.2 Inventory and classification

| Path | Side | Notes |
|------|------|-------|
| `core/` | capability | `pyramid_core` runtime services (UUID, logging, event, job) |
| `pim/` (generator, `backends/`, `cpp/`, `ada/`, `mbse/`, `topic_metadata/`) | capability | contract-agnostic since the generic-layout + manifest work |
| `pim/test/` | capability (fixture) | the MBSE-generated proving contract tree; also Part 2's target contract — see decision D1.3 |
| `pim/test_harness/` | capability | proves bindings/plugins against the MBSE tree; no app coupling |
| `proto/` | consumers | legacy component contracts (tactical objects, sensor data interpretation, autonomy backend). `pyramid/options/pyramid.options.proto` is capability-owned and ships with the generator (already duplicated per tree) |
| `plugins/` | capability | coupled gRPC/ROS2 plugin sources |
| `cmake/` (+ `cmake/tests/`) | capability | manifest/binding-source helpers |
| `ros2/`, `ros2_msgs/` | capability | transport adapter + generated interface package build (contract-parameterised) |
| `sdk_template/` | capability | offline SDK template |
| `scripts/` | split | capability: `generate_bindings.*`, `build_plugins.*`, `package_sdk.*`, `create_sdk.*`, `stage_plugin_deploy.*`, `build_ros2_transport.*`, `build_gnat_pyramid_cabi_marshal_libs.*`, `pick_loopback_port.py`, `gen_requirement_trace.py`. consumers: `test_cpp_app_client.*`, `test_cpp_bridge_client.*`, `test_ada_*`, `test_pyramid_bridge_e2e.*`, `test_tobj_master_conformance.bat`, `coverage_tactical_objects.*`, `build_ada.*` |
| `tactical_objects/` | consumers | runtime, component, `StandardBridge`, app |
| `pyramid_bridge/` | consumers | Ada/C++ bridge demo apps |
| `examples/` | consumers | hand-written Ada/C++ clients (tobj interest client, evidence provider, sensor demo) |
| `tests/` (python generator suite, `cmake/tests` driver, plugin/facade C++ tests) | capability | `test_generic_*`, `test_binding_manifest.py`, `test_manifest_cmake_helper.py`, `test_no_domain_literals.py`, `test_topic_metadata.py`, `test_ros2_*`, `test_plugin_only_fail_closed.cpp`, `test_codec_plugin_swap.cpp`, `test_codec_registry_bridge.cpp`, `test_transport_codec_plugin_composition.cpp`, `test_pcl_generated_component_stream_handle.cpp`, `test_grpc_*` |
| `tests/` (contract-coupled binding + app tests) | consumers | `tests/tactical_objects/**`, `tests/ada/**` (tobj/active-find/grpc-interop e2e), `tobj_grpc_server.cpp`, `test_pcl_proto_bindings.cpp`, `test_sensor_data_interpretation_bindings.cpp`, `test_codec_dispatch_e2e.cpp`, `test_binding_performance.cpp` — all compile against `proto/`-generated `pyramid_services_*` names |
| `doc/architecture/` | split | capability keeps `generated_bindings.md`, `transport_codec_plugin_system.md`, `ros2_transport_semantics.md`, `generic_contract_layout.md`, `pyramid_interaction_semantics.md`, `sdk_packaging.md`; `tactical_objects/` + `doc/requirements/tactical_objects/` move with the component |

Borderline case worth naming: some "capability" C++ tests today compile
against the tactical-objects contract because it is the only contract in
the default build (e.g. `test_codec_dispatch_e2e`). Near-term they move
with the consumers (they are de facto contract-conformance tests);
long-term the capability should regain equivalents pinned to a neutral
fixture contract (`pim/test/` or a minimal synthetic contract), so the
capability test suite is self-contained. That is Phase P4.

## 1.3 Target layout

```
subprojects/PYRAMID/                # binding/plugin capability (future standalone SDK repo)
    core/  pim/  plugins/  cmake/  ros2/  ros2_msgs/  sdk_template/
    scripts/        (capability scripts only)
    tests/          (generator + plugin/facade tests, fixture-contract only)
    doc/

subprojects/PYRAMID_COMPONENTS/     # example components + apps consuming the capability
    contracts/proto/                (moved from PYRAMID/proto; legacy tree)
    tactical_objects/  pyramid_bridge/  examples/
    tests/          (tobj unit/integration/e2e, Ada e2e, conformance)
    scripts/        (app/e2e/coverage scripts)
    doc/            (tactical objects architecture + requirements)
```

Dependency direction is one-way: `PYRAMID_COMPONENTS` links capability
targets (`pyramid::core`, generated binding targets, `pcl_core`); the
capability never references a consumer path. That property is what makes
the later repo extraction "minimal path churn", and it should be enforced
by a source-guard test (grep for `PYRAMID_COMPONENTS` under
`subprojects/PYRAMID/` — the `test_no_domain_literals.py` pattern).

## 1.4 The real engineering: a per-contract CMake API

The move itself is mechanical; the enabling work is replacing the
top-of-file singleton in `subprojects/PYRAMID/CMakeLists.txt` (globals
`PYRAMID_PROTO_DIR`, `PYRAMID_CPP_BINDINGS_DIR`, one
`pyramid_cpp_bindings_codegen` target, one `pyramid_generated_codecs`, one
plugin-target loop) with a callable seam:

```cmake
pyramid_add_contract(<name>
    PROTO_DIR   <dir>
    BACKENDS    json;flatbuffers;protobuf;grpc;ros2   # subset per options
    LAYOUT      pyramid|generic
)
```

producing namespaced targets (`<name>_bindings_codegen`,
`<name>_generated_marshal`, `<name>_generated_codecs`,
`<name>_codec_<content>_<component>` plugins, `<name>_grpc_transport`,
`<name>_ros2_transport`) driven by that contract's
`binding_manifest.json`. This is a packaging of machinery that already
exists: manifest mode covers every source-selection site (TODO C2, done
2026-07-04), and the generator is already invoked per proto dir. The
compatibility layer is a default invocation
`pyramid_add_contract(pyramid PROTO_DIR ${PYRAMID_PROTO_DIR} ...)` plus
alias targets carrying today's names, so consumers and tests migrate
incrementally.

This seam is also a hard prerequisite for Part 2: the Tactical Objects
migration needs the legacy contract and the `pim/test` contract generated
and compiled **in the same build tree** during the transition, which the
singleton design cannot do (`PYRAMID_BUILD_TESTS` must currently be OFF
when `PYRAMID_PROTO_DIR` targets `pim/test`).

## 1.5 Phases

| Phase | Work | Acceptance gate |
|-------|------|-----------------|
| P1 — CMake seam | Implement `pyramid_add_contract()` over the manifest accessors; re-express the current build as one default invocation + alias targets. No file moves. | Full CTest suite: same registered test set, all green; generated bindings byte-identical; `build_plugins.sh` (both proto dirs) and `package_sdk.sh` unchanged in behaviour |
| P2 — Move consumers | Create `subprojects/PYRAMID_COMPONENTS/`; move `tactical_objects/`, `pyramid_bridge/`, `examples/`, `proto/` (→ `contracts/proto/`), consumer tests + scripts; root `CMakeLists.txt` adds the new subproject (gated by a `UNMANNED_BUILD_PYRAMID_COMPONENTS` option replacing `PYRAMID_BUILD_TESTS`) | Same test set and names green from new paths; capability configures with **no contract at all** (bindings generation becomes consumer-invoked); `.bat`/`.sh` script pairs fixed together |
| P3 — Docs + scripts split | Move tactical-objects docs/requirements; update both READMEs, `CLAUDE.md` test counts/paths, doc cross-links (many docs hardlink `subprojects/PYRAMID/...` paths) | Link check over `doc/` and both subproject `doc/` trees; stakeholder-facing docs reviewed |
| P4 — Capability test self-sufficiency | Re-pin capability C++ binding tests to a fixture contract (`pim/test/` or a minimal synthetic one) so `subprojects/PYRAMID` tests run with zero consumer content | Capability-only configure+build+ctest green in isolation (the "own repository" rehearsal) |
| P5 — Guard | Source-guard test for dependency direction; CI job building capability standalone | Guard green in CI |

## 1.6 Risks

- **Path churn** is the big one: dozens of scripts, docs, and CMake files
  hardcode `subprojects/PYRAMID/...`. Mitigate by doing P2 as a single
  mechanical commit (moves + path rewrites only, no logic changes) so it
  is reviewable as a rename.
- **Windows parity**: every consumer script has a `.bat` twin; move and
  fix them together (TODO C5 discipline).
- **CMake cache compatibility**: `PYRAMID_PROTO_DIR`,
  `PYRAMID_BINDING_SOURCE_MODE`, `PYRAMID_BUILD_TESTS` are documented
  knobs; keep them working (with deprecation notes) through P2, remove in
  a later release.
- **Naming**: `PYRAMID_COMPONENTS` vs `PYRAMID_APPS` vs folding Tactical
  Objects into its own `TACTICAL_OBJECTS` subproject. **Resolved (D1.1):
  `PYRAMID_COMPONENTS`** (it holds several components plus examples, not
  only apps); revisit per-component repos only when a second real
  component ships.
- **`pim/test/` ownership** (Decision D1.3): it is simultaneously the
  generator's proving fixture and Part 2's production-bound contract.
  **Resolved (D1.3): promote it to `PYRAMID_COMPONENTS/contracts/pim/`**
  and leave a reduced synthetic fixture behind for generator tests. (This
  departs from keeping it under the capability: the split is taken now
  rather than deferred until the Osprey contract is a delivered artifact —
  P4's capability-test re-pinning therefore targets the synthetic fixture
  from the outset.)

---

# Part 2 — Gap analysis: Tactical Objects onto the new-style proto (PIM Osprey)

## 2.1 Target shape

The new tree (`subprojects/PYRAMID/pim/test/`, emitted by
`pim/mbse/proto_generator.py` from `pim/mbse/test.json`) has a closed port
grammar: every service is either a **Request port**

```proto
service X_Service {
  rpc Create(X_Service_Request) returns (common.Ack);        // SUBSCRIBE <proj>.<iface>.request
  rpc Read(common.Query) returns (stream X_Service_Requirement); // PUBLISH <proj>.<iface>.requirement
  rpc Update(X_Service_Requirement) returns (common.Ack);
  rpc Cancel(base.Identifier) returns (common.Ack);
}
```

or an **Information port** (`Read(Empty) → stream T`), with
`pyramid.options.pyramid_op` interaction options (pattern/topic/QoS)
stamped mechanically, oneof wrapper messages for refined payload variants
(including a `cancel` variant on request wrappers), and acceptance
semantics on `Achievement` (`acceptance`, `acceptance_reason` —
`pim/test/.../pyramid.data_model.common.proto:65-74`). This is exactly the
"request/cancel/read/update" surface the migration targets.

**What the generated Osprey tactical-objects contract contains today**
(`pyramid.components.pim_osprey.tactical_objects.services.provided.proto` /
`...consumed.proto`):

| Service | Kind | Origin |
|---------|------|--------|
| `Capability_Service` | Information, provided | inherited from `GenericComponent` |
| `Authorisation_Dependency_Service` | Request, provided | inherited from `GenericComponent` |
| `Capability_Evidence_Service` | Information, consumed | inherited from `GenericComponent` |

**None of the tactical-objects domain ports are emitted.** The legacy
surface (`Object_Of_Interest`, `Matching_Objects`,
`Specific_Object_Detail`, evidence topics) has no counterpart in the
generated Osprey contract yet. The gaps below explain why and what closes
them.

## 2.2 Verified current state (evidence)

Traced through `pim/mbse/test.json` and `proto_generator.py` on this tree:

1. The model **does** contain the component and ports. Three
   `TacticalObjects` blocks exist: the legacy-PIM one, the
   `1. Common PIM Components / 7. Tactical Objects` one (ports
   `pObjectOfInterest`, `pMatchingObjects`, `pSpecificObjectDetail`
   provided; `pObjectSolutionEvidence`, `pObjectEvidence` consumed), and
   the Osprey project one (`2. Projects / 2. PIM Osprey / 1. Components /
   1. Tactical Objects`), which **generalizes** the Common block (so it
   inherits all five domain ports) and adds two project ports
   (`pTPTrackControl`, `pTPRadarTrackData`).
2. Port inheritance works: `_component_ports()` returns all nine inherited
   + own ports for the Osprey block.
3. The domain ports are dropped at the **kind gate**
   (`proto_generator.py:182`): `port.kind not in ('request','information')`.
   Measured kinds: `pObjectOfInterest`, `pSpecificObjectDetail`,
   `pObjectSolutionEvidence`, `pObjectEvidence`, `pTPTrackControl`,
   `pTPRadarTrackData` all have `kind: null`; only the GenericComponent
   ports carry `request`/`information` kinds.
4. `pMatchingObjects` **does** have `kind: information` (on the Common
   block) but is excluded by `_port_disabled_by_structure_override()` — an
   Osprey project structure connector wires it to a lower-0 override,
   i.e. the model deliberately disables Matching Objects for Osprey.
5. The port-type interfaces already classify their payloads correctly
   (`_classify_iface_payloads`):

   | Interface (port type) | request payload | requirement payload | information payload |
   |---|---|---|---|
   | `Object_Of_Interest_Requirement` | `TOBRequest` | `TOBRequirement` | — |
   | `Matching_Objects` | — | — | `ObjectMatch` |
   | `Specific_Object_Detail` | — | — | `ObjectDetail` |
   | `Object_Solution_Evidence` (consumed) | `TOBCreateTrackRadarRequest` | `TOBCreateTrackRadarRequirement` | — |
   | `Object_Evidence` (consumed) | — | — | `ObjectDetail` |

6. The supporting data model already exists in the new tree
   (`pyramid.data_model.common_pim_components.tactical_objects.proto`):
   `ObjectDetail`, `ObjectMatch`, `ObjectDetailRequest`,
   `ObjectDetailRequirement`, `TOBRequest`/`TOBRequirement`/
   `TOBDependency`/`TOBDependencyRequirement`, positioned detail subtypes
   (`SinglePointObject`, `CircleAreaObject`, `PolyAreaObject`), plus the
   Osprey radar-specific request/requirement family
   (`pyramid.data_model.pim_osprey.tactical_objects.proto`).

So the migration is **not** blocked on generator capability (topics, QoS,
acceptance states, oneof wrappers, codec plugins, Ada, ROS2 typed wire are
all delivered per the pub/sub plan ledger and TODO WS-A/WS-B/WS-E). It is
blocked on model classification, a handful of contract-semantics
decisions, the build seam (Part 1), and the adapter/app/test rework.

## 2.3 Gap register

### G1 — Model: port kinds not stamped (blocking, small)

The five domain ports need `kind` set in the model/export so the generator
emits their services:

| Port | Kind to stamp | Resulting service (provided file unless noted) |
|------|---------------|--------------------------|
| `pObjectOfInterest` | `request` | `Object_Of_Interest_Requirement_Service` — Create/Read/Update/Cancel over `TOBRequest`/`TOBRequirement` wrappers; topics `pim_osprey.object_of_interest_requirement.{request,requirement}` |
| `pSpecificObjectDetail` | `information` | `Specific_Object_Detail_Service` — `Read(Empty) → stream ObjectDetail` wrapper; topic `pim_osprey.specific_object_detail.information` |
| `pObjectEvidence` | `information` | consumed: subscribe `pim_osprey.object_evidence.information` (`ObjectDetail`) |
| `pObjectSolutionEvidence` | `request` | consumed: publish requests / subscribe requirements on `pim_osprey.object_solution_evidence.{request,requirement}` (`TOBCreateTrackRadarRequest`/`Requirement`) |
| `pTPTrackControl`, `pTPRadarTrackData` | decide (D2.5) | Osprey radar/track ports — out of scope for parity with the legacy surface, but the same fix applies |

Fix home: the SysML model (and its `test.json` export) — the same place
the working ports (`pSENRequirement`, `pCapability`, …) carry kinds. Then
regenerate `pim/test/` and confirm drift is exactly the new services.

Also verify under G1 that the **refinement edges** land the intended oneof
variants in the wrappers: `Object_Of_Interest`'s payloads are the *base*
types `TOBRequest`/`TOBRequirement`; the useful variants
(`ObjectDetailRequest`/`ObjectDetailRequirement`, and for Osprey the
`TOBCreateRadar*` family) must be modelled as refinements so
`Object_Of_Interest_Requirement_Service_Request` becomes a oneof over them
(plus the auto-added `cancel` variant). If the wrapper degenerates to the
bare base type, interest criteria cannot be expressed — check the first
regeneration.

### G2 — Model/product: Matching Objects disabled for Osprey (decision D2.1)

`pMatchingObjects` is correctly classified but explicitly disabled by a
lower-0 structure override in the Osprey project. The legacy surface's
`ReadMatch(Query) → stream ObjectMatch` and the `standard.entity_matches`
high-rate stream have **no Osprey counterpart while this stands**. Either:
(a) re-enable the port for Osprey (information port publishing
`ObjectMatch` on `pim_osprey.matching_objects.information` — the direct
analogue of `standard.entity_matches`), or (b) accept that matching is
delivered as `Object_Of_Interest` requirement transitions only, and retire
the match stream. Recommendation: (a) — the match stream is the wire-
efficient high-rate path the runtime already produces
(`standard_alignment.md`, Object Match Stream), and (b) would push
full-detail payloads onto every stream tick, the exact thing the legacy
design avoided.

### G3 — Contract semantics deltas (rpc-by-rpc mapping)

| Legacy (proto/) | New (pim/test, after G1) | Delta / work |
|---|---|---|
| `Object_Of_Interest_Service.CreateRequirement(ObjectInterestRequirement) → Identifier` | `Create(Request-wrapper) → Ack` | New `Ack` carries `identifier` (`common.proto:121-125`), so the server-assigned-id flow survives — but on PUBSUB routes there is no sync Ack: acceptance arrives as `Achievement.acceptance` transitions on the requirement topic, and ids must then be client-supplied (`Entity.id`). Adapter + clients must handle both (D2.4) |
| `ReadRequirement(Query) → stream ObjectInterestRequirement` | `Read(Query) → stream Requirement-wrapper` | Same shape; `Query` is id-list + `one_shot` in both trees |
| `UpdateRequirement(...) → Ack` | `Update(Requirement-wrapper) → Ack` | Same shape; payload becomes the requirement wrapper |
| `DeleteRequirement(Identifier) → Ack` | `Cancel(Identifier) → Ack` | Semantic shift: **cancellation, not deletion** — the A-GRA-derived pattern retains cancelled request objects for history/debrief; runtime must cancel the interest but keep the requirement record with a cancelled/rejected transition (harness already sequence-checks cancel → requirement transition) |
| `Matching_Objects_Service.ReadMatch(Query) → stream ObjectMatch` | Information port (D2.1 = re-enable): `Read(Empty) → stream ObjectMatch` | **Query parameter disappears** from the information port — but per D2.2 the criteria are carried by the paired Request port and each `ObjectMatch` tick is tagged with the originating requirement id, so consumers correlate by requirement id rather than filter blindly on header ids. No per-query streams |
| `Specific_Object_Detail_Service.ReadDetail(Query) → stream ObjectDetail` | criteria-carrying Request port (`ObjectDetailRequest`) + `Specific_Object_Detail_Service.Read(Empty) → stream ObjectDetail` wrapper | Per D2.2 the Request port is in scope from the start: it carries the `ObjectDetailRequest` criteria (the data model already defines it), and the information-port detail ticks carry the matching requirement id for tracing |
| publish `standard.entity_matches` (`ObjectMatch[]`) | `pim_osprey.matching_objects.information` | wire-name change ripples to every consumer/config; array topics use the stream-of-element convention already |
| publish `standard.evidence_requirements` (`ObjectEvidenceRequirement`) | consumed Request port `Object_Solution_Evidence`: publish `TOBCreateTrackRadarRequest` on `pim_osprey.object_solution_evidence.request`, subscribe requirement transitions | evidence flow becomes a proper correlated request/requirement pair (acceptance + progress observable) instead of a fire-and-forget topic |
| consume `standard.object_evidence` (`ObjectDetail`) | consumed Information port `Object_Evidence`: subscribe `pim_osprey.object_evidence.information` | direct analogue |

### G4 — Data-model deltas (field-level)

- **DataPolicy / active-find trigger has no home.** Legacy
  `ObjectInterestRequirement.policy` (Query→ReadCurrent, Obtain→ActiveFind)
  drives the runtime's core mode split. In the new tree `DataPolicy`
  appears on the Osprey radar types (`TOBCreateRadarRequest`,
  `RadarCreateDependency`, …) but **not** on
  `ObjectDetailRequest`/`ObjectDetailRequirement`. Either add `policy` to
  the common interest-request type in the model, or make active-find an
  Osprey-variant-only capability (`TOBCreateRadar*` oneof arm). Decision
  D2.3.
- **Location criteria narrowed.** Legacy interest takes
  `oneof location {PolyArea, CircleArea, Point}`; new
  `ObjectDetailRequest` has only `optional position` + `optional
  poly_area` — `CircleArea` criteria are gone (the type still exists in
  `common`). Runtime already reduces all shapes to bounding boxes, so this
  is a contract-expressiveness regression, not a runtime one; decide
  whether to add `circle_area` to the model or drop circle support.
- **`ObjectDetail` has no `position` field** in the new tree; positioned
  details are the refined subtypes (`SinglePointObject`,
  `CircleAreaObject`, `PolyAreaObject`). The adapter's detail projection
  (kinematics → `position`) must instead emit a subtype variant
  (presumably `SinglePointObject`), and consumers must read position from
  the variant. Verify these subtypes are refinement-wired into the
  `Specific_Object_Detail` information wrapper (same check as G1's
  refinement audit).
- **`ObjectEvidenceRequirement` mapping.** Its fields (location oneof,
  `policy`, `dimension[]`) map onto the `TOBCreateTrackRadarRequest` /
  `RadarCreateDependency` family (which carry `policy`, `dimension`,
  `area`/`location`), but not 1:1 (point/circle again absent; radar types
  are `Location`/`PolyArea`-based). A field-by-field mapping table is
  implementation work for the adapter, with any missing fields fed back
  into the model.
- **Units:** both trees share the `GeodeticPosition` contract; the
  radians-at-the-boundary rule from `standard_alignment.md` carries
  over unchanged — but re-verify against `GeoPosition_Record`
  (`int32` lat/lon) if the Osprey radar/track ports (D2.5) come into
  scope.

### G5 — Generator/runtime capability residuals (small, known)

Delivered already: contract-stamped topics + QoS, acceptance states,
correlated pairs proven over PCL and PUBSUB-only routes, JSON/FB/protobuf
codec plugins for the new tree, Ada object-compile 174/174, typed ROS2
wire, facade-owned pub/sub and local routing (TODO WS-A, WS-B, WS-E1–E4,
pub/sub plan phases 0–5). Residuals that touch this migration:

- FlatBuffers codec plugins still need the generated JSON codec closure as
  a wrapper-conversion bridge (WS-D trigger: "before treating binary
  codecs as fully independent") — acceptable, but Tactical Objects is the
  first production-shaped consumer, so this trigger arguably fires here.
- Scalar-wrapper alias payloads (e.g. bare `Identifier`) use the envelope
  fallback on the typed ROS2 wire (A3 note) — `Cancel(Identifier)` rides
  the request topic inside the oneof wrapper, so this is a non-issue on
  the main path; note it for any direct alias-typed topic.

### G6 — Build: one contract per tree (depends on Part 1 P1)

`tactical_objects_component` hardwires legacy binding sources by name
(`pyramid_services_tactical_objects_provided.cpp` / `_consumed.cpp`,
`tactical_objects/CMakeLists.txt:43-45`), and the whole consumer block is
gated OFF when `PYRAMID_PROTO_DIR` targets `pim/test`. The migration needs
both contracts generated side by side (legacy app + PIM app in one tree
for A/B conformance), which is precisely Part 1's
`pyramid_add_contract()` seam. **Sequence Part 1 P1 before Part 2 M3.**

### G7 — Adapter and app rework (the main implementation)

`StandardBridge` (~production adapter, still on raw PCL wiring — TODO E5)
is rebuilt against the generated component facade for the Osprey contract:

- Register the four fixed rpcs per Request port; dispatch oneof wrapper
  variants (interest criteria variants; `cancel` variant on the request
  topic when routed over PUBSUB).
- Publish requirement transitions with `Achievement.acceptance`
  (RECEIVED/REJECTED + reason) and progress — replacing both the legacy
  sync-ack-only flow and the fire-and-forget evidence topic.
- Cancel path: interest removal + stream-subscriber removal + retained,
  transitioned requirement record (not row deletion).
- Evidence flow inversion: derived evidence becomes `Create` on the
  consumed `Object_Solution_Evidence` request port (correlated pair)
  instead of a bare publish; incoming evidence arrives on the
  `Object_Evidence` information subscription.
- Detail/match projections re-targeted to the
  `common_pim_components.tactical_objects` types (position via subtype,
  enum re-alignment — new tree's `standard_identifer` spelling included).
- Doing this on the generated facade **is** TODO E5's "migrate" option;
  the legacy `StandardBridge` then becomes the documented raw-PCL
  exception only if it is kept alive for the legacy contract (D2.6).

The internal runtime (`TacticalObjectsRuntime`, stores, correlation,
spatial, interest manager) is deliberately untouched — the
standard-alignment design already keeps generated types at the boundary.

### G8 — Clients, examples, tests

Everything that speaks the legacy service/wire names must be ported or
duplicated: C++/Ada examples (`tobj_interest_client`,
`tobj_evidence_provider`, `ada_tobj_client`), the app test client, the
tobj E2E/conformance suites (`tobj_cpp_app_client_*`,
`tobj_ada_active_find_*`, socket/shm/grpc servers,
`standalone_bridge`), and the master conformance script. The
`pim/test_harness` comms test already demonstrates the new-style client
pattern (correlated request/requirement pair incl. Cancel, JSON + FB) and
is the template. New sequence-level checks to add: create →
acceptance transition, cancel → cancelled transition (the harness pattern,
now against the real app).

## 2.4 Migration sequence

| Step | Work | Acceptance gate |
|------|------|-----------------|
| M0 | Resolve decisions D2.1–D2.6 (below) | ✅ done 2026-07-07 — recorded in this doc (§2.5 + Decision resolutions) |
| M1 | Model edits: stamp port kinds (G1), re-enable/leave `pMatchingObjects` per D2.1, add missing fields/refinements (G4); regenerate `pim/test/` | regeneration drift is exactly the intended new services/fields; wrapper oneofs carry the interest variants; viability + comms harness green; Ada object-compile stays 174/174 (plus new units) |
| M2 | Part 1 P1 lands (per-contract CMake seam) | legacy build byte-identical via compat invocation |
| M3 | New adapter (`PimBridge` or `StandardBridge` v2) on the generated facade + `tactical_objects_app` variant hosting the Osprey contract; legacy app untouched | app serves Create/Read/Update/Cancel + information topics over JSON; facade-only (no raw PCL) — E5 satisfied for the new path |
| M4 | Port clients/examples/tests; run legacy and PIM apps side by side; conformance parity matrix (JSON/FB/protobuf codecs × socket/shm transports × C++/Ada clients, plus gRPC + typed-ROS2 smoke) | parity matrix green; sequence checks (create→acceptance, cancel→transition) green; legacy suite still green untouched |
| M5 | Switchover: PIM contract becomes the shipped Tactical Objects surface; `standard_alignment.md` rewritten as the new-current-state reference (legacy sections marked historical); decide legacy contract retirement vs frozen-compat (D2.6) | docs updated; TODO E5 + the "Tactical Objects bulk-detail" WS-D row re-evaluated against the new stream design |

## 2.5 Open decisions — resolved 2026-07-07

| ID | Decision | Resolution |
|----|----------|------------|
| D2.1 | Re-enable `pMatchingObjects` for Osprey (structure override) vs drop the match stream | **Re-enable** — keeps the compact high-rate path (G2) |
| D2.2 | Query-parameterised detail/match reads: accept flat-topic + client-side ID filtering, or add a criteria-carrying Request port using `ObjectDetailRequest` | **Hybrid: the Request port carries the criteria; the information port then emits the details tagged with the matching requirement id for tracing.** Correlation is by requirement id, not client-side header filtering — so the criteria-carrying Request port (using `ObjectDetailRequest`) is in scope from the start, and detail/match stream ticks carry the originating requirement id so consumers correlate rather than filter blindly. |
| D2.3 | Home for `DataPolicy` (Query/Obtain → ReadCurrent/ActiveFind) | **Add `policy` to `ObjectDetailRequest`/`Requirement`** in the model — active-find is a common capability, not radar-specific |
| D2.4 | Requirement id policy: server-assigned (via `Ack.identifier`, RPC routes) vs client-supplied `Entity.id` (PUBSUB routes) | **Client-supplied UUIDs everywhere** (A-GRA discipline); `Ack.identifier` echoes it, keeping one rule across transports |
| D2.5 | Scope of `pTPTrackControl` / `pTPRadarTrackData` (Osprey radar/track ports) in this migration | **Out of scope** for legacy parity; stamp kinds when the radar integration work picks them up |
| D2.6 | Legacy contract fate after switchover | **Frozen compat** (matches the `standard_topics.py` precedent): legacy app + suite kept building until the first external consumer confirms migration, then retire |
| D2.7 | **OPEN** — surfaced by §2.6 MM5: add `circle_area` to the common `ObjectDetailRequest`/`Requirement`, or drop circle support as a contract regression | Recommend **add it** (type already exists in `common`; one field keeps the new contract a superset of the legacy criteria surface). Not yet confirmed. |

## 2.6 Where the MBSE model is currently lacking

The migration has **no generator, transport, codec, or runtime feature
gap** (§2.7/§2.2). Every remaining piece of *upstream* work lives in the
SysML model and its `test.json` export
(`subprojects/PYRAMID/pim/mbse/test.json`) — the single source of truth the
generator consumes. This section consolidates those model-side
deficiencies (the modeller's to-do list), separated from the downstream
adapter/build/test work (G6–G8), with the root cause verified against the
current tree on 2026-07-07. All line references are on
`pim/mbse/proto_generator.py` / `sysml_parser.py`.

**How port `kind` is actually derived (context).** `kind` is *not* a
literal on the port — `sysml_parser._resolve_port_kind()`
(`sysml_parser.py:628-654`) walks the port **type's** generalization +
«Refine» chain and returns `request` if it reaches `RequestService`,
`information` if it reaches `ProviderService`, else `None`. The generator
then drops any port whose resolved kind is neither
(`proto_generator.py:182`, `:697`). So "stamp the kind" (G1) concretely
means: **give each domain interface block the missing base-service
ancestor.** Verified contrast on this tree:

- `Capability` interface → `generalizes ProviderService` → `information` ✅
- `Matching_Objects` (Common, `_2024x…` id) →
  `generalizes Information_Dependency` → `ProviderService` → `information` ✅
- `Authorisation_Dependency` → `generalizes RequestService` → `request` ✅
- `Object_Of_Interest_Requirement`, `Specific_Object_Detail`,
  `Object_Solution_Evidence`, `Object_Evidence` → **`generalizes []` and
  `refines []`** → `kind = None` → dropped ❌

### Model deficiencies register

| # | Deficiency (model-side) | Evidence on this tree | Fix in model / `test.json` | Ties to |
|---|---|---|---|---|
| MM1 | **Domain interface blocks inherit no base-service stereotype.** The four request/information interfaces have empty `generalizes` and empty «Refine», so `_resolve_port_kind` returns `None` and their ports never emit a service. | `Object_Of_Interest_Requirement`, `Specific_Object_Detail`, `Object_Solution_Evidence`, `Object_Evidence` all `generalizes=[] refines=[]` (vs `Capability`→`ProviderService`, `Matching_Objects`→`Information_Dependency`) | Make each domain interface generalize the correct base: request ports (`Object_Of_Interest_Requirement`, `Object_Solution_Evidence`) → `RequestService` (or `Request_Dependency`); information ports (`Specific_Object_Detail`, `Object_Evidence`) → `ProviderService`/`Information_Dependency`. This is the *only* thing missing for the ports to emit. | G1, M1 |
| MM2 | **`pMatchingObjects` is force-disabled for Osprey by a lower-0 structure override.** The Common block classifies it `information` correctly, but a project-structure connector in the Osprey project sets its lower multiplicity to 0. | `_port_disabled_by_structure_override()` suppresses it (`proto_generator.py:410`, `:423-460`); Osprey `pMatchingObjects` kind resolves to `information` yet is excluded | Per D2.1 (re-enable): remove/raise the lower-0 structure override on `pMatchingObjects` in the Osprey project structure so the port publishes `ObjectMatch` on `pim_osprey.matching_objects.information`. | G2, D2.1 |
| MM3 | **Refinement edges missing → request/information wrappers degenerate to bare base types.** The base payload types are empty placeholders; the useful variants are not «Refine»-wired to them, so the generated oneof wrapper has nothing to be a oneof *over*. | `TOBRequest` / `TOBRequirement` have **zero properties**; the concrete variants (`ObjectDetailRequest`/`ObjectDetailRequirement`, Osprey `TOBCreateRadar*`) are separate types not wired as refinements of the base | Model the concrete variants as «Refine» of the base payloads so `Object_Of_Interest_Requirement_Service_Request` becomes a oneof over them (+ the auto-added `cancel` arm). Same audit for the positioned detail subtypes into the `Specific_Object_Detail` information wrapper (see MM6). Without this, interest criteria cannot be expressed. | G1 (refinement audit), M1 |
| MM4 | **`DataPolicy` (Query→ReadCurrent / Obtain→ActiveFind) has no home on the common interest request.** The active-find trigger that drives the runtime's core mode split is absent from the common request type; it exists only on Osprey radar types. | `DataPolicy` is an enumeration whose only field users are `TOBCreateRadarRequest.policy`, `RadarCreateDependency.policy`, `RadarCreateTrackDependency.policy`, `TOBCreateRadarRequirement.policy`, `RadarCreateDependencyRequirement.policy` — **not** `ObjectDetailRequest`/`Requirement` | Per D2.3: add `policy: DataPolicy` to the common `ObjectDetailRequest` **and** `ObjectDetailRequirement` in the model. | G4, D2.3 |
| MM5 | **Location criteria narrowed — no `circle_area`.** Legacy interest took `oneof {PolyArea, CircleArea, Point}`; the new request carries only `position` + `polyArea`. The `CircleArea` type still exists in `common` but is not wired to the request. | `ObjectDetailRequest` / `ObjectDetailRequirement` properties: `…, position, …, polyArea` — no circle field | Decide (open sub-decision, see below): either add a `circle_area` field to the common request in the model, or accept the expressiveness regression (runtime already reduces all shapes to bounding boxes, so it is contract-only). | G4 |
| MM6 | **`ObjectDetail` carries no `position`; positioned details are refined subtypes that must be wrapper-wired.** The streamed detail type has no location field, so the adapter must emit a positioned *subtype*, which only works if those subtypes are refinement-wired into the information wrapper. | `ObjectDetail` properties: `speed, course, dimension, standard_identifer, source, creationTime, quality, length` — **no `position`**; positioned variants are `SinglePointObject` / `CircleAreaObject` / `PolyAreaObject` | Ensure the positioned subtypes are «Refine»-wired into the `Specific_Object_Detail` information wrapper (same class of fix as MM3), so the detail stream can carry position via a subtype variant. | G4 |
| MM7 | **No correlation / requirement-id field on the streamed elements** — required by the D2.2 resolution. The chosen design tags each detail/match tick with the originating requirement id for tracing, but the streamed element types carry no such id today. | `ObjectMatch` has a single property `matchingObjectId`; `ObjectDetail` has no id/correlation field at all | Add a requirement-id (correlation) field to the streamed element or its wrapper — or confirm the id rides an envelope/`Entity.id` level the adapter can populate. Needed before D2.2's "correlate rather than filter blindly" holds. | D2.2 (new), G3 |
| MM8 | **`pTPTrackControl` / `pTPRadarTrackData` share the MM1 defect** (kind-less), but are **deferred**. | Both Osprey project ports resolve `kind=None` (types `TP_RadarTrackControlInterface`, `RadarTrackData` reach no base service) | Out of scope per D2.5; apply the MM1 fix when the radar/track integration work picks them up. Listed only so the model gap is not mistaken for complete. | D2.5 |
| MM9 | **Field-name defect baked into the model: `standard_identifer`** (missing the second `i`). Consumers and the adapter must match the misspelling, or it must be corrected once across the model. | field `standard_identifer` on `ObjectDetail` / `ObjectDetailRequest` / `ObjectDetailRequirement` | Low priority: either fix the spelling in the model (ripples to all generated bindings + consumers) or accept and document it. Flag now so it is a conscious choice, not a silent typo. | G7 |

**One sub-decision this surfaces (MM5).** The `circle_area` narrowing is
the only genuinely open model choice not already covered by D2.1–D2.6: add
`circle_area` to the common request, or drop circle support as a contract
regression. Recommendation: **add it** — the type already exists in
`common`, the cost is one field, and it keeps the new contract a superset
of the legacy criteria surface (avoids a silent capability loss even though
the runtime tolerates it). Treat as **D2.7**.

**Sequencing.** MM1–MM4, MM6, MM7 are all prerequisites for M1
(regenerate `pim/test/`) and must land together in the model so the first
regeneration drift is exactly the intended new services/fields. MM5/MM9 are
independent contract-quality choices; MM8 is deferred. None of these touch
generator code — every one is a model edit consumed by existing machinery,
which is what keeps the migration a "model classification" problem (§2.7).

## 2.7 What this does *not* require

Worth stating to bound the work: no PCL runtime changes, no generator
feature work beyond the model edits (ports, fields, refinements — all
existing machinery), no new transport or codec capability, and no changes
to the Tactical Objects business runtime. The delivered pub/sub,
plugin, manifest, facade, and typed-ROS2 workstreams were, in effect, the
enabling programme for exactly this migration; the remaining work is
model classification, one build seam, one adapter, and test porting.
