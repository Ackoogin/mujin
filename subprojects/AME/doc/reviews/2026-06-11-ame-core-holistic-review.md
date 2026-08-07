# ame_core Holistic Review — SOLID, HLR Adherence, Clean Architecture, Hard-Coding

**Date:** 2026-06-11
**Scope:** the `ame_core` static library — `subprojects/AME/src/lib/`, `subprojects/AME/src/nodes/`, `subprojects/AME/include/ame/` — reviewed against `doc/requirements/core_HLR.md` (AMC.001–064, design decisions D1–D10).
**Out of scope:** `ame_foxglove`, `ame_neuro` internals, the ROS2 node layer, LAPKT, PCL/PYRAMID subprojects (touched only where they leak into `ame_core`).

Findings are ordered by severity. Each has an ID (`F-nn`) used by the remediation plan
at the end. Items needing a user decision are tagged `[DECISION Dx]` and collected in
the *Decisions Requested* section.

---

## 1. Critical — fail-open behaviour (violates D5 fail-closed model)

These are the highest-priority findings: each one allows the engine to report success
for work that did not verifiably happen, directly contradicting D5 and the workspace
"no hacky fallbacks / fail closed" rule.

### F-01 Unregistered actions compile to "apply effects without doing anything"
`PlanCompiler::emitActionUnit` (`src/lib/plan_compiler.cpp:66-69`) emits the action's
implementation **only if** `registry.hasAction(name)`. If the action is not
registered, the emitted unit is: precondition checks → *nothing* → `SetWorldPredicate`
effect writes. The plan step "succeeds" and writes its predicted effects as BELIEVED
state with no physical action ever dispatched. This is a textbook phantom-success
path and violates AMC.027 ("the registry is the sole bridge"), AMC.033, and D5.

**Fix (D-2 ACCEPTED — hard error at compile time):** compilation fails closed —
`compile()`/`compileSequential()` return an explicit error (or throw) when any plan
step's action name has no registration. No emit-with-error-leaf fallback; fail at the
earliest possible point.

### F-02 `AmeDispatchNode` treats a lost result as SUCCESS
`src/nodes/ame_dispatch_node.cpp:93-96` (`onRunning`): if the sink no longer reports
the command pending but has **no terminal result**, the node returns
`BT::NodeStatus::SUCCESS` ("treat as done"). A result lost to a sink `reset()`, a
crashed integration, or a contract bug silently becomes mission progress.
`IdentifyTarget` (`src/nodes/identify_target.cpp`) returns FAILURE in the same
situation — the two nodes disagree on fail direction. Violates D5, AMC.046.

**Fix:** return FAILURE (consistent with `IdentifyTarget`), and log/emit the anomaly.

### F-03 Silent precondition/effect dropping during grounding
`WorldModel::groundActionSchema` (`src/lib/world_model.cpp:404-428`): when a
precondition/add/del template fails to resolve to a fluent index, it is **silently
skipped**. A typo'd predicate, an un-grounded template, or an unsupported PDDL
construct produces a ground action with *fewer preconditions than the domain
declared* — the planner will then schedule actions whose guards were silently
relaxed. Violates AMC.005 and D5; also undermines the SACE assurance argument, which
assumes the grounded model is faithful to the domain.

**Fix:** unresolved templates must be a hard grounding error (with the offending
template + schema named), not a skip. Parameter-free mismatches (template referencing
objects outside the cross-product) should be distinguished from typos if needed, but
the default must be fail-closed.

### F-04 PDDL parser silently mangles unsupported constructs
`src/lib/pddl_parser.cpp:192-227`: `parsePrecondition`/`parseEffect` only understand
bare atoms, `(and …)`, and `(not …)` in effects. A `(not …)` **precondition**, `(or
…)`, `(forall …)`, `(= …)`, `(when …)` are parsed as if they were atoms — producing
garbage templates like `"(not)"` that F-03 then silently drops. The net effect: a
domain using one unsupported construct loads "successfully" with quietly altered
semantics. There is also no `:requirements` validation and no `(:constants)` support.
Violates AMC.017 (declared STRIPS-only scope must be *enforced*) and D5.

**Fix (D-4 ACCEPTED — reject + audit domains):** the parser must reject (throw with
location/context) any construct outside the supported STRIPS subset, and validate
`:requirements`. This will break any existing domain that silently relied on the
lenient behaviour, so `domains/` is audited for affected files in the same change.

### F-05 `PerceptionBridge` writes perception facts as BELIEVED
`src/lib/perception_bridge.cpp` (`flush()`): calls `wm_.setFact(fact, value, source)`
without an authority argument, so perception-sourced facts land with the default
`FactAuthority::BELIEVED`. **Direct violation of AMC.011** ("facts written through
the perception ingestion boundary shall be recorded with CONFIRMED authority").
The parallel path `WorldModelComponent::applyDetection` gets it right
(`enqueueMutation(..., FactAuthority::CONFIRMED)`), so the engine has two perception
boundaries with different authority semantics — exactly what D9 forbids. Downstream,
`hasAuthorityConflict` and `required_authority="confirmed"` guards are blind to
anything ingested via `PerceptionBridge`.

**Fix:** `PerceptionBridge::flush` must pass `FactAuthority::CONFIRMED` (and should
use the mutation queue, see F-16). Consider deleting `PerceptionBridge` entirely in
favour of the queue-based boundary — two ingestion paths is one too many.

### F-06 `FollowRoute` is a stub that always succeeds
`src/nodes/follow_route.cpp`: an empty/missing route returns SUCCESS ("stub/test
mode" comment, lines 73-78), and `onRunning` "advances through all waypoints
immediately" (lines 86-91). A compiled plan containing `FollowRoute` reports mission
progress with zero physical motion — the exact failure mode the gate-demo rule
("demos must prove tasking is physically carried out") exists to catch. Additionally
`std::stod` is unguarded (malformed waypoint → exception through the tick), and the
route format is an ad hoc `k=v;` / `|`-separated string protocol.

**Fix (D-5 ACCEPTED — convert to sink dispatch):** make `FollowRoute` a real
sink-dispatching node like `AmeDispatchNode`, with empty/missing route = FAILURE (not
SUCCESS) and guarded waypoint parsing. It stays in `ame_core` because it has a real
dispatch role; the immediate-advance stub behaviour is removed.

### F-07 No empty-goal rejection in the planner
`Planner::solve` (`src/lib/planner.cpp:18`) never checks
`wm.goalFluentIds().empty()`. An empty goal set projects to an empty LAPKT goal,
which is trivially satisfied → success with an empty plan → `PlanCompiler` emits an
empty `<Sequence/>` that succeeds. **Direct violation of AMC.024** ("shall report
failure for a request with no goal"). `PlannerComponent::solveGoal` and the phase
node both forward un-validated goal lists into this hole.

**Fix:** `solve()` returns `success=false, error_msg="no goal fluents"` when the goal
set is empty; add the same validation to `PlannerComponent::solveGoal`.

### F-08 Unknown authority enum coerced to CONFIRMED
`CurrentAmeBackendAdapter::toWorldModelAuthority`
(`src/lib/autonomy_backend.cpp:500-509`): the fall-through return after the switch is
`FactAuthority::CONFIRMED`. An out-of-range/corrupt `FactAuthorityLevel` (e.g. from a
deserialised transport message) is promoted to *observed ground truth*. Fail-open in
the trust-escalating direction.

**Fix:** fall through to `BELIEVED` (lower trust) — or better, throw, since an
unknown enum is a contract violation.

### F-09 `AgentDispatcher` reports success when no port exists
`src/lib/agent_dispatcher.cpp:131-136`: if the agent has no registered BT publisher
port, dispatch returns `result.success = true` ("in-process / test mode" comment).
A misconfigured roster (agent in WorldModel but not in `agent_ids` param) silently
"succeeds" without sending anything. Test-mode behaviour living in the production
path.

**Fix:** missing port = failure. Test code can register a loopback port.

---

## 2. Major — architecture & HLR boundary violations

### F-10 `ame_core` links PCL and PYRAMID — direct AMC.063 violation
`src/CMakeLists.txt:64-71`: `target_link_libraries(ame_core PUBLIC behaviortree_cpp
lapkt_core pcl_core pyramid_core pyramid_generated_codecs)`. The HLR is unambiguous:
*"`ame_core` shall depend only on the Behaviour Tree library and the planner core,
with no ROS2, DDS, socket, or other transport dependency"* (AMC.063, D2). Today
`ame_core` contains:

- three `pcl::Component` lifecycle subclasses (`world_model_component.cpp`,
  `planner_component.cpp`, `executor_component.cpp`) doing `pcl_port_publish`,
  `addService`, `addSubscriber`;
- `agent_dispatcher.cpp` (PCL pub/sub fan-out);
- `pcl_msg_json.cpp` (hand-rolled transport codecs);
- `pyramid_autonomy_bridge.cpp` (conditionally, behind `UNMANNED_BUILD_PYRAMID`).

This is the single largest divergence between the documented architecture and the
build reality. It also means the pure algorithmic core (WorldModel/Planner/
PlanCompiler/nodes) cannot be consumed without dragging PCL in.

**Fix:** split into two libraries — `ame_core` (WorldModel, TypeSystem, Planner,
PlanCompiler, ActionRegistry, PddlParser, BT nodes, loggers, execution sink) and
`ame_pcl_components` (the four PCL components + `pcl_msg_json` +
`pyramid_autonomy_bridge`). The components already depend on core only through its
public headers, so this is mostly a CMake/file move. **(D-3 ACCEPTED — split
approved.)** Downstream consumers (`_ame_py`, ROS2 layer, apps) relink against the
new `ame_pcl_components` library.

### F-11 Hard-coded verb table in `ExecutorComponent::on_configure`
`src/lib/executor_component.cpp:120-129`: fourteen verb strings
(`"navigate-to-waypoint"`, `"search-area"`, … `"takeoff"`, `"land"`, plus underscore
variants) are compiled into the executor and registered against `AmeDispatchNode`.
This is exactly the "hardcoded verb table" anti-pattern the workspace rules prohibit:
adding a domain action requires editing and rebuilding core C++ (OCP violation), and
the duplicated hyphen/underscore variants are a smell that the naming contract is
unresolved. AMC.042 says "a dispatch leaf **per registered verb**" — the registration
source should be the `ActionRegistry`/domain, not a literal list.

**Fix (D-6 ACCEPTED — auto-register from `ActionRegistry`):** derive dispatch-leaf
registration from the `ActionRegistry` contents at tree-load time (BT.CPP allows
registering a fallback factory or registering node types just before
`createTreeFromText` from the names present in the compiled XML). Fail closed on names
that are neither built-in nodes nor registered verbs. The hard-coded 14-verb table is
deleted.

### F-12 Shared-WorldModel goal clobbering (D1 violation)
Four call sites mutate the *single authoritative* WorldModel's goal set as a way of
passing a parameter to `Planner::solve`, and none restore it:

- `ExecutePhaseAction::planDirect` (`src/nodes/execute_phase_action.cpp` —
  `wm->setGoal(goals)`): a phase node *permanently replaces the parent mission's
  goals*. After the phase, the compiled goal-guard (`emitGoalGuardOpen`) and
  `CurrentAmeBackendAdapter::goalsSatisfied()` evaluate the **phase** goals, not the
  mission goals.
- `DelegateToAgent::onStart` (`src/nodes/delegate_to_agent.cpp`) — same, and two
  delegations in parallel flows race on the same field.
- `AgentDispatcher::dispatchToAgent` (`src/lib/agent_dispatcher.cpp`) — sequential
  per-agent dispatch each overwrites the previous goal set.
- `PlannerComponent::snapshotWorldModel` is the only correct pattern (it copies the
  WM first).

This is a real correctness bug today, not just a style issue.

**Fix (D-7 ACCEPTED — per-solve goal API):** make goals a per-solve input —
`Planner::solve(const WorldModel&, const std::vector<unsigned>& goal_ids)` (keep a
thin compat overload reading WM goals during migration), and have phase/delegate nodes
plan against a copied/projected WM like `PlannerComponent` does. `WorldModel::setGoal`
is deprecated as the planning input; mission goals are owned by the backend, not the
WM. The four `setGoal` clobber sites are deleted.

### F-13 `WorldModel` is a god class (SRP)
`include/ame/world_model.h` — one class owns: the type system facade, predicate
schemas, eager grounding, the action schema store + grounding, the fact bitset +
metadata, audit callback, goal set, the perception mutation queue, snapshots, STRIPS
projection, **and the multi-agent registry** (`AgentInfo`, registerAgent/getAgent/
availableAgentIds). The agent roster has nothing to do with grounded boolean state
(it isn't fluent-indexed, versioned, audited, or lock-protected) and exists only for
`GoalAllocator`/`DelegateToAgent`.

Compounding issues:
- `getAgent` returns a raw mutable `AgentInfo*` into a `std::vector` —
  `registerAgent` can reallocate and dangle it, and `available` flips are unlocked
  and un-audited.
- Copy ctor/assignment don't lock `other`'s mutex — copying while a perception
  thread writes is a data race (used by `PlannerComponent::snapshotWorldModel`).
- `version_` is a plain `uint64_t` read without the lock by `version()`.

**Fix:** extract `AgentRegistry` as its own component (audited availability writes);
fix copy-under-lock; make `version_` atomic. Longer term, separate "planning model"
(types/predicates/schemas/grounding) from "state store" (bits/metadata/queue).

### F-14 Executor introspects compiled-tree shape with magic strings
`ExecutorComponent::on_tick` repair-hook path
(`src/lib/executor_component.cpp:190-245`): computes the failed-step index by
`dynamic_cast`-walking the live tree, special-casing the goal guard by checking
`first->name() == "GoalCheck"` and inferring "action unit vs flow" from whether the
first child is a `ControlNode`. The executor is now structurally coupled to the
private XML emission conventions of `PlanCompiler` — change the compiler's shape and
the repair seam silently miscounts. Violates D3 (compiler output is supposed to be an
opaque contract) and OCP.

**Fix:** have `PlanCompiler` emit step metadata (e.g. a step-index attribute on each
action unit, or a sidecar step map returned with the XML) and have the executor read
that, not the tree topology.

### F-15 `ActionRegistry` implements 2 of the 3 required levels
AMC.028 requires *simple node*, *template*, and **pre-authored BT XML file** action
implementations. `include/ame/action_registry.h` has `SimpleNode` and
`SubTreeTemplate` only. Callers can fake the third by reading a file into
`registerActionSubTree`, but then positional `{paramN}` substitution applies and
file/load errors surface at the wrong layer. **(D-8 ACCEPTED — implement.)** Add
`registerActionFile(pddl_name, path)` (loaded and validated at registration time, fail
closed) as the third AMC.028 level; the HLR is not amended.

### F-16 Tick reads live state instead of the snapshot (AMC.014/D9 gap)
`WorldModel::captureSnapshot()`/`WorldStateData` exist precisely so a BT tick reads a
stable state, but nothing in the execution path uses them: `CheckWorldPredicate`,
`SetWorldPredicate`, and the guards all hit the live `WorldModel` per-node, taking
the shared lock per fact. Consequences: (a) one tick can observe perception writes
mid-traversal (the inconsistency AMC.014 exists to prevent); (b) `projectToSTRIPS`
likewise reads `getFact(i)` one lock at a time, so a planning projection can be torn
across concurrent writes — undermining AMC.021/022 determinism in deployed (threaded)
configurations. `SnapshotManager::publish` has the same torn-read pattern.

**Fix:** capture one snapshot at tick start (executor sets it on the blackboard;
condition nodes read the snapshot, action nodes write through the live WM), and build
the STRIPS init vector from a single locked capture.

### F-17 Stringly-typed core contracts and duplicated parsers
The signature `verb(a,b)` and fluent `(pred a b)` formats are parsed/produced by ad
hoc string code in at least five places:

- `PlanCompiler::actionName/actionParams` (`src/lib/plan_compiler.cpp:17-37`) and a
  duplicate pair in `CurrentAmeBackendAdapter` (`src/lib/autonomy_backend.cpp:468-498`);
- `WorldModel::substituteTemplate` (`src/lib/world_model.cpp:357-372`) — **naive
  substring replacement with a real prefix-collision bug**: with params `?s` and
  `?sector`, substituting `?s` first corrupts `?sector` occurrences (e.g.
  `"(at ?sector)"` → `"(at uav1ector)"`). Any domain whose parameter names share a
  prefix grounds incorrectly *silently* (compounded by F-03 swallowing the fallout).
- `StubSpatialOracle` (`src/lib/spatial_oracle.cpp`) — prefix matching where
  `fname.substr(0,3) == "(at"` also matches `(attack …)`, `(attached …)`, etc.;
- `GoalAllocator::extractSector` — "last token before `)` is the sector".

**Fix:** one token-aware utility (parse fluent key → predicate + args; format back),
token-boundary-safe template substitution, and replace prefix matching with
predicate-name equality. The duplicate signature parsers collapse into it.

### F-18 Hard-coded domain vocabulary inside core C++
`StubSpatialOracle` bakes `(reachable …)`/`(nearest …)`/`(at …)` semantics — and a
"first alphabetical reachable location" policy — into the engine.
`GoalAllocator` bakes a "goals ending in the same token belong to the same sector,
round-robin sectors over agents" allocation policy. Both are mission/domain logic
living in C++ rather than in the domain model (the same class of violation as the
AutoMTK "domain-driven planning" rule). The `Stub` prefix acknowledges this for the
oracle, but it ships in `ame_core` and is callable from production wiring.

**Fix:** move both behind explicitly-named interfaces (`ISpatialOracle`,
`IGoalAllocator`) injected at the backend boundary, keep the stub implementations in
test/demo targets only, and document that real implementations derive vocabulary
from the loaded domain, not hard-coded predicate names.

### F-19 Authorisation-gate authority contract is split across languages
`AuthorisationGuard` (`src/nodes/authorisation_guard.cpp`) defaults
`required_authority="confirmed"`, but the in-tree comment admits the Python subtree
compiler deliberately emits `required_authority="any"` because operator
authorisations arrive as BELIEVED facts from source `"operator"`. Net effect: the
native default is safe-looking but every real emitted gate runs in the weaker mode,
and the safety semantics live in a *comment* spanning two codebases. For a
fail-closed permission gate (AMC.042, D5) this needed a single, explicit decision.

**Fix (D-1 ACCEPTED — CONFIRMED via ingestion):** operator authorisation writes go
through the perception/operator ingestion boundary as CONFIRMED authority (with
`operator` a recognised confirming source). `AuthorisationGuard` keeps its native
`required_authority="confirmed"` default, and the Python subtree compiler must stop
emitting `required_authority="any"` — the two codebases converge on the
confirmed-authority contract rather than encoding it in a comment.

---

## 3. Moderate — robustness, observability, consistency

### F-20 Pervasive silent exception swallowing
`catch (const std::exception&) {}` / `catch (...) {}` followed by a bare FAILURE or
skip appears throughout (`execute_phase_action.cpp`, `delegate_to_agent.cpp`,
`invoke_service.cpp`, `world_model_component.cpp`, `planner_component.cpp`,
`spatial_oracle.cpp`). The mission fails with no diagnostic anywhere in the 6-layer
observability stack. Worst case: `WorldModelComponent::loadDomainFromStrings`
re-applies preserved facts with `catch (...) {}` — facts not present in the new
domain vanish silently, **and** all preserved facts lose their authority and
timestamp (a CONFIRMED perception fact becomes BELIEVED after a domain reload —
an AMC.009/D9 provenance loss). Blackboard-miss catches should distinguish
"entry absent" from real errors, and every degraded path should emit one structured
event.

### F-21 Audit-record gaps (AMC.055/062)
- `ExecutePhaseAction::planViaComponent` records its episode against a
  default-constructed `WorldModel{}` — `init_facts` is always empty, so the episode
  is not "independently reconstructable" (AMC.055).
- `Planner::solve` sets `heuristic_source = "neural_hook"` *before* checking whether
  the hook returned scores; an empty return still reports neural provenance
  (AMC.062 accuracy).
- `PlanAuditLog::Episode` has `session_id`-less records from the component path
  while `DecisionRecord` carries it — cross-stream correlation (AMC.058) relies on
  timestamps alone there.

### F-22 Unbounded in-memory observability buffers
`AmeBTLogger::events_`, `WmAuditLog::entries_`, `PlanAuditLog::episodes_` all grow
without bound for the process lifetime. On a long mission the "non-perturbing"
observability stack (AMC.057) becomes a slow memory leak. Cap, ring-buffer, or drop
the in-memory copies (the JSONL files already persist everything).

### F-23 `jsonEscape` triplicated and incomplete
Three identical private copies (`plan_audit_log.cpp`, `wm_audit_log.cpp`,
`bt_logger.cpp`); none escapes control characters below 0x20, so a fact/source
containing one emits invalid JSONL. Consolidate into one header utility and complete
the escape set. Same for the XML emission in `PlanCompiler`/`ActionRegistry::resolve`
— attribute values are not XML-escaped (`&`, `<`, `"` in an object name or template
parameter breaks the compiled tree at load time; fail at *emission* time instead).

### F-24 Sink resubmission contract is ambiguous
`FormationHold::onRunning` and `IdentifyTarget::onRunning` re-`submit()` the *same
command_id* every tick while pending; `CommandQueueExecutionSink::submit`
unconditionally appends to `command_queue_`, so downstream consumers see the same
command N times (deduplicating by id is implicit, undocumented). `AmeDispatchNode`
submits once. Pick one contract (submit-once + poll is the obvious one — it is what
`IExecutionSink`'s documentation implies), enforce it in the sink (reject duplicate
ids), and fix the two streaming nodes to poll.

### F-25 Compilation determinism is implementation-defined (AMC.036)
`extractFlows` (`src/lib/plan_compiler.cpp:207-218`) groups flows via
`std::unordered_map` and emits them in hash-iteration order. For a given libstdc++
build this is stable, but it is not *specified* — across standard libraries or
hash-seed changes the same plan can compile to differently-ordered Parallel children,
breaking byte-identical reproducibility claims. Order flows by their minimum step
index (a `std::map` or post-sort).

### F-26 Grounding performance is O(N²)-ish and re-does work
`groundActionSchema` checks duplicates with a linear scan over all existing ground
actions (quadratic in actions); `groundNewObject` clears and re-grounds **all**
schemas on every `addObject`. Fine for demo domains, hostile to the "repeated
replanning without rebuilding" goal (D8) at scale. Use an `unordered_set<string>` of
signatures and ground incrementally per new object.

### F-27 `WorldModel` registration is not thread-safe relative to readers
`registerPredicate`/`addObject`/`registerAction` mutate `fluent_names_`,
`fluent_index_`, `ground_actions_` with **no lock**, while `setFact` reads
`fluent_names_[id]` under `state_mutex_` and other threads may call `getFact`
concurrently. The implicit rule "ground everything before going multi-threaded"
exists nowhere in the headers. Either document + assert that invariant (single
"sealed" flag flipped before execution starts) or take the write lock during
registration. The unlocked `version()` read and unlocked copy-ctor (F-13) belong to
the same family (AMC.015).

### F-28 Misleading names and dead/odd API surface
- `InvokeService` ports `request_json`/`response_json` carry `k=v;k=v` strings, not
  JSON.
- `ExecutorComponent` registers `EnsureAltitude`/`FormationHold`/`IdentifyTarget`
  but `AmeDispatchNode` duplicates `action_command_builder.h` logic inline
  (`kMaxParams` defined twice) — fold it into the shared builder.
- `kMaxParams = 8` silently truncates >8-parameter actions at dispatch; assert or
  derive from the registry.
- `WorldModel::move` ctor "behaves like copy" — surprising; either delete moves or
  implement them properly.
- `TypeSystem::isSubtype` recurses with no cycle guard — a cyclic `:types` section
  (parse-able today) gives infinite recursion; `addObject` appends duplicates to
  `object_order_`.

---

## 4. What is in good shape

For balance — the review also confirmed substantial strengths worth preserving:

- The **WorldModel bitset + stable fluent index** design (AMC.004–008) is clean and
  matches the HLR exactly; the mutation queue (AMC.013) is the right shape.
- **`IExecutionSink`** (`execution_sink.h`) is a well-designed seam: the
  `RequirementBindingExecutionSink` policy ladder (`CommandOnly` /
  `PreferTypedPlacement` / `RequireTypedPlacement`) correctly *rejects* unbound
  commands with `FAILED_PERMANENT` in the strict mode — a good fail-closed pattern
  the rest of the codebase should copy.
- The **neuro seams** (heuristic reorder-only hook, repair hook falling back to the
  baseline failure path, `AME_NEURO` compiled out by default) faithfully implement
  AMC.059–061/D10 — the hook can bias search order but cannot change plan
  admissibility, and exceptions are contained.
- The **causal-graph compiler** (AMC.031–034) implements all three edge types
  (add→pre, del→pre, interference) as specified.
- `CommandQueueExecutionSink::pushResult` and the backend's
  `pushCommandResult`/`pushDispatchResult` **throw on unknown ids** — correct
  fail-closed posture at the result-ingestion boundary.
- The observability layering (BT events ↔ wm_version correlation in `AmeBTLogger`)
  satisfies the AMC.054/058 correlation requirement.

---

## 5. Decisions requested

**All eight decisions resolved 2026-06-14 (operator interview). Every decision was
accepted at the reviewer's recommended option.** The `Decision` column below is
authoritative; affected findings' `Fix:` text has been updated to record the resolved
intended action. The remediation plan (§6) already references each decision as
`(per D-x)` and, because every decision matched its recommendation, those items stand
unchanged.

| ID | Question | Recommendation | Decision (2026-06-14) |
|----|----------|----------------|-----------------------|
| **D-1** | Operator authorisation gates (F-19): should `(authorised <gate>)` writes from the operator path be CONFIRMED authority (gate keeps `required_authority="confirmed"`), or do we formalise BELIEVED+source-tag as the operator trust model (gate checks source, not authority)? This decides the cross-language contract with the Python subtree compiler. | Make operator writes CONFIRMED via the perception/operator ingestion boundary; keep the native default. | **ACCEPTED.** Operator authorisations written CONFIRMED via the perception/operator ingestion boundary; `AuthorisationGuard` keeps `required_authority="confirmed"`. The Python subtree compiler must stop emitting `required_authority="any"`. |
| **D-2** | Unregistered plan action at compile time (F-01): hard error (throw / failed compile result) or emit-with-error-leaf (a leaf that always returns FAILURE at runtime)? | Hard error at compile time — earliest possible failure. | **ACCEPTED.** Hard error at compile time: `compile()`/`compileSequential()` fail closed on any unregistered action name. |
| **D-3** | Approve the library split (F-10): `ame_core` (pure, BT.CPP+LAPKT only) + new `ame_pcl_components` (PCL components, msg codecs, PYRAMID bridge)? Mostly CMake/file moves, but downstream consumers (`_ame_py`, ROS2 layer, apps) must relink. | Yes — this is the HLR-mandated boundary (AMC.063/064). | **ACCEPTED.** Split into `ame_core` (pure, BT.CPP+LAPKT only) + new `ame_pcl_components`; downstream consumers relink. |
| **D-4** | PDDL parser strictness (F-04): reject unsupported constructs outright? This may break existing demo domains that load today with silently-altered semantics. | Reject. Audit `domains/` for affected files in the same change. | **ACCEPTED.** Parser rejects (throws with location/context) any non-STRIPS construct and validates `:requirements`; `domains/` audited for breakage in the same change. |
| **D-5** | `FollowRoute` stub (F-06): delete from `ame_core` (move to tests), or convert to a real sink-dispatching node? | Convert to sink dispatch (it has a real role); empty route = FAILURE. | **ACCEPTED.** Convert `FollowRoute` to a real sink-dispatching node; empty/missing route = FAILURE. |
| **D-6** | Dispatch verb registration (F-11): drive from configuration (PCL param / domain file), or auto-register from compiled-XML node names with a registered-verb allowlist from `ActionRegistry`? | Auto-register from `ActionRegistry` at load; unknown names fail tree load (fail closed). | **ACCEPTED.** Auto-register dispatch leaves from `ActionRegistry` at tree-load time; names that are neither built-in nodes nor registered verbs fail tree load. |
| **D-7** | Goal scoping API (F-12): change `Planner::solve(wm)` → `solve(wm, goal_ids)` and deprecate `WorldModel::setGoal` as the planning input (kept only as "mission goal" state owned by the backend)? Breaking API change for callers including `_ame_py`. | Yes — per-solve goals; mission goals stay on the backend, not the WM. | **ACCEPTED.** Per-solve goal API `solve(wm, goal_ids)`; mission goals owned by the backend, not the WM. Thin compat overload kept during migration. |
| **D-8** | AMC.028 third level (F-15): implement `registerActionFile()` pre-authored support, or amend the HLR to two levels? | Implement — it is small and the HLR is already baselined. | **ACCEPTED.** Implement `registerActionFile()` (pre-authored BT XML, loaded + validated at registration time, fail closed). |

---

## 6. Remediation plan

Phased so that every phase leaves the build green and tests passing; each item names
its findings. P0 items are independent and small; P1–P2 are structural.

### P0 — Fail-closed correctness (no API breaks beyond error paths)
1. **F-01**: `PlanCompiler` fails on unregistered actions (per D-2).
2. **F-02**: `AmeDispatchNode` lost-result → FAILURE + anomaly event.
3. **F-03**: grounding errors on unresolved templates.
4. **F-05**: `PerceptionBridge` writes CONFIRMED via the mutation queue.
5. **F-07**: empty-goal → planner failure (planner + `PlannerComponent`).
6. **F-08**: unknown authority → BELIEVED/throw.
7. **F-09**: `AgentDispatcher` missing port → failure.
8. **F-17 (bug subset)**: token-boundary-safe `substituteTemplate`; fix
   `StubSpatialOracle` `(at`-prefix match.
9. Tests: one regression test per item (the phantom-success compile, the lost-result
   dispatch, the prefix-collision grounding, the BELIEVED-perception conflict-detect
   miss, the empty-goal trivial plan).

### P1 — Parser strictness + library boundary
1. **F-04**: strict STRIPS validation in `PddlParser` (+ `:requirements`,
   `(:constants)`); audit `domains/*` for breakage (per D-4).
2. **F-10**: split `ame_pcl_components` out of `ame_core`; CMake + include moves;
   update `_ame_py`, apps, ros2 layer, tests (per D-3).
3. **F-15**: `registerActionFile()` pre-authored level (per D-8).

### P2 — Goal scoping and WorldModel decomposition
1. **F-12**: per-solve goal API (per D-7); migrate phase/delegate/dispatcher to
   copied-WM planning; delete the four `setGoal` clobber sites.
2. **F-13**: extract `AgentRegistry`; fix copy-under-lock; atomic `version_`.
3. **F-16**: snapshot-per-tick on the blackboard; locked single-capture STRIPS
   projection; fix `SnapshotManager` torn read.
4. **F-27**: registration sealing or write-locked registration.

### P3 — Coupling, contracts, hygiene
1. **F-11**: registry-driven dispatch-verb registration (per D-6).
2. **F-14**: compiler emits step metadata; executor stops tree-shape introspection.
3. **F-06**: `FollowRoute` per D-5.
4. **F-17**: shared fluent/signature parse-format utility; delete duplicate parsers.
5. **F-18**: `ISpatialOracle`/`IGoalAllocator` interfaces; stubs out of core wiring.
6. **F-19**: implement D-1; align Python subtree compiler emission.
7. **F-24**: submit-once sink contract; duplicate-id rejection; fix
   `FormationHold`/`IdentifyTarget`.

### P4 — Observability and robustness polish
1. **F-20**: structured degraded-path events; fix domain-reload provenance loss.
2. **F-21**: real init-facts in component-path episodes; accurate
   `heuristic_source`; session id on episodes.
3. **F-22**: bounded in-memory log buffers.
4. **F-23**: shared `jsonEscape` (full control-char set) + XML attribute escaping.
5. **F-25**: deterministic flow ordering.
6. **F-26**: signature-set grounding; incremental per-object grounding.
7. **F-28**: naming/API cleanups (ports, `kMaxParams`, move semantics, type-cycle
   guard).

---

## 7. Traceability summary

| HLR / principle | Status | Findings |
|---|---|---|
| D1 single source of truth | **Violated** (goal clobbering, raw agent ptrs) | F-12, F-13 |
| D2 / AMC.063–064 middleware-agnostic core | **Violated** (PCL/PYRAMID linked) | F-10 |
| D4 / AMC.022, .036 determinism | At risk (torn reads, hash ordering) | F-16, F-25 |
| D5 fail-closed | **Violated** in 8 places | F-01…F-09 |
| D9 / AMC.009–015 authority + concurrency | Partially violated | F-05, F-13, F-16, F-20, F-27 |
| AMC.024 goal requirement | **Violated** | F-07 |
| AMC.028 three impl levels | Gap | F-15 |
| AMC.042 registrable guard/dispatch leaves | Hard-coded | F-11 |
| AMC.055/058/062 audit completeness | Gaps | F-21 |
| D10 / AMC.059–061 neuro seams | **Compliant** | — |
| AMC.031–034 causal compilation | Compliant (ordering caveat) | F-25 |
| SOLID: SRP | WorldModel god class; step() state machine | F-13 |
| SOLID: OCP | verb table, tree-shape introspection | F-11, F-14 |
| SOLID: DIP | oracle/allocator concrete in core | F-18 |
| Hard-coding | verbs, predicates, sector heuristic, k=v protocols | F-11, F-17, F-18, F-28 |
