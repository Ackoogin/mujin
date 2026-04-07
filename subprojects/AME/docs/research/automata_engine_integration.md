# AME ↔ PYRAMID Integration Analysis

## 4. Functional Comparison: AME vs automata_engine

This section compares the two systems on functionality alone (SACE assurance is covered in §1).

### 4.1 Capability Matrix

| Capability | AME | automata_engine | Notes |
|------------|-----|-----------------|-------|
| **Classical planning** | ✅ PDDL + LAPKT BRFS (complete, step-optimal) | ✅ GOAP A\* (cost-optimal with admissible heuristic) | Complementary — different optimality criteria |
| **Behaviour tree execution** | ✅ Full BT.CPP integration, compiled from plans | ⚠️ BtStrategy traverses a tree definition but doesn't use BT.CPP | AME's BT execution is more mature |
| **FSM execution** | ❌ | ✅ FsmStrategy + FsmPlanExecutor | Useful for procedural/scripted behaviours |
| **Reinforcement learning** | ❌ | ✅ QLearningStrategy (Q-table lookup) | Niche — requires offline training |
| **Cost-based optimisation** | ⚠️ BRFS step count only | ✅ Composable cost calculators (spatial, temporal, urgency, schedule-conflict) | Major gap in AME |
| **Temporal planning** | ❌ (researched, not implemented) | ✅ TemporalPlanExecutor, deadline monitoring, time-window constraints, TemporalConstraint | Major gap in AME |
| **Resource tracking** | ❌ | ✅ ResourceTracker (fuel, battery, ammunition with threshold depletion) | Major gap in AME |
| **Multi-asset coordination** | ✅ AgentInfo, DelegateToAgent, GoalAllocator | ✅ Asset-capability matching, CapabilityMatcher, asset-specific costs | Both have multi-asset; automata_engine's is richer for heterogeneous fleets |
| **Spatial awareness** | ❌ | ✅ GeodeticPositionDetail, SpatialCostCalculator, area decomposition | Major gap in AME |
| **Parameter resolution** | ✅ ActionRegistry param binding, `{param0}` templates | ✅ Multi-scope ParameterContext, ComputedParameterEngine, expression-based | automata_engine's is significantly more flexible |
| **Hierarchical planning** | ✅ ExecutePhaseAction (sub-planning per phase) | ✅ GoalResolver (failure-driven sub-goal decomposition) | Different strategies — AME is phase-based, AE is failure-driven |
| **Reactive replanning** | ✅ MissionExecutor replan-on-failure | ✅ triggerReactivePlanning on world state changes | AE is more event-driven; AME is more BT-tick-driven |
| **Observability** | ✅ 5-layer stack (BT events, WM audit, plan audit, Foxglove, JSONL) | ⚠️ EventBus + IPlanningEventListener (no file/WebSocket sinks) | AME is far ahead |
| **Causal-graph parallelism** | ✅ PlanCompiler analyses dependencies for parallel BT branches | ❌ Solutions are sequential action lists | Unique to AME |
| **External service integration** | ✅ InvokeService BT node (async, timeout) | ❌ Actions are in-process `IAction::execute()` | AME designed for distributed systems |
| **Perception integration** | ✅ PerceptionBridge (buffered, atomic flush, source-tagged) | ❌ WorldState updated directly | AME designed for sensor fusion |
| **Dependency injection** | ❌ Manual wiring | ✅ ServiceContainer | Useful for testability |
| **Validation framework** | ❌ | ✅ Diagnostic and validation subsystem | Useful for configuration correctness |
| **World state typing** | Boolean predicates (STRIPS) | Polymorphic DetailBase (bool, numeric, string, geodetic, timestamp, etc.) | Trade-off: AME is simpler/verifiable; AE is more expressive |

### 4.2 Summary

**AME strengths**: formal PDDL semantics, BT.CPP execution maturity, observability/auditability, causal-graph parallelism, distributed service calls, perception integration.

**automata_engine strengths**: cost optimisation, temporal/spatial awareness, resource tracking, rich typed state, multiple planning paradigms (GOAP/FSM/BT/QL), capability matching, parameter resolution, validation framework.

---

## 5. Use Cases for GOAP under PDDL Strategic Planning

When PDDL is the top-level strategic planner (mission-level goals), GOAP fills a distinct niche as a **tactical planner** — faster, cost-aware, and closer to execution. Below are concrete use cases.

### 5.1 Tactical Action Selection (Cost-Optimised)

**Scenario**: PDDL plan says "survey area A". Multiple assets with different capabilities, positions, and fuel levels could execute it. GOAP selects the cheapest assignment.

**Why GOAP, not PDDL?** PDDL's boolean predicates cannot natively represent floating-point costs, distances, or fuel levels. GOAP's typed `WorldState` with `SpatialCostCalculator` and `ResourceTracker` handles this naturally. PDDL decides *what* to do; GOAP decides *who does it and how*.

**automata_engine components used**: `GoapStrategy`, `SpatialCostCalculator`, `CapabilityMatcher`, `ResourceTracker`.

### 5.2 Real-Time Replanning on Execution Failure

**Scenario**: A PDDL-planned action fails mid-execution (e.g., waypoint blocked, sensor degraded). The system needs a fast tactical replan without re-running the full PDDL pipeline.

**Why GOAP, not PDDL?** LAPKT BRFS explores the full grounded state space — suitable for strategic planning but too slow for sub-second replanning. GOAP's A\* with a good heuristic prunes aggressively and finds a cost-optimal local fix quickly. The `GoalResolver` can decompose the failed sub-goal into achievable steps.

**automata_engine components used**: `GoapStrategy`, `GoalResolver`, `triggerReactivePlanning`.

### 5.3 Resource-Constrained Task Sequencing

**Scenario**: Multiple tasks to execute, assets have finite fuel/battery. The system must sequence tasks so no asset runs out of resources mid-mission.

**Why GOAP, not PDDL?** STRIPS has no native numeric fluents (AME uses boolean predicates). Modelling fuel as discretised boolean levels (fuel_high, fuel_medium, fuel_low) is brittle and explodes the state space. GOAP's `ResourceTracker` with `NumericDetail` handles continuous quantities directly.

**automata_engine components used**: `ResourceTracker`, `GoapStrategy` with resource-aware preconditions, `TemporalCostCalculator`.

### 5.4 Multi-Asset Area Decomposition

**Scenario**: PDDL plan says "search region R". The region must be divided among N assets based on position, capability, and fuel.

**Why GOAP, not PDDL?** This is fundamentally a spatial optimisation problem. PDDL can assign assets to sectors, but the sector decomposition itself (geographic area partitioning based on asset positions) is beyond PDDL's expressiveness.

**automata_engine components used**: `GeographicAreaDetail`, area decomposition engine, `SpatialCostCalculator`, `CapabilityMatcher`.

### 5.5 Deadline-Aware Task Prioritisation

**Scenario**: Multiple concurrent mission objectives with different deadlines. The system must prioritise tasks to meet time constraints.

**Why GOAP, not PDDL?** Classical PDDL has no temporal operators (AME's temporal extension research is unimplemented). GOAP's `TemporalCostCalculator` and `UrgencyCostCalculator` can weight actions by urgency and deadline proximity. `TemporalPlanExecutor` monitors deadlines during execution.

**automata_engine components used**: `TemporalCostCalculator`, `UrgencyCostCalculator`, `DeadlineDetail`, `TemporalPlanExecutor`.

### 5.6 Scripted Procedures via FSM

**Scenario**: Certain mission phases require strict procedural sequences (pre-flight checks, emergency procedures, docking sequences) that don't benefit from planning.

**Why FSM, not PDDL or GOAP?** These are deterministic, pre-authored sequences. Planning them wastes compute and adds unnecessary failure modes. An FSM with verified transitions is simpler and more assurable.

**automata_engine components used**: `FsmStrategy`, `FsmPlanExecutor`.

### 5.7 Learned Preferences via Q-Learning

**Scenario**: Repetitive operational decisions (e.g., preferred patrol routes, sensor mode selection) where historical performance data is available.

**Why Q-Learning?** The decision is too context-dependent for hand-authored rules and too low-level for PDDL. A pre-trained Q-table maps (state, action) → expected reward, enabling fast policy lookup without search.

**automata_engine components used**: `QLearningStrategy`.

### 5.8 Layered Architecture Summary

```
┌─────────────────────────────────────────────┐
│  PDDL + LAPKT (Strategic / Mission-Level)   │  "What needs to happen"
│  Complete, formally verifiable, auditable    │  Minutes-scale planning
├─────────────────────────────────────────────┤
│  GOAP + A* (Tactical / Task-Level)          │  "Who does it, how, at what cost"
│  Cost-optimal, resource-aware, fast replan   │  Sub-second planning
├─────────────────────────────────────────────┤
│  FSM (Procedural / Phase-Level)             │  "Execute this sequence exactly"
│  Verified transitions, no search overhead    │  Deterministic
├─────────────────────────────────────────────┤
│  Q-Learning (Preference / Micro-Level)      │  "Which option worked best before"
│  Offline-trained, policy lookup              │  Sub-millisecond
├─────────────────────────────────────────────┤
│  BT.CPP Execution Engine                    │  Tick-based execution, reactivity
│  AME's existing execution layer              │  Continuous
└─────────────────────────────────────────────┘
```

---

## 6. Reuse Opportunities: automata_engine Components → AME

This section identifies specific components from automata_engine that can be adapted or reused to extend AME, ordered by impact and feasibility.

### 6.1 High Impact, Low Effort — Direct Reuse

#### 6.1.1 Cost Calculators → AME Planner Heuristics

**What**: Port `ICostCalculator` interface and implementations (`SpatialCostCalculator`, `TemporalCostCalculator`, `CompositeCostCalculator`) into AME.

**How**: AME's `Planner::solve()` currently uses BRFS (uniform-cost). LAPKT supports weighted A\* — replacing BRFS with A\* and using automata_engine's cost calculators as the heuristic function gives AME cost-optimal planning without changing the PDDL front-end.

**Benefit**: AME gains cost-aware planning (spatial distance, temporal urgency) while retaining PDDL semantics and the full observability stack. The cost calculators are self-contained with minimal dependencies (just `DetailBase` types and `nlohmann/json`).

**Changes needed**:
- Abstract `ICostCalculator` adapted for AME's `WorldModel` (translate fluent keys ↔ DetailBase)
- LAPKT search algorithm changed from BRFS to A\* (LAPKT supports this natively)
- Cost calculator configuration added to planner config

#### 6.1.2 ResourceTracker → WorldModel Extension

**What**: Port `ResourceTracker` to track consumable resources (fuel, battery, mission time) within AME's `WorldModel`.

**How**: Add numeric resource tracking alongside boolean predicates. `ResourceTracker` is a standalone class that maintains resource levels, consumption rates, and threshold alerts. Wire it as a `WorldModel` extension — resource consumption becomes an additional effect applied alongside predicate changes.

**Benefit**: Enables resource-constrained planning without fundamentally changing the STRIPS model. Resource violations surface as additional precondition failures triggering replanning.

**Changes needed**:
- `ResourceTracker` integrated as WorldModel component
- New BT node `CheckResource` / `ConsumeResource`
- PlanCompiler emits resource consumption effects
- WmAuditLog extended with resource change events

#### 6.1.3 TemporalConstraint Types → Plan Execution

**What**: Adopt `DeadlineDetail`, `TimeWindowDetail`, `DurationDetail`, and `TemporalConstraint` data types.

**How**: Even without full temporal planning, AME's execution layer can use these types for **deadline monitoring**. The `MissionExecutor` / `ExecutorComponent` checks temporal constraints at each BT tick and triggers replanning if a deadline is about to be missed.

**Benefit**: Adds time-awareness to execution without the complexity of a temporal planner. This is the pragmatic middle ground identified in AME's `temporal_extension_research.md`.

**Changes needed**:
- Temporal data types added to WorldModel (or blackboard)
- ExecutorComponent checks deadlines pre-tick
- PlanAuditLog extended with temporal metrics

### 6.2 High Impact, Medium Effort — Adaptation Required

#### 6.2.1 GoapStrategy as Tactical Replanner

**What**: Integrate `GoapStrategy` as a fast tactical replanner invoked when a PDDL-planned action fails at execution time.

**How**: When a BT action node returns FAILURE, instead of immediately re-running the full PDDL pipeline, invoke GOAP with the current world state and the failed action's postconditions as goals. If GOAP finds a local fix, splice it into the BT. If not, escalate to full PDDL replanning.

**Architecture**:
```
BT Action FAILURE
  → GOAP tactical replan (sub-second)
    → Success? Splice fix into BT, continue
    → Failure? Full PDDL replan (seconds)
```

**Benefit**: Dramatically reduces replanning latency for common failure modes. PDDL only runs when the tactical planner can't find a local solution.

**Changes needed**:
- WorldModel ↔ WorldState adapter (boolean fluents ↔ typed DetailBase)
- GoapStrategy integrated as AME dependency (or compiled alongside)
- New BT meta-node `TacticalReplan` wrapping GOAP invocation
- PlanAuditLog extended with tactical replan episodes

#### 6.2.2 CapabilityMatcher → Multi-Agent Task Allocation

**What**: Port `AssetCapabilityRegistry`, `CapabilityMatcher`, and capability-based action filtering.

**How**: AME's current `GoalAllocator` does round-robin sector-based allocation. Replace with capability-aware allocation: each agent declares capabilities, each action declares required capabilities, the matcher assigns actions to capable agents with lowest cost.

**Benefit**: Enables heterogeneous fleet operations (e.g., UAV for survey, UGV for sample collection) where different agents have fundamentally different action repertoires.

**Changes needed**:
- `Capability` and `AssetCapabilityRegistry` adapted for AME's agent model
- `GoalAllocator` replaced/extended with capability-aware allocation
- PlanCompiler generates agent-capability-filtered BTs
- Action registration extended with capability requirements

#### 6.2.3 GoalResolver → Hierarchical Failure Recovery

**What**: Port `GoalResolver`'s failure-driven sub-goal decomposition.

**How**: AME has `ExecutePhaseAction` for pre-defined hierarchical decomposition. `GoalResolver` adds **automatic** decomposition: when planning fails, it analyses which preconditions are unsatisfied, creates sub-goals to establish them, and recursively plans. This is more flexible than pre-authored phases.

**Benefit**: Self-healing planning — the system can discover and execute prerequisite actions that weren't in the original mission specification.

**Changes needed**:
- GoalResolver adapted to work with AME's WorldModel
- Remediation action registry populated from PDDL domain
- Integration with MissionExecutor's replan loop
- Max recursion depth as safety parameter

### 6.3 Medium Impact, Higher Effort — Deeper Integration

#### 6.3.1 ParameterContext → Rich Action Parameterisation

**What**: Port the multi-scope `ParameterContext` and `ComputedParameterEngine`.

**How**: Replace AME's `{param0}`-style string substitution with automata_engine's scoped parameter resolution (action → requirement → global → computed). Computed parameters enable expressions like "distance_to_target * speed_factor" evaluated at planning time.

**Benefit**: More flexible action parameterisation, especially for cost expressions and conditional parameters.

#### 6.3.2 FsmStrategy → Procedural Phase Execution

**What**: Integrate `FsmStrategy` + `FsmPlanExecutor` as an alternative execution mode for deterministic mission phases.

**How**: Register FSM definitions for procedural phases (pre-flight, emergency, docking). The PlanCompiler emits FSM references instead of BT subtrees for these phases.

**Benefit**: Simpler verification for deterministic procedures; no planning overhead.

#### 6.3.3 Validation Framework → Configuration Assurance

**What**: Port the validation/diagnostic subsystem for AME's PDDL domain and problem files.

**How**: Validate PDDL domains at load time: check for unreachable predicates, actions with unsatisfiable preconditions, type mismatches. Currently AME only validates syntax (PddlParser), not semantic correctness.

**Benefit**: Catches configuration errors before planning, reducing runtime failures.

#### 6.3.4 ServiceContainer → AME Dependency Injection

**What**: Adopt `ServiceContainer` for AME's component wiring.

**How**: Replace manual component construction (WorldModel → Planner → PlanCompiler → Executor) with DI container registration. Components resolve dependencies at runtime.

**Benefit**: Easier testing (mock injection), cleaner component lifecycle, simpler configuration.

### 6.4 Reuse Roadmap

```
Phase 1 — Quick Wins (no architectural change)
  ├── 6.1.2 ResourceTracker integration
  ├── 6.1.3 Temporal constraint types for deadline monitoring
  └── 6.3.3 Validation framework

Phase 2 — Cost-Aware Planning (planner enhancement)
  ├── 6.1.1 Cost calculators → LAPKT A* heuristics
  └── 6.2.2 CapabilityMatcher for heterogeneous fleets

Phase 3 — Tactical Replanning (execution enhancement)
  ├── 6.2.1 GOAP as tactical replanner
  └── 6.2.3 GoalResolver for failure recovery

Phase 4 — Architecture Enrichment (deeper integration)
  ├── 6.3.1 ParameterContext
  ├── 6.3.2 FsmStrategy for procedures
  └── 6.3.4 ServiceContainer DI
```

### 6.5 What NOT to Reuse

| Component | Reason to skip |
|-----------|---------------|
| `BtStrategy` | AME's BT.CPP integration is more mature; AE's BT strategy is a simple tree traversal, not a full BT engine |
| `QLearningStrategy` | Requires offline training data that doesn't exist yet; add later when operational data is available |
| `WorldState` (wholesale) | AME's boolean-predicate WorldModel is tightly coupled to PDDL semantics and the observability stack; replacing it would break too much. Instead, adapt specific `DetailBase` types as extensions |
| `EventBus` | AME's callback-based observability is more auditable; AE's EventBus is loosely-typed and harder to assure |

---

# AME ↔ PYRAMID Integration: SACE Assurance for GOAP + AME as Strategy

## 1. SACE Assurance Applicability to GOAP

Ame's existing [SACE assurance plan](../roadmaps/autonomy_assurance_plan.md) is tightly coupled to the **PDDL + LAPKT + BT.CPP** architecture. Below is a stage-by-stage analysis of how each SACE stage — and its evidence arguments — applies (or must adapt) when the planning algorithm is automata_engine's **GOAP A\*** rather than LAPKT BRFS.

### 1.1 Property Comparison: LAPKT BRFS vs GOAP A\*

| Assurable Property | LAPKT BRFS (current ame) | GOAP A\* (automata_engine) | Assurance Impact |
|---|---|---|---|
| **Soundness** | ✅ Guaranteed — STRIPS semantics; preconditions checked per step | ✅ Guaranteed — `IAction.getPreconditions()` checked at each A\* node expansion | Equivalent — both validate preconditions before applying effects |
| **Completeness** | ✅ BRFS is complete for finite STRIPS | ⚠️ A\* is complete **iff** heuristic is admissible and graph is finite; GOAP uses `max_depth` bound | New evidence needed: demonstrate heuristic admissibility or document loss of completeness |
| **Optimality** | ⚠️ BRFS finds shortest plan (step count), not cost-optimal | ✅ A\* with admissible heuristic finds cost-optimal plan vs. configured `ICostCalculator` | Trade-off reversal: GOAP is cost-optimal but may miss step-optimal plans |
| **Termination** | ✅ Finite grounded fluent space → guaranteed termination | ⚠️ Depends on `max_depth` config + state space finiteness | Must document `GoapConfigOptions.max_depth` as a safety parameter |
| **Determinism** | ✅ Same PDDL → same plan (BRFS is deterministic) | ⚠️ Depends on action ordering in `ActionRegistry` and tie-breaking in priority queue | Tie-breaking policy must be deterministic and documented |
| **Transparency** | ✅ 5-layer audit stack (JSONL, Foxglove, BT events) | ⚠️ `IPlanningEventListener` exists but no file/WebSocket sinks by default | Must either port observability layers or bridge `EventBus` → existing sinks |
| **State integrity** | ✅ Boolean bitset with version counter + `AuditCallback` | ⚠️ Typed `WorldState` with `DetailBase` polymorphism; no built-in audit trail | Need to add audit mechanism to `WorldState` mutations |

### 1.2 SACE Stage Mapping

#### Stage 1 — Operating Context

| Aspect | PDDL-specific argument | GOAP equivalent | Gap |
|--------|----------------------|-----------------|-----|
| Domain model | PDDL domain file is the formal specification | JSON `ActionDefinition` + `GoapDefinition` | JSON schema is less formal than PDDL; needs validation specification |
| Capability specification | Derived from PDDL predicates | `AssetCapabilityRegistry` + `Capability` structs | ✅ Richer — automata_engine explicitly models capabilities |
| Scenario catalogue | Based on PDDL problem files | Based on `PlanningRequest` instances | Equivalent — both enumerate scenarios via problem instances |

> **Important:** PDDL has well-established formal semantics (McDermott 1998). GOAP's `ActionDefinition` with `Condition` + `ActionEffect` is functionally equivalent but lacks a published formal semantics. For SACE Stage 1, a formal specification of `ActionDefinition` semantics (precondition satisfaction, effect application, cost monotonicity) is needed.

#### Stage 2 — Hazardous Scenarios

The existing hazard table (H1–H9 in `../roadmaps/autonomy_assurance_plan.md`) maps as follows:

| Hazard | PDDL Argument | GOAP Adaptation |
|--------|---------------|-----------------|
| H1 (Incorrect plan) | STRIPS soundness | A\* soundness + precondition check in `GoapNode` expansion |
| H2 (Incomplete plan) | BRFS completeness | **New hazard**: A\* with depth limit may miss valid plans |
| H3 (Stale world model) | `setFact()` freshness | **Worse**: typed `WorldState` has more state dimensions to go stale |
| H4 (Causal misordering) | Causal graph in `PlanCompiler` | **Not applicable** if GOAP feeds `BasicPlanExecutor` instead of BT |
| H5 (Effect misattribution) | `SetWorldPredicate` nodes | **Equivalent**: `ActionEffect` application via `IAction.getEffects()` |
| H6 (Replan livelock) | `MissionExecutor` counter | **Equivalent**: `GoalResolver` has `max_depth` parameter for recursive replanning |
| H7 (Timing violation) | `max_iterations` in LAPKT | `max_depth` in `GoapConfigOptions` + A\* expansion limit |
| H8 (Action–BT mismatch) | `ActionRegistry.resolve()` | `ActionRegistry` (pyramid) + `IAction.execute()` — tighter coupling, less mismatch risk |
| **H10 (NEW)** | n/a | **Heuristic inadmissibility** — if h(n) overestimates, A\* may return suboptimal plan or miss solutions |
| **H11 (NEW)** | n/a | **Cost calculator error** — `SpatialCostCalculator` uses great-circle distance; errors in position data propagate to cost |
| **H12 (NEW)** | n/a | **Type mismatch in DetailBase** — wrong `DetailBase` subtype cast during `WorldState` evaluation |

#### Stage 3 — Safe Operating Concept

| SOC Rule | PDDL applicability | GOAP adaptation needed |
|----------|--------------------|-----------------------|
| Bounded planning time | `max_iterations` in BRFS | `max_depth` + wall-clock timeout in `GoapStrategy` |
| Precondition gating | `CheckWorldPredicate` BT nodes | `IAction.getPreconditions()` checked during A\* expansion |
| Replan limit | `MissionExecutor` counter | `GoalResolver.max_depth` + executor replan counter |
| Safe-state fallback | BT safe-state sub-tree | Must define equivalent in `BasicPlanExecutor` / `TemporalPlanExecutor` |

#### Stage 4 — Safety Requirements

New/modified safety requirements for GOAP:

| Req ID | Requirement | Traces to |
|--------|-------------|-----------|
| SR-10 | GOAP heuristic SHALL be documented as admissible or the completeness loss SHALL be recorded in the hazard log | H10 |
| SR-11 | `ICostCalculator` implementations SHALL validate input position data before computing costs | H11 |
| SR-12 | `WorldState` mutations SHALL be logged with source identification (equivalent to ame's `AuditCallback`) | H3 |
| SR-13 | `GoapStrategy` SHALL expose planning statistics (nodes expanded, depth reached, time elapsed) for audit | SR-02 equivalent |

#### Stage 5 — Design Assurance

| Design argument (PDDL) | GOAP equivalent | Status |
|------------------------|-----------------|--------|
| Single authoritative WorldModel | Single `WorldState` per `PlanningRequest` | ✅ Equivalent |
| Stateless planner | `GoapStrategy` holds `goap_definition_` but plans are pure functions of request | ⚠️ `planning_world_state_` is set during `determineSolution` — mutable internal state |
| Compiler-generated BT | N/A — GOAP outputs `Solution` directly to executor | ✅ Simpler — no BT compilation step to assure |
| Observability layers 1–5 | `EventBus` + `IPlanningEventListener` | ⚠️ Less mature — needs evidence of audit completeness |

#### Stage 6 — Hazardous Failures

GOAP adds new failure modes:

| Failure | Detection | Response |
|---------|-----------|----------|
| A\* search exhaustion (max depth) | `GoapConfigOptions.max_depth` reached | Return no solution + log; trigger replanning via `GoalResolver` |
| Cost calculator exception | Exception in `calculateCost()` | Fallback to static `IAction.getCost()` |
| `DetailBase` cast failure | `dynamic_cast` returns nullptr | Error logged; action skipped; plan invalidated |

#### Stages 7–8 (Out-of-Context + Verification)

Verification additions needed for GOAP:

| Gap | Test type | What to verify |
|-----|-----------|----------------|
| Heuristic admissibility | Property-based | h(n) ≤ true cost for random world states |
| Cost calculator accuracy | Unit test | `SpatialCostCalculator` vs known geodetic distances |
| A\* depth limit safety | Integration test | Planner returns gracefully at `max_depth` |
| Typed state consistency | Unit test | `DetailBase` round-trip through `WorldState` mutations |
| Determinism | Property-based | Same `PlanningRequest` → same `Solution` across runs |

### 1.3 DSTL Cross-Cut Analysis for GOAP

| DSTL Dimension | PDDL status | GOAP status |
|----------------|-------------|-------------|
| **Requirements** | Traceable via PDDL domain | Traceable via `ActionDefinition` JSON + `Condition` structs |
| **Algorithms** | BRFS formally characterised | A\* well-characterised but heuristic must be specified and assured |
| **Data** | Boolean bitset (simple, verifiable) | Typed `DetailBase` (richer, harder to verify exhaustively) |
| **Integration** | WorldModel → Planner → Compiler → Executor | WorldState → GoapStrategy → Solution → BasicPlanExecutor |
| **Adversarial** | PDDL injection, perception spoofing | Same + JSON config injection, cost manipulation |

---

## 2. AME as an `IPlanningStrategy` within AutomataEngine

### 2.1 Concept

Wrap ame's entire PDDL+LAPKT pipeline as an implementation of `pyramid::automata_engine::IPlanningStrategy`, registered alongside GOAP, FSM, Q-Learning, and BT strategies in `AutomataEngine`.

```
AutomataEngine
  ├-- GoapStrategy      (A* + cost calculators)
  ├-- PddlStrategy      (ame WorldModel + LAPKT BRFS)  ← NEW
  ├-- FsmStrategy
  ├-- BtStrategy
  └-- QLearningStrategy
```

### 2.2 Interface Mapping

`IPlanningStrategy` requires:

```cpp
class IPlanningStrategy {
  virtual std::shared_ptr<Solution> determineSolution(const PlanningRequest& request) = 0;
  virtual bool validateParameterContext(const ParameterContext&) { return true; }
};
```

The adapter (`PddlStrategy`) would:

| Step | What happens | Implementation |
|------|-------------|----------------|
| 1. Extract world state | `PlanningRequest.context` → `ame::WorldModel` | Translate typed `WorldState` → boolean fluents (threshold/equality checks) |
| 2. Extract goals | `PlanningRequest.requirements` → PDDL goal fluents | Map `ExpectedEffect` → `setGoal()` fluent keys |
| 3. Load domain | From PDDL file path or embedded in `strategy_config` JSON | `PddlParser::parseDomain()` + `parseProblem()` |
| 4. Solve | `ame::Planner::solve(wm)` | Returns `PlanResult` with `PlanStep` indices |
| 5. Compile to BT XML | `PlanCompiler::compile()` with causal-graph parallelism | (Optional — can skip if executor uses `Solution` directly) |
| 6. Map to Solution | `PlanResult.steps` → `vector<SolutionElement>` | One `SolutionElement` per `GroundAction` with `ElementType::ACTION` |
| 7. Return | `shared_ptr<Solution>` | Includes `total_estimated_cost` (step count) and constraint validation |

### 2.3 Skeleton Code

```cpp
// pddl_strategy.h — ame as an IPlanningStrategy
#pragma once
#include <interfaces/IPlanningStrategy.h>
#include "ame/world_model.h"
#include "ame/planner.h"
#include "ame/pddl_parser.h"

namespace pyramid { namespace automata_engine {

class PddlStrategy : public IPlanningStrategy {
public:
  PddlStrategy(const std::string& domain_pddl_path,
               const std::string& problem_pddl_path);

  std::shared_ptr<Solution> determineSolution(
      const PlanningRequest& request) override;

private:
  // Translate AutomataEngine WorldState → ame WorldModel fluents
  void syncWorldState(const WorldState& ae_state, ame::WorldModel& wm);

  // Translate AutomataEngine Requirements → ame goal fluents
  void syncGoals(const std::vector<std::shared_ptr<Requirement>>& reqs,
                 ame::WorldModel& wm);

  // Convert ame PlanResult → AutomataEngine Solution
  std::shared_ptr<Solution> convertResult(
      const ame::PlanResult& result,
      const ame::WorldModel& wm,
      const PlanningRequest& request);

  std::string domain_path_;
  std::string problem_path_;
};

}} // namespace pyramid::automata_engine
```

### 2.4 Key Design Decisions

| Decision | Options | Recommendation |
|----------|---------|----------------|
| **State sync direction** | (a) AE WorldState → WM fluents on each `determineSolution()`, (b) Maintain persistent WM | **(a)** — stateless per request, matches ame's design philosophy and aids assurance |
| **BT compilation** | (a) Compile to BT XML and embed in `SolutionElement.details`, (b) Return `SolutionElement` per action, let AE executor handle | **(b)** — more consistent with AE's `BasicPlanExecutor` expectations |
| **Cost reporting** | (a) Report LAPKT step count as cost, (b) Ignore cost (BRFS doesn't optimize cost) | **(a)** — step count is a valid cost metric; document it as uniform-cost |
| **When to select PDDL** | (a) Explicit `preferred_strategy_id`, (b) Auto-select when request has no typed state | **(a)** — explicit selection avoids surprising strategy switches |

### 2.5 Assurance Advantage of Offering Both

Registering `PddlStrategy` alongside `GoapStrategy` in `AutomataEngine` creates a **cross-validation** opportunity:

```
PlanningRequest --► GoapStrategy --► Solution A (cost-optimal)
                └-► PddlStrategy --► Solution B (complete, formally assured)
                                          │
                               Cross-Validate: same goal reached?
                               same actions available? divergence → flag
```

This directly supports **SACE Stage 8 (Verification)** by enabling:

- **Diverse redundancy** — two different algorithms solving the same problem
- **Regression detection** — if GOAP stops finding plans that PDDL can find, the heuristic may be inadmissible
- **Cost-quality trade-off analysis** — compare GOAP's cost-optimal plan vs PDDL's step-optimal plan

---

## 3. Recommended Next Steps

| # | Action | Effort | Prerequisite |
|---|--------|--------|-------------|
| 1 | **Write formal semantics for `ActionDefinition`** — precondition satisfaction rules, effect application order, cost monotonicity | 1 week | None |
| 2 | **Implement `PddlStrategy`** adapter — wrap ame pipeline behind `IPlanningStrategy` | 2 weeks | Build system integration (ame_core as dependency of automata_engine) |
| 3 | **Extend SACE hazard table** with H10–H12 (heuristic inadmissibility, cost calculator error, type mismatch) | 1 week | None |
| 4 | **Add audit trail to `WorldState`** — equivalent to ame's `AuditCallback` for typed state | 1 week | None |
| 5 | **Cross-validation harness** — run both strategies on UAV search domain, compare plan equivalence | 2 weeks | Steps 1–2 |
| 6 | **Update SACE–PDDL integration guide** to cover GOAP as a second planning algorithm | 1 week | Steps 1, 3 |

