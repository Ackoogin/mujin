# AME ↔ PYRAMID Integration: SACE Assurance for GOAP + AME as Strategy

## 1. SACE Assurance Applicability to GOAP

Ame's existing [SACE assurance plan](autonomy_assurance_plan.md) is tightly coupled to the **PDDL + LAPKT + BT.CPP** architecture. Below is a stage-by-stage analysis of how each SACE stage — and its evidence arguments — applies (or must adapt) when the planning algorithm is automata_engine's **GOAP A\*** rather than LAPKT BRFS.

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

The existing hazard table (H1–H9 in `autonomy_assurance_plan.md`) maps as follows:

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
