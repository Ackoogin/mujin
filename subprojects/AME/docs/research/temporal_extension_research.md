# Temporal Extension Research: Planner Options and Integration Strategy

**Purpose:** Evaluate temporal planning backends, BT execution strategies, and supporting infrastructure for Extension 7 (PDDL 2.1 durative actions with STN conversion).

**Status:** Research complete. No implementation started.

---

## 1. Current Baseline

AME uses LAPKT BRFS (Breadth-First Search) over eagerly-grounded STRIPS problems. Actions are instantaneous and sequential. The PlanCompiler produces linear `BT::Sequence` trees. LAPKT does **not** natively support temporal planning or durative actions.

---

## 2. Temporal Planner Candidates

### 2.1 OPTIC

| Attribute | Detail |
|-----------|--------|
| **Description** | Forward-chaining temporal planner with PDDL 3 preference support and continuous numeric change |
| **PDDL Support** | PDDL 2.1 durative actions (full start/end semantics, self-overlapping), PDDL 3.0 trajectory constraints, numeric fluents, continuous linear effects |
| **Language** | C++ |
| **Licence** | Freely available (academic). `optic-clp` variant has CBC licence restrictions incompatible with GPL |
| **Origin** | King's College London (Coles & Coles) |
| **Maintenance** | Stable but not actively developed; mirrors on GitHub (KavrakiLab/optic, dbanda/optic) |
| **Strengths** | Best combination of temporal + trajectory constraints; well-tested in IPC competitions; C++ native (easier integration); handles continuous linear effects |
| **Weaknesses** | Academic licence may restrict commercial use; limited to linear continuous effects; `optic-clp` variant has reduced functionality (no preferences, unreliable TIL abstraction); no active development |
| **Integration** | **Medium.** C++ subprocess or static library link. Parse temporal plan output (timed action list) into STN. Needs CBC or CPLEX LP solver dependency |

### 2.2 POPF / POPF2

| Attribute | Detail |
|-----------|--------|
| **Description** | Forward-chaining partial-order temporal planner; introduces only necessary ordering constraints |
| **PDDL Support** | PDDL 2.1 durative actions, linear continuous numeric effects |
| **Language** | C++ |
| **Licence** | Freely available (academic, KCL) |
| **Origin** | King's College London; built on COLIN |
| **Maintenance** | Stable; community fork (fmrico/popf) with modern compiler fixes |
| **Strengths** | Partial-order approach means fewer unnecessary sequencing constraints (better concurrency); anytime search (WA*); well-understood semantics |
| **Weaknesses** | Does not support negative preconditions; no PDDL 3.0 preferences; requires CBC solver; older codebase |
| **Integration** | **Medium.** Same approach as OPTIC (subprocess + parse). Simpler than OPTIC but fewer features. Good stepping-stone planner |

### 2.3 COLIN

| Attribute | Detail |
|-----------|--------|
| **Description** | Forward-chaining heuristic search planner for COntinuous LINear numeric change |
| **PDDL Support** | Full PDDL 2.1 temporal semantics, continuous linear effects, duration-dependent effects with duration inequalities |
| **Language** | C++ |
| **Licence** | Freely available (academic, KCL) |
| **Origin** | King's College London; ancestor of POPF and OPTIC |
| **Maintenance** | Stable; superseded by POPF/OPTIC for most use cases |
| **Strengths** | Tightly coupled temporal-numeric reasoning via LP; handles duration-dependent effects; well-documented theory |
| **Weaknesses** | Superseded by POPF/OPTIC; less efficient partial-order handling than POPF |
| **Integration** | **Medium.** Same COLIN/POPF/OPTIC family — similar integration path |

### 2.4 Temporal Fast Downward (TFD)

| Attribute | Detail |
|-----------|--------|
| **Description** | Temporal extension of Fast Downward using context-enhanced additive heuristic |
| **PDDL Support** | PDDL 2.1 durative actions, numeric features |
| **Language** | C++ / Python (translate layer) |
| **Licence** | GPL |
| **Origin** | Albert-Ludwigs-Universität Freiburg (Eyerich, Mattmüller, Röger) |
| **Maintenance** | Stable; GitHub mirrors exist. Standard Fast Downward does NOT support temporal — TFD is separate |
| **Strengths** | Anytime planner (progressively improves plans); multi-valued variable encoding for efficient search; well-proven in IPC 2008; PlanSys2 integration available |
| **Weaknesses** | GPL licence; Python translate step adds complexity; separate codebase from standard Fast Downward; not as actively maintained as standard FD |
| **Integration** | **Medium-High.** Subprocess invocation (like FD). Requires Python for SAS+ translation. Parse numbered plan outputs. PlanSys2 package shows ROS2 integration path |

### 2.5 Aries

| Attribute | Detail |
|-----------|--------|
| **Description** | Modern plan-space planner targeting hierarchical and temporal problems via constraint satisfaction |
| **PDDL Support** | Classical, numeric (integers), temporal (durative actions, intermediate conditions/effects, timed initial literals, timed goals), hierarchical (HDDL) |
| **Language** | Rust |
| **Licence** | Open source (plaans/aries on GitHub) |
| **Origin** | LAAS-CNRS (Arthur Bit-Monnot) |
| **Maintenance** | **Actively developed** (IPC 2023 participant); unified-planning Python plugin |
| **Strengths** | Modern codebase; built-in Difference Logic engine (essentially an STN solver); combined temporal + hierarchical; native handling of optional variables; clause learning from SAT solvers; constraint satisfaction approach naturally handles resource conflicts |
| **Weaknesses** | Rust — requires FFI bridge for C++ integration; plan-space search means no optimality proof or unsolvability proof for action-based planning; younger project with smaller community; integer-only numeric support |
| **Integration** | **Medium-High.** Rust binary invoked as subprocess. Built-in STN solver is architecturally appealing — could potentially share STN representation. Alternatively, use the unified-planning Python interface |

### 2.6 SGPlan6

| Attribute | Detail |
|-----------|--------|
| **Description** | Partition-and-resolve planner that decomposes problems by subgoals |
| **PDDL Support** | PDDL 2.1 temporal + PDDL 3.0 (partial), derived predicates |
| **Language** | C |
| **Licence** | Open source |
| **Maintenance** | Minimal; IPC-era code |
| **Strengths** | Scalable for large problems via subgoal partitioning; solved all crew-planning problems in IPC 2008 |
| **Weaknesses** | Older C codebase; limited maintenance; partition approach may not suit tightly coupled temporal constraints |
| **Integration** | **Medium.** Subprocess. Older build system may need adaptation |

### 2.7 LPG (Local Search for Planning Graphs)

| Attribute | Detail |
|-----------|--------|
| **Description** | Stochastic local search planner using planning graphs with temporal extensions |
| **PDDL Support** | PDDL 2.1 durative actions, numeric fluents |
| **Language** | C |
| **Licence** | Freely available (academic) |
| **Maintenance** | Stable but not actively developed |
| **Strengths** | Fast for satisficing temporal plans; local search handles large domains well |
| **Weaknesses** | Stochastic — non-deterministic results; older codebase; less predictable than systematic search |
| **Integration** | **Medium.** Subprocess |

---

## 3. Planner Comparison Summary

| Planner | PDDL 2.1 | Numeric | Continuous | PDDL 3.0 | Language | Licence | Active | Integration |
|---------|-----------|---------|------------|-----------|----------|---------|--------|-------------|
| **OPTIC** | Full | Yes | Linear | Yes | C++ | Academic | Low | Medium |
| **POPF** | Full | Yes | Linear | No | C++ | Academic | Low | Medium |
| **COLIN** | Full | Yes | Linear | No | C++ | Academic | Low | Medium |
| **TFD** | Full | Yes | No | No | C++/Py | GPL | Low | Medium-High |
| **Aries** | Full | Integer | No | No | Rust | Open | **High** | Medium-High |
| **SGPlan6** | Full | Yes | No | Partial | C | Open | Low | Medium |
| **LPG** | Full | Yes | No | No | C | Academic | Low | Medium |

---

## 4. Recommendation: Planner Selection Strategy

### Primary: OPTIC (near-term) + Aries (medium-term)

**Phase 1 — OPTIC as subprocess (near-term):**
- Lowest integration friction: C++, well-documented output format, proven on temporal domains
- Invoke via `std::system()` or `popen()`, parse timed plan output
- Sufficient for PDDL 2.1 durative actions + trajectory constraints
- No changes to LAPKT — keep LAPKT for STRIPS fallback/comparison

**Phase 2 — Aries evaluation (medium-term):**
- Actively maintained, modern architecture with built-in STN solver
- Supports both hierarchical and temporal planning (aligns with existing Extension 6)
- Evaluate once Extension 7 foundations (parser, WorldModel numeric support) are stable
- Rust subprocess invocation or unified-planning Python bridge

**Fallback: TFD**
- Well-proven alternative if OPTIC licence is problematic
- PlanSys2 integration demonstrates ROS2 compatibility
- GPL licence is more permissive than academic-only

### LAPKT Retention
LAPKT remains the execution planner for STRIPS domains. The temporal planner is invoked when the domain contains `:durative-actions`. This preserves the existing fast STRIPS path and allows incremental adoption.

---

## 5. Simple Temporal Network (STN) Integration

### What is an STN?

An STN is a weighted directed graph where:
- **Nodes** represent time-points (action start, action end, plan start)
- **Edges** represent temporal difference constraints: `Y - X <= w` (edge from X to Y with weight w)
- **Consistency** is checked by detecting negative cycles (Bellman-Ford or Floyd-Warshall)
- **Dispatchability** computes minimal time windows for executing each time-point

### STN Role in AME

The temporal planner produces a partially-ordered plan with time annotations. The PlanCompiler converts this into an STN:

```
Temporal Plan Output          STN Representation           BT Structure
---------------------    →    ------------------    →    --------------
0.000: (move uav1 a b) [5]   t0 --[0,0]-- t_start     Parallel {
5.000: (scan uav1 b) [3]     t_start --[5]-- t1          Sequence {
0.000: (move uav2 c d) [4]   t1 --[3]-- t2                 move(uav1,a,b)  // 5s
                              t_start --[4]-- t3            scan(uav1,b)    // 3s
                              ...                         }
                                                          move(uav2,c,d)    // 4s (concurrent)
                                                        }
```

### Implementation Approach

```cpp
struct TimePoint {
    int id;
    std::string label;        // e.g., "move_uav1_start", "move_uav1_end"
    int action_index;         // index into plan action list
    enum Kind { START, END }; // start or end of durative action
};

struct TemporalConstraint {
    int from, to;             // time-point indices
    double lower_bound;       // minimum delay
    double upper_bound;       // maximum delay (INF if unconstrained)
};

class SimpleTemporalNetwork {
    std::vector<TimePoint> time_points_;
    std::vector<TemporalConstraint> constraints_;
public:
    bool isConsistent() const;              // Bellman-Ford negative cycle check
    std::vector<double> computeSchedule();  // earliest start times via SSSP
    std::vector<std::vector<int>> topologicalLayers(); // concurrent action groups
};
```

The STN implementation is straightforward (~200-300 LOC). Floyd-Warshall for the all-pairs shortest path is adequate for plan-sized graphs (typically <100 nodes). For larger problems, incremental Bellman-Ford is preferred.

---

## 6. BehaviorTree.CPP Temporal Execution

### Relevant BT.CPP Primitives

| Node Type | Purpose for Temporal Execution |
|-----------|-------------------------------|
| `ParallelNode` | Execute concurrent actions; `success_threshold` / `failure_threshold` control synchronisation |
| `StatefulActionNode` | Non-blocking actions returning `RUNNING`; supports `onStart()`, `onRunning()`, `onHalted()` |
| `TimeoutNode` (decorator) | Enforce maximum duration on child node — maps directly to STN upper bounds |
| `DelayNode` (decorator) | Wait before starting child — maps to STN lower bounds |
| `ReactiveSequence` | Re-evaluates conditions each tick — useful for temporal invariant monitoring |

### Compilation Strategy: STN → BT

The PlanCompiler's temporal extension converts STN layers into BT structure:

1. **Topological sort** the STN to identify concurrent action groups (actions with no ordering constraint between them)
2. **Wrap each durative action** in a `StatefulActionNode` that returns `RUNNING` for the action's duration
3. **Group concurrent actions** under `ParallelNode` with `success_threshold = N` (all must succeed)
4. **Apply deadline decorators**: `TimeoutNode` with the STN upper-bound duration
5. **Apply delay decorators**: `DelayNode` for actions that must start after a minimum offset
6. **Chain sequential groups** in a `Sequence` node

```xml
<!-- Example: Two concurrent moves followed by a scan -->
<Sequence>
  <Parallel success_threshold="2">
    <Timeout msec="5500">
      <MoveAction uav="uav1" from="a" to="b"/>  <!-- 5s duration -->
    </Timeout>
    <Timeout msec="4500">
      <MoveAction uav="uav2" from="c" to="d"/>  <!-- 4s duration -->
    </Timeout>
  </Parallel>
  <Timeout msec="3500">
    <ScanAction uav="uav1" sector="b"/>          <!-- 3s, starts after move completes -->
  </Timeout>
</Sequence>
```

### Temporal Invariant Monitoring

PDDL 2.1 `over all` conditions (invariants that must hold throughout a durative action) can be monitored using `ReactiveSequence`:

```xml
<ReactiveSequence>
  <CheckCondition fact="(battery-ok uav1)"/>   <!-- re-checked every tick -->
  <MoveAction uav="uav1" from="a" to="b"/>    <!-- halted if condition fails -->
</ReactiveSequence>
```

If the invariant fails mid-execution, the `ReactiveSequence` halts the action and returns `FAILURE`, triggering replanning.

---

## 7. Plan Validation: VAL

**VAL** (KCL-Planning/VAL on GitHub) is the standard plan validator for PDDL, used in IPC competitions.

| Feature | Support |
|---------|---------|
| PDDL 2.1 durative actions | Full (discretized and continuous) |
| Continuous effects | Linear and polynomial |
| Derived predicates | Yes (PDDL 2.2) |
| Timed initial literals | Yes |
| Temporal mutex checking | Yes (concurrent activity non-interference) |
| Plan repair advice | Yes |
| Licence | BSD |
| Language | C++ |

**Integration:** Run VAL as a post-planning validation step. The temporal planner produces a timed plan; VAL validates it against the PDDL 2.1 domain before the PlanCompiler converts it to a BT. This catches temporal constraint violations, mutex conflicts, and invariant breaches before execution.

```
Temporal Planner → timed plan → VAL validation → PlanCompiler → BT
                                     ↓ (if invalid)
                                  replan with adjusted constraints
```

---

## 8. Required Infrastructure Changes

### 8.1 PDDL 2.1 Parser Extensions

The existing `PddlParser` must be extended to handle:

| Feature | PDDL Syntax | Priority |
|---------|-------------|----------|
| Durative actions | `:durative-action` with `:duration`, `:condition`, `:effect` | **Required** |
| Temporal conditions | `(at start ...)`, `(at end ...)`, `(over all ...)` | **Required** |
| Numeric fluents | `(:functions ...)`, `(increase ...)`, `(decrease ...)`, `(assign ...)` | **Required** |
| Duration constraints | `(= ?duration 5)`, `(<= ?duration 10)` | **Required** |
| Timed initial literals | `(at 10 (signal-green crossing1))` | Nice-to-have |
| Continuous effects | `(increase (fuel) (* #t -0.5))` | Future |

### 8.2 WorldModel Numeric State

The current `WorldModel` uses a bitset for Boolean facts. Numeric fluents require:

- A typed value store alongside the bitset (e.g., `std::unordered_map<std::string, double>`)
- Numeric projection for the planner (`projectToSTRIPS()` → `projectToNumericState()`)
- Audit callbacks for numeric changes (Layer 2 integration)

### 8.3 PlanCompiler Temporal Mode

The PlanCompiler needs a second compilation path:

- **STRIPS path (existing):** Linear plan → `Sequence` of action nodes
- **Temporal path (new):** Timed plan → STN → topological layers → `Parallel`/`Sequence` with deadline decorators

The temporal path activates when the plan contains duration annotations.

### 8.4 Planner Abstraction

Introduce a `PlannerBackend` interface to support multiple solvers:

```cpp
class IPlannerBackend {
public:
    virtual ~IPlannerBackend() = default;
    virtual PlanResult solve(const PlanningProblem& problem) = 0;
    virtual bool supportsTemporalPlanning() const = 0;
};

class LapktBackend : public IPlannerBackend { /* existing BRFS */ };
class OpticBackend : public IPlannerBackend { /* subprocess invocation */ };
```

---

## 9. Implementation Roadmap

| Phase | Work Item | Depends On | Effort |
|-------|-----------|------------|--------|
| **7a** | PDDL 2.1 parser extensions (durative actions, numeric fluents) | — | Medium |
| **7b** | WorldModel numeric fluent store + audit integration | — | Medium |
| **7c** | `IPlannerBackend` abstraction + OPTIC subprocess backend | 7a | Medium |
| **7d** | STN data structure + consistency check + scheduling | — | Low |
| **7e** | PlanCompiler temporal mode (STN → BT with Parallel/Timeout) | 7a, 7c, 7d | High |
| **7f** | VAL integration for post-planning validation | 7c | Low |
| **7g** | Temporal invariant monitoring (`ReactiveSequence` pattern) | 7e | Low |
| **7h** | End-to-end temporal planning tests | All above | Medium |
| **7i** | Aries evaluation + potential migration | 7a-7h stable | Medium |

Phases 7a, 7b, and 7d can proceed in parallel. Phase 7e is the critical-path item.

---

## 10. Risk Assessment

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| OPTIC licence incompatible with deployment context | Must switch to TFD or Aries | Medium | Evaluate licence early; TFD (GPL) and Aries (open) as fallbacks |
| Temporal plan output parsing fragile across planner versions | Broken integration | Low | VAL validation as gatekeeper; integration tests against known plan outputs |
| STN scheduling produces invalid BT structure | Runtime failures | Medium | VAL post-validation; STN consistency check before compilation; round-trip tests |
| BT.CPP single-threaded ticking limits temporal fidelity | Drift from scheduled times | Low | Use wall-clock tracking in `StatefulActionNode`; TimeoutNode enforces hard deadlines |
| Numeric fluent support increases WorldModel complexity | Regression risk in existing STRIPS path | Medium | Feature-flag numeric support; keep bitset path unchanged for Boolean-only domains |
| LAPKT ↔ temporal planner result format mismatch | Two parsing paths to maintain | Low | `IPlannerBackend` abstraction normalises output format |

---

## References

- [OPTIC — Planning Wiki](https://planning.wiki/ref/planners/optic)
- [POPF — Planning Wiki](https://planning.wiki/ref/planners/popf)
- [COLIN — Planning with Continuous Linear Numeric Change (arXiv)](https://arxiv.org/abs/1401.5857)
- [Temporal Fast Downward (GitHub)](https://github.com/neighthan/tfd)
- [Aries — Automated Planning Toolbox (GitHub)](https://github.com/plaans/aries)
- [SGPlan6 (GitHub)](https://github.com/felisd/sgplan6-pddl-planner)
- [VAL Plan Validator (GitHub)](https://github.com/KCL-Planning/VAL)
- [BehaviorTree.CPP Asynchronous Nodes](https://www.behaviortree.dev/docs/3.8/tutorial-advanced/asynchronous_nodes/)
- [BehaviorTree.CPP Parallel Node](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/src/controls/parallel_node.cpp)
- [PDDL 2.1 Specification (JAIR)](https://jair.org/index.php/jair/article/view/10352)
- [Temporal Planning — Planning Wiki](https://planning.wiki/ref/planners/tags/temporal)
- [Temporal Planning Algorithms (AIG-UPF)](https://github.com/aig-upf/temporal-planning)
- [Concurrent BTs for Parallel Execution (ResearchGate)](https://www.researchgate.net/publication/327644052_Improving_the_Parallel_Execution_of_Behavior_Trees)

