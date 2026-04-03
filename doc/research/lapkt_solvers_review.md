# LAPKT Solvers Review: Benefits for AME

## Current State

AME uses a single LAPKT solver: **BRFS (Breadth-First Search)** over a forward-chaining STRIPS search space (`planner.cpp:14`). BRFS is uninformed — it expands states in generation order without heuristic guidance. It is complete (guaranteed to find a plan if one exists) and step-optimal (finds the shortest plan in terms of action count), but explores the state space exhaustively. No heuristics, novelty measures, or search budgets are currently configured.

This is adequate for AME's current UAV search-and-classify domain, which is small enough that BRFS terminates quickly. However, as domains grow in object count, predicate arity, or goal complexity, BRFS will encounter scaling limits. The LAPKT toolkit provides several alternative solvers that could address these limits.

---

## Available LAPKT Solvers

### 1. IW(k) — Iterated Width

**What it is:** A blind search algorithm that prunes states based on *novelty*. A state's novelty w(s) is the size of the smallest tuple of atoms that appears for the first time in s. IW(k) only expands states with novelty <= k. Typically k=1 or k=2.

**How it works:** IW(1) runs breadth-first but discards any state whose every individual atom has already been seen. If IW(1) fails, IW(2) retries keeping states with novel pairs, and so on. In practice, most IPC benchmark problems have effective width 1 or 2 for atomic goals.

**Benefit to AME:**
- Dramatically reduces node expansions compared to BRFS for domains with large state spaces but low effective width (which includes most robotics/logistics domains).
- Remains a blind search — no heuristic function to design or tune.
- Particularly well-suited to exploration and navigation sub-problems where the branching factor is high but progress is structurally simple.

**Limitations:**
- Incomplete for goals with high width. Must fall back to a complete solver if IW(k) fails.
- Only handles single atomic goals natively; conjunctive goals require SIW (see below).

**Integration effort:** Low. The IW engine header is available in the LAPKT include path. Instantiation follows the same pattern as BRFS — template on `Fwd_Search_Problem`.

---

### 2. SIW — Serialized Iterated Width

**What it is:** Extends IW to handle conjunctive goals by *serializing* them. SIW picks one unsatisfied goal atom at a time, runs IW to achieve it, then repeats until all goal atoms are satisfied.

**How it works:** SIW maintains the set of goal atoms. At each iteration it selects an unsatisfied goal, runs IW(k) from the current state targeting that goal, applies the resulting partial plan, and continues. The order in which goals are tackled affects efficiency but not correctness (with fallback).

**Benefit to AME:**
- Handles AME's conjunctive goals (e.g., "all areas searched AND all targets classified") efficiently by decomposing them into IW-tractable sub-problems.
- Competitive with heuristic search on IPC benchmarks while remaining heuristic-free.
- Natural fit for AME's replan-on-failure architecture: if one sub-goal fails, SIW can re-serialize from the current state.

**Limitations:**
- Can get stuck if achieving one sub-goal undoes another (delete interactions between serialized goals). In such cases, falls back to BFS(f) or full search.
- Not optimal — plan quality depends on goal serialization order.

**Integration effort:** Low-medium. SIW is available as a LAPKT engine. Requires specifying the IW width bound (typically 2).

---

### 3. BFS(f) — Best-First Search with Novelty

**What it is:** A best-first search that uses a composite evaluation function f combining a heuristic estimate (typically goal count or landmark count) with a novelty measure. States are ranked lexicographically: first by heuristic bucket, then by novelty within each bucket.

**How it works:** BFS(f) maintains an open list sorted by (h, novelty). Within states of equal heuristic value, novel states (those containing previously unseen atom tuples) are expanded first. This balances exploitation (heuristic guidance toward the goal) with exploration (novelty-driven diversification).

**Benefit to AME:**
- Best of both worlds: heuristic-guided search with novelty-based tie-breaking prevents plateaux that trap pure best-first search.
- IPC competition performance: BFS(f) was among the fastest planners in the AGILE track of IPC 2014.
- Can be configured with AME's existing compiled `landmark_graph.cxx` for the heuristic component — no new source files needed.

**Limitations:**
- Requires choosing and instantiating a heuristic (goal count, landmark count, or additive h).
- More complex to configure than BRFS or IW.

**Integration effort:** Medium. Requires instantiating a heuristic evaluator and wiring it into the BFS(f) engine template.

---

### 4. SIW-then-BFS(f) — Two-Phase Portfolio

**What it is:** A solver portfolio that runs SIW first (fast, novelty-based) and falls back to BFS(f) (complete, heuristic-guided) if SIW fails.

**How it works:** Phase 1 runs SIW with a width bound (typically k=2). If SIW solves the problem, its plan is returned immediately. If SIW fails (e.g., due to high-width goals or delete interactions), Phase 2 runs BFS(f) which is complete and will find a solution if one exists.

**Benefit to AME:**
- The most production-appropriate solver strategy. Fast path for easy problems, robust fallback for hard ones.
- Matches AME's existing roadmap item: "Solver portfolio: try fast heuristic first, fall back to complete search" (`doc/roadmaps/plan.md:162`).
- Used in IPC competition planners and the editor.planning.domains web service (handling over 1M calls).

**Limitations:**
- Two-phase approach means configuration of both SIW and BFS(f) parameters.
- SIW phase may waste time on problems that are inherently high-width.

**Integration effort:** Medium. Both SIW and BFS(f) engines are available in LAPKT. Implementation involves running them sequentially with a shared `STRIPS_Problem`.

---

### 5. BFWS — Best-First Width Search

**What it is:** The state-of-the-art novelty-based planner. BFWS uses evaluation functions that combine novelty with goal-relevant information (landmark counts, goal counts, or both). Multiple variants exist: BFWS(f5), DUAL-BFWS, polynomial BFWS.

**How it works:** BFWS(f5) evaluates states using a tuple (#goals-achieved, novelty, hFF-estimate). DUAL-BFWS maintains two open lists — one novelty-based, one heuristic-based — alternating expansion between them. 1-BFWS restricts to novelty-1 states for polynomial complexity guarantees.

**Benefit to AME:**
- Highest coverage of any single-algorithm planner on IPC benchmarks. Won 1st and 2nd place in the Sparkle Planning Challenge.
- DUAL-BFWS provides strong anytime behaviour: finds an initial plan quickly, then improves it.
- Polynomial variants (1-BFWS) provide predictable runtime bounds — valuable for safety-critical autonomous systems where planning latency must be bounded.

**Limitations:**
- Most complex to integrate — requires landmark computation, relevance analysis, and multi-queue management.
- BFWS implementations may exist as standalone planners outside the LAPKT core library; integration may require pulling additional source files.

**Integration effort:** High. Would require compiling additional LAPKT source files and potentially porting code from the BFWS-public repository.

---

### 6. WA* — Weighted A*

**What it is:** A* search with the evaluation function f(s) = g(s) + W * h(s), where W > 1 inflates the heuristic to trade optimality for speed.

**How it works:** With W=1 it is standard A* (optimal but slow). As W increases, search becomes greedier — faster but with bounded sub-optimality (plan cost <= W * optimal cost). Anytime WA* (also called RWA*) decreases W iteratively to improve plan quality over time.

**Benefit to AME:**
- Provides bounded sub-optimal plans with predictable quality guarantees (W-admissible).
- Anytime variant fits AME's execution model: return a fast initial plan, improve it during execution if time permits.
- LAPKT provides WA* engine templates with landmark count heuristic examples.

**Limitations:**
- Requires an admissible heuristic for the sub-optimality bound to hold. The landmark count heuristic (already compiled in `lapkt_core`) is admissible and suitable.
- Performance depends heavily on heuristic quality.

**Integration effort:** Medium. WA* engine is available in LAPKT. Requires instantiating a heuristic (landmark count is already compiled).

---

### 7. DFS+ — Depth-First Search with Novelty Pruning

**What it is:** A depth-first search variant that uses novelty pruning to avoid redundant exploration. Part of the IW family of algorithms.

**How it works:** DFS+ performs depth-first search but prunes branches where all generated states have novelty greater than the bound k. This gives it the memory efficiency of DFS with the pruning power of novelty.

**Benefit to AME:**
- Lower memory footprint than BFS-based approaches — relevant for embedded or resource-constrained deployments.
- Was among the fastest planners in IPC AGILE track alongside BFS(f) and SIW+.

**Limitations:**
- Not optimal. May find longer plans than BFS-based approaches.
- Incomplete without fallback for high-width problems.

**Integration effort:** Low-medium. Similar instantiation pattern to IW/SIW.

---

## Available Heuristics

LAPKT provides several heuristic evaluators that can be paired with the search engines above. AME already compiles `landmark_graph.cxx` but does not use it.

| Heuristic | Description | Admissible | Compiled in AME |
|-----------|-------------|:----------:|:---------------:|
| **Landmark Count (h_LM)** | Counts unsatisfied landmarks (facts that must be true at some point in any plan). | Yes | Yes |
| **h_add (Additive)** | Sum of costs to achieve each goal atom independently (relaxation-based). | No | No |
| **h_max** | Maximum cost to achieve any single goal atom (relaxation-based). | Yes | No |
| **h_FF (FastForward)** | Extracts a relaxed plan from the planning graph; uses its length as estimate. | No | No |
| **Goal Count** | Simply counts unsatisfied goal atoms. Trivial but effective as a secondary heuristic. | No | N/A (trivial) |

**Recommendation:** Start with the landmark count heuristic — it is already compiled into `lapkt_core` and provides admissible estimates suitable for WA* and as a component of BFS(f).

---

## Recommendations for AME

### Short Term: SIW-then-BRFS Portfolio

Replace the single BRFS solver with a two-phase approach:
1. Run SIW(2) first — fast novelty-based search for the common case.
2. Fall back to BRFS if SIW fails — preserves current completeness guarantee.

This requires minimal code changes (instantiate SIW engine, try it before BRFS) and provides immediate performance improvement for larger domains. The `Planner` interface already returns `PlanResult` with timing metrics, so the caller is unaffected.

### Medium Term: BFS(f) with Landmark Heuristic

Replace the BRFS fallback with BFS(f) using the landmark count heuristic:
1. Compute landmarks from the `STRIPS_Problem` (infrastructure already compiled).
2. Instantiate BFS(f) with `(h_LM, novelty)` evaluation.
3. Add a configurable time/node budget to `Planner::solve()`.

This brings AME's planner to IPC-competitive performance while using only already-compiled LAPKT components.

### Long Term: Solver Configuration and BFWS

1. Add a `SolverConfig` parameter to `Planner::solve()` with solver selection, heuristic choice, and time/node budgets.
2. Investigate BFWS integration for domains where coverage and bounded runtime are critical.
3. Consider the `IPlannerBackend` abstraction (already planned for Extension 7) to unify LAPKT solvers and future temporal planners behind a common interface.

---

## Integration Architecture

The current `Planner::solve()` method is stateless and creates the LAPKT engine locally. This pattern extends naturally to a solver portfolio:

```
Planner::solve(wm, config)
  |
  +-- projectToSTRIPS(strips)
  |
  +-- Phase 1: SIW(config.iw_bound)
  |     +-- if success: return plan
  |
  +-- Phase 2: BFS(f) with h_LM + novelty
  |     +-- if success: return plan
  |
  +-- Phase 3: BRFS (complete fallback)
        +-- return plan or failure
```

Each phase shares the same `STRIPS_Problem` and `Fwd_Search_Problem` wrapper. The `PlanResult` struct already captures expanded/generated counts and solve time, providing observability into which phase succeeded.

---

## Summary Table

| Solver | Speed | Completeness | Optimality | Memory | Integration Effort | AME Benefit |
|--------|:-----:|:------------:|:----------:|:------:|:-----------------:|-------------|
| **BRFS** (current) | Slow on large domains | Complete | Step-optimal | High | N/A (in use) | Baseline |
| **IW(k)** | Fast | Incomplete | No | Low | Low | Prune large state spaces |
| **SIW** | Fast | Incomplete* | No | Low | Low-medium | Handle conjunctive goals efficiently |
| **BFS(f)** | Fast | Complete | No | Medium | Medium | Heuristic + novelty guidance |
| **SIW-then-BFS(f)** | Fast + robust | Complete | No | Medium | Medium | Best portfolio for production |
| **BFWS** | Fastest coverage | Complete | No | Medium | High | State-of-the-art performance |
| **WA*** | Configurable | Complete | Bounded | Medium | Medium | Quality guarantees |
| **DFS+** | Fast | Incomplete* | No | Low | Low-medium | Memory-constrained deployments |

*Incomplete solvers require a complete fallback (BRFS or BFS(f)) in the portfolio.

---

## LAPKT Feature Audit

This section audits every LAPKT feature compiled into `lapkt_core` or available via include paths, documenting whether AME actively uses it, compiles it as dead code, or has access to it but does not compile it.

### Audit Summary

| Feature | Category | Compiled | Header Included | Instantiated | Called by AME | Status |
|---------|----------|:--------:|:---------------:|:------------:|:-------------:|--------|
| STRIPS_Problem | Model | Yes | Yes | Yes | Yes | **Active** |
| Action | Model | Yes | Yes | Yes | Yes | **Active** |
| Fluent | Model | Yes | Yes | Yes | Yes | **Active** |
| STRIPS_State | Model | Yes | Yes | Yes | Yes | **Active** |
| Fwd_Search_Problem | Model | Yes | Yes | Yes | Yes | **Active** |
| Successor Generator (succ_gen) | Model | Yes | Yes | Yes (internal) | Yes (internal) | **Active** |
| Conditional Effects (cond_eff) | Model | Yes | No | No | No | **Dead code** |
| Conjunctive Component Problem | Model | Yes | No | No | No | **Dead code** |
| Fluent Conjunction (fl_conj) | Model | Yes | No | No | No | **Dead code** |
| Mutex Sets | Model | Yes | No | No | No | **Dead code** |
| Match Tree | Component | Yes | No | No | No | **Dead code** |
| Reachability Analysis | Component | Yes | No | No | No | **Dead code** |
| Watched Literals Successor Gen | Component | Yes | No | No | No | **Dead code** |
| Landmark Graph Heuristic | Heuristic | Yes | No | No | No | **Dead code** |
| Bit Array / Bit Set | Utility | Yes | Yes (internal) | Yes (internal) | Yes (internal) | **Active** |
| Memory / Resources Control | Utility | Yes | Yes (internal) | Yes (internal) | Yes (internal) | **Active** |
| BRFS Engine | Engine | Header-only | Yes | Yes | Yes | **Active** |
| IW / SIW Engines | Engine | Header-only | No | No | No | **Available** |
| BFS(f) / Best-First Engines | Engine | Header-only | No | No | No | **Available** |
| WA* / RWA* Engines | Engine | Header-only | No | No | No | **Available** |
| BFWS Engines | Engine | Header-only | No | No | No | **Available** |
| Novelty Evaluators | Node Eval | Not compiled | No | No | No | **Available** |
| h_add / h_max / h_FF Heuristics | Node Eval | Not compiled | No | No | No | **Available** |

---

### Active Features (Used by AME)

#### Agnostic Problem Representation (`STRIPS_Problem`, `Fwd_Search_Problem`)

LAPKT's core design principle is the *agnostic interface*: a language-independent problem representation that decouples the planning model from any particular parser or input language. The `STRIPS_Problem` class holds fluents, actions (with preconditions, add effects, delete effects), initial state, and goal specification without any dependency on PDDL syntax.

**How AME uses it:** `WorldModel::projectToSTRIPS()` (`world_model.cpp:438`) constructs a `STRIPS_Problem` programmatically by iterating over AME's internal fluent and ground action registries. This is exactly the intended use of the agnostic interface — AME never passes PDDL text to LAPKT; instead it builds the planning model from its own data structures.

`Fwd_Search_Problem` wraps a `STRIPS_Problem` to provide a forward search model (initial state, goal test, successor generation). All LAPKT search engines are templated on this wrapper.

**What AME gains:** Clean separation between AME's world model and LAPKT internals. The PDDL parser populates the WorldModel, the WorldModel projects to STRIPS, and the planner operates on STRIPS. Each layer can be replaced independently.

**Unused potential:** The agnostic interface also supports SAS+ representations (finite-domain variables instead of Boolean fluents). This could be beneficial if AME moves to more expressive state representations (e.g., numeric fluents for fuel levels or battery charge).

#### Action Representation (`action.cxx`)

LAPKT's `Action` class stores preconditions, add effects, and delete effects as sorted vectors of fluent indices. The static method `STRIPS_Problem::add_action()` creates and registers actions.

**How AME uses it:** `projectToSTRIPS()` calls `STRIPS_Problem::add_action()` for each ground action, passing precondition, add, and delete vectors built from the WorldModel's `GroundAction` struct. After all actions are added, `make_action_tables()` builds internal lookup structures for successor generation.

**What AME gains:** Efficient action applicability testing via LAPKT's internal tables. The `make_action_tables()` call builds match trees and applicability indices that are used by the successor generator during search.

#### State Representation (`strips_state.cxx`)

LAPKT `State` objects are bitset-backed representations of the current set of true fluents, with a hash for open-list membership testing.

**How AME uses it:** `WorldModel::currentStateAsSTRIPS()` (`world_model.cpp:471`) creates a `State` object, sets bits for all currently-true fluents, and calls `update_hash()`. This state is passed to the search engine as the initial state.

**Note:** The state is heap-allocated via raw `new` and ownership is transferred to the engine. This is a LAPKT convention; the engine manages the lifetime.

#### Successor Generator (`succ_gen.cxx`)

The successor generator computes the set of applicable actions for a given state. It is called internally by LAPKT search engines during node expansion.

**How AME uses it:** Implicitly, via `Fwd_Search_Problem`. When the BRFS engine expands a state, it calls the successor generator to enumerate applicable actions. AME does not call the successor generator directly.

**What AME gains:** Efficient applicability testing — the successor generator uses the action tables built by `make_action_tables()` to avoid testing every action against every state.

#### Utility Libraries (`bit_array.cxx`, `bit_set.cxx`, `memory.cxx`, `resources_control.cxx`)

Low-level infrastructure used internally by all LAPKT components. Bit arrays back the state representation; memory utilities manage search node allocation; resources control tracks time and memory usage.

**How AME uses it:** Implicitly, through all LAPKT operations. These are foundational and cannot be removed.

---

### Dead Code (Compiled but Unused)

These features are compiled into the `lapkt_core` static library because the CMakeLists.txt includes them, but no AME code instantiates or calls them. They add a small amount to compile time and binary size but have no runtime cost.

#### Conditional Effects (`cond_eff.cxx`)

**What it provides:** Support for actions whose effects depend on conditions beyond the preconditions. In PDDL, these appear as `(when (condition) (effect))` clauses within an action.

**How AME uses it:** Not at all. `projectToSTRIPS()` passes an empty `Conditional_Effect_Vec` for every action. All AME actions are simple STRIPS — fixed preconditions, fixed effects.

**Potential benefit:** Conditional effects would allow more compact action representations. For example, a `search(robot, sector)` action could have a conditional effect: "if a target is present in the sector, then `target_found` becomes true." Currently, this must be modelled as separate actions or handled at the BT execution level. Enabling conditional effects would require:
1. Extending `WorldModel::GroundAction` to store conditional effects.
2. Populating them during PDDL parsing (requires PDDL `:conditional-effects` requirement).
3. Passing them to `STRIPS_Problem::add_action()` instead of the empty vector.

#### Conjunctive Component Problem (`conj_comp_prob.cxx`)

**What it provides:** Decomposes a planning problem into conjunctive sub-problems — one per goal atom or per connected component of the causal graph. Used internally by some LAPKT solvers for goal serialization.

**How AME uses it:** Not at all. Could be useful if AME implements goal decomposition at the planner level (currently handled by the PlanCompiler's causal graph analysis).

**Potential benefit:** Automated goal serialization for SIW. Rather than SIW picking goals arbitrarily, the conjunctive component analysis could provide an informed ordering based on causal dependencies.

#### Fluent Conjunction (`fl_conj.cxx`)

**What it provides:** Represents conjunctions of fluents as first-class objects. Used by landmark-based heuristics and by some novelty computations that track tuples of atoms.

**How AME uses it:** Not at all. Would become relevant if AME enables the landmark heuristic or novelty-based search.

**Potential benefit:** Required infrastructure for landmark computation and for novelty measures beyond novelty-1. If AME moves to BFS(f) or BFWS, fluent conjunctions will be needed.

#### Mutex Sets (`mutex_set.cxx`)

**What it provides:** Computes and stores sets of mutually exclusive fluents — pairs of atoms that cannot both be true in any reachable state. Derived from the planning graph.

**How AME uses it:** Not at all. The BRFS engine does not use mutex pruning.

**Potential benefit:** Mutex information can be used to:
1. **Prune the search space** — discard states containing mutex pairs.
2. **Improve heuristic estimates** — h_max and landmark heuristics can use mutex information for tighter bounds.
3. **Validate domain models** — detect inconsistencies in PDDL domain definitions (e.g., actions that claim to add two mutually exclusive fluents simultaneously).

Integration would require calling the mutex computation API after `make_action_tables()` and passing mutex sets to heuristic constructors.

#### Match Tree (`match_tree.cxx`)

**What it provides:** A decision-tree data structure for efficient action matching. Given a state, the match tree quickly identifies which actions are applicable by traversing a pre-built tree based on precondition structure, avoiding the need to check every action.

**How AME uses it:** Not directly, but `make_action_tables()` may build match trees internally as part of successor generation setup. The match tree is an optimisation that benefits larger domains with many ground actions.

**Potential benefit:** Already providing value transparently through the successor generator. For domains with hundreds or thousands of ground actions (e.g., multi-robot logistics with many locations), match trees significantly reduce per-node expansion cost.

#### Reachability Analysis (`reachability.cxx`)

**What it provides:** Forward reachability analysis from the initial state using relaxed planning graph expansion. Identifies which fluents and actions are reachable (potentially achievable) and which are unreachable (provably dead).

**How AME uses it:** Not at all.

**Potential benefit:**
1. **Domain validation** — detect unreachable goals before starting search, providing immediate failure feedback instead of exhaustive exploration.
2. **Problem simplification** — remove unreachable fluents and inapplicable actions from the STRIPS problem, reducing the effective state space.
3. **Heuristic computation** — reachability analysis is the foundation of relaxation-based heuristics (h_add, h_max, h_FF). Enabling reachability is a prerequisite for these heuristics.
4. **Safety assurance** — provably demonstrate that certain dangerous states are unreachable, supporting the SACE/AMLAS safety case (`doc/roadmaps/autonomy_assurance_plan.md`).

#### Watched Literals Successor Generator (`watched_lit_succ_gen.cxx`)

**What it provides:** An alternative successor generation strategy inspired by SAT solver watched literal propagation. Instead of checking all preconditions of every action, it "watches" specific precondition fluents and only re-evaluates an action when a watched fluent changes.

**How AME uses it:** Not at all. The default successor generator is used.

**Potential benefit:** More efficient successor generation for domains with long action precondition lists. The watched literal approach has sub-linear amortised cost per state expansion compared to the basic successor generator's linear cost. Most beneficial for domains with many actions and multi-atom preconditions.

#### Landmark Graph Heuristic (`landmark_graph.cxx`)

**What it provides:** Computes a *landmark graph* — a directed graph of facts and actions that must appear in every valid plan. The landmark count heuristic h_LM counts unsatisfied landmarks as an estimate of remaining plan cost. It is admissible (never overestimates).

**How AME uses it:** Compiled but never instantiated. This is the most immediately useful piece of dead code in the build.

**Potential benefit:**
1. **Heuristic search** — pair with WA* for bounded sub-optimal plans, or with BFS(f) for novelty-guided heuristic search.
2. **Plan quality estimation** — landmark count provides a lower bound on remaining actions, useful for progress reporting and timeout decisions.
3. **Goal ordering** — the landmark graph reveals causal ordering between sub-goals, which could inform SIW's goal serialization or the PlanCompiler's flow decomposition.

---

### Available but Not Compiled

These features exist in the LAPKT source tree (headers are on the include path) but no `.cxx` implementation files are compiled into `lapkt_core`.

#### Novelty Evaluators (`node_eval/novelty/`)

**What they provide:** Functions that compute the novelty w(s) of a state — the size of the smallest tuple of atoms appearing for the first time in s. Used by IW, SIW, BFS(f), and BFWS.

**Status:** Header directory is on the include path (`${LAPKT_SRC}/node_eval/novelty`) but no source files are compiled. Novelty evaluators may be header-only templates (common in LAPKT) or may require compilation — this needs verification at build time.

**Required for:** IW, SIW, BFS(f), BFWS, DFS+, and any novelty-based search.

#### Relaxation-Based Heuristics (`node_eval/heuristic/`)

**What they provide:** h_add (additive), h_max (max-cost), and h_FF (FastForward relaxed plan) heuristics. These estimate the cost-to-go by solving a relaxed version of the problem that ignores delete effects.

**Status:** The heuristic include directory is available but only `landmark_graph.cxx` is compiled. Other heuristic source files (if they exist as `.cxx` rather than header-only templates) are not compiled.

**Required for:** A*, WA*, greedy best-first search, and as secondary heuristics in BFS(f) and BFWS.

#### Search Engines (`engine/`)

**What they provide:** All LAPKT search engine implementations — BRFS, IW, SIW, BFS(f), AT_BFS_f, BFWS, WA*, RWA*, DFS+, and others. LAPKT search engines are C++ template classes, so they are header-only and require no additional compilation.

**Status:** The engine include directory is on the path. Only `brfs.hxx` is currently included by AME code. All other engines are available for immediate use by adding the appropriate `#include` directive.

**Integration pattern:** All engines follow the same template:
```cpp
#include <engine_header.hxx>
using Engine = aptk::search::EngineType<aptk::agnostic::Fwd_Search_Problem>;
Engine engine(fwd_prob);
engine.start(init_state);
engine.find_solution(cost, plan);
```

---

### LAPKT API Usage Inventory

AME's LAPKT integration is concentrated in exactly two source files:

**`world_model.cpp`** — 12 LAPKT API calls in `projectToSTRIPS()` and `currentStateAsSTRIPS()`:
- `STRIPS_Problem::add_fluent()` — register fluent names
- `STRIPS_Problem::add_action()` — register ground actions with pre/add/del
- `STRIPS_Problem::set_init()` — set initial state fluents
- `STRIPS_Problem::set_goal()` — set goal fluents
- `STRIPS_Problem::make_action_tables()` — build internal lookup structures
- `State(prob)` constructor, `State::set()`, `State::update_hash()` — create initial state

**`planner.cpp`** — 8 LAPKT API calls in `Planner::solve()`:
- `Fwd_Search_Problem(&strips)` — wrap STRIPS problem for forward search
- `BRFS engine(fwd_prob)` — instantiate search engine
- `engine.set_verbose(false)` — suppress LAPKT console output
- `engine.start(init)` — set initial state for search
- `engine.find_solution(cost, plan)` — run search
- `engine.expanded()`, `engine.generated()` — retrieve search statistics

**No other AME files reference LAPKT APIs.** The integration boundary is clean and narrow, making it straightforward to add new solvers or features without affecting the rest of the codebase.

---

### MSVC Compatibility Layer

LAPKT was developed for Linux/GCC and uses POSIX headers that are unavailable on Windows. AME provides shim headers in `cmake/compat/` that are injected before LAPKT's own include paths when building with MSVC:

| POSIX Header | Shim Behaviour |
|-------------|----------------|
| `sys/time.h` | Redirects to Windows time APIs |
| `unistd.h` | Redirects to `<io.h>` and provides minimal POSIX stubs |
| `sys/resource.h` | Provides minimal stub for resource tracking |

These shims are transparent to AME code and only affect the `lapkt_core` build target. Adding new LAPKT source files may require extending the shims if they use additional POSIX headers.

---

### Build Optimisation Opportunities

**Current state:** 17 LAPKT source files compiled; 7 are dead code (41% of LAPKT compilation).

**Files safe to remove from `lapkt_core` if only BRFS is needed:**
- `cond_eff.cxx` — empty conditional effects vector is the only reference
- `conj_comp_prob.cxx` — never instantiated
- `fl_conj.cxx` — never instantiated
- `mutex_set.cxx` — never instantiated

**Files to keep even though not directly called:**
- `match_tree.cxx` — may be used internally by `make_action_tables()`
- `reachability.cxx` — may be used internally by action table construction
- `watched_lit_succ_gen.cxx` — may be used as an alternate successor generator internally
- `landmark_graph.cxx` — recommended for near-term heuristic integration

**Recommendation:** Keep all files compiled. The build cost is negligible, and they will be needed as AME adopts more LAPKT features. Removing them risks build failures when new solvers are added.

---

## References

- Lipovetzky, N. and Geffner, H. (2012). "Width and Serialization of Classical Planning Problems." ECAI 2012.
- Lipovetzky, N. and Geffner, H. (2017). "Best-First Width Search: Exploration and Exploitation in Classical Planning." AAAI 2017.
- LAPKT Documentation: https://lapkt-dev.github.io/docs/
- LAPKT Modules and Planners: https://lapkt-dev.github.io/docs/modules/
- LAPKT GitHub Repository: https://github.com/LAPKT-dev/LAPKT-public
- BFWS Public Repository: https://github.com/nirlipo/BFWS-public
