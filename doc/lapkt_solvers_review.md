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
- Matches AME's existing roadmap item: "Solver portfolio: try fast heuristic first, fall back to complete search" (`doc/plan.md:162`).
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

## References

- Lipovetzky, N. and Geffner, H. (2012). "Width and Serialization of Classical Planning Problems." ECAI 2012.
- Lipovetzky, N. and Geffner, H. (2017). "Best-First Width Search: Exploration and Exploitation in Classical Planning." AAAI 2017.
- LAPKT Documentation: https://lapkt-dev.github.io/docs/
- LAPKT Modules and Planners: https://lapkt-dev.github.io/docs/modules/
- LAPKT GitHub Repository: https://github.com/LAPKT-dev/LAPKT-public
- BFWS Public Repository: https://github.com/nirlipo/BFWS-public
