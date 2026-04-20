# Extensions Roadmap (Future + Cross-Cutting)

This document now focuses on **future extensions and roadmap-level guidance**.

Completed extension material has been consolidated into the main architecture documents:

- Perception integration + thread safety → [02-world-model.md](02-world-model.md)
- Runtime extension nodes (`InvokeService`, `ExecutePhaseAction`, `DelegateToAgent`) → [04-execution.md](04-execution.md)
- ROS2 wiring for extension nodes → [06-ros2.md](06-ros2.md)

---

## Temporal Planning (Extension 7) — Future

PDDL 2.1 durative actions with STN (Simple Temporal Network) conversion. See [`temporal_extension_research.md`](../../../../doc/research/AME/temporal_extension_research.md) for full planner evaluation.

### Planner Strategy

LAPKT does not natively support temporal planning. Recommended approach uses an external temporal planner alongside the existing LAPKT STRIPS path:

| Option | Role | Rationale |
|--------|------|-----------|
| **OPTIC** (primary) | Near-term temporal backend | C++, PDDL 2.1 + 3.0 trajectory constraints, proven IPC planner, subprocess integration |
| **Aries** (evaluate) | Medium-term replacement candidate | Rust, actively developed, built-in STN/Difference Logic solver, hierarchical + temporal |
| **TFD** (fallback) | Alternative if OPTIC licence is problematic | GPL, PlanSys2 ROS2 integration exists |
| **LAPKT** (retained) | STRIPS execution planner | Unchanged for non-temporal domains; fast, auditable |

Integration pattern: `IPlannerBackend` abstraction selects LAPKT for STRIPS domains and the temporal planner for `:durative-actions` domains.

### Proposed Architecture

```
PDDL 2.1 Domain --▶ PddlParser (extended) --▶ IPlannerBackend
                                                   │
                                    ┌--------------┼--------------┐
                                    ▼              ▼              ▼
                               LapktBackend   OpticBackend   (future)
                               (STRIPS)       (subprocess)   AriesBackend
```

### Implementation Phases

| Phase | Work Item | Effort |
|-------|-----------|--------|
| 7a | PDDL 2.1 parser (`:durative-action`, `:functions`) | Medium |
| 7b | WorldModel numeric fluent store + audit | Medium |
| 7c | `IPlannerBackend` abstraction + OPTIC subprocess | Medium |
| 7d | STN data structure + consistency check | Low |
| 7e | PlanCompiler temporal mode (STN → BT `Parallel`/`Timeout`) | High |
| 7f | VAL integration for temporal plan validation | Low |
| 7g | Temporal invariant monitoring (`ReactiveSequence`) | Low |
| 7h | End-to-end temporal tests | Medium |
| 7i | Aries evaluation + migration | Medium |

---

## Neuro-Symbolic Integration (Extension 8) — Candidate

Neural components should remain advisory while the symbolic stack remains authoritative.
See [`neuro_symbolic_reasoning.md`](../../../../doc/research/AME/neuro_symbolic_reasoning.md) and [`neuro_symbolic_reasoning_review.md`](../../../../doc/reviews/AME/neuro_symbolic_reasoning_review.md).

Adoption order:
1. Low-risk: goal interpretation and offline mission analysis
2. Medium-risk: planner-adjacent guidance and repair suggestions
3. High-risk: learned heuristics in the online control path

