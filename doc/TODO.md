# Remaining Work

Consolidated from `plan.md` (gap analysis) and `extensions.md`. All vertical slice steps (1–8) and extensions 1–6 are complete.

---

## Extension 7: Temporal Planning (not started)

PDDL 2.1 durative actions with STN (Simple Temporal Network) conversion. Requires:

- Temporal planner backend (LAPKT temporal extensions or external solver like OPTIC)
- PDDL 2.1 parser support (`:durative-actions`, `:fluents`)
- Numeric fluents in WorldModel (extend bitset to typed value store)
- Conditional effects (effect evaluation at tick time, not compile time)
- Temporal causal links in PlanCompiler (start/end time points)
- Durative actions → STN → timed Parallel/Sequence with deadline decorators

**Depends on:** Core vertical slice + hierarchical planning (both done).

---

## Hardening (production-readiness, no new architecture)

### Planner
- Solver portfolio: try fast heuristic first, fall back to complete search
- Configurable time/node budget with timeout handling
- Domain-specific heuristic configuration

### ActionRegistry
- Dynamic registration from config file (YAML/JSON)
- Type-checked parameter binding against PDDL schema
- Startup validation: check all PDDL actions have registered implementations

### BT Nodes
- Failure taxonomy: TRANSIENT (retry), PERMANENT (replan with blacklist), FATAL (abort)
- `WaitForFact`, `Timeout` decorator, `RetryWithBackoff` nodes

### PlanCompiler
- Serialise compiled BT to file for offline inspection
- Emit DOT graph of causal structure for debugging
- Complex DAG join-point synchronisation

### MissionExecutor
- Progressive replan strategy: retry → local replan → full replan → relax goal → abort
- Replan budget (max N replans before abort)
- Pre-replan world model consistency checks

### PDDL Parser
- Structured error messages with line/column info
- Schema validation: predicate arities, type consistency, unreachable goals

### Configuration & Packaging
- YAML config file for domain paths, solver selection, action mappings, logging sinks
- ROS2 lifecycle graceful shutdown with state persistence
- Debian/colcon package, Docker image, CI/CD pipeline

### Testing
- Property-based / fuzz tests for predicate/object registration ordering
- Systematic fault injection (random failures, perception delays, stale state)
- Planning time vs. domain size benchmarks
- GitHub Actions CI: build matrix, ctest, coverage

---

## Post-Extension (future, not yet planned)

- Plan quality optimisation (anytime search, LPG-style local improvement)
- Advanced heuristics (landmark-based for larger domains)
- Multi-agent planning (MA-STRIPS or task allocation)
- Dynamic object lifecycle (creation/destruction mid-mission with re-grounding)
- Resource-aware flow scheduling (mutex on shared resources)
- Gazebo/Isaac Sim integration for simulation testing
