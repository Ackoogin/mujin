# Autonomous Mission Engine (AME) — Stakeholder Summary

## What This System Does

The AME autonomy module enables unmanned platforms to **plan and execute missions automatically**. Given a description of the mission objectives and the current situation, it works out what actions to take, carries them out, and adapts if things go wrong — all without requiring step-by-step human programming for each mission.

Think of it as a mission planner and executor combined: the system receives goals (e.g., "search these sectors and classify anything found"), figures out the steps needed, runs them — potentially in parallel — and recovers automatically if an action fails.

## Why This Approach

### Formal Planning, Not Hard-Coded Scripts

Traditional autonomy software uses hand-written decision scripts ("if X happens, do Y"). These become brittle and hard to maintain as missions grow in complexity. Our approach uses **formal AI planning** — the same class of techniques used in logistics, space mission planning, and industrial automation:

- Mission objectives and available actions are described in a standard language called **PDDL** (Planning Domain Definition Language), widely used in academic and industrial AI planning
- A **solver** automatically searches for a valid sequence of actions to achieve the objectives
- The resulting plan is compiled into an executable **behaviour tree** that handles real-time execution, including parallel actions and reactive monitoring

This means new mission types can be defined by writing new PDDL descriptions rather than rewriting execution code.

### Key Design Principles

| Principle | What It Means in Practice |
|-----------|--------------------------|
| **Single source of truth** | One central "world model" holds all known facts. Every component reads from and writes to the same place, preventing conflicting views of the world. |
| **Automatic recovery** | If an action fails, the system captures the current state, generates a new plan from that state, and continues — no human intervention required for routine failures. |
| **Full auditability** | Every state change, every planning decision, and every execution step is logged with timestamps and attribution. Post-mission, you can reconstruct exactly what happened and why. |
| **Separation of concerns** | The planning logic, execution engine, and robot/sensor interfaces are independent modules. You can change the planner, swap the robot platform, or update sensors without rewriting the whole system. |
| **ROS-agnostic core** | The core planning and execution library has no dependency on ROS2 or any specific robotics framework. ROS2 integration is provided as an optional wrapper layer. |

## How It Works — In Plain Terms

```
1. DEFINE the mission
   (what actions are possible, what the world looks like, what goals to achieve)
                                        |
                                        v
2. PLAN automatically
   (solver finds a sequence of actions that achieves the goals)
                                        |
                                        v
3. ANALYSE the plan for parallelism
   (identify which actions are independent and can run simultaneously)
                                        |
                                        v
4. EXECUTE the plan
   (behaviour tree engine runs actions, checks preconditions, applies effects)
                                        |
                                        v
5. MONITOR and ADAPT
   (if something fails or the world changes, replan from the current state)
                                        |
                                        v
6. AUDIT everything
   (every decision and state change recorded for review)
```

## What Makes This Different

### Compared to Hard-Coded Autonomy

| Aspect | Hard-coded scripts | This system |
|--------|--------------------|-------------|
| Adding new missions | Requires rewriting code | Write a new PDDL domain file |
| Handling failures | Each failure case must be anticipated and coded | Automatic replanning from current state |
| Parallel execution | Must be manually designed | Automatically identified from plan structure |
| Auditability | Depends on what was logged | Comprehensive by design — every decision recorded |
| Verification | Test each script path | Formal properties of the planner can be verified |

### Compared to Neural/AI-Only Approaches

| Aspect | Neural-only (e.g., LLM-driven) | This system |
|--------|--------------------------------|-------------|
| Predictability | Probabilistic — may produce different outputs each time | Deterministic — same inputs always produce the same plan |
| Explainability | "Black box" reasoning | Every plan step traceable to preconditions and effects |
| Safety assurance | Difficult to certify | Formal model enables structured safety arguments (SACE, AMLAS) |
| Training data | Requires extensive training data | Works from domain descriptions — no training required |

The architecture also supports **optional neural integration** (see roadmap) in an advisory role — e.g., using AI to suggest plans faster or interpret operator commands — while keeping the formal planner as the authority that validates all decisions.

## Current Status

### What's Built and Working

- **Core planning and execution pipeline** — end-to-end from PDDL to plan to behaviour tree execution, with replan-on-failure
- **Full observability stack** (5 layers) — BT execution events, world state audit trail, plan audit trail, live Foxglove Studio monitoring
- **ROS2 integration** — lifecycle node wrappers for distributed deployment
- **Comprehensive test suite** — 73 tests covering all components
- **UAV search-and-classify example** — working demonstration domain

### What's Next

| Priority | Feature | Status |
|----------|---------|--------|
| 1 | Perception integration (external sensors updating world state) | Ready — core APIs in place, needs ROS2 service wiring |
| 2 | PYRAMID service node integration | Awaiting SDK availability |
| 3 | Thread safety for multi-node deployment | Designed, not yet needed |
| 4 | Hierarchical planning (multi-phase missions) | Future |
| 5 | Temporal planning (time-bounded actions) | Future |
| 6 | Neuro-symbolic integration (AI-assisted planning) | Designed — see integration options document |

## Observability and Assurance

### Real-Time Monitoring

The system includes a live monitoring capability via **Foxglove Studio** (free, cross-platform desktop application). Operators can observe:

- Which actions are currently running, succeeding, or failing
- How the world model state is changing over time
- The full planning and execution audit trail

No ROS2 is required for monitoring — the system runs its own WebSocket server.

### Safety Assurance Framework

A structured autonomy assurance plan has been developed, mapped to established frameworks:

- **SACE** (Safety Assurance of Autonomous Systems in Complex Environments) — University of York / AAIP
- **AMLAS** (Assurance of ML for Autonomous Systems) — for future neural components
- **DSTL Biscuit Book** — cross-cutting assurance dimensions
- **Defence Standard 00-56** — safety management requirements

The formal planning model (PDDL) directly supports several assurance activities: hazard analysis can be performed systematically on the action model, plans can be verified against domain constraints, and the complete audit trail provides post-incident evidence.

See `doc/autonomy_assurance_plan.md` for the full assurance plan.

## Technical Summary

| Aspect | Detail |
|--------|--------|
| Language | C++17 |
| Build system | CMake with automatic dependency management |
| Planning | LAPKT classical AI planner (breadth-first search) |
| Execution | BehaviorTree.CPP 4.6.2 |
| Domain language | PDDL (Planning Domain Definition Language) |
| Monitoring | Foxglove Studio via WebSocket |
| ROS2 support | Optional — Jazzy, lifecycle nodes |
| Tests | 73 tests (Google Test) |
| Platforms | Windows (MSVC), Linux (GCC/Clang) |
