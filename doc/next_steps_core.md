# Next Steps: Core Extensions Roadmap 

## Purpose
This document maps out the path forward for the core repository, building upon the completed features defined in [`plan.md`](plan.md) and [`extensions.md`](extensions.md). Observability (Layers 1-5) and ROS2 configurations are complete. The following items detail the strategy for completing Extensions 3 through 7.

## 1. Extension 3 & 5: Perception & Thread Safety
These two extensions are deeply intertwined. A single authoritative `WorldModel` accessed by concurrent perception callbacks and a running BT tick thread requires strict synchronisation.

### 1.1 Implement Versioned Snapshots
**Objective:** Provide lock-free or highly predictable memory access for the execution thread while perception updates the state.
**Actions:**
- Update `WorldModel` to maintain a concept of double-buffering or RCU (Read-Copy-Update) semantics.
- Let the BT executor read from a consistent snapshot (captured at the start of the tick).
- Apply all queued `setFact()` mutations from perception endpoints between BT ticks to increment the `version()` counter and swap the active state.

### 1.2 Perception Integration
**Objective:** Interface the `WorldModelNode` with real telemetry and perception classifiers.
**Actions:**
- Add a dedicated interface in the ROS2 `WorldModelNode` to subscribe to external `/detections` or `/telemetry` topics.
- Map these ROS messages into programmatic `setFact()` calls.
- Enforce the new **State Authority Semantics** (see `next_steps_neuro_symbolic.md`): mark perception-sourced facts as `CONFIRMED`.

## 2. Extension 4: PYRAMID Service Nodes
**Objective:** Provide native integration with external planning/strategic services using the PYRAMID architecture.
**Actions:**
- Implement the `InvokeService` BT node inheriting from `BT::StatefulActionNode`.
- This node will translate PDDL action bindings (from the `ActionRegistry`) into asynchronous service requests (e.g., via ROS2 Action Clients or direct SDK API calls).
- Handle the asynchronous wait returning `BT::NodeStatus::RUNNING` until the PYRAMID service finishes.
- Implement robust timeout handling and graceful transition to `FAILURE` if the SDK service is unreachable.

## 3. Extension 6: Hierarchical Planning
**Objective:** Allow top-level BT trees to act as mission-phase coordinators that launch dedicated planners for scoped sub-goals.
**Actions:**
- Finalize the `ExecutePhaseAction` BT node concept.
- Integrate the `PlannerNode` such that `ExecutePhaseAction` can invoke a ROS2 action call to the planner, retrieve a dynamically compiled child tree, and tick it using BT.CPP's dynamic sub-tree features.
- Update `PlanAuditLog` to capture causal links between parent mission phases and dynamically planned sub-trees to retain full observability.

## 4. Extension 7: Temporal Planning (Long-Term)
**Objective:** Transition from classical STRIPS to PDDL 2.1 durative actions to handle bounded resources, explicit timing, and asynchronous operations.
**Actions:**
- Evaluate swapping LAPKT BRFS for an explicitly temporal backend (e.g., LAPKT temporal extensions, OPTIC, or TFD).
- Re-write the `PlanCompiler` to emit temporal causal links into a Simple Temporal Network (STN).
- Compile STNs into explicitly timed `BT::Sequence` nodes utilizing native `BT::Timeout` decorators to guarantee schedule adherence.
