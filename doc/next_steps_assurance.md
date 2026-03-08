# Next Steps: Autonomy Assurance

## Purpose
This document outlines the concrete engineering and documentation steps required to close the verification and assurance gaps identified in [`autonomy_assurance_plan.md`](autonomy_assurance_plan.md). It serves as an actionable backlog for improving the system's safety and robustness.

## 1. High-Priority Verification Gaps

According to the SACE Stage 8 analysis, the following technical verification mechanisms are missing and must be implemented:

### 1.1 Property-Based Testing for the Planner
**Objective:** Prove solver soundness and completeness invariants over random but valid domains.
**Actions:**
- Write a PDDL fuzzer that generates randomized, solvable STRIPS domains and problems.
- Create a test harness invoking the LAPKT solver on fuzzed problems.
- Assert that all returned plans simulate successfully from initial state to goal state.

### 1.2 Adversarial & Fault Injection Testing (Perception)
**Objective:** Ensure the system does not act on malicious, stale, or logically inconsistent perception data.
**Actions:**
- Implement a fault-injection middleware for the `WorldModel::setFact()` ROS2 service.
- Construct tests injecting:
  - High-frequency contradictory facts (livelock testing).
  - Facts involving unregistered types/objects.
  - Stale timestamps (simulating replay attacks or severe latency).

### 1.3 Safe-State Integration Harnes
**Objective:** Verify that the system degrades to a safe state when normal execution is impossible.
**Actions:**
- Create an end-to-end integration test isolating the execution loop.
- Trigger failures: planning timeouts, complete loss of communication (ROS2 lifecycle INACTIVE), and unmapped PDDL actions.
- Assert that the executor successfully loads and transitions to the fallback safe-state BT (e.g., Loiter or Return to Base).

## 2. System and Boundary Definitions

### 2.1 Out-of-Context (OOC) Detection (SACE Stage 7)
**Objective:** Detect when the AS is operating outside its validated Operational Domain Model (ODM).
**Actions:**
- Implement strict bounds-checking on the `WorldModel` (e.g., limits on fluent capacity, explicit geofencing boundaries).
- Introduce a `CheckContext` BT node that runs in a parallel reactive sequence to monitor operational limits.
- Design the handover procedure (Level 3 supervisor alert vs. Level 2 operator takeover) when an OOC condition is flagged.

### 2.2 Threat Modelling (DSTL Adversarial)
**Objective:** Provide a formal adversarial analysis.
**Actions:**
- Author a dedicated `threat_model.md` focusing on spoofing, PDDL injection, and denial of planning constraints.
- Formalise required cryptographic / integrity checks for loading `.pddl` files in production environments.

## 3. Medium-Priority Testing Work

### 3.1 Performance and Timing Benchmarks
**Objective:** Establish regression limits on planning latency and BT tick rates.
**Actions:**
- Enforce CI thresholds on LAPKT solver execution time for the `uav_search` domain.
- Create stress tests checking BT tick rates against 50Hz requirements under heavy load.

### 3.2 Formal Compiler Correctness
**Objective:** Guarantee the Plan-to-BT Compiler extracts causal dependencies perfectly.
**Actions:**
- Develop a suite that converts generated BTs back into linear action sequences.
- Verify that every linear topological sort of the generated BT's execution preserves the preconditions and effects established by the LAPKT sequential plan.
