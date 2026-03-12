# Next Steps: Neuro-Symbolic Integration

## Purpose
This document distills the recommendations from [`neuro_symbolic_reasoning_review.md`](neuro_symbolic_reasoning_review.md) into concrete implementation steps. The core philosophy of this plan is: **Neural components assist, but the symbolic system remains the absolute authority for correctness.**

## 1. Architectural Pre-Requisites

Before bridging any LLM or neural component into the system, the underlying invariants must be clarified to prevent logic or safety drift.

### 1.1 Clarify State-Authority Semantics
**Objective:** Resolve the ambiguity around "world-state" vs "believed-state", an issue highlighted in the neuro-symbolic review.
**Actions:**
- Update `concept.md` to define **Believed Facts** (optimistic plan effects applied by the BT) versus **Confirmed Facts** (authoritative states verified by perception).
- Enforce explicit tagging in the `WorldModel` to differentiate between the two, preventing the planner from making authoritative decisions on provisional data without a timeout or verification check.

### 1.2 Define Neural Acceptance Criteria
**Objective:** Establish boundaries for neural tools.
**Actions:**
- **Latency Budget:** Neural calls in the planning path (e.g., heuristics) must adhere to a strict timeout (e.g., 500ms).
- **Fallback Policy:** Any timeout, malformed output, or unreachable endpoint must immediately revert execution to the deterministic LAPKT solver.
- **Audit Logging:** Every LLM prompt, response, and action must be captured in the Observability stack (Layer 5 or a new Layer 6).

## 2. Phase 1 Implementation (Low-Risk, High-Value)

We will proceed with the immediate adoption sequence recommended in the review, focusing on user-experience and observability enhancements outside the critical execution path.

### 2.1 LLM Goal Interpreter (Option E)
**Objective:** Translate human operator natural language into grounded symbolic goals for the LAPKT planner.
**Implementation Steps:**
1. Write a Python/C++ integration wrapper interfacing with an LLM.
2. Provide the LLM with the parsed `DomainModel` (predicates and objects via the `TypeSystem`).
3. Require the LLM to output ONLY a JSON array of grounded PDDL goal fluents.
4. **Validation:** The symbolic `WorldModel` strictly validates the output. If the fluents do not match the grounded bitset schemas, the goal is rejected, and the operator is prompted for clarification.
5. Create a `tests/test_goal_interpreter.py` to ensure adversarial inputs do not result in malformed goals or planning crashes.

### 2.2 LLM Mission Analyst (Option F)
**Objective:** Provide a batch-processing tool to analyze the extensive SQLite/JSONL audit logs and generate human-readable incident and mission summaries.
**Implementation Steps:**
1. Create an offline Python utility (`scripts/mission_analyst.py`).
2. Implement log ingestion to read `ame_plan_audit.jsonl`, `ame_bt_events.jsonl`, and `ame_wm_audit.jsonl`.
3. Compress and window the transition logs into logical decision epochs.
4. Prompt the LLM to analyze the sequences (e.g., "Why did the UAV replan at t=1702345679000?").
5. **Validation:** Enforce evidence citation. The LLM must cite specific `wm_version` numbers and BT node names to justify its explanations.

## 3. Future Phases (Deferred)

Once Phase 1 is proven and stable, the following options will be investigated under the constraints outlined in the [Assurance Plan](next_steps_assurance.md):
- **Phase 2:** Planner-adjacent integrations (Option A: Heuristic Guide, Option B: Plan Repair).
- **Phase 3:** Embedded Data-Driven Optimization (Option D: Learned Heuristic, Option G: Anomaly Detector) relying on actual mission data sets spanning multiple deployments.
- **Phase 4:** Offline authoring tools (Option C).

## 4. Documentation Updates
- Add **Neuro-Symbolic Reasoning** formally as "Extension 8" in `extensions.md`, incorporating Phase 1.
- Update `autonomy_assurance_plan.md` to ensure "Autonomous Goal Decomposition" references the LLM Goal Interpreter architecture rather than assuming it is already implicitly built into the STRIPS solver.
