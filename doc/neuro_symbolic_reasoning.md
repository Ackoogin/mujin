# Neuro-Symbolic Reasoning Integration

Options for integrating neural reasoning (LLMs and other models) into the PDDL planning + BT execution pipeline. The primary integration point is the **planning layer** (heuristic guidance, domain authoring, plan repair). A secondary integration point is **evidence review** over observability data.

---

## Current Architecture (Baseline)

```
PDDL Domain/Problem
        |
        v
   WorldModel ──► Planner (LAPKT BRFS) ──► PlanCompiler ──► BT Executor
   (bitset)          pure symbolic              causal          tick loop
                     search                     graph
```

The planner is deterministic, complete (for finite STRIPS), and auditable. Every plan step maps 1:1 to a ground action with known preconditions and effects. The observability stack (Layers 1–5) records every state transition, fact change, and planning episode.

Any neural integration must preserve these properties or explicitly document which guarantees it relaxes.

---

## Integration Point 1: Planning Layer

### Option A — LLM as Heuristic Guide (Preferred Starting Point)

**Concept**: Keep LAPKT as the sound solver but use an LLM to supply a heuristic ordering or seed search, reducing the number of nodes expanded.

**How it works**:

```
WorldModel snapshot (init facts + goal fluents)
        |
        v
   LLM Heuristic ──► ranked action preferences / initial plan sketch
        |
        v
   LAPKT solver (BRFS / best-first) with LLM-informed h(n)
        |
        v
   Verified plan (still sound w.r.t. PDDL semantics)
```

1. Before calling `Planner::solve()`, serialize the current world state and goal into a natural-language or structured prompt.
2. The LLM returns a ranked list of promising actions or a candidate plan sketch.
3. Feed rankings into LAPKT as action-ordering preferences (tie-breaking in BRFS) or as the initial state for a plan-repair search.
4. LAPKT still validates every step against PDDL preconditions — the LLM cannot produce an invalid plan.

**Integration surface**:

```cpp
// New: pluggable heuristic provider
class NeuralHeuristic {
public:
    virtual ~NeuralHeuristic() = default;

    // Given current state + goal, return action preferences
    // Higher score = try this action earlier in search
    virtual std::vector<ActionScore> scoreActions(
        const WorldModel& wm,
        const std::vector<unsigned>& goal_fluents) = 0;
};

struct ActionScore {
    unsigned ground_action_id;
    float score;  // higher = preferred
};

// Planner gains an optional heuristic
class Planner {
public:
    void setNeuralHeuristic(std::shared_ptr<NeuralHeuristic> h);
    PlanResult solve(const WorldModel& wm);  // unchanged API
};
```

**Pros**: Sound plans guaranteed. Graceful degradation — if LLM is unavailable or slow, BRFS proceeds with default ordering. Fully auditable (log the LLM scores alongside the plan episode in Layer 5).

**Cons**: LLM latency added to planning time (mitigated by async pre-computation). Benefit depends on domain complexity — small STRIPS problems already solve in <50ms.

**When to use**: Domains with large grounded action spaces (>1000 actions) where BRFS expansion is the bottleneck. Multi-objective scenarios where the LLM encodes soft preferences not expressible in PDDL.

---

### Option B — LLM-Driven Plan Repair

**Concept**: When the BT executor encounters a failure and replanning is triggered, use an LLM to propose a repair patch rather than replanning from scratch.

**How it works**:

```
BT Failure detected
        |
        v
   Failure context:
     - Failed action + step index
     - Current world state (WM snapshot)
     - Original plan + causal graph
     - Recent WM audit entries (what changed unexpectedly)
        |
        v
   LLM Plan Repair ──► candidate repair (partial plan suffix)
        |
        v
   Validate repair against PDDL ──► if valid, compile to BT
                                     if invalid, fall back to full BRFS replan
```

**Integration surface**:

```cpp
class NeuralPlanRepairer {
public:
    struct RepairContext {
        unsigned failed_step_index;
        std::string failed_action;
        std::vector<std::string> current_facts;     // true fluents
        std::vector<std::string> goal_fluents;
        std::vector<std::string> original_plan;     // full plan as action signatures
        std::vector<WmAuditLog::Entry> recent_changes;  // what diverged
    };

    // Returns a candidate suffix plan starting from current state
    virtual std::optional<std::vector<PlanStep>> repair(
        const RepairContext& ctx,
        const WorldModel& wm) = 0;
};
```

The returned plan steps are validated by applying them to a WorldModel copy (forward simulation). If all preconditions hold and the goal is reached, the repair is accepted. Otherwise, fall back to `Planner::solve()`.

**Pros**: Faster recovery in large domains where full replanning is expensive. LLM can reason about *why* the failure occurred (using audit log context) and avoid repeating the same failure.

**Cons**: Repair validation adds a forward-simulation pass. LLM may produce plausible-looking but invalid repairs, wasting time before fallback.

---

### Option C — Neural PDDL Domain Authoring

**Concept**: Use an LLM to generate or extend PDDL domain definitions from natural-language mission descriptions, sensor specifications, or operator intent.

**How it works**:

```
Operator intent (natural language)
  + Existing domain.pddl (if any)
  + Object/type inventory from WorldModel
        |
        v
   LLM Domain Author ──► candidate PDDL domain + problem
        |
        v
   PDDL Validator ──► syntax + type check
        |
        v
   WorldModel::registerPredicate() / registerAction()
        |
        v
   Standard planning pipeline
```

This is an **offline** or **mission-setup-time** integration — not in the real-time planning loop.

**Integration surface**:

```cpp
class DomainAuthor {
public:
    struct DomainDraft {
        std::string domain_pddl;
        std::string problem_pddl;
        std::vector<std::string> warnings;  // LLM confidence notes
    };

    virtual DomainDraft generateDomain(
        const std::string& mission_description,
        const std::string& existing_domain_pddl,  // empty if new
        const std::vector<std::string>& available_types,
        const std::vector<std::string>& available_objects) = 0;
};
```

**Pros**: Dramatically lowers the barrier to creating new PDDL domains. Operators describe missions in natural language; the system generates formal specifications. Pairs well with the existing `PddlParser`.

**Cons**: Generated PDDL must be validated and likely human-reviewed before deployment. Not suitable for safety-critical domains without verification.

---

### Option D — Learned Heuristic (Non-LLM Neural Model)

**Concept**: Train a small, fast neural network (GNN, transformer encoder, or MLP) on historical planning data to predict action costs or goal distances, replacing or augmenting LAPKT's default heuristic.

**How it works**:

```
Training (offline):
   PlanAuditLog episodes ──► (state, goal, optimal_cost) pairs
        |
        v
   Train h(s, g) model (small GNN/MLP)
        |
        v
   Export to ONNX / TorchScript

Inference (online):
   WorldModel state + goal
        |
        v
   h(s, g) → estimated cost-to-go
        |
        v
   LAPKT best-first search with neural h(n)
```

**Integration surface**:

```cpp
class LearnedHeuristic : public NeuralHeuristic {
public:
    LearnedHeuristic(const std::string& model_path);  // ONNX or TorchScript

    std::vector<ActionScore> scoreActions(
        const WorldModel& wm,
        const std::vector<unsigned>& goal_fluents) override;

private:
    // ONNX Runtime or libtorch session
    // Input: binary state vector (from WM bitset) + goal vector
    // Output: per-action score or state-value estimate
};
```

**Pros**: Microsecond inference (no LLM latency). Can be trained on domain-specific data from `PlanAuditLog` episodes. Proven effective in learning-for-planning literature (e.g., LAMA-style landmark heuristics, GNN-based planners).

**Cons**: Requires training data (many solved episodes). Domain-specific — must retrain for new domains. Model quality directly affects search efficiency.

**Dependencies**: ONNX Runtime (available via FetchContent, ~5MB static lib) or libtorch. Keeping it optional (`MUJIN_NEURAL_HEURISTIC` CMake flag, similar to `MUJIN_FOXGLOVE`).

---

### Option E — LLM as Goal Interpreter

**Concept**: Translate high-level operator commands into formal PDDL goal specifications.

**How it works**:

```
Operator: "Search all sectors and classify anything suspicious"
        |
        v
   LLM Goal Interpreter
     + WorldModel object inventory (sectors, robots)
     + Domain predicate list
        |
        v
   Goal fluents: [(searched sector_a), (searched sector_b),
                   (classified sector_a), (classified sector_b)]
        |
        v
   WorldModel::setGoal() → standard planning pipeline
```

This is the simplest integration point and the lowest risk. The LLM output is a set of ground fluent names, trivially validated against the WorldModel's fluent index.

---

## Integration Point 2: Evidence Review Tool

### Option F — LLM-Powered Mission Analyst

**Concept**: Post-execution (or during long missions), use an LLM to analyze the observability data streams (Layers 2, 3, 5) and surface anomalies, explain failures, or answer operator questions.

**Data sources** (all already produced by the existing pipeline):

| Source | Layer | Content |
|--------|-------|---------|
| `mujin_bt_events.jsonl` | 2 | Every BT node status transition |
| `mujin_wm_audit.jsonl` | 3 | Every fact change with source attribution |
| `mujin_plan_audit.jsonl` | 5 | Full planning episodes (init, goal, plan, BT XML) |

**Capabilities**:

1. **Anomaly detection** — "Why did the plan fail at step 3?" Feed the plan audit episode + WM changes around the failure timestamp to the LLM. It correlates the expected effects with actual state changes and identifies the divergence.

2. **Causal explanation** — "Why is uav1 at sector_b instead of sector_a?" Trace the WM audit log backward from the fact `(at uav1 sector_b)` to find the BT node that set it, the plan step that required it, and the goal that motivated it.

3. **Mission summary** — "Summarize what happened in the last 5 minutes." Aggregate BT events and WM changes into a natural-language narrative.

4. **Counterfactual reasoning** — "What would have happened if sector_a had already been searched?" Modify the init state in a plan audit episode, replay planning, and compare the resulting plan.

5. **Safety audit** — "Were all preconditions satisfied before each action executed?" Cross-reference plan steps with WM state at each BT node transition to verify that the symbolic model was respected.

**Integration surface**:

```cpp
class MissionAnalyst {
public:
    struct AnalysisRequest {
        std::string question;                            // natural language
        std::vector<MujinBTLogger::Event> bt_events;    // relevant window
        std::vector<WmAuditLog::Entry> wm_changes;      // relevant window
        std::vector<PlanAuditLog::Episode> episodes;     // relevant episodes
    };

    struct AnalysisResult {
        std::string explanation;        // natural language answer
        std::vector<std::string> evidence;  // specific log entries cited
        float confidence;               // 0-1 self-assessed confidence
    };

    virtual AnalysisResult analyze(const AnalysisRequest& req) = 0;
};
```

**Deployment options**:

- **Batch**: Run after mission completion over JSONL files. No real-time requirements.
- **Streaming**: Attach as a callback sink (like FoxgloveBridge) that buffers events and periodically queries an LLM for anomaly alerts.
- **Interactive**: Operator asks questions via CLI/web UI; system retrieves relevant log windows and sends to LLM.

**Pros**: Leverages the full observability stack already in place. No changes to the core pipeline. High value for operator understanding and post-incident review. Works with any LLM backend (Claude API, local models, etc.).

**Cons**: LLM latency makes real-time alerting challenging (batch or periodic is more practical). Large missions produce voluminous logs — needs a retrieval/windowing strategy to stay within context limits.

---

### Option G — Neural Anomaly Detector (Non-LLM)

**Concept**: Train a small sequence model (LSTM, transformer) on nominal BT event streams to detect execution anomalies in real time.

```
Training (offline):
   mujin_bt_events.jsonl from successful missions
        |
        v
   Train sequence model: P(next_event | event_history)

Inference (online):
   Live BT event stream
        |
        v
   If P(observed_event) < threshold → flag anomaly
        |
        v
   Alert operator + log to anomaly audit trail
```

**Pros**: Real-time (millisecond inference). No LLM dependency. Learns domain-specific execution patterns.

**Cons**: Requires representative training data from successful missions. May produce false positives in novel but valid scenarios.

---

## Recommended Integration Sequence

| Phase | Integration | Risk | Effort | Value |
|-------|------------|------|--------|-------|
| **Phase 1** | **E: Goal Interpreter** | Low | ~200 LOC + LLM API | Immediate UX improvement |
| **Phase 1** | **F: Mission Analyst (batch)** | Low | ~300 LOC + LLM API | Leverages existing observability |
| **Phase 2** | **A: LLM Heuristic Guide** | Medium | ~400 LOC in Planner | Scaling to larger domains |
| **Phase 2** | **B: LLM Plan Repair** | Medium | ~500 LOC + validation | Faster failure recovery |
| **Phase 3** | **D: Learned Heuristic** | Medium | ONNX dep + training pipeline | Production-grade speed |
| **Phase 3** | **G: Neural Anomaly Detector** | Medium | Training pipeline + model serving | Real-time monitoring |
| **Phase 4** | **C: Domain Authoring** | Higher | Validation framework needed | New mission types |

Phase 1 can start immediately — it requires no changes to the core planning or execution pipeline. Phase 2 modifies the planner internals but preserves soundness guarantees. Phase 3 adds ML infrastructure dependencies. Phase 4 involves generating safety-critical artifacts and requires additional verification.

---

## Architecture: Where Neural Components Fit

```
                                    ┌─────────────────┐
                                    │  Goal Interpreter│ (Option E)
                                    │  (LLM)          │
                                    └────────┬────────┘
                                             │ goal fluents
                                             v
Operator ──► Mission Description ──► WorldModel ──► Planner ──► PlanCompiler ──► BT Executor
                                         │              │                            │
                                         │         ┌────┴─────┐                      │
                                         │         │ Neural   │ (Options A, D)       │
                                         │         │ Heuristic│                      │
                                         │         └──────────┘                      │
                                         │                                           │
                                         │              ┌──────────────┐             │
                                         │              │ Plan Repair  │ (Option B)  │
                                         │              │ (LLM)        │◄────────────┘
                                         │              └──────────────┘   on failure
                                         │
                                         ▼
                              Observability Layers 2,3,5
                                         │
                              ┌──────────┴──────────┐
                              │                     │
                      ┌───────▼──────┐    ┌─────────▼────────┐
                      │Mission Analyst│    │Anomaly Detector  │ (Options F, G)
                      │(LLM, batch)  │    │(neural, real-time)│
                      └──────────────┘    └──────────────────┘
```

---

## Soundness and Safety Considerations

All neural integrations follow a **propose-verify** pattern:

1. **Neural component proposes** — action rankings, plan repairs, goal interpretations, explanations.
2. **Symbolic component verifies** — LAPKT validates plans against PDDL semantics. WorldModel validates goal fluents exist. Forward simulation validates repair steps.

The symbolic pipeline remains the authority. Neural components are advisory. This is consistent with the safety case documented in `doc/autonomy_assurance_plan.md` and the SACE-PDDL evidence in `doc/sace-pddl/`.

**Audit trail**: Every neural contribution is logged:
- LLM heuristic scores recorded in `PlanAuditLog` episodes (new field: `heuristic_source`)
- Plan repair attempts logged whether accepted or rejected
- Goal interpretations logged with the original natural-language input
- Mission analyst outputs stored with cited evidence for traceability

**Fallback behavior**: If any neural component is unavailable (network failure, timeout, model error), the system degrades to pure symbolic operation. No neural component is in the critical path — they all have bypass routes to the existing deterministic pipeline.

---

## LLM Backend Options

| Backend | Latency | Context | Deployment | Best For |
|---------|---------|---------|------------|----------|
| Claude API (claude-sonnet-4-6) | ~1-2s | 200k tokens | Cloud | Mission analyst, domain authoring |
| Claude API (claude-haiku-4-5) | ~0.3-0.5s | 200k tokens | Cloud | Goal interpretation, plan repair |
| Local LLM (Llama, Mistral) | ~0.1-1s | 8-32k tokens | Edge device | Latency-sensitive planning guidance |
| ONNX model (custom) | <1ms | N/A | Embedded | Learned heuristic, anomaly detection |

For edge deployment (the primary use case), a tiered approach works well:
- **Hot path** (planning heuristic): local ONNX model or small local LLM
- **Warm path** (plan repair, goal interpretation): local LLM or fast cloud API
- **Cold path** (mission analysis, domain authoring): full cloud API

---

## Dependencies and Build Integration

New optional dependencies, following the `MUJIN_FOXGLOVE` pattern:

```cmake
option(MUJIN_NEURAL_HEURISTIC "Build with ONNX Runtime for learned heuristics" OFF)
option(MUJIN_LLM "Build with LLM integration (requires libcurl)" OFF)

if(MUJIN_NEURAL_HEURISTIC)
    FetchContent_Declare(onnxruntime ...)
    target_link_libraries(mujin_core PRIVATE onnxruntime)
endif()

if(MUJIN_LLM)
    find_package(CURL REQUIRED)
    add_library(mujin_llm STATIC
        src/llm/llm_client.cpp
        src/llm/goal_interpreter.cpp
        src/llm/mission_analyst.cpp
        src/llm/plan_repairer.cpp
    )
    target_link_libraries(mujin_llm PUBLIC mujin_core PRIVATE CURL::libcurl)
endif()
```

Library boundaries remain clean:
- `mujin_core` — no neural dependencies (unchanged)
- `mujin_llm` — optional, LLM integrations (Options A, B, C, E, F)
- `mujin_neural` — optional, ONNX-based models (Options D, G)
- `mujin_foxglove` — unchanged

---

## Relation to Extension Roadmap

This document proposes **Extension 8: Neuro-Symbolic Reasoning** in the roadmap (`doc/extensions.md`). It depends on:

- Extensions 1 (Observability) — **done**, provides data for evidence review
- Extension 2 (ROS2 Wrappers) — **done**, provides deployment surface
- Extension 3 (Perception Integration) — provides real-world state updates that make neural plan repair valuable

It is independent of Extensions 4–7 (PYRAMID, thread safety, hierarchical/temporal planning) but composes well with them — e.g., hierarchical planning (ext 6) benefits from LLM-guided sub-goal decomposition, and temporal planning (ext 7) benefits from learned duration estimates.
