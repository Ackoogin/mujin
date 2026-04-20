# SACE/PDDL Documentation Approach Validity Review

Date: 2026-03-07

## Scope and review method

This review assesses the **validity of the documentation approach itself**, rather than the implementation status of the codebase. The focus is the SACE/PDDL documentation set under `doc/assurance/AME/sace-pddl/`, with particular attention to whether the assurance claims made for PDDL artefacts are methodologically sound.

The review was static and argument-focused. It did not assess runtime behavior, test results, or code correctness except where the documentation itself relies on distinctions between planning, model checking, validation, and execution.

## Findings

### 1. High: Stage 7 overstates single-scenario planning evidence as proof of a universal reachability claim

**Evidence**

- `doc/assurance/AME/sace-pddl/08-stage7-out-of-context.md:13-14` says planning from out-of-context states demonstrates whether safe fallback actions are always reachable.
- `doc/assurance/AME/sace-pddl/08-stage7-out-of-context.md:28-40` describes the out-of-context reachability analysis as a "formal proof of safe-state reachability".
- `doc/assurance/AME/sace-pddl/12-e2e-example.md:189-193` more carefully states the actual question for Stage 7 as universal reachability and notes that a model checker is needed for the "from every state" property.

**Why this matters**

This is the clearest place where the prose outruns the method. A planner run from one initial state can provide strong scenario-specific evidence that a safe state is reachable from that state in the model. It does not establish the universal claim that safe recovery exists from **all** relevant out-of-context states.

That universal claim is a valid Stage 7 goal, but it needs a stronger analysis method than a collection of individual plan-existence checks.

**Recommendation**

Revise Stage 7 so that:

- individual plans are described as fallback evidence for specific modeled states,
- "formal proof" is reserved for model-checking style analyses,
- and the universal claim is explicitly marked as requiring additional tooling beyond the current planner.

### 2. Medium: Stage 3, Stage 5, and parts of Stage 8 use stronger prose than the GSN structure actually requires

**Evidence**

- `doc/assurance/AME/sace-pddl/04-stage3-safe-operating-concept.md:13-14` says constrained planning provides a "constructive proof" that the SOC is achievable.
- `doc/assurance/AME/sace-pddl/06-stage5-design-assurance.md:13-14` says PDDL verifies that the architecture satisfies safety constraints under nominal and degraded configurations.
- `doc/assurance/AME/sace-pddl/09-stage8-verification.md:13-14` says each valid plan is a constructive proof.
- `doc/assurance/AME/sace-pddl/01-gsn-role.md:44-60` already explains that PDDL artefacts primarily populate **solution** nodes, while model fidelity and abstraction limits belong in **assumption** nodes.

**Why this matters**

The core GSN structure is mostly sound here. The documentation generally treats plan traces as evidence items attached to solution nodes, not as the entire argument. Within the formalism, a valid plan is legitimately constructive evidence that the modeled goal is reachable from the modeled initial state.

The issue is therefore mostly one of presentation, not argument structure:

- Stage 3 and Stage 5 should speak more clearly in terms of modeled feasibility and scenario-specific evidence.
- Stage 8 can retain the formal notion of constructive proof, but it should be qualified as proof **within the model**, with relevance to reality mediated by the existing assumption nodes on model fidelity and traceability.

**Recommendation**

Tighten the prose rather than redesigning the GSN mapping. The key fix is to distinguish:

- evidence that a plan exists in the formal model,
- from stronger claims about sufficiency, adequacy, or real-world completeness.

### 3. Medium: Stage 2 should qualify its "unknown-unsafe" language, but the underlying argument is assumption-based rather than circular

**Evidence**

- `doc/assurance/AME/sace-pddl/03-stage2-hazardous-scenarios.md:35-44` frames Pattern [I] as an argument that hazardous scenarios have been identified, while also explicitly stating the assumption that the hazard predicate set is complete.
- `doc/assurance/AME/sace-pddl/03-stage2-hazardous-scenarios.md:44-44` clearly identifies hazard-set completeness as an assumption requiring review against the preliminary hazard analysis.
- `doc/assurance/AME/sace-pddl/03-stage2-hazardous-scenarios.md:48-48` says PDDL state-space search addresses the unknown-unsafe quadrant.

**Why this matters**

The earlier version of this review overstated the problem as circularity. The document does, in fact, use GSN in the intended way: completeness of the hazard encoding is an explicit assumption, not something proven from within the PDDL model.

The real issue is narrower. The phrase "addresses the unknown-unsafe quadrant" risks sounding broader than the method actually supports. PDDL search can systematically explore the **modeled** unknown-unsafe space, but it does not by itself establish coverage of the broader real-world unknown-unsafe space.

**Recommendation**

Keep the assumption-based structure, but qualify the prose so that Stage 2 says PDDL exploration addresses the modeled unknown-unsafe space and converts candidate hazards into explicit counterexamples or negative results.

### 4. Medium: The residual-risk prose overreaches beyond the caveats already present in the assumption nodes

**Evidence**

- `doc/assurance/AME/sace-pddl/10-residual-risk.md:107-118,141-145` says PDDL coverage metrics can support or bound arguments about unknown-unsafe residual risk.
- `doc/assurance/AME/sace-pddl/10-residual-risk.md:171-171` gives an illustrative example based on "94% of reachable states".
- `doc/assurance/AME/sace-pddl/09-stage8-verification.md:43-47` similarly links state-space coverage to unknown-unsafe evidence.
- `doc/assurance/AME/sace-pddl/09-stage8-verification.md:43-43` already lists as an assumption that PDDL state-space coverage is a meaningful proxy for real-world scenario coverage.

**Why this matters**

The structure of the argument is better than the body prose suggests. The assumptions section already acknowledges the key gap: abstract state-space coverage is not automatically the same thing as operational scenario coverage or exposure-weighted residual risk.

The problem is therefore one of consistency. Parts of the body read as if coverage metrics directly support residual-risk bounds, while the assumption nodes more carefully acknowledge that a confidence argument is still required.

**Recommendation**

Align the body text with the existing caveats. Describe coverage metrics as:

- model-exploration evidence,
- confidence indicators for systematic analysis,
- or inputs to a broader residual-risk argument,

unless and until the docs add an explicit bridge from abstract states to operational exposure.

### 5. Medium: Stage 4 mixes direct PDDL contributions with stronger outputs that require additional tooling

**Evidence**

- `doc/assurance/AME/sace-pddl/05-stage4-safety-requirements.md:13-13` references HTN extensions or multi-level modelling as if they establish requirement decomposition support.
- `doc/assurance/AME/sace-pddl/05-stage4-safety-requirements.md:28-30,39-42` lists a completeness analysis showing that sub-domain constraints jointly imply system constraints.
- `doc/assurance/AME/sace-pddl/12-e2e-example.md:126-136` later states more precisely that proving joint implication requires a theorem prover or model checker.

**Why this matters**

Stage 4 contains genuine PDDL value:

- decomposition structure,
- traceability,
- and per-tier satisfaction evidence.

What it does not clearly separate is which outputs come directly from the PDDL/planning approach and which require a stronger formal method. The current text risks blurring "useful decomposition evidence" with "proof of compositional correctness".

**Recommendation**

Keep per-tier plan traces and traceability as legitimate Stage 4 evidence, but split the outputs into:

- evidence the current PDDL approach can provide directly, and
- stronger compositional claims that require additional tooling.

Also qualify or remove the HTN reference unless the documentation is explicitly discussing future or external methods rather than current repository capability.

### 6. Medium: The "single source of truth" section oversells a point that the assumptions table already qualifies correctly

**Evidence**

- `doc/assurance/AME/sace-pddl/13-tooling-analysis.md:261-267` presents shared PDDL as a "single source of truth" ensuring the analyzed model is the executed model.
- `doc/assurance/AME/sace-pddl/13-tooling-analysis.md:279-281` more carefully states the actual requirement: the STRIPS execution domain must be semantically compatible with the richer analysis domains, supported by a formal subsumption argument.

**Why this matters**

The substance is mostly there. The documentation already identifies the need for a conformance or subsumption argument between the execution model and richer offline evidence models.

The problem is presentation and emphasis. The section heading creates a stronger impression than the main technical claim can justify on its own. Once richer offline tools are introduced, validity depends not just on shared PDDL artefacts, but on an explicit argument that properties proven offline still apply to the restricted runtime semantics.

**Recommendation**

Rename or soften the "single source of truth" framing and promote the subsumption/conformance argument into the main text, rather than leaving it primarily in the assumptions table.

## Overall assessment

The overall documentation approach is more valid than this review's earlier draft implied. The GSN structure generally does the right thing:

- plan traces are treated as solution-node evidence,
- abstraction and model-fidelity gaps are represented as assumptions,
- and the tooling analysis correctly recognizes when stronger methods are needed.

The main issue is not that the argument structure is fundamentally wrong. It is that some of the prose, especially in the stage documents, sometimes overstates what the evidence shows and does not always stay aligned with the more careful caveats already present elsewhere in the documentation set.

The approach is strongest when read as:

- a structured way to model scenarios,
- generate formal evidence within the modeled abstraction,
- organize assumptions explicitly,
- and connect offline analysis artefacts into a GSN-based safety argument.

It is weakest where the prose appears to collapse the distinction between:

- modeled feasibility,
- universal properties,
- residual-risk justification,
- and real-world adequacy.

## Recommended next steps

1. Fix Stage 7 first by removing "formal proof" language for universal reachability unless backed by model checking.
2. Soften Stage 3 and Stage 5 wording to emphasize modeled feasibility and scenario-specific evidence.
3. Qualify Stage 8 language so "constructive proof" is clearly stated as proof within the formal model, with real-world relevance mediated by assumption nodes.
4. Update Stage 2 wording around "unknown-unsafe" so it refers to the modeled unknown-unsafe space, while preserving the explicit completeness assumption.
5. Align the residual-risk body text with its existing assumption nodes so coverage metrics are not presented as direct residual-risk bounds without an exposure bridge.
6. Split Stage 4 outputs into direct PDDL contributions versus outputs requiring stronger formal methods.
7. Replace or soften "single source of truth" in the tooling analysis and promote the subsumption argument into the main narrative.

