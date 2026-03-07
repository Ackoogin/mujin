# SACE/PDDL Documentation Approach Validity Review

Date: 2026-03-07

## Scope and review method

This review assesses the **validity of the documentation approach itself**, rather than the implementation status of the codebase. The focus is the SACE/PDDL documentation set under `doc/sace-pddl/`, with particular attention to whether the assurance claims made for PDDL artefacts are methodologically sound.

The review was static and argument-focused. It did not assess runtime behavior, test results, or code correctness except where the documentation itself relies on distinctions between planning, model checking, validation, and execution.

## Findings

### 1. High: The docs often treat plan existence as proof of safety claims that are actually universal or adequacy claims

**Evidence**

- `doc/sace-pddl/04-stage3-safe-operating-concept.md:13-14` says constrained planning provides a "constructive proof" that the SOC is achievable.
- `doc/sace-pddl/06-stage5-design-assurance.md:13-14` says PDDL verifies that the architecture satisfies safety constraints under nominal and degraded configurations.
- `doc/sace-pddl/08-stage7-out-of-context.md:13-14,28-40` says planning from out-of-context states demonstrates whether fallback actions are always reachable and calls the analysis a "formal proof".
- `doc/sace-pddl/09-stage8-verification.md:13-14,47-47` says each valid plan is a constructive proof and that constrained plan traces prove mitigation of known-unsafe scenarios.

**Why this matters**

A planner finding a plan demonstrates that at least one action sequence exists in the model for a given initial state and goal. That is much weaker than the kinds of SACE claims the stage documents are trying to support, such as:

- all relevant degraded states are recoverable,
- the SOC is sufficient across the hazardous scenario class,
- mitigation is effective in general rather than for one modelled case,
- the real system will behave consistently with the abstract plan.

Existential feasibility evidence is useful, but it is not by itself proof of safety sufficiency, universal recoverability, or verification completeness.

**Recommendation**

Reword these sections so that planning traces are described as:

- feasibility evidence,
- scenario-specific evidence,
- or partial formal evidence within the modeled abstraction,

rather than as standalone proof of the stage-level safety claim. Reserve stronger language such as "proof" for cases where the documentation explicitly introduces model checking or another method that can justify the stronger property.

### 2. High: The Stage 2 hazard-identification argument is circular about unknown-unsafe discovery and completeness

**Evidence**

- `doc/sace-pddl/03-stage2-hazardous-scenarios.md:13-14` says PDDL exploration enables automated hazard discovery.
- `doc/sace-pddl/03-stage2-hazardous-scenarios.md:35-44` frames the argument as showing that all hazardous scenarios have been identified and that the identification is complete and correct.
- `doc/sace-pddl/03-stage2-hazardous-scenarios.md:48-48` says PDDL search addresses the unknown-unsafe quadrant.
- `doc/sace-pddl/03-stage2-hazardous-scenarios.md:44-44` simultaneously assumes the hazard predicate set is complete.

**Why this matters**

The modeled state space can only reveal hazards that have already been represented through predicates, actions, abstractions, and fault injections. That makes PDDL exploration valuable for systematic analysis of the **modeled** scenario space, but it cannot by itself justify the stronger claim that:

- all hazardous scenarios have been identified, or
- the unknown-unsafe space has been covered in the broader SOTIF sense.

As written, the argument risks using the model to prove the completeness of the very modeling choices that limit what the model can express.

**Recommendation**

Narrow the claim to something like:

"PDDL provides systematic exploration of the modeled hazard space and can convert manually identified or systematically generated candidate scenarios into explicit counterexamples or negative results."

Then keep completeness as an argued assumption supported by external hazard analysis, domain review, scenario derivation, and model-fidelity evidence.

### 3. High: The residual-risk discussion overstates what PDDL coverage metrics can support

**Evidence**

- `doc/sace-pddl/10-residual-risk.md:35-42` links residual-risk acceptance to exploring the unknown-unsafe space to a defined coverage threshold.
- `doc/sace-pddl/10-residual-risk.md:107-118,141-145` says PDDL state-space coverage metrics can bound unknown-unsafe residual risk.
- `doc/sace-pddl/10-residual-risk.md:171-171` gives an example based on "94% of reachable states".
- `doc/sace-pddl/09-stage8-verification.md:29-29,43-47` makes the same move from state-space coverage to verification completeness and unknown-unsafe evidence.

**Why this matters**

Coverage over an abstract planner state space is not the same thing as:

- real-world operational scenario coverage,
- exposure-weighted scenario coverage,
- or a quantitative residual-risk bound in the ISO 21448 sense.

Even if the state count is accurate, a percentage of reachable abstract states does not directly tell you the probability mass, operational significance, or representational adequacy of what remains unexplored.

**Recommendation**

Reframe coverage metrics as internal model-exploration indicators unless the documentation also introduces:

- an explicit mapping from abstract states to scenario classes,
- a defensible exposure model,
- and a justified argument for why the metric correlates with residual risk.

Without that bridge, the safer claim is that coverage metrics support confidence in systematic exploration of the model, not that they bound residual risk.

### 4. Medium: Stage 4 presents requirement decomposition as if PDDL structure itself can prove compositional correctness

**Evidence**

- `doc/sace-pddl/05-stage4-safety-requirements.md:13-13` says PDDL supports hierarchical decomposition via HTN extensions or multi-level modelling.
- `doc/sace-pddl/05-stage4-safety-requirements.md:28-30,39-42` says the outputs and solution nodes include a completeness analysis showing sub-domain constraints jointly imply system constraints.
- `doc/sace-pddl/05-stage4-safety-requirements.md:43-51` uses ISO 34502 categories to justify the decomposition structure.
- `doc/sace-pddl/12-e2e-example.md:126-136` later concedes that proving joint implication requires a theorem prover or model checker.

**Why this matters**

There is an important difference between:

- using PDDL to organise requirements and provide traceability, and
- proving that the decomposition is complete, consistent, and semantically preserves the system-level safety requirement.

HTN-style structure or multi-level domain modelling can help document decomposition, but it does not by itself prove joint implication. Also, ISO 34502 risk-factor categories are useful for scenario derivation, but they do not automatically justify the architectural decomposition of safety requirements.

**Recommendation**

Position Stage 4 PDDL artefacts primarily as decomposition and traceability aids unless a stronger formal method is added. Align the stage-level claims with the more cautious wording already used in the end-to-end example.

### 5. Medium: The "single source of truth" argument is weaker than the docs claim once richer offline tools are introduced

**Evidence**

- `doc/sace-pddl/13-tooling-analysis.md:254-259` explicitly separates STRIPS runtime execution from richer offline analysis tooling.
- `doc/sace-pddl/13-tooling-analysis.md:263-267` then says all tools consume the same PDDL files "or minimal extensions", ensuring that the analyzed model is the executed model.
- `doc/sace-pddl/13-tooling-analysis.md:279-281` acknowledges the key assumption: the STRIPS execution domain must be semantically compatible with the richer analysis domains.

**Why this matters**

Once the assurance case depends on constrained planners, model checkers, temporal planners, or probabilistic models, the argument is no longer simply "the same model was analyzed and executed". Instead, the validity of the approach depends on a further claim:

- that the richer offline model is a conservative or otherwise sound extension of the execution model, and
- that properties proven offline continue to apply to the restricted runtime semantics.

That can be a valid approach, but only if it is treated as an explicit conformance or subsumption argument, not as a direct consequence of shared filenames or closely related PDDL sources.

**Recommendation**

Keep the separation-of-execution-and-analysis architecture, but soften the "single source of truth" claim. Replace it with a stronger requirement for:

- model correspondence,
- semantic compatibility,
- and explicit traceability between offline evidence models and runtime execution models.

## Overall assessment

The overall direction is credible as a **documentation and evidence-generation framework** for a SACE-oriented planning system. The strongest part of the approach is the recognition, especially in `13-tooling-analysis.md` and `12-e2e-example.md`, that different assurance questions require different tools.

The weakest part is that several stage documents still overstate what planning artefacts can prove. In particular, they blur the distinction between:

- existential plan feasibility,
- universal safety properties,
- completeness arguments,
- and residual-risk arguments.

As currently written, the documentation is strongest when read as:

- a structured way to model scenarios,
- generate formal scenario-specific evidence,
- organise assumptions,
- and connect offline analysis artefacts into a GSN safety case.

It is much less convincing when read as claiming that PDDL planning alone can close the SACE argument for hazard completeness, mitigation sufficiency, out-of-context recoverability, or residual-risk acceptability.

## Recommended next steps

1. Replace "proof" language in the stage documents with more precise terms such as "scenario-specific evidence", "feasibility evidence", or "partial formal evidence".
2. Explicitly separate "modeled hazard-space coverage" from "real-world hazard completeness" and keep the latter as an assumption requiring external support.
3. Recast state-space coverage metrics as confidence indicators unless a formal bridge to operational exposure and residual risk is added.
4. Align Stage 4 with the more careful wording already present in `12-e2e-example.md` about the need for stronger formal methods for joint implication.
5. Strengthen the documented correspondence argument between offline evidence models and runtime execution semantics.
