# Residual Risk Management

[Back to Index](00-index.md) | [Previous: Stage 8](09-stage8-verification.md) | [Next: Cross-Stage Summary](11-cross-stage-summary.md)

---

## Purpose

Residual risk management is a cross-cutting concern that draws on the outputs of all eight SACE stages. Its purpose is to:

1. Specify the acceptance criteria for residual risk
2. Systematically identify and evaluate potential functional insufficiencies and their triggering conditions
3. Define the expected system responses to each
4. Demonstrate that the residual risk after all mitigations is below the acceptance threshold

This section is driven primarily by ISO 21448, which makes residual risk evaluation central to achieving SOTIF.

## Relationship to SACE Stages

Residual risk management is not a single SACE stage but an integrating activity that consumes artefacts from across the process:

- **Stage 2** identifies hazardous scenarios
- **Stage 3** defines mitigations via the SOC
- **Stage 6** manages hazardous failures from design decisions
- **Stage 8** provides verification evidence

Residual risk management takes all of these and asks: *what remains unmitigated, and is it acceptable?* The answer feeds back into the top-level GSN safety argument as the final claim required before the system can be accepted as sufficiently safe.

---

## Key Activities

### 1. Define Residual Risk Acceptance Criteria

Before residual risk can be evaluated, the acceptance threshold must be defined. ISO 21448 requires that the combined area of known-unsafe scenarios (after mitigation) and unknown-unsafe scenarios be demonstrably below an acceptable level.

The acceptance criteria must be specific, measurable, and agreed with stakeholders. They may be expressed:

- **Quantitatively** -- e.g. a maximum probability of encountering an unmitigated hazardous scenario per operating hour
- **Qualitatively** -- e.g. all identified hazardous scenarios have a documented mitigation with verified evidence, and the unknown-unsafe scenario space has been explored to a defined coverage threshold

PDDL contributes by providing formal, quantifiable coverage metrics. The proportion of the reachable state space explored by the planner, the number of fault-injected domain variants exercised, and the number of ISO 34502 annex scenarios with corresponding PDDL verification results all feed into the acceptance criteria evaluation.

### 2. Identify and Evaluate Functional Insufficiencies

A functional insufficiency is a gap between what the system is specified to do and what it can actually achieve under real-world conditions. ISO 21448 requires systematic identification of these insufficiencies across the entire system.

For a PDDL-based autonomous system, functional insufficiencies manifest in several ways:

| Category | Description | Examples |
|----------|-------------|----------|
| **Predicate grounding insufficiencies** | The PDDL model assumes predicates are correctly grounded by real perception and communication systems. Any condition under which the real system cannot reliably determine the truth value of a predicate is an insufficiency. | Sensor degradation in adverse weather; EM interference affecting communications; ML model limitations under out-of-distribution inputs |
| **Action model insufficiencies** | The PDDL model assumes action effects faithfully represent physical outcomes. Any divergence between real actuation and modelled effects is an insufficiency. | Actuator saturation under extreme loads; control degradation in turbulence; timing violations in real-time execution |
| **State space insufficiencies** | The PDDL model necessarily abstracts the real world into a finite set of predicates and types. Any real-world condition outside the representational capacity of the model is an insufficiency. | This is the most subtle category and the hardest to identify, as it represents what the model cannot even express |

Each identified functional insufficiency must be evaluated for its potential to contribute to a hazardous scenario, considering the likelihood of manifestation (given the defined operating context) and the severity of the consequences.

### 3. Identify and Evaluate Triggering Conditions

A triggering condition is a specific condition in the operating environment that activates a functional insufficiency, causing the system to behave in a potentially hazardous way. ISO 21448 requires systematic identification for each functional insufficiency. ISO 34502 provides the structured framework through its three risk factor categories:

**Perception triggering conditions** -- environmental factors that degrade sensor or perception performance:
- Snow, fog, rain, dust, or sand interfering with camera or LIDAR systems
- Sun glare causing false negatives
- RF interference degrading radar returns
- Camouflaged or low-observable objects in the operating environment

**Judgement and planning triggering conditions** -- situations that expose limitations in decision-making logic:
- Novel combinations of environmental states not represented in the PDDL domain
- Conflicting mission objectives forcing the planner into unsafe trade-offs
- Adversarial behaviour by other agents that the planner's model does not anticipate

**Control triggering conditions** -- physical conditions that degrade actuation or vehicle control:
- Extreme wind gusts exceeding actuator authority
- Icing affecting aerodynamic surfaces
- Mechanical wear reducing control precision

**Misuse triggering conditions** -- ISO 21448 explicitly classifies reasonably foreseeable misuse as a triggering condition category:
- Operator commanding the system outside its defined ODD
- Incorrect mission parameter entry
- Overriding safety constraints without authorisation
- Reliance on the autonomous system beyond its assured capability envelope

Misuse conditions are modelled in PDDL by encoding the misuse as an action or predicate state and checking whether safety constraints can still be satisfied.

### 4. Define Expected Responses

For each identified functional insufficiency and triggering condition pair, the expected system response must be defined. PDDL contributes by verifying that the defined response is achievable.

| Response Type | Description | PDDL Verification |
|---------------|-------------|-------------------|
| **Continued safe operation** | Insufficiency is minor or system has redundancy | Plans satisfying all safety constraints exist even with the relevant predicate or action degraded |
| **Reduced operating domain** | System restricts autonomous capability to a subset of the ODM | State transition to a restricted problem with tighter goal constraints |
| **Safe state transition** | System transitions to a defined safe state (loiter, return-to-base, operator handoff) | Reachability of the safe state from the triggered condition (as in Stage 7) |
| **Mission abort** | System immediately terminates mission and enters minimal-risk condition | Abort action sequence is always available |

Where PDDL cannot verify that a defined response achieves a safe state, this constitutes an unresolvable residual risk that must be explicitly documented and accepted.

### 5. Evaluate Residual Risk

After all functional insufficiencies have been identified, triggering conditions enumerated, and responses defined, the residual risk is evaluated against the acceptance criteria. Residual risk arises from three sources:

1. **Known-unsafe scenarios with verified mitigations** -- Residual risk is the probability that the mitigation fails. PDDL provides evidence of mitigation effectiveness through constrained plan traces, but the residual depends on the fidelity of the PDDL model to reality (captured as model assumption confidence).

2. **Known-unsafe scenarios with incomplete mitigations** -- Where PDDL analysis reveals that a mitigation does not fully eliminate the hazardous state (e.g. a safe-state is reachable but with non-zero probability of transient unsafe behaviour during transition), the residual risk must be explicitly quantified.

3. **Unknown-unsafe scenarios** -- Residual risk from scenarios not yet discovered. PDDL state-space coverage metrics provide evidence about how much of the *modelled* scenario space has been explored. The acceptance argument combines two elements: (a) coverage metrics showing that the unexplored region of the abstract state space is small, and (b) a confidence argument that the model's state space is a meaningful proxy for real-world scenario diversity. The latter depends on model fidelity evidence and the systematic derivation of the model from ISO 34502 risk factor categories. Abstract state coverage alone does not directly quantify real-world exposure-weighted risk.

---

## PDDL Contribution Summary

PDDL contributes to residual risk management at four levels:

1. Provides the **formal model** against which functional insufficiencies are identified -- every model assumption is a potential insufficiency if violated
2. Enables **systematic triggering condition exploration** through fault-injected domain variants parameterised by ISO 34502 risk factors
3. **Verifies** that defined responses achieve safe states
4. Provides **quantitative coverage metrics** over the abstract state space, which support -- but do not alone establish -- the argument that residual risk from unknown-unsafe scenarios is acceptable. A confidence argument linking model coverage to real-world risk is also required

Critically, PDDL also reveals its own limits as residual risk contributors. The assumptions that the PDDL domain is a faithful abstraction, that the predicate set is complete, and that action effects are accurate are themselves sources of residual risk. These must be captured as GSN assumption nodes with explicit confidence arguments.

---

## GSN Mapping

Residual risk management contributes to the top-level GSN safety argument. It does not have a single SACE argument pattern but draws on and integrates multiple patterns.

| GSN Element | Content |
|-------------|---------|
| **Goal** | The residual risk from the operation of the AS is below the defined acceptance criteria |
| **Strategy** | Argue that: (a) known-unsafe scenarios are mitigated with verified evidence, (b) the functional insufficiency and triggering condition analysis is systematic and complete, (c) expected responses are verified as achieving safe states, and (d) the unknown-unsafe scenario space has been sufficiently explored. Decompose into sub-goals for each source of residual risk |
| **Context** | The residual risk acceptance criteria. The complete set of identified functional insufficiencies and triggering conditions. The ISO 21448 four-quadrant scenario classification. The ISO 34502 risk factor coverage achieved |
| **Solution** | PDDL state-space coverage reports quantify exploration completeness. Constrained plan traces demonstrate mitigation effectiveness for known-unsafe scenarios. Fault-injected domain variant results demonstrate response verification for each triggering condition. Misuse scenario analysis results demonstrate safe behaviour under foreseeable misuse conditions |
| **Assumption** | The PDDL model is a sufficiently faithful abstraction. The functional insufficiency identification is complete. The triggering condition catalogue derived from ISO 34502 risk factors covers the relevant scenario space. Each assumption requires a confidence argument |
| **Justification** | ISO 21448 provides the conceptual framework for residual risk evaluation through the four-quadrant model and the requirement to demonstrate that unknown-unsafe area is acceptably small. ISO 34502 provides the systematic scenario derivation methodology that justifies the claim of completeness in triggering condition identification |

---

## ISO 21448 (SOTIF) Impact

Residual risk management is the heart of ISO 21448. The standard requires that the development process systematically reduces the area of known-unsafe and unknown-unsafe scenarios until the residual is acceptable. PDDL operationalises this by providing:

- Formal identification of known-unsafe scenarios through state-space exploration
- Verified mitigations through constrained planning
- Coverage metrics over the abstract state space that inform (but do not directly bound) the unknown-unsafe residual -- the link between model coverage and real-world risk requires a confidence argument grounded in model fidelity evidence

The acceptance criteria must account for both the probability of encountering an unmitigated scenario and the severity of the potential outcome, consistent with ISO 21448's risk-based approach.

ISO 21448 also requires that reasonably foreseeable misuse be treated as a triggering condition rather than excluded from the analysis. PDDL models must include misuse scenarios -- operator actions that violate intended use patterns -- and demonstrate that safety constraints are either maintained or that the system transitions to a safe state.

## ISO 34502 Impact

ISO 34502's risk factor categories provide the structured framework for triggering condition identification. By systematically combining perception, judgement, and control risk factors, the triggering condition catalogue achieves the completeness that the residual risk argument requires. ISO 34502 Annex K (constrained random testing to identify unknown critical scenarios) directly supports the unknown-unsafe residual risk argument.

The ISO 34502 annex structure also provides a natural checklist: every Annex B traffic disturbance, Annex C perception disturbance, and Annex D control disturbance should have been considered as a potential triggering condition for each identified functional insufficiency.

---

## UAV Example

**Functional insufficiency:** the camera-based obstacle detector cannot reliably detect obstacles when snow is present on the lens or when operating in heavy snowfall conditions.

**Triggering condition:** snow accumulation on sensor housing (ISO 34502 Annex C perception disturbance).

**Expected response:** the system detects degraded camera confidence via a built-in test, activates LIDAR-only obstacle detection, and transitions to a reduced operating domain with increased separation distances.

**PDDL verification:** a fault-injected domain variant sets `(camera-degraded)` true, restricts obstacle detection to LIDAR predicates only, and tightens the minimum-separation constraint. The planner demonstrates that missions within the reduced ODM are achievable with the tightened constraint.

**Misuse triggering condition:** the operator commands the UAV to operate in airspace classified as denied. The PDDL domain includes an action `(override-airspace-restriction ?zone)` available only when `(operator-override-active)` is true. A trajectory constraint `(always (implication (denied-airspace ?zone) (not (in ?uav ?zone))))` is maintained regardless of operator override status -- the planner will not produce plans entering denied airspace even under operator command. The safety constraint is hardcoded and not overridable. Residual risk: the operator may fly the system manually (bypassing the planner entirely). This residual is managed through operational procedures and documented as an accepted residual risk.

**Residual risk evaluation:** after all mitigations, the state-space coverage report shows that 94% of reachable states have been explored with no unmitigated safety violations. The remaining 6% corresponds to extreme multi-factor degradation states (simultaneous camera, LIDAR, and GPS failure) for which the system enters immediate mission abort. The acceptance argument references ISO 21448's requirement for the unknown-unsafe area to be acceptably small, supported by ISO 34502 systematic risk factor coverage as a justification for the exploration methodology.

---

[Next: Cross-Stage Summary](11-cross-stage-summary.md)

