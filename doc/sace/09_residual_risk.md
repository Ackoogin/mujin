# SACE PDDL Contributions — Residual Risk Management

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**Cross-cutting activity — integrates artefacts from all eight SACE stages.**

---

## Purpose

Residual risk management is a cross-cutting concern that draws on the outputs of all eight SACE stages. Its purpose is to:

1. Specify the acceptance criteria for residual risk
2. Systematically identify and evaluate potential functional insufficiencies and their triggering conditions
3. Define the expected system responses to each
4. Demonstrate that the residual risk after all mitigations is below the acceptance threshold

This section is driven primarily by ISO 21448, which makes residual risk evaluation central to achieving SOTIF.

---

## Relationship to SACE Stages

Residual risk management is not a single SACE stage but an **integrating activity** that consumes artefacts from across the process:

| SACE Stage | Contribution to Residual Risk Management |
|-----------|------------------------------------------|
| Stage 2 | Identifies hazardous scenarios |
| Stage 3 | Defines mitigations via the SOC |
| Stage 6 | Manages hazardous failures from design decisions |
| Stage 8 | Provides verification evidence |
| **All stages** | Residual risk management asks: what remains unmitigated, and is it acceptable? |

The answer feeds back into the top-level GSN safety argument as the final claim required before the system can be accepted as sufficiently safe.

---

## Key Activity 1: Define Residual Risk Acceptance Criteria

Before residual risk can be evaluated, the acceptance threshold must be defined. ISO 21448 requires that the combined area of known-unsafe scenarios (after mitigation) and unknown-unsafe scenarios be demonstrably below an acceptable level.

Acceptance criteria may be expressed as:
- **Quantitative:** maximum probability of encountering an unmitigated hazardous scenario per operating hour
- **Qualitative:** all identified hazardous scenarios have a documented mitigation with verified evidence, and the unknown-unsafe scenario space has been explored to a defined coverage threshold

**PDDL contribution:** Formal, quantifiable coverage metrics — proportion of reachable state space explored, number of fault-injected domain variants exercised, number of ISO 34502 annex scenarios with corresponding PDDL verification results — feed into the acceptance criteria evaluation.

---

## Key Activity 2: Identify and Evaluate Functional Insufficiencies

A functional insufficiency is a gap between what the system is specified to do and what it can actually achieve under real-world conditions. For a PDDL-based autonomous system, functional insufficiencies manifest in three categories:

### Predicate Grounding Insufficiencies
The PDDL model assumes predicates like `(obstacle-at ?wp)` or `(comms-available)` are correctly grounded by the real perception and communication systems. Any condition under which the real system cannot reliably determine the truth value of a predicate is a functional insufficiency.

*Examples:* sensor degradation in adverse weather; electromagnetic interference; ML model limitations under out-of-distribution inputs.

### Action Model Insufficiencies
The PDDL model assumes that action effects faithfully represent physical outcomes. Any condition under which real actuation diverges from the modelled effect is a functional insufficiency.

*Examples:* actuator saturation under extreme loads; control performance degradation in turbulence; timing violations in real-time execution.

### State Space Insufficiencies
The PDDL model necessarily abstracts the real world into a finite set of predicates and types. Any real-world condition that falls outside the representational capacity of the model is a functional insufficiency. This is the most subtle category — it represents what the model cannot even express.

---

## Key Activity 3: Identify and Evaluate Triggering Conditions

A triggering condition is a specific condition in the operating environment that activates a functional insufficiency. ISO 34502 provides the structured framework for systematic triggering condition identification:

### Perception Triggering Conditions (ISO 34502 Annex C)
Environmental factors that degrade sensor or perception performance.

*UAV examples:* snow/fog/rain/dust interfering with camera or LIDAR; sun glare causing false negatives; RF interference degrading radar; camouflaged or low-observable objects.

### Judgement and Planning Triggering Conditions (ISO 34502 Annex B)
Situations that expose limitations in decision-making logic.

*UAV examples:* novel combinations of environmental states not in the PDDL domain; conflicting mission objectives forcing unsafe trade-offs; adversarial behaviour by other agents not anticipated in the model.

### Control Triggering Conditions (ISO 34502 Annex D)
Physical conditions that degrade actuation or vehicle control.

*UAV examples:* extreme wind gusts exceeding actuator authority; icing affecting aerodynamic surfaces; mechanical wear reducing control precision.

### Misuse Triggering Conditions (ISO 21448 requirement)
ISO 21448 explicitly classifies reasonably foreseeable misuse as a triggering condition category. PDDL models misuse by encoding the misuse as an action or predicate state and checking whether safety constraints can still be satisfied.

*UAV examples:* operator commanding system outside its defined ODD; incorrect mission parameter entry; overriding safety constraints without authorisation; reliance on the autonomous system beyond its assured capability envelope.

---

## Key Activity 4: Define Expected Responses

For each functional insufficiency and triggering condition pair, the expected system response must be defined. PDDL contributes by verifying that the defined response is achievable:

| Response Type | When Used | PDDL Verification |
|--------------|-----------|-------------------|
| **Continued safe operation** | Minor insufficiency or redundancy available | Plans satisfying all constraints exist with degraded predicate/action |
| **Reduced operating domain** | System restricts autonomous capability | State transition to restricted problem with tighter constraints |
| **Safe state transition** | Moderate insufficiency | Safe state reachability from triggered condition (Stage 7 analysis) |
| **Mission abort** | Severe insufficiency | Abort action sequence is always available |

Where PDDL cannot verify that a defined response achieves a safe state, this constitutes an **unresolvable residual risk** that must be explicitly documented and accepted.

---

## Key Activity 5: Evaluate Residual Risk

After all functional insufficiencies have been identified, triggering conditions enumerated, and responses defined, residual risk is evaluated against the acceptance criteria. Residual risk arises from three sources:

| Source | Nature | PDDL Evidence |
|--------|--------|---------------|
| **Known-unsafe with verified mitigations** | Residual = probability that mitigation fails; depends on model fidelity | Constrained plan traces demonstrating mitigation effectiveness |
| **Known-unsafe with incomplete mitigations** | Residual = probability of transient unsafe behaviour during transition | Failure-mode variant results quantifying unsafe exposure |
| **Unknown-unsafe scenarios** | Residual = probability of encountering an undiscovered scenario | State-space coverage metrics + ISO 34502 systematic coverage argument |

---

## GSN Mapping

Residual risk management integrates multiple SACE patterns and contributes to the top-level safety argument.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | The residual risk from the operation of the AS is below the defined acceptance criteria | Top-level safety claim |
| **Strategy** | Argue that: (a) known-unsafe scenarios are mitigated with verified evidence, (b) functional insufficiency and triggering condition analysis is systematic and complete, (c) expected responses are verified as achieving safe states, (d) the unknown-unsafe scenario space has been sufficiently explored | Multi-source argument |
| **Context** | Residual risk acceptance criteria; complete set of identified functional insufficiencies and triggering conditions; ISO 21448 four-quadrant classification; ISO 34502 risk factor coverage achieved | Acceptance criteria as context |
| **Solution** | State-space coverage reports (exploration completeness); constrained plan traces (mitigation effectiveness); fault-injected variant results (response verification); misuse scenario analysis results | PDDL artefacts as solution nodes |
| **Assumption** | PDDL model is sufficiently faithful; functional insufficiency identification is complete; triggering condition catalogue covers the relevant scenario space — each requires a confidence argument | Three explicit assumptions |
| **Justification** | ISO 21448 provides the four-quadrant residual risk framework; ISO 34502 provides the systematic scenario derivation justifying completeness claims | Standards as justification nodes |

---

## ISO 21448 (SOTIF) Impact

Residual risk management is the heart of ISO 21448. The standard requires that the development process systematically reduces the area of known-unsafe and unknown-unsafe scenarios until the residual is acceptable.

PDDL operationalises this by providing:
1. **Formal identification** of known-unsafe scenarios through state-space exploration
2. **Verified mitigations** through constrained planning
3. **Coverage metrics** that bound the unknown-unsafe residual

ISO 21448 also requires that **reasonably foreseeable misuse** be treated as a triggering condition rather than excluded from the analysis. PDDL models must include misuse scenarios and demonstrate that safety constraints are either maintained or that the system transitions to a safe state.

---

## ISO 34502 Impact

ISO 34502's risk factor categories provide the structured framework for triggering condition identification. The annex structure provides a completeness checklist:

- Every **Annex B** traffic disturbance → considered as planning/judgement triggering condition
- Every **Annex C** perception disturbance → considered as perception triggering condition
- Every **Annex D** control disturbance → considered as control triggering condition
- **Annex K** (constrained random testing) → implemented as randomised PDDL problem generation for unknown-unsafe residual argument

---

## UAV Example

### Snow-induced Camera Degradation

**Functional insufficiency:** Camera-based obstacle detector cannot reliably detect obstacles in heavy snowfall or with snow on lens.

**Triggering condition:** Snow accumulation on sensor housing (ISO 34502 Annex C).

**Expected response:** System detects degraded camera confidence via built-in test, activates LIDAR-only obstacle detection, transitions to reduced operating domain with increased separation distances.

**PDDL verification:** Fault-injected variant sets `(camera-degraded)` true, restricts obstacle detection to LIDAR predicates only, tightens the minimum-separation constraint. Planner demonstrates missions within the reduced ODM are achievable.

---

### Misuse: Denied Airspace Entry Command

**Triggering condition:** Operator commands UAV to operate in airspace classified as denied.

**PDDL implementation:**
```pddl
;; Action available only when operator override is active
(:action override-airspace-restriction
  :parameters (?zone - airspace-zone)
  :precondition (operator-override-active)
  :effect (override-active ?zone))

;; Safety constraint: hardcoded, not overridable
(:constraints
  (always
    (implication (denied-airspace ?zone)
                 (not (in ?uav ?zone)))))
```

The trajectory constraint is maintained regardless of operator override status — the planner will not produce plans entering denied airspace even under operator command.

**Residual risk:** Operator may attempt manual flight (bypassing the planner entirely). This residual is managed through operational procedures and documented as an accepted residual risk with severity and probability assessment.

---

### Residual Risk Evaluation Summary

| Source | Coverage | Outcome |
|--------|----------|---------|
| Mitigated known-unsafe scenarios | 100% of Stage 2 scenarios have verified mitigations | PASS |
| Incomplete mitigations | 2 scenarios with transient exposure documented | Risk accepted with probability assessment |
| Unknown-unsafe state space | 94% of reachable states explored | Coverage report supports acceptance argument |
| Extreme multi-failure states (6%) | Enter immediate mission abort | Abort action always available (verified) |

---

## Evidence Checklist

- [ ] Residual risk acceptance criteria defined and agreed with stakeholders
- [ ] Functional insufficiency register produced covering all three categories
- [ ] Triggering conditions identified per ISO 34502 Annex B, C, D structure
- [ ] Misuse scenarios identified and analysed per ISO 21448 requirement
- [ ] Expected responses defined for each insufficiency/triggering condition pair
- [ ] PDDL response verification completed for each expected response
- [ ] State-space coverage report produced against coverage target
- [ ] Residual risk evaluated against acceptance criteria
- [ ] Unresolvable residuals documented with explicit risk acceptance
- [ ] All three GSN assumption nodes documented with confidence arguments

---

*Previous: [Stage 8 — Verification Assurance](08_stage8_verification.md)*
*Next: [Cross-Stage Summary](10_cross_stage_summary.md)*
