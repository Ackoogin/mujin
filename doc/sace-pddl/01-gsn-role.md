# GSN and Its Role in SACE

[Back to Index](00-index.md)

---

## GSN Elements

GSN uses a small set of graphical elements to build structured safety arguments. Understanding these is essential for seeing how PDDL artefacts fit into the safety case.

| Element | Shape | Description | Example |
|---------|-------|-------------|---------|
| **Goal** | Rectangle | A claim about the system that must be shown to be true. Goals decompose into sub-goals. | "The AS is acceptably safe in its defined operating context." |
| **Strategy** | Parallelogram | The reasoning step that explains how a goal is decomposed into sub-goals. | "Argument over each identified hazardous scenario." |
| **Solution** | Circle | A reference to an item of evidence that directly supports a goal. **This is where PDDL artefacts attach.** | "Plan validity certificate for scenario X." |
| **Context** | Rounded rectangle | Information that scopes or qualifies a goal or strategy. | "Defined operating context [C1]" or "ISO 21448 four-quadrant scenario classification." |
| **Assumption** | Oval with A | A statement taken to be true without proof. | "PDDL domain is a faithful abstraction of the real system." |
| **Justification** | Oval with J | A statement providing the rationale for a strategy. | "ISO 34502 risk factor decomposition ensures systematic scenario coverage." |
| **Assurance Claim Point** | Black square | Used in SACE specifically to mark points where additional confidence arguments are required. These are characteristic of SACE patterns and indicate where the argument must demonstrate not just that something is true, but that there is justified confidence in the claim. | — |

---

## SACE Argument Pattern Structure

SACE defines a top-level safety argument that decomposes into two principal claims: that the AS is safe within its defined operating context, and that it remains safe when outside that context. The first claim decomposes further through an argument that all hazardous scenarios have been identified and sufficiently mitigated. Each SACE stage produces a named GSN argument pattern that slots into this hierarchy.

### SACE Argument Patterns

| Code | Pattern Name | Produced In | Argument |
|------|-------------|-------------|----------|
| **[G]** | Operating Context Assurance | Stage 1 | The defined operating context is sufficient and correctly specified |
| **[I]** | Hazardous Scenarios Identification | Stage 2 | All hazardous scenarios have been identified and are complete |
| **[N]** | SOC Assurance | Stage 3 | The Safe Operating Concept sufficiently mitigates all identified hazardous scenarios |
| **[S]** | Safety Requirements Assurance | Stage 4 | Safety requirements are correctly decomposed, complete, and traceable |
| **[X]** | Design Assurance | Stage 5 | The design at each tier satisfies the allocated safety requirements |
| **[DD]** | Hazardous Failures Management | Stage 6 | Potential hazardous failures introduced through design decisions have been identified and are acceptably managed |
| **[PP]** | Out of Context Operation | Stage 7 | The system remains safe when outside its defined operating context |
| **[UU]** | Verification Assurance | Stage 8 | Verification evidence is appropriate and trustworthy |

---

## How PDDL Artefacts Map to GSN Elements

PDDL artefacts primarily populate GSN solution nodes — they are the formal evidence that supports safety claims. However, PDDL also contributes to other GSN elements.

### PDDL as Solution Nodes

Plan traces, constraint satisfaction proofs, state-space coverage reports, and reachability analyses are all items of evidence that attach as solutions to specific goals in the SACE patterns.

### PDDL as Context Nodes

The PDDL domain file itself serves as a context node, providing the formal definition of autonomous capabilities and operating context that scopes the argument.

### PDDL Informing Strategy Nodes

The choice to use formal planning-based verification as part of the safety argument is itself a strategy. This strategy node would be justified by reference to ISO 34502's scenario-based evaluation approach and ISO 21448's requirement for systematic scenario exploration.

### PDDL Generating Assumption Nodes

Every PDDL model involves abstraction assumptions: that the predicate set is sufficient, that action models are faithful, that the state space is adequately bounded. These must be explicitly captured as GSN assumption nodes, with associated confidence arguments explaining why each assumption is reasonable.

### ISO Standards in GSN

The ISO standards primarily populate strategy, context, and justification nodes. ISO 21448's four-quadrant model justifies the strategy of decomposing the hazard argument into known and unknown scenario sub-goals. ISO 34502's risk factor categories justify the strategy of decomposing scenario identification along perception, judgement, and control axes.

---

[Next: Stage 1 — Operating Context Assurance](02-stage1-operating-context.md)
