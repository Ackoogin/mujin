# Graphical Autonomy Authoring Tool Options

**Purpose:** Assess practical options for building a stakeholder-friendly tool for graphical authoring and validation of autonomy behaviours, using the existing AME DevEnv as the starting point and avoiding GPL/LGPL/AGPL and similar licence risk.

**Status:** Research and recommendation. No implementation started.

---

## 1. Executive Summary

There is no obvious off-the-shelf, licence-clean, stakeholder-friendly PDDL workbench that fits AME well enough to adopt directly. If we want a usable graphical authoring and validation tool, we should assume we will need to build one ourselves.

The best path is to **build on the existing DevEnv backend**, but **move the rich authoring surface to a web-style canvas and editor** rather than trying to force everything into the current Dear PyGui text-centric UI. In practice, that means:

- keep the current Python/ROS2/Foxglove/AME integration as the backend and local launcher
- introduce a richer graphical authoring UI for actions, predicates, objects, goals, scenarios, and plan views
- treat **generated PDDL as an artefact**, not the primary user-facing authoring surface
- use AME itself, plus optional offline validators, to check syntactic, semantic, planning, and execution-level correctness

The recommended product direction is therefore:

1. **Near term:** deliver a structured authoring overlay in the current DevEnv so non-experts can define scenarios and domain elements without hand-editing PDDL.
2. **Mainline solution:** evolve DevEnv into a **hybrid desktop workbench** with a local embedded web UI.
3. **Long term:** expose the same backend through a browser-first deployment if stakeholder review and model sharing become more important than desktop packaging.

---

## 2. Starting Point

The current DevEnv already gives us several important foundations:

- a local desktop application under [`subprojects/AME/tools/devenv`](../../../subprojects/AME/tools/devenv)
- live connection to the AME ROS2 nodes and Foxglove event streams, documented in [`subprojects/AME/doc/guides/devenv_ros2_quickstart.md`](../../../subprojects/AME/doc/guides/devenv_ros2_quickstart.md)
- an existing PDDL editor tab in [`subprojects/AME/tools/devenv/ui/pddl_editor_tab.py`](../../../subprojects/AME/tools/devenv/ui/pddl_editor_tab.py)
- permissive licensing already recorded in [`doc/THIRD_PARTY_LICENSES.md`](../../THIRD_PARTY_LICENSES.md)

Today, however, the PDDL experience is still fundamentally:

- file-oriented
- text-oriented
- developer-oriented
- only lightly validated in the client

That is a good engineering baseline, but it is not yet a tool that systems engineers or stakeholders will find comfortable for routine autonomy behaviour authoring.

---

## 3. Design Constraints

The solution should satisfy the following constraints.

### 3.1 User Constraints

- Non-programmers must be able to create and review autonomy behaviour definitions.
- The tool should hide raw PDDL by default and make it optional.
- It should support both **authoring** and **validation**, not just diagram drawing.

### 3.2 Technical Constraints

- Build on the existing DevEnv and AME backend rather than replacing them outright.
- Work well on Windows developer/operator machines.
- Support offline or local-only use.
- Reuse the existing AME planner, WorldModel, compiler, execution, and observability pipeline.

### 3.3 Licensing Constraints

- Avoid GPL, LGPL, AGPL, and similar copyleft obligations.
- Prefer **MIT**, **BSD**, **Apache-2.0**, **BSL-1.0**, and **public domain** dependencies.
- Treat custom licences, weak copyleft, and commercial-runtime-key models as exceptions requiring deliberate approval.

---

## 4. What the Tool Actually Needs to Do

To be genuinely useful, the tool needs more than a pretty graph canvas. It should support:

- **Domain authoring:** actions, parameters, predicates, effects, invariants, constraints
- **Problem/scenario authoring:** objects, initial facts, goals, environmental assumptions
- **Behaviour validation:** syntax checks, semantic checks, reachability/planning checks, execution checks
- **Scenario review:** nominal and off-nominal cases, expected outcomes, keystone scenarios
- **Traceability:** link scenarios, generated PDDL, plans, BTs, and observed execution runs
- **Expert escape hatch:** show generated PDDL and allow expert edits when needed

This means the real product problem is not "a PDDL editor". It is a **model-driven autonomy authoring and validation workbench** that happens to emit and consume PDDL.

---

## 5. Product Options

### 5.1 Option 1: Extend the Current Dear PyGui DevEnv In Place

### Shape

Keep the current Python + Dear PyGui application and add:

- form-based editors for actions, predicates, objects, and scenarios
- table views for domain/problem elements
- guided wizards for common scenario patterns
- generated graph previews for causal relations or mission flow
- deeper validation buttons that call into AME

### Source of Truth

PDDL remains the canonical model. The graphical and form-based UI acts as a higher-level projection over it.

### Third-Party Elements

- **Dear PyGui** (MIT) is already in use in DevEnv
- **VAL** could be added as an optional offline validator later; the current VAL repository is BSD-3-Clause

### Pros

- Lowest disruption to the current codebase
- Reuses the existing packaging, launch scripts, and backend integration
- Fastest route to a first non-text workflow
- No major new runtime stack

### Cons

- Dear PyGui is not the most natural foundation for a polished, highly interactive graph editor
- Diagramming, docking, rich text editing, and complex canvas behaviour will be harder to make feel modern
- The result is likely to remain more "developer tool" than "stakeholder workbench"

### Best Use

- Short-term pilot
- Internal engineering users
- A first step before a richer UI rework

### Indicative Effort

- **Low to medium**

---

### 5.2 Option 2: Hybrid DevEnv Desktop App With Embedded Web Authoring UI

### Shape

Keep DevEnv as the local Python launcher/backend, but embed a web-based front end inside the desktop tool.

The Python side remains responsible for:

- ROS2 and Foxglove connectivity
- AME planner/executor integration
- local file access
- local project storage
- validation orchestration

The web side becomes responsible for:

- graphical authoring
- structured forms and tables
- side-by-side advanced text editing
- diagram export and presentation-grade views

### Source of Truth

A **structured internal model** becomes canonical, for example JSON or SQLite-backed project data. PDDL is generated from that model and can also be imported into it where feasible.

### Candidate Stack

| Element | Role | Licence |
|---------|------|---------|
| `pywebview` | Native desktop shell hosting local HTML UI | BSD-3-Clause |
| `FastAPI` | Local backend API from Python to browser UI | MIT |
| `uvicorn` | Local ASGI server | BSD-3-Clause |
| `React Flow` | Editable node/edge canvas for authoring | MIT |
| `dagre` | Automatic graph layout | MIT |
| `Monaco Editor` | Advanced PDDL/code pane for expert users | MIT |
| `Mermaid` | Read-only exported diagrams in docs/reports | MIT |
| `VAL` | Optional external plan/model validation | BSD-3-Clause |
| `SQLite` | Local project/scenario store | Public Domain |

### Pros

- Best balance of usability, engineering reuse, and licence cleanliness
- Lets us keep the current AME/ROS2 integration where it already works
- Gives us a modern graph and forms experience without replatforming the whole backend
- Suitable for both systems engineers and developers
- Creates a clean path to a future browser-first deployment

### Cons

- Introduces a second UI stack
- Requires frontend build tooling and packaging discipline
- Round-trip import/export between generated PDDL and structured models must be designed carefully

### Best Use

- Mainline implementation
- Stakeholder-facing workbench
- Path to long-term productisation

### Indicative Effort

- **Medium**

---

### 5.3 Option 3: Browser-First Workbench, Served Locally by DevEnv

### Shape

Turn DevEnv into a backend service and launcher, and serve the full authoring tool into the user's normal browser.

This can still be a local tool at first:

- DevEnv starts the backend
- the browser opens to `localhost`
- AME remains local
- storage remains local

Later, the same architecture can be moved to a shared intranet deployment if required.

### Source of Truth

Structured internal model, with generated PDDL and execution artefacts.

### Candidate Stack

Largely the same as Option 2, but without the embedded desktop shell requirement.

### Pros

- Best future path for collaboration, review, and organisational rollout
- Simplest UI technology story once built
- Easiest route to later integration with web portals, Cameo-adjacent workflows, or review dashboards

### Cons

- Slightly less "single desktop app" in feel
- Packaging and offline expectations must be managed carefully
- Authentication/hosting questions appear earlier if the tool grows beyond local use

### Best Use

- If the tool is expected to become a shared engineering platform rather than a local developer utility

### Indicative Effort

- **Medium**

---

### 5.4 Option 4: Full Visual Programming Studio

### Shape

Make the primary user experience a node-based behaviour graph rather than forms plus structured model tables.

This is the most "approachable-looking" option on first contact. Users manipulate blocks, nodes, and connections rather than PDDL constructs.

### Source of Truth

The graph itself becomes canonical, and PDDL plus validation artefacts are compiled from it.

### Candidate Libraries

- **LiteGraph.js** is MIT-licensed and includes its own editor and JSON export
- **Drawflow** is MIT-licensed, dependency-free, and supports import/export

### Pros

- Highly visual and presentation-friendly
- Good for mission flow demonstrations and review workshops
- Can look very approachable for non-programmers

### Cons

- Highest semantic risk: node/dataflow tooling does not naturally map to planning semantics
- Easy to create a visually appealing but formally weak model
- Strong temptation to invent a second planning language accidentally
- Harder to preserve the clean relationship between action models, scenario models, and generated PDDL

### Best Use

- Mission storyboard layer
- Plan review layer
- Optional visualisation layer on top of a stronger structured model

### Indicative Effort

- **Medium to high**

---

## 6. Source-of-Truth Strategies

Independently of UI choice, we need to decide what is canonical.

### 6.1 PDDL-First

Users ultimately edit PDDL, with forms and graphs as helpers.

### Benefits

- Minimal semantic mismatch
- Works naturally with existing AME code
- Easy to inspect and diff

### Drawbacks

- Hard to make truly approachable
- Round-tripping from formatted PDDL back into rich UI structures is fragile

### 6.2 Structured Model First

Users edit a typed internal model; PDDL is generated from it.

### Benefits

- Best fit for stakeholder-friendly authoring
- Easier validation before PDDL generation
- Easier to attach metadata such as rationale, constraints, status, ownership, and scenario tags

### Drawbacks

- Requires clear code generation rules
- Importing arbitrary existing PDDL into the structured model becomes a separate problem

### 6.3 Graph First

Users edit a node graph as the authoritative model.

### Benefits

- Maximum visual appeal

### Drawbacks

- Most likely to drift from planning semantics
- Hardest to make rigorous

### Recommendation

Use **Structured Model First** as the main target architecture. Keep PDDL visible as a generated expert-facing artefact.

---

## 7. Validation Architecture

The tool should validate at several levels, not just "does this parse?".

### 7.1 Level 1: Authoring-Time Structural Checks

- missing names, types, parameters, or references
- undeclared predicates or objects
- malformed action signatures
- contradictory or incomplete scenario inputs

### 7.2 Level 2: PDDL Generation and Parse Checks

- generate domain/problem PDDL from the internal model
- parse it using the AME parser
- surface parser errors back into the structured UI

### 7.3 Level 3: Planning Feasibility Checks

- call the AME planner against selected scenarios
- explain `plan found`, `no plan`, or `model invalid`
- show unsatisfied preconditions, unreachable goals, and action coverage gaps where possible

### 7.4 Level 4: External Validation

- optionally validate plans/models with **VAL**, whose current repository is BSD-3-Clause
- use this as a second opinion and broader standards check, especially for interchange and assurance workflows

### 7.5 Level 5: Behaviour Execution Validation

- run the generated behaviour through AME
- inspect generated plans, BT structure, and runtime events
- compare actual execution traces against expected scenario outcomes

### 7.6 Level 6: Scenario Regression Pack

- save keystone scenarios as reusable regression assets
- rerun them automatically after model changes
- use them as both engineering and assurance evidence

---

## 8. Candidate UI/Graph Libraries

### 8.1 Recommended Candidates

| Library | Licence | Why it is interesting | Main concern | Recommendation |
|---------|---------|-----------------------|--------------|----------------|
| **React Flow** | MIT | Popular node-based UI library; good fit for editable authoring canvases | Requires a frontend stack | **Best default choice** |
| **Monaco Editor** | MIT | Strong expert-mode text editing beside graphical authoring | Browser-only component | **Use for advanced mode** |
| **dagre** | MIT | Simple automatic graph layout | Layout only, not an editor | **Use with React Flow** |
| **Cytoscape.js** | MIT | Strong graph visualisation/analysis; production-proven | Better viewer/analysis tool than authoring canvas | **Use for dependency/analysis views** |
| **Mermaid** | MIT | Good for generated documentation and review diagrams | Text-to-diagram, not interactive authoring | **Use for export/report views** |
| **LiteGraph.js** | MIT | Node graph editor included; JSON export; no heavy dependencies | Semantics skew toward visual programming/dataflow | **Use only if graph-first** |
| **Drawflow** | MIT | Simple, dependency-free, fast to stand up | Lighter-weight and less semantically rich | **Prototype option only** |
| **VAL** | BSD-3-Clause | Well-known plan/model validator | Additional toolchain complexity | **Good optional validator** |

### 8.2 Libraries to Avoid or Treat as Exceptions

| Library | Licence position | Why not default |
|---------|------------------|-----------------|
| **JointJS** | MPL-2.0 for community core, commercial for JointJS+ | Weak copyleft/commercial split is avoidable; not aligned with the default permissive-only goal |
| **GoJS** | Commercial licence key model | Adds procurement/runtime licensing friction |
| **yFiles** | Commercial licensing | Strong product, but contrary to the goal of staying with clean permissive components |
| **bpmn-js** | Uses the `bpmn.io` licence rather than a simple MIT/BSD/Apache licence | Would need legal review before adoption |
| **GPL/LGPL/AGPL planning tools or UI elements** | Copyleft | Excluded by policy |

---

## 9. Recommended Technical Direction

### 9.1 Recommendation

The best overall route is:

#### Product Architecture

- **Option 2:** Hybrid DevEnv desktop app with embedded web authoring UI

#### Canonical Model

- **Structured model first**

#### Core Validation Engine

- **AME itself first**, with optional **VAL** as an external validator

#### Core UI Stack

- `pywebview` + `FastAPI` + `uvicorn` + `React Flow` + `dagre` + `Monaco`

#### Visualisation Add-Ons

- `Cytoscape.js` for read-only graph analysis views
- `Mermaid` for exported documentation/report diagrams

### 9.2 Why This Combination Wins

It gives us:

- a local desktop experience
- a richer, modern UI than Dear PyGui alone
- continuity with the current DevEnv and AME runtime
- low licence risk
- a straightforward path from developer utility to stakeholder-facing tool
- a clean way to keep PDDL formalism while hiding it from most users

---

## 10. Suggested Delivery Plan

### 10.1 Phase 0: Clarify the Product Model

Define the internal structured model for:

- predicates
- actions
- parameters
- objects
- scenario templates
- initial state
- goals
- validation messages

This is the most important design step. Without it, the UI will be superficial.

### 10.2 Phase 1: Improve Current DevEnv for Immediate Value

Add to the current Dear PyGui tool:

- structured tables/forms for domain/problem data
- stronger validation by calling AME
- generated plan/BT previews
- saved keystone scenarios

This reduces immediate pain without waiting for a larger rework.

### 10.3 Phase 2: Build the Hybrid Graphical Workbench

Introduce:

- local FastAPI service
- embedded web UI shell
- React Flow authoring canvas
- Monaco expert pane
- generated PDDL panel
- richer validation explanation

### 10.4 Phase 3: Add Scenario Regression and Assurance Views

Add:

- scenario pack management
- expected vs actual outcome comparison
- audit/replay links into observability data
- report exports for review boards and stakeholders

### 10.5 Phase 4: Optional Browser-First Repackaging

If the tool gains wider uptake:

- serve the same UI into a normal browser
- keep the same backend contracts
- add organisational hosting, identity, and review workflow only when needed

---

## 11. Practical Guidance on PDDL Exposure

To keep the tool approachable:

- do **not** make users start from a blank PDDL file
- do show generated PDDL in an "Advanced" panel
- do allow experts to inspect and export the generated PDDL
- do store keystone scenarios as first-class objects
- do explain validation failures in mission language first, and planning language second

The goal is that a stakeholder can say:

> "In this scenario, the system should search sector B, classify the contact, and return safely if comms drop."

and the tool should help convert that into:

- structured scenario data
- generated PDDL
- a checkable plan
- an executable behaviour
- validation evidence

---

## 12. Final Recommendation

If we only want the fastest improvement, extend the current Dear PyGui DevEnv.

If we want the right long-term tool, build a **hybrid DevEnv workbench** with:

- Python backend continuity
- structured model as the canonical source
- generated PDDL as an artefact
- React Flow as the main graphical authoring surface
- AME and optional VAL as the validation engines

That route gives the best mix of:

- usability
- formal rigor
- incremental adoption
- licence cleanliness
- future extensibility

---

## 13. Sources

- Existing DevEnv quickstart and architecture in this repository:
  - [`subprojects/AME/doc/guides/devenv_ros2_quickstart.md`](../../../subprojects/AME/doc/guides/devenv_ros2_quickstart.md)
  - [`subprojects/AME/tools/devenv/ui/pddl_editor_tab.py`](../../../subprojects/AME/tools/devenv/ui/pddl_editor_tab.py)
  - [`doc/THIRD_PARTY_LICENSES.md`](../../THIRD_PARTY_LICENSES.md)
- `pywebview` GitHub repository and licence notes: [github.com/hycool/pywebview](https://github.com/hycool/pywebview)
- `FastAPI` GitHub repository and MIT licence: [github.com/FastAPI/FastAPI](https://github.com/FastAPI/FastAPI)
- `uvicorn` GitHub repository and BSD-3-Clause licence: [github.com/Kludex/uvicorn](https://github.com/Kludex/uvicorn)
- `React Flow` / xyflow open-source licensing statement: [xyflow.com/open-source](https://xyflow.com/open-source)
- `Monaco Editor` GitHub repository and MIT licence: [github.com/microsoft/monaco-editor](https://github.com/microsoft/monaco-editor)
- `dagre` licence file: [github.com/dagrejs/dagre/blob/master/LICENSE](https://github.com/dagrejs/dagre/blob/master/LICENSE)
- `Cytoscape.js` documentation: [js.cytoscape.org](https://js.cytoscape.org/)
- `LiteGraph.js` GitHub repository: [github.com/jagenjo/litegraph.js](https://github.com/jagenjo/litegraph.js)
- `Drawflow` GitHub repository: [github.com/jerosoler/Drawflow](https://github.com/jerosoler/Drawflow)
- `VAL` GitHub repository and BSD-3-Clause licence: [github.com/KCL-Planning/VAL](https://github.com/KCL-Planning/VAL)
- `JointJS` licensing page: [jointjs.com/license](https://www.jointjs.com/license)
- `GoJS` deployment/licensing notes: [gojs.net/latest/intro/deployment.html](https://gojs.net/latest/intro/deployment.html)
- `yFiles` licence types: [yworks.com/products/yfiles-wpf/license-types](https://www.yworks.com/products/yfiles-wpf/license-types)
- `bpmn-js` repository licence statement: [github.com/bpmn-io/bpmn-js](https://github.com/bpmn-io/bpmn-js)
