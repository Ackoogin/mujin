# Making PDDL Authoring Usable: Navigation and Non-Programmer Options

**Purpose:** Follow-on research for the graphical authoring tool now built under
[`subprojects/AME/src/authoring`](../../../subprojects/AME/src/authoring). It answers two
questions that the earlier option papers did not cover: how to make a large PDDL domain
navigable when the whole thing does not fit on one canvas, and what authoring styles
(block-based editors and similar) actually help people who are not software engineers.

**Status:** Research and recommendation. No implementation started.

**Related documents:**

- [`graphical_autonomy_authoring_tool_options.md`](graphical_autonomy_authoring_tool_options.md) — the original product-level options paper
- [`graphical_authoring_option1_extend_existing.md`](graphical_authoring_option1_extend_existing.md) — the node-editor library assessment that led to the current C++ tool
- [`doc/plans/AME/graphical_authoring_tool_workplan.md`](../../plans/AME/graphical_authoring_tool_workplan.md) — work items WI-0.1 through WI-5.4, which built what exists today

---

## 1. Executive Summary

The problem in the brief is really two problems, and they need different fixes.

**Problem one is navigation.** A PDDL domain is wide and shallow: many actions and many
predicates, each connected to a handful of others, with almost no nesting. Drawing all of
them on one canvas produces a wall of boxes with no reading order. This is not fixed by
better node styling. It is fixed by changing *how much is on screen at once* and *which
relationship the picture actually shows*.

**Problem two is vocabulary.** Preconditions, add effects, delete effects, typed parameters
and free variables are unfamiliar to anyone who has not written a planning model before.
This is not fixed by a graph at all. It is fixed by changing the *editing* surface, so that
people fill in guided forms and sentences rather than composing logic.

The recommended direction is:

1. **Stop using one canvas to hold the whole domain.** Replace it with a view that shows one
   selected element plus its immediate neighbours, and let the user walk outwards one step at
   a time, with back and forward history. This is the interaction model used by the
   Sourcetrail code explorer, and it is designed for exactly this shape of graph.
2. **Add a second view that splits the domain by object type.** For each type, show the small
   state machine that objects of that type move through. A domain that is unreadable as one
   200-node picture is usually readable as eight separate 6-state pictures. This is the
   established object-centred approach from the GIPO tool, and the same structure that
   modern planners compute automatically when they convert PDDL into finite-domain variables.
3. **Draw the relationship people actually navigate by.** The current canvas draws only
   hand-authored causal links, which carry no meaning into the generated PDDL. The
   relationship the brief describes — "from a predicate, find every action that changes it" —
   is already in the model but is never drawn.
4. **Move editing off the canvas into guided forms and fill-in-the-blank sentences.** Keep
   the graph for understanding, review and explanation. Every published evaluation of visual
   PDDL tools that measured expert productivity found graph-first authoring *slower* than
   typing text, so the graph should not be where the typing happens.

Block-based editing in the style of Scratch is worth adopting, but for a narrower job than
it first appears: it suits the tree-shaped logical expression inside a single action's
precondition, not the wide flat structure of the domain as a whole.

---

## 2. What the Current Canvas Does, and Why It Reads Badly

Four specific properties of
[`domain_graph_panel.cpp`](../../../subprojects/AME/src/authoring/domain_graph_panel.cpp)
combine to produce the flat wall of nodes.

### 2.1 Everything is drawn, always

`render()` loops over `model.predicates` and then `model.actions` and emits a node for every
entry, with no filtering. The palette filter box in
[`app_shell.cpp:864`](../../../subprojects/AME/src/authoring/app_shell.cpp) narrows the
sidebar list only; it does not change what the canvas draws. Screen area therefore grows
linearly with domain size and nothing ever leaves the view.

### 2.2 The default layout is two long rows

Unpositioned predicates are placed at `y = 60` spaced 230 pixels apart
([`domain_graph_panel.cpp:85`](../../../subprojects/AME/src/authoring/domain_graph_panel.cpp)),
and unpositioned actions at `y = 300` spaced 260 pixels apart (line 129). A domain with 40
predicates opens as a 9,000-pixel-wide strip. After that first frame, tidying the layout is
entirely manual, and the positions are saved into the project file as `posX` / `posY` on
each `PredicateDef` and `ActionDef`, so the manual arrangement becomes something the user
has to maintain by hand as the domain grows.

### 2.3 The edges on screen are not the edges that matter

This is the most important point. The only links drawn are `model.causalLinks`
(lines 197–202), which the user draws by hand between one action's add-effect pin and
another action's precondition pin. Searching the tool for that field shows it is written,
drawn, serialised, and type-checked — and then never read by
[`pddl_generator.cpp`](../../../subprojects/AME/src/authoring/pddl_generator.cpp), by any
validator, or by the planner. They are decoration.

Meanwhile the relationship the brief asks to navigate — predicate to the actions that touch
it — *is* fully present in the model, as the `predicateName` field of every `EffectRef`
inside `action.preconditions`, `action.addEffects` and `action.delEffects`. It is simply
never turned into an edge. Predicate nodes get a single output pin labelled `"o"`
(lines 108–110) that connects to nothing.

So the canvas is mostly disconnected boxes: the graph is flat because the graph structure has
not been drawn, not because the domain lacks structure.

### 2.4 Nodes have no compact form

Each action node renders every parameter, every precondition and every effect inline as its
own pin row. A ten-precondition action is a very tall node. There is no collapsed
representation, so zooming out to see the whole domain produces unreadably small text rather
than progressively simpler shapes.

---

## 3. Part A — Making a Wide Domain Navigable

### 3.1 Focused Neighbourhood View (primary recommendation)

**Idea.** Never render the whole domain. Render the currently selected element in the centre,
everything one step away from it around the edge, and nothing else. Clicking a neighbour
re-centres the view on that neighbour. Browser-style back and forward buttons walk the
history.

This is often called a *focus-plus-context* view: a small amount of detail at the focus, and
just enough surrounding material to know where you are. The formal version of the idea is
Furnas's degree-of-interest function, later extended from trees to general graphs by van Ham
and Perer, whose paper is subtitled with the design rule this recommendation follows:
"search, show context, expand on demand". The best-known working implementation for
developers is Sourcetrail, which shows the selected symbol with all its incoming and outgoing
dependencies and lets you step through them; the shape of a code base — many symbols, each
related to a few others — is very close to the shape of a PDDL domain.

**How it maps onto AME's model.** Two node kinds and two edge kinds are enough:

| From | Edge | To | Derived from |
|------|------|-----|--------------|
| Predicate | *is required by* | Action | `action.preconditions[].predicateName` |
| Predicate | *is made true by* | Action | `action.addEffects[].predicateName` |
| Predicate | *is made false by* | Action | `action.delEffects[].predicateName` |
| Action | *may enable* | Action | an add effect of A matching a precondition of B |

The last row is the causal relationship that `causalLinks` currently asks the user to draw by
hand. It can be computed instead: an action A may enable action B when A adds a predicate
that B requires and the parameter types line up. `causalLinkCompatible()` in
[`project_model.cpp`](../../../subprojects/AME/src/authoring/project_model.cpp) already
contains the type-compatibility test needed to do this.

**Sizing the view.** Show one step by default. Offer a "show two steps" control, and a per-node
expand control for the cases where the user wants to grow the view selectively rather than
uniformly. Cap the number of neighbours drawn (for example twenty) and summarise the rest as a
single "+37 more" node that opens a list — this keeps a heavily used predicate such as
`(at ?obj ?loc)` from flooding the view.

**Effort.** Medium. It needs an index built from the project model, a small layout routine, and
history tracking. It does not need a new library; `imgui-node-editor` already draws
everything required.

**Why this is the first thing to build.** It removes the layout maintenance problem entirely
(positions become computed, not stored), it makes domain size irrelevant to readability, and
it directly implements the navigation the brief describes.

### 3.2 Object Lifecycle View (second primary recommendation)

**Idea.** Group the domain by object type rather than by element kind. For each type in the
type hierarchy, show the states an object of that type can be in and the actions that move it
between them. A `robot` might have states `idle`, `moving`, `scanning`; a `sample` might have
`unclassified`, `classified`, `stowed`.

**Why this is the structural answer to "wide rather than deep".** A domain is wide because it
describes many objects at once. Slice it per type and each slice is small and deep enough to
draw as an ordinary state diagram — the kind of picture a systems engineer can read without
any planning background. Eight readable diagrams beat one unreadable one.

This is the central idea of GIPO, the Graphical Interface for Planning with Objects, which
built domain models from per-object state machines and generated PDDL from them, explicitly to
hide the technical detail of operator specification. The same structure appears on the planner
side: when Fast Downward translates PDDL into its finite-domain representation, it uses
invariant synthesis to find groups of mutually exclusive facts, and each group becomes one
multi-valued variable with a *domain transition graph* — a state machine over that variable's
values. So the decomposition is not a modelling convention we would be inventing; it is
recoverable from the domain automatically.

**How it maps onto AME's model.** Candidate state variables come from predicates whose first
argument is of the type in question and whose remaining arguments are either absent or fixed
per state. For each such predicate group, the transitions are the actions that delete one
member and add another. A first version does not need full invariant synthesis: start by
offering the user a simple grouping step ("these three predicates are alternative states of a
`robot`"), store the grouping in the project model, and derive the diagram from it. Automatic
candidate detection can be added later.

**Effort.** Medium, and higher if automatic grouping is attempted. Worth staging: manual
grouping first, automatic suggestion second.

### 3.3 Automatic Layout

Once the canvas shows a computed subgraph rather than a hand-arranged whole domain, positions
should be computed too. Two layouts cover the need:

- **Layered layout** for the neighbourhood and lifecycle views. Preconditions on the left,
  the focused action in the middle, effects on the right; or predecessor actions above and
  successor actions below.
- **Two-column layout** for any "all predicates against all actions" overview that is
  retained, since that graph is bipartite — every edge runs between a predicate and an
  action, never within a group.

C++ options are limited. The Open Graph Drawing Framework has a well-regarded implementation
of the standard layered (Sugiyama) algorithm but is GPL-licensed and therefore excluded by
project policy. `demekgraph` is a small C++ Sugiyama implementation whose repository is
labelled BSD-3-Clause while its README says MIT — either is acceptable, but the discrepancy
would need resolving before adoption. Given the graphs here are small once filtered (tens of
nodes, not thousands), writing a few hundred lines of layered layout directly is a reasonable
alternative and avoids the dependency question entirely.

### 3.4 Semantic Zoom

Make nodes change shape rather than just size as the canvas zooms out:

| Zoom level | Action node shows |
|------------|-------------------|
| Close | Name, parameters, every precondition and effect as pins |
| Medium | Name, parameters, counts such as "3 preconditions, 2 effects" |
| Far | Name only, as a small coloured box |

`imgui-node-editor` exposes the current canvas zoom, so this is a rendering change inside the
existing node loop with no new dependency. It is a modest amount of work with a large effect
on the overview case, and it pairs well with grouping (3.5) because collapsed groups can be
drawn as single boxes when zoomed out.

### 3.5 Grouping and Saved Views

Two lighter mechanisms that go a long way:

- **Groups.** Let the user put related actions and predicates into a named group
  ("navigation", "sensing", "communications"), drawn as a labelled box that can be collapsed
  to a single node. `imgui-node-editor` supports group nodes directly. The grouping is
  presentation metadata and needs one new field on the project model.
- **Saved views.** Let the user name and save a particular focus and filter — "the
  communications loss contingency" — and return to it. This turns the canvas into a set of
  small curated diagrams suitable for review meetings, which is closer to how stakeholders
  will actually use the tool than free exploration is.

### 3.6 Non-Graph Navigators

Some of the navigation the brief describes does not need a canvas at all, and these are the
cheapest items on the list:

- **A relations panel.** When a predicate is selected, show three lists beside the canvas:
  actions that require it, actions that add it, actions that delete it. When an action is
  selected, show its predicates and the actions that may enable or be enabled by it. Every
  entry is clickable and re-focuses the selection. This alone delivers the "predicate to
  actions, then action to related actions" walk described in the brief, in a fraction of the
  effort of the graph work, and it is useful whether or not the graph views get built.
- **A predicate-by-action matrix.** A grid with predicates down the side and actions across
  the top, each cell marked to show whether that action requires, adds or deletes that
  predicate. For a wide domain a matrix is often more readable than a graph, and it exposes
  gaps immediately: an empty row is a predicate nothing ever changes, an empty column is an
  action that touches nothing.
- **Selection history.** Back and forward through previously selected elements. Cheap, and it
  is what makes step-by-step exploration feel safe.

---

## 4. Part B — Authoring Styles for People Who Are Not Programmers

### 4.1 Guided Forms and Sentence Templates (recommended primary editing surface)

The single most effective change for non-specialist users is to stop asking them to write
logical expressions and start asking them to complete sentences with dropdown menus:

```
Action:  move a robot between locations

  It involves:   [robot] called [?r]      [location] called [?from]      [location] called [?to]

  Before it can happen:
     [?r] must be  [at]  [?from]                            (+ add a condition)
     [?from] and [?to] must be  [connected]

  Afterwards:
     [?r] is  [at]  [?to]                                    becomes true
     [?r] is  [at]  [?from]                                  becomes false
```

Every bracketed slot is a dropdown restricted to values that are legal at that point, so a
type error becomes impossible to express rather than something to be reported afterwards. The
words "precondition", "add effect" and "delete effect" do not appear; the generated PDDL is
available in the existing PDDL tab for anyone who wants it.

This is the same principle behind the strongest usability result in the prior art: MyPDDL
scored 89.6 on the System Usability Scale, and its type-diagram generator cut the time to
answer questions about a type hierarchy by 48 percent — from generated pictures and
structured assistance, not from graph-based editing.

**Effort.** Low to medium. The tool already has a selected-element editor
(`renderSelectedElementEditor()` in `app_shell.cpp`); this is a redesign of that panel rather
than new infrastructure.

### 4.2 Block-Based Editing (Scratch-style)

**What it is.** Interlocking shaped blocks that only fit together in valid combinations, so
the shapes enforce the grammar. Blockly, Google's toolkit, is Apache-2.0; Scratch Blocks, the
Scratch Foundation's fork, is also Apache-2.0. The rest of Scratch 3.0 is AGPL-3.0 and is not
a candidate. There is a directly relevant project: `blockly-pddl` from the AI-Planning
organisation on GitHub, MIT-licensed, which translates between Blockly blocks and PDDL 1.2 and
also covers behaviour trees and state machines. It is small (single-digit stars) and limited
to PDDL 1.2, so it is best treated as a design reference rather than a component to adopt.

**Where blocks genuinely help.** Blocks are good at *tree-shaped* content — nested `and`,
`or`, `not`, `forall`, `exists` inside a single precondition. That is the one place in a PDDL
domain that is deep rather than wide, and it is exactly where a form with dropdown menus
starts to struggle. If AME's PDDL support grows beyond flat conjunctions of predicates, a
block editor for the condition expression inside one action becomes the natural control.

**Where blocks do not help.** Blocks do nothing for the wide problem. A domain of 60 actions
becomes 60 block stacks, which is the same wall of boxes in a different costume. Blocks are
also a poor fit for typed parameter declarations, which are better as a small table.

**The practical obstacle.** Blockly is a web toolkit. Embedding it in the current Dear ImGui
C++ application means embedding a browser engine, which is a large dependency for one panel
and reopens the desktop-versus-web question that
[the original options paper](graphical_autonomy_authoring_tool_options.md) settled. The
realistic route is to reimplement the *interaction* — shaped, snap-together condition blocks —
directly in ImGui for the expression editor only. That is a bounded piece of custom drawing,
not a port of Blockly.

**Verdict.** Adopt the idea, scoped to logical expressions inside one action. Do not adopt it
as the top-level domain editing metaphor, and do not embed a browser for it.

### 4.3 Object-Oriented Modelling (itSIMPLE-style)

itSIMPLE lets users describe a planning domain as UML diagrams — class diagrams for the types
and their relationships, state diagrams for object behaviour — and generates PDDL from them.
Later versions added Petri net translation for checking dynamic behaviour, and from version 4
onwards allowed direct PDDL projects without the UML step.

For AME the attraction is that many defence and systems-engineering stakeholders already read
UML, and a class diagram of the type hierarchy is a genuinely useful artefact. The state
diagram half of it is the same idea as the object lifecycle view in section 3.2, which is the
part worth building. A full UML modelling surface is a much larger undertaking than the
navigation work and should not be attempted; a generated, read-only class diagram of the type
hierarchy is a small addition with good presentational value.

### 4.4 Example-First Authoring

Ask the user to describe a concrete situation and the outcome they want, before asking them to
describe the rules. "The vehicle is at the dock, the sample is in sector B, and I want the
sample stowed on board." The tool then reports which parts of that story the current domain
can already support and which it cannot, and offers to create the missing predicates and
actions.

This inverts the usual order and matches how people actually think about missions. The pieces
needed are largely present: the scenario editor from WI-4.1, the feasibility check from
WI-2.4, and the batch runner from WI-4.2. What is missing is the diagnostic step that turns
"no plan found" into "there is no action that makes `(stowed ?sample)` true" — a report the
tool can generate directly from the same predicate-to-action index that section 3.1 needs.

That diagnostic is worth building on its own merits. "No plan found" is the least helpful
message a planning tool can give a non-expert, and the index makes a much better one cheap.

### 4.5 Language-Model Drafting Inside the Existing Envelope

Recent work on using language models to write PDDL is consistent on one point: models do
reasonably well on problem files and badly on domain files, so the useful shape is
draft-then-verify rather than generation. AME already has the right structure for this in the
propose-verify-fallback envelope described in
[`08-neuro-symbolic.md`](../../../subprojects/AME/doc/architecture/08-neuro-symbolic.md): a
model proposes, a symbolic verifier checks, and a known-good fallback applies if the check
fails.

Applied to authoring, that means a "describe what you want in plain English" box that produces
a *proposed* action or predicate, presented as a form the user reviews and edits before it
enters the model, with the existing structural, parse, grounding and planning checks run
against it first. This should come after the navigation and form work, not before — a
drafting aid is only useful once reviewing and correcting a draft is easy.

---

## 5. Prior Art and What It Teaches

| Tool | Approach | What to take from it |
|------|----------|---------------------|
| **GIPO** | Object-centred; per-object state machines generate PDDL operators | The lifecycle decomposition in section 3.2 — the strongest available answer to domain width |
| **itSIMPLE** | UML class and state diagrams, Petri net checking, PDDL generation | Type-hierarchy class diagram as a generated, read-only artefact |
| **VIZ** | Simplified graphical domain modelling without PDDL syntax | Evidence that hiding the syntax is achievable; also that a thin graphical layer alone is not enough |
| **KAVI** | Visual modelling plus a reusable domain knowledge base and validation-driven refinement | Reusable building blocks and the refine-from-validation loop |
| **MyPDDL** | Text-first with generated type diagrams and structured helpers | The best measured usability result in the field, from generated diagrams rather than graph editing |
| **blockly-pddl** | Blockly blocks translated to and from PDDL 1.2 (MIT) | Reference for block shapes over PDDL constructs |
| **vscode-pddl** | Text editing with generated state and plan visualisations | Generated pictures beside authoritative text; a good default division of labour |
| **Sourcetrail** | Selection-focused neighbourhood graph with browser-style history | The navigation model in section 3.1 |

**The caveat that matters most.** The KAVI paper's stated motivation is that existing visual
modelling tools, itSIMPLE and VIZ specifically, are *less efficient than an expert hand-coding
PDDL in a text editor*. This is the recurring result in this field, and it should shape the
design: the graphical tool's job is to make the domain understandable and to let non-experts
contribute safely, not to be the fastest way for an experienced engineer to write a domain.
Keeping the PDDL tab as a real, editable, round-trippable path — which
[`pddl_importer.cpp`](../../../subprojects/AME/src/authoring/pddl_importer.cpp) already
supports — is not a fallback, it is part of the design.

---

## 6. Libraries and Licences

| Component | Licence | Fit | Recommendation |
|-----------|---------|-----|----------------|
| `imgui-node-editor` (current) | MIT | Already integrated; supports groups, zoom, custom node drawing | Keep. Note the upstream repository has been quiet for around two years; the fork maintained as part of ImGui Bundle is the more active line and is worth tracking |
| `demekgraph` | BSD-3-Clause per repository, MIT per README | Small C++ layered graph layout | Only if a layout library is wanted; resolve the licence discrepancy first |
| **OGDF** | GPL | High-quality layered layout | **Excluded by project policy** |
| Hand-written layered layout | n/a | Tens of nodes per view after filtering | **Preferred** — avoids the dependency and the licence question |
| **Blockly** / **Scratch Blocks** | Apache-2.0 | Block-based editing, but web-only | Design reference; reimplement the interaction in ImGui rather than embedding a browser |
| `blockly-pddl` | MIT | PDDL 1.2 block vocabulary | Design reference |
| Scratch 3.0 (other than Scratch Blocks) | AGPL-3.0 | — | **Excluded by project policy** |

Nothing in the recommended direction requires a new third-party dependency.

---

## 7. Recommendation

Build the navigation work first, in this order. Each step is useful on its own and none
invalidates the code already written.

1. **Build a relation index over the project model.** One structure answering "which actions
   require, add or delete this predicate" and "which actions may enable this action". Everything
   else in this document consumes it, and it is the foundation of the better "no plan found"
   diagnostics in section 4.4.
2. **Add the relations panel** (section 3.6). Cheapest item, delivers the navigation described
   in the brief immediately, and works without touching the canvas.
3. **Convert the canvas to a focused neighbourhood view** (section 3.1) with computed layout
   and selection history. Retire the stored `posX` / `posY` positions for this view. Retain the
   current whole-domain canvas as an explicit "show everything" mode for small domains.
4. **Rebuild the selected-element editor as guided forms and sentence templates**
   (section 4.1). This is where non-specialist users will spend their time.
5. **Add the object lifecycle view** (section 3.2), starting with user-declared groupings and
   adding automatic suggestion later.
6. **Add semantic zoom and grouping** (sections 3.4, 3.5) as polish once the views above are
   in place.

Then, if the tool is taken further:

7. Block-based editing for logical expressions inside one action, once AME's PDDL support
   includes nested conditions (section 4.2).
8. Language-model drafting inside the propose-verify-fallback envelope (section 4.5).

**Decide what `causalLinks` is for.** It is currently drawn and stored but never used. Either
make it derived and read-only — which is what section 3.1 needs — or remove it. Keeping a
hand-drawn field that has no effect on the generated PDDL will confuse every user who finds it.

---

## 8. Suggested Work Items

These extend
[`doc/plans/AME/graphical_authoring_tool_workplan.md`](../../plans/AME/graphical_authoring_tool_workplan.md),
whose phases 0 through 5 are complete.

| Item | Scope | Depends on | Effort |
|------|-------|-----------|--------|
| **WI-6.1** | Relation index over `ProjectModel`; derived predicate-to-action and action-to-action edges | WI-1.4 | Low |
| **WI-6.2** | Relations panel with clickable navigation and selection history | WI-6.1, WI-5.2 | Low |
| **WI-6.3** | Focused neighbourhood canvas mode with computed layered layout and neighbour capping | WI-6.1, WI-1.1 | Medium |
| **WI-6.4** | Guided form and sentence-template editor replacing the current selected-element panel | WI-1.4, WI-1.2 | Medium |
| **WI-6.5** | Predicate-by-action matrix view | WI-6.1 | Low |
| **WI-6.6** | Object lifecycle view with user-declared state groupings | WI-6.1, WI-1.2 | Medium |
| **WI-6.7** | Semantic zoom and collapsible groups on the canvas | WI-6.3 | Low-medium |
| **WI-6.8** | Failure explanation using the relation index ("nothing makes X true") | WI-6.1, WI-2.4 | Low-medium |
| **WI-6.9** | Saved named views | WI-6.3 | Low |
| **WI-6.10** | Resolve `causalLinks`: derive it or remove it | WI-6.1 | Low |

---

## 9. Sources

**Prior art in planning domain authoring**

- GIPO — Graphical Interface for Planning with Objects: [ipc05.icaps-conference.org/papers/paper4.pdf](https://ipc05.icaps-conference.org/papers/paper4.pdf)
- Planning domain definition using GIPO, McCluskey et al.: [eprints.hud.ac.uk/495](https://eprints.hud.ac.uk/id/eprint/495/1/McCluskeyPlanning.pdf)
- itSIMPLE 2.0 — An Integrated Tool for Designing Planning Domains: [cdn.aaai.org/ICAPS/2007/ICAPS07-043.pdf](https://cdn.aaai.org/ICAPS/2007/ICAPS07-043.pdf)
- The itSIMPLE tool for modeling planning domains: [ipc05.icaps-conference.org/papers/paper5.pdf](https://ipc05.icaps-conference.org/papers/paper5.pdf)
- KAVI — An Integrated Development Environment for Planning Domain Modeling, Li and Zhuo: [arxiv.org/abs/1804.07013](https://arxiv.org/abs/1804.07013)
- MyPDDL — Tools for efficiently creating PDDL domains and problems: [arxiv.org/pdf/2008.11069](https://arxiv.org/pdf/2008.11069)
- Planning in the Wild — Modeling Tools for PDDL: [arxiv.org/pdf/1511.07500](https://arxiv.org/pdf/1511.07500)

**Block-based and visual programming**

- Visual Planning Domain Design for PDDL using Blockly (ICAPS 2021 demo): [icaps21.icaps-conference.org/demos/demos/385.pdf](https://icaps21.icaps-conference.org/demos/demos/385.pdf)
- `blockly-pddl` (MIT): [github.com/AI-Planning/blockly-pddl](https://github.com/AI-Planning/blockly-pddl)
- Blockly (Apache-2.0): [en.wikipedia.org/wiki/Blockly](https://en.wikipedia.org/wiki/Blockly)
- Scratch Blocks licence (Apache-2.0): [github.com/scratchfoundation/scratch-blocks/blob/develop/LICENSE](https://github.com/LLK/scratch-blocks/blob/develop/LICENSE)
- Scratch source code licensing overview: [en.scratch-wiki.info/wiki/Scratch_Source_Code](https://en.scratch-wiki.info/wiki/Scratch_Source_Code)

**Graph navigation and visualisation**

- "Search, Show Context, Expand on Demand": Supporting Large Graph Exploration with Degree-of-Interest, van Ham and Perer: [perer.org/papers/adamPerer-DOIGraphs-InfoVis2009.pdf](https://perer.org/papers/adamPerer-DOIGraphs-InfoVis2009.pdf)
- Sourcetrail documentation — graph view and node navigation: [github.com/CoatiSoftware/Sourcetrail/blob/master/DOCUMENTATION.md](https://github.com/CoatiSoftware/Sourcetrail/blob/master/DOCUMENTATION.md)
- `imgui-node-editor`: [github.com/thedmd/imgui-node-editor](https://github.com/thedmd/imgui-node-editor)
- ImGui Bundle discussion on the maintained node-editor fork: [github.com/pthom/imgui_bundle/discussions/428](https://github.com/pthom/imgui_bundle/discussions/428)
- `demekgraph` — C++ Sugiyama layered layout: [github.com/gml4gtk/demekgraph](https://github.com/gml4gtk/demekgraph)
- OGDF layered layout algorithms (GPL): [ogdf.github.io/doc/ogdf/group__gd-layered.html](https://ogdf.github.io/doc/ogdf/group__gd-layered.html)

**Planner-side structure**

- The Fast Downward Planning System, Helmert — multi-valued planning tasks, causal graphs and domain transition graphs: [jair.org/index.php/jair/article/download/10457/25068/19419](https://www.jair.org/index.php/jair/article/download/10457/25068/19419)
- Concise finite-domain representations for PDDL planning tasks, Helmert: [researchgate.net/publication/222429384](https://www.researchgate.net/publication/222429384_Concise_finite-domain_representations_for_PDDL_planning_tasks)

**Language models and planning models**

- LLMs as Planning Formalizers — a survey: [arxiv.org/pdf/2503.18971](https://arxiv.org/pdf/2503.18971)
- Generating consistent PDDL domains with Large Language Models: [alphaxiv.org/overview/2404.07751v1](https://www.alphaxiv.org/overview/2404.07751v1)
- Large Language Models as Planning Domain Generators: [arxiv.org/pdf/2405.06650](https://arxiv.org/pdf/2405.06650)

**Text-first tooling for comparison**

- VS Code PDDL extension: [github.com/jan-dolejsi/vscode-pddl](https://github.com/jan-dolejsi/vscode-pddl)
- `pddl-gantt` plan visualisation: [github.com/jan-dolejsi/pddl-gantt](https://github.com/jan-dolejsi/pddl-gantt)
