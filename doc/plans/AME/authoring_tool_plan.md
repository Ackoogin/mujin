# AME Authoring Tool — Plan

**What this is.** The single plan for the AME graphical authoring tool: what it is for,
what it can already do, what is left to build, and what still has to be reviewed before
a release.

**Status in one paragraph.** All four workstreams are built: the authoring surface,
simulation runs, review and assurance artefacts, and the workstream D update that keeps the
tool's PDDL reader and project format aligned with `ame_core`. One thing is outstanding, and
it is acceptance rather than code: the
non-specialist walkthrough in section 5 is a release gate and has not been run end to end
by someone who does not write software.

**Background reading.**

| Document | What it gives you |
|----------|-------------------|
| [`doc/research/AME/graphical_autonomy_authoring_tool_options.md`](../../research/AME/graphical_autonomy_authoring_tool_options.md) | Why a tool was built rather than adopted, and the licence constraints |
| [`doc/research/AME/graphical_authoring_option1_extend_existing.md`](../../research/AME/graphical_authoring_option1_extend_existing.md) | Why the tool is a C++ application linked against `ame_core` |
| [`doc/research/AME/pddl_authoring_usability_and_navigation.md`](../../research/AME/pddl_authoring_usability_and_navigation.md) | The navigation and non-programmer authoring research, with prior art and sources |
| [`doc/design/AME/pddl_authoring_hmi_concepts.html`](../../design/AME/pddl_authoring_hmi_concepts.html) | The screen concepts the domain-authoring views were built from |
| [`doc/design/AME/simulation_run_hmi_concepts.html`](../../design/AME/simulation_run_hmi_concepts.html) | The screen concepts the simulation-run views were built from, with the six review questions and their answers |
| [`subprojects/AME/doc/guides/authoring_tool_user_guide.md`](../../../subprojects/AME/doc/guides/authoring_tool_user_guide.md) | How to build, launch and use what exists today |

---

## 1. Purpose and audience

The authoring tool is a local desktop workbench for creating, reviewing and checking AME
mission models. It keeps a structured project as its working format, generates PDDL from
that project, and uses the same parser, world model, planner and plan compiler as the
runtime stack, so what the tool says about a model is what the runtime will do with it.

Its users are mission and systems engineers, safety and assurance reviewers, and
operational subject-matter experts. Some of them write software. Most of them do not, and
the ones who do not are the people the tool exists for. An autonomy developer can already
write PDDL in a text editor faster than any graphical tool will let them; that is a
consistent finding in the published evaluations, and this plan does not try to beat it.
The tool earns its place by letting people who cannot write PDDL contribute to a mission
model safely, and by letting reviewers understand a model they did not write.

### The requirement that outranks the others

**A person who does not write software must be able to author, run and review a mission
model without help.** In practice that means five rules, which apply to anything built
from here on as much as they applied to what exists:

1. **No planning vocabulary appears as a label.** "Precondition", "add effect", "delete
   effect", "fluent", "grounding" and "predicate" belong in the generated PDDL, not on a
   form. The screen says "Before it can happen", "Afterwards", and "fact". The generated
   PDDL stays visible for anyone who wants it.
2. **An illegal choice is impossible to express, not reported afterwards.** Every slot the
   user fills in is a list restricted to what is legal at that point. Illegal choices stay
   visible, greyed, with the reason beside them, so the user learns the rule rather than
   wondering where the option went.
3. **Every failure is explained in terms of the mission, not the machinery.** The tool says
   which fact could not be brought about, what would have produced it, and offers the two
   or three edits that would fix it.
4. **Every view is readable without training.** If a screen needs a briefing before a
   reviewer can use it, the screen is wrong.
5. **The tool never asks the user to maintain something the model does not use.** Anything
   that can be computed is computed, rather than hand-maintained.

---

## 2. What exists today

The tool builds as `ame_authoring_tool` behind the `authoring` CMake preset, on Windows
with MSVC 2022 and on Linux with GCC or Clang. It has a headless `--self-test` mode that
renders frames off-screen and writes a PNG plus a JSON result, so an automated agent can
check the user interface without a display attached. Source lives in
[`subprojects/AME/src/authoring`](../../../subprojects/AME/src/authoring); its own test
suites run under the `authoring-release` and `authoring-debug` test presets.

### Authoring a model

| Capability | What works | Where |
|------------|-----------|-------|
| **Project format** | Versioned JSON project holding types, objects, facts, actions, scenarios, state groups and behaviour-tree bindings, with round-trip tests | `project_model.cpp` |
| **Domain editing** | Type hierarchy and objects in the sidebar; facts and actions as nodes; guided sentence editor with type-aware dropdowns; type rename that reaches every use; parameter reordering; copy and paste; quick-add and fit-to-contents from the keyboard | `app_shell.cpp`, `guided_editor_model.cpp`, `model_edits.cpp`, `type_hierarchy_panel.cpp` |
| **Undo and redo** | Covers structural edits and typing, with a run of keystrokes in one field folded into one undoable step; the Edit menu names what would be undone | `command_stack.cpp` |
| **Files and projects** | Native dialogs, a recent-projects list held with the user's settings, a prompt on quitting with unsaved work, and a recovery copy written every half minute while there are unsaved changes | `app_shell.cpp`, `recent_projects.cpp` |
| **Navigation** | Relation index over the whole model; relations panel with back and forward history; focused neighbourhood canvas with one- and two-step depth and a neighbour cap; whole-domain canvas with semantic zoom; saved named views held in the project | `relation_index.cpp`, `neighbourhood_model.cpp`, `domain_graph_panel.cpp` |
| **Presentation groups** | A named set of facts and actions drawn as one labelled box on the whole-domain canvas, which closes to a single box standing in for its contents. Lines that reached a member reach the box instead, lines that would then say the same thing twice are drawn once, and a line with both ends inside a closed group is not drawn at all. Anything can be in at most one group, and the refusal says why. Nothing about a group reaches the generated PDDL | `presentation_groups.cpp`, `domain_graph_panel.cpp` |
| **PDDL** | Generation with live preview and export; one problem file per scenario; import through a dedicated reader that preserves names and condition structure. Covers the finite subset `PddlParser` accepts: negative and alternative conditions, finite quantifiers, equality filters, union input types, goal alternatives, domain constants and confirmed facts | `pddl_generator.cpp`, `pddl_importer.cpp` |
| **Import merging** | Importing into a non-empty project shows what would be added, overwritten or left alone, with the cost of each replacement stated, and asks before doing any of it. Replacing an action keeps its behaviour-tree binding, run settings and canvas position. Imported elements are laid out by their relationships | `import_merge.cpp` |
| **Checking** | Four levels: continuous structural checks while editing, parse through `PddlParser`, grounding through `WorldModel`, and feasibility through `Planner` | `structural_validator.cpp`, `pddl_validator.cpp` |
| **Problem reporting** | One sentence per problem; clicking a row selects the element it names and opens the Domain tab on it; parser wording is translated, with the raw message behind a details toggle. A test asserts the word "predicate" appears nowhere in the list | `problem_list.cpp` |
| **Failure explanation** | Backward chain from a failed goal to the first fact nothing produces, with buttons that apply the likely fixes | `failure_explainer.cpp` |
| **Behaviour-tree binding** | Per-action node type, reactive flag and subtree template with parameter placeholders, previewed against stand-in arguments | `app_shell.cpp` |

### Running and watching a model

| Capability | What works | Where |
|------------|-----------|-------|
| **Scenarios** | Scenario list with starting facts and goals chosen from the grounded set rather than typed, with typing kept as an alternative held to the same rules; planning and execution expectations per scenario | `app_shell.cpp`, `fact_chooser.cpp` |
| **Simulation runs** | A scenario runs against the real compiled tree with stand-in action nodes bound to every action. Run, Pause, Step, Stop and Reset; per-action run settings defaulting to four ticks and always succeeds; a saved random seed. Starting facts are recorded as observed and everything a run produces as predicted, so an action that requires an observed fact refuses a merely predicted one | `simulation_engine.cpp` |
| **Watching a run** | The timeline is the primary view, one row per action against a tick axis, with fact changes marked on the action that caused them. Beside it, the facts as they stand and the compiled tree coloured by what each node is doing. Clicking the timeline moves the whole screen to that tick without disturbing a live run | `app_shell.cpp`, `bt_graph_panel.cpp` |
| **Faults and replanning** | Two mechanisms only: fail one action attempt, or set one grounded fact at a chosen tick. Lifecycle alternatives stay consistent. A failed tree replans from current state and continues the same run, with the abandoned and replacement plans shown together and the reason stated in a sentence | `simulation_engine.cpp`, `app_shell.cpp` |
| **Batch regression** | Each scenario is planned and simulated against its expectations — goal reached, action-count bounds, actions that must or must not run, acceptable replans, behaviour under a named fault — with progress, a Stop button, a reason per result and a JSON report. Older project files take defaults that preserve their previous meaning | `scenario_runner.cpp`, `mission_commands.cpp` |
| **Recorded runs and replay** | A run is saved in the runtime's three JSONL formats plus a `run.json` manifest naming project, scenario, seed and injected faults. Recordings from this tool or a real system replay through the same views without an open project; a recording with no manifest is labelled a real-system run rather than assumed simulated | `run_record.cpp` |
| **Comparing runs** | The current in-memory run against a saved one, or two saved folders, compared by first differing tree tick, differing actions and differing final facts, stated first in one sentence | `run_record.cpp` |
| **Contingency analysis** | Per-scenario declaration of which facts represent a contingency and which a safe state, with the old inference kept as the default. No fact-count cap; a domain too large to enumerate is refused with the numbers and advice. Results say how much of the space was covered. The exhaustive search and its pruning are `ame_core`'s `ContingencySearch`, shared with the contingency verifier | `contingency_analyser.cpp`, `ame/contingency_search.h` |

### Producing artefacts

| Capability | What works | Where |
|------------|-----------|-------|
| **Review views** | Fact-by-action matrix with CSV and Markdown export; object lifecycle view derived from user-declared groups of alternative facts; plan view as a causal graph; compiled behaviour-tree view parsed from the generated XML | `fact_action_matrix.cpp`, `lifecycle_model.cpp`, `plan_graph_panel.cpp`, `bt_graph_panel.cpp` |
| **Review packs** | One export writes a dated, indexed folder holding the generated domain, a problem file per scenario, both matrix formats, a scenario results table, a replayable run and a domain summary | `review_pack.cpp` |
| **Assurance evidence** | A generated Markdown report of what the model contains, what nothing brings about, what each action is bound to, how every scenario behaved and what stayed reachable under each contingency — ending with what the report does *not* tell you. Reachable from the File menu, from the review pack, and from `ame_mission_cli evidence` | `assurance_report.cpp` |
| **Command line** | `ame_mission_cli` with `run`, `record`, `batch` and `evidence`, linking only the core and the project-model library, so a build agent needs no display stack. Conventions match the contingency verifier: positional project file, `--json` naming the machine-readable report, `--help`, and an exit code that states the verdict | `mission_cli_main.cpp`, `mission_commands.cpp` |

### One core change the tool depended on

The plan compiler used to surround every action with helper nodes, one `CheckWorldPredicate`
per precondition and one `SetWorldPredicate` per effect. It now emits **one node per plan
step**, carrying its grounded preconditions and effects as attributes on the emitted
element, so a saved tree still describes itself without the project that produced it. A
planned-action base class in `ame_core` checks the preconditions and applies the effects
around the action's own work; simulation uses a stand-in derived from that base class.
Neither helper node type appears in generated output any more.

This mattered for two reasons. A three-step mission used to arrive on screen as roughly
twenty boxes, of which three were the mission. And writing an effect as a node asserts that
the world changed because the action returned success, which is reasonable for a simulation
but wrong for a deployed system, where establishing the state after an action is the
deployment's business. The follow-ups a review of this work raised are tracked in
[`doc/todo/AME/TODO.md`](../../todo/AME/TODO.md) under "Planned-action contract:
follow-ups". One consequence to know about: an action with no registered implementation now
aborts the compile unless stub mode is on, which the authoring tool switches on.

---

## 3. Non-goals

Unchanged and still deliberate.

- Web or browser deployment. The tool is a local desktop application.
- Conditional effects (`when`), numeric fluents and arithmetic, and temporal or durative
  actions. `PddlParser` rejects all of these outright, so they are out of scope for the
  whole of AME rather than for the tool alone. Everything else the parser has since gained
  is now in scope for the tool and is workstream D in section 4; the sentence that used to
  stand here, saying AME supports STRIPS with typing and the tool targets exactly that
  subset, was true when this plan was written and is no longer.
- Replacing DevEnv. DevEnv keeps live monitoring of real ROS2 systems; the authoring tool
  simulates offline.
- Live ROS2 connection from the authoring tool. Simulation runs are in-process against the
  world model, with no transport involved.
- Multi-user collaboration and concurrent editing.
- Graph-first authoring, where a domain is built by wiring nodes together. Every published
  evaluation that measured it found it slower than typing. The graph is for looking, not
  for writing.

Also considered and dropped, so they are not re-proposed: a light theme (one dark theme is
the house style across DevEnv and this tool); icons for node types (a legend to learn works
against rule 4); dragging from the palette onto the canvas (the canvas is not where
authoring happens); per-flow colour-coding in the plan view (a second colour language that
holds on one screen only); layout presets (the tool uses fixed tabs, and saved views cover
the underlying need); and a custom animation layer for canvas navigation (the node editor
already animates panning and zooming).

One dropped decision needs its reasoning kept in full, because workstream D asks for it to
be reconsidered. **The importer was built on its own reader rather than on `PddlParser`**,
because the parser discards the names, comments and structure the importer exists to
preserve, and because a round-trip test against the generator keeps the dedicated reader
honest. That reasoning still holds as far as it goes. What has changed is that the two
readers now accept visibly different languages, which is what produced workstream D, so the
trade is no longer the one that was originally judged.

---

## 4. What remains to be built

Everything in workstreams A, B, C and D is built. The remaining item is the acceptance
walkthrough in section 5.

### Workstream D — catch the tool up with the PDDL the core accepts

**Built.** Project format version 2 preserves condition trees, union input types, goal
alternatives and domain constants while version 1 projects still load with their old meaning.
The reader and generator round-trip every construct in the table below, and the generated
files are parsed again through `PddlParser` in tests. The guided editor uses the wording from
[`condition_authoring_hmi_concepts.html`](../../design/AME/condition_authoring_hmi_concepts.html).
Relations, both domain views, the matrix, failure explanation, contingency analysis and the
assurance report distinguish a fact that must be false, a fact that must be true, and a fact
that is one alternative from the others.

**Reader decision.** The dedicated reader remains. `PddlParser` deliberately lowers
alternatives and finite quantifiers into grounded action schemas, so using it alone would lose
the source structure that the editor must show and write back. The drift is instead guarded by
round-trip cases for every accepted construct followed by a parse through the core reader.

**Why this is here.** When this plan was written, `PddlParser` accepted STRIPS with typing
and the tool targeted exactly that. The parser has since gained a good deal more, and
because the tool keeps its own reader and its own project format rather than sharing the
core's, none of it arrived in the tool. The result is not a missing convenience. **A domain
that uses any of these cannot be opened in the tool at all**, so the people this tool exists
for — the reviewers and subject-matter experts of section 1 — are locked out of exactly the
domains that are hardest to read unaided.

**What the original gap was, measured rather than assumed.** Each row below was tried against
the reader before D. The completed-work column records the corresponding behaviour now.

| PDDL the core accepts | Completed work | Core reference |
|-----------------------|--------------------------|----------------|
| Negative precondition, `(not (q ?x))` | Preserved and shown as “must be false” | `PddlParserNegPre.AcceptsNegatedAtomInPrecondition` |
| Disjunctive precondition, `(or ...)`, split into several grounded schemas | Preserved and shown as “any one of these is enough” | `PddlParser.DisjunctivePreconditionSplitsIntoSchemas` |
| Universal quantifier, `(forall (?y - t) ...)`, expanded over the objects | Preserved and shown as “for every” | `PddlParser.UniversalExpandsToConjunction` |
| Existential quantifier, `(exists (?y - t) ...)`, expanded to a disjunction | Preserved and shown as “for at least one” | `PddlParser.ExistentialExpandsToDisjunctionSplit` |
| Equality and inequality as binding filters, `(= ?a ?b)`, `(not (= ?a ?b))` | Preserved and shown as “same” or “different” | `PddlParser.EqualityKeepsOnlyMatchingPairs` |
| Union parameter type, `?x - (either a b)` | Preserved as the legal set of accepted types | `PddlParser.EitherTypeGroundsOverUnionOnly` |
| Disjunctive goal, setting goal alternatives | Preserved and shown as goal choices | `PddlParser.DisjunctiveGoalSetsAlternatives` |
| Domain constants, `(:constants ...)` | Kept in the domain and available in every scenario | `PddlParser.ParsesDomainConstants` |

Confirmed predicates, `(:confirmed-predicates ...)`, are the one recent addition the tool
already carries end to end, and are the model to follow.

**What was built.** Four pieces, in dependency order.

- **D1. The project format holds these constructs.** `EffectRef` records whether a fact must
  be false, and a recursive condition expression holds alternatives, quantifiers and
  comparisons. Saved projects use version 2; version 1 projects still load with their old
  meaning.
- **D2. The reader and generator preserve them.** `pddl_importer.cpp` and
  `pddl_generator.cpp` remain separate from the core parser for the reason above. A
  round-trip case covers every construct and sends the generated result through the core
  parser.
- **D3. The guided editor says these things without planning vocabulary.** It uses phrases
  such as "must be false", "any one of these is enough", "for every", "for at least one",
  "the same" and "different". The editor offers legal names and types as choices. The
  concept pack fixed the wording before implementation.
- **D4. Every view that reads a condition understands the new shapes.** The fact-by-action
  matrix, relation index, neighbourhood canvas, failure explainer, contingency analyser and
  assurance report distinguish positive, negative and alternative conditions. The concise
  backward failure chain stops when an alternative or quantified condition cannot be
  reduced safely to one mandatory fact.

**Non-specialist acceptance.** A reviewer opens a domain written by someone else that uses
a negative precondition and a "one of these" condition, reads what each action needs in
plain words, runs a scenario against it, and exports a review pack — without being told
that the domain uses anything unusual.

**Effort.** Large. D1 and D2 are mechanical; D3 is a design problem with a concept pack in
front of it; D4 is a sweep whose size is not knowable until D1 settles the shape.

**One judgement to make before starting.** The tool duplicating the core's parser is what
let this drift happen, and D2 makes the duplicate larger. It is worth asking, once, whether
the tool should read PDDL through `PddlParser` and keep its own reader only for what the
parser discards — the names, comments and layout the importer exists to preserve. That was
looked at once before and rejected, and the reason is recorded in section 3's dropped list.
The trade has changed now that the two readers accept visibly different languages, so the
answer may be different; it should be decided deliberately rather than by writing D2 first.

### Presentation groups

Built, and described in section 2. Three decisions taken while building them are worth
keeping:

- **Membership is stored by name, and unresolvable members are dropped.** A fact or action
  that is deleted stops being a member, and a group left holding nothing is removed. This
  happens inside the same undoable step as the deletion, so one press of undo puts both
  back. The same pruning covers a member that a rename left pointing at nothing.
- **Nothing about a group's shape is stored.** The box is computed from where its contents
  are, and a closed group first appears in the middle of what it holds. Section 1 rule 5
  says the tool never asks the user to maintain something the model does not read, and a
  box that had to be dragged back over its own contents would be exactly that.
- **Anything can be in at most one group.** Two boxes each claiming to stand for the same
  fact would make closing them ambiguous. The refusal says so in a sentence rather than
  hiding the option, which is rule 2.

Everything else in this plan is an ongoing practice rather than an item:

**Concept mockups before code.** Any new screen gets a concept pack first, in the form the
two existing packs use: one standalone HTML page under [`doc/design/AME/`](../../design/AME/),
no external files, the tool's own theme values, real content from
`subprojects/AME/domains/mission_autonomy`, ending in the specific questions the review
needs answered. The format has earned its place — it settled the guided editor's wording
before any of it was built, and the simulation pack's six questions decided what B2 to B4
became. A pack whose questions nobody answers has failed and should not be repeated.

The D3 pack is
[`condition_authoring_hmi_concepts.html`](../../design/AME/condition_authoring_hmi_concepts.html):
how the guided editor says "must not be true", "any one of these", "every one of these" and
"two different ones" without using the words a planning textbook would.

---

## 5. What needs reviewing

### The walkthrough, which is a release gate

This is the acceptance test for the requirement in section 1. **It is run by someone who
does not write software, working alone, with no help and no documentation open.** A release
does not go ahead until every step covering something in that release has been completed by
such a person. A step that needs help is a defect against the tool, not against the user,
and the release waits for the fix.

The one exception: if nobody suitable is available, the person who owns the release may
accept the gap **in writing**, naming which steps were not run and why. That waiver is
recorded with the release and does not carry over to the next one. A developer running the
steps themselves is not a substitute — a developer cannot tell whether a screen makes sense
to someone who has never seen a planning model.

**Main session**, one sitting, in the order a real user meets these tasks.

| Step | Task |
|------|------|
| 1 | Open the tool and open a mission project saved yesterday, from the recent list |
| 2 | Find the action that makes a sector count as searched, using the relations panel |
| 3 | Add an action the domain did not previously have, using the guided editor |
| 4 | Rename a type that four actions and two scenarios already use, and check nothing broke |
| 5 | Rename the new action, decide against it, and undo with one keystroke |
| 6 | Fix the three problems the tool reports, by clicking each one |
| 7 | Build a starting situation of six facts for a scenario |
| 8 | Open the compiled tree for the scenario and say what every box on it is for |
| 9 | Run the scenario and watch it complete |
| 10 | Say when a particular fact stopped being true |
| 11 | Make one action fail, run again, and explain why the mission took a different route |
| 12 | Run the whole scenario set and say which one failed and why |
| 13 | Change one action, re-run, and say from the comparison what the change did |
| 14 | Export a review pack and name what each file in it is |

**Separate checks**, which need a second person, a second machine, or a document read away
from the tool. Each is still run by someone who does not write software.

| Check | Task |
|-------|------|
| 15 | Import a second domain into the project and see, before committing, which existing actions it would overwrite |
| 16 | Open a saved view by name and get the picture the author saved |
| 16a | Draw the communications-related part of a domain as one named box, close it, and still say what the rest of the picture shows |
| 16b | Open a domain written by someone else that uses a negative precondition and a "one of these" condition, say in their own words what each action needs, run a scenario against it, and export a review pack (workstream D) |
| 17 | Mark a fact as a contingency and another as a safe state, run the analysis, and read whether the safe state is reachable |
| 18 | Save a run, send the folder to a colleague, and have the colleague step through the same mission |
| 19 | Read a concept mockup pack in a browser and answer its questions without asking what anything means |
| 20 | Read the generated assurance report without opening the tool and say which parts of the mission have been checked |

### Risks that stay live

| Risk | Likelihood | Impact | What we do about it |
|------|-----------|--------|--------------------|
| Simulated behaviour diverges from real execution, so a mission that works in the tool fails in the field | Medium | High | The simulation uses the real compiled tree, the real world model and the real planner; the substitution is one stand-in node per action and nothing else, given the same preconditions and effects a deployed node would get. The difference is that the stand-in predicts the state that follows an action where a deployment establishes it. Each substitution is visible with its settings shown, and the tool never claims a run proves field behaviour |
| Non-specialist acceptance gets signed off by developers | Medium | High | The walkthrough above is a release gate with a written-waiver exception, and nothing else. Two defects found so far were of exactly the kind only this gate catches: a cascading type rename that was written and tested but had no button calling it, and a way to make a fact false that existed, worked and was tested, but whose button read the same as the button beside it. Neither is a bug a test suite can see |
| The tool falls behind the core's PDDL again, because it keeps its own reader and generator | Medium | High | Workstream D records every accepted construct and has a reader-generator round-trip case for each, followed by a parse through `PddlParser`. Adding a core construct without extending that contract is now a visible test change rather than silent drift |
| Simulation settings per action become a modelling burden of their own | Medium | Medium | Defaults that need no configuration: four ticks, always succeeds. Settings are opened only for a specific fault, or an action whose duration matters to the mission |
| Fault injection grows into a scripting language | Medium | Medium | Two mechanisms only, fixed: fail an action on a given attempt, and set a fact at a given tick. Anything more expressive is a separate decision, not a slow accumulation |
| The timeline and tree views drift apart from DevEnv's, so the two tools disagree | Low | Medium | Both read the same three JSONL schemas, and the files are interchangeable in both directions, which is testable |
| Assurance evidence is read as stronger than it is | Medium | Medium | The generated report always ends by stating its own limits: every run is a simulation of stand-in actions, a passing scenario means the model matched what somebody wrote down rather than that what they wrote down was right, and it counts the scenarios declaring no contingency and the actions with nothing bound |
| The upstream node-editor library stays quiet | Medium | Low | It is MIT-licensed and stable, the fork maintained as part of ImGui Bundle is the more active line, and nothing outstanding needs new features from it |

### Two decisions worth re-reading before extending the tool

**The headless commands live outside the graphical binary.** `ame_authoring_tool` links SDL2
and OpenGL, so anything headless that lives there forces a build agent to carry a display
stack. New headless work belongs in `ame_mission_cli`, whose conventions match the
contingency verifier. The graphical tool keeps `--self-test` alone, because that genuinely
needs a window. The machine-readable report is written only when `--json` names a file:
standard output belongs to the summary a person reads, and the planner writes progress
lines there too.

**The contingency search is one algorithm.** The exhaustive safe-state reachability search
and its monotonicity pruning live in `ame_core`. The verifier, the authoring tool's analyser
and the assurance report all call it. Do not grow a second copy — two implementations of the
same proof with nothing checking they agree is worse than either alone.
