# AME Authoring Tool — Plan

**What this is.** The single plan for the AME graphical authoring tool. It states what
the tool is for, what it can already do, and what is left to build.

**What it replaces.** This document supersedes `graphical_authoring_tool_workplan.md`,
which recorded every work item from the first line of build code onwards, each with a
tick box and a note about what was deferred. That per-item history has served its
purpose and has been folded into section 3, which describes the tool as it stands
rather than the order in which it was built. The reasoning behind the design, and the
survey of comparable tools, stays where it belongs, in the research documents listed
below.

**Background reading.**

| Document | What it gives you |
|----------|-------------------|
| [`doc/research/AME/graphical_autonomy_authoring_tool_options.md`](../../research/AME/graphical_autonomy_authoring_tool_options.md) | Why a tool was built rather than adopted, and the licence constraints |
| [`doc/research/AME/graphical_authoring_option1_extend_existing.md`](../../research/AME/graphical_authoring_option1_extend_existing.md) | Why the tool is a C++ application linked against `ame_core` |
| [`doc/research/AME/pddl_authoring_usability_and_navigation.md`](../../research/AME/pddl_authoring_usability_and_navigation.md) | The navigation and non-programmer authoring research, with prior art and sources |
| [`doc/design/AME/pddl_authoring_hmi_concepts.html`](../../design/AME/pddl_authoring_hmi_concepts.html) | The last round of screen concepts, which the current views were built from |
| [`subprojects/AME/doc/guides/authoring_tool_user_guide.md`](../../../subprojects/AME/doc/guides/authoring_tool_user_guide.md) | How to build, launch and use what exists today |

---

## 1. Purpose and audience

The authoring tool is a local desktop workbench for creating, reviewing and checking
AME mission models. It keeps a structured project as its working format, generates
PDDL from that project, and uses the same parser, world model, planner and plan
compiler as the runtime stack, so what the tool says about a model is what the runtime
will do with it.

Its users are mission and systems engineers, safety and assurance reviewers, and
operational subject-matter experts. Some of them write software. Most of them do not,
and the ones who do not are the people the tool exists for. An autonomy developer can
already write PDDL in a text editor faster than any graphical tool will let them; that
is a consistent finding in the published evaluations, and this plan does not try to
beat it. The tool earns its place by letting people who cannot write PDDL contribute
to a mission model safely, and by letting reviewers understand a model they did not
write.

---

## 2. The requirement that outranks the others

**A person who does not write software must be able to author, run and review a
mission model without help.** Every work item in this plan is measured against that
before it is measured against anything else. In practice it means five rules.

1. **No planning vocabulary appears as a label.** The words "precondition", "add
   effect", "delete effect", "fluent", "grounding" and "predicate" belong in the
   generated PDDL, not on a form. The screen says "Before it can happen", "Afterwards",
   and "fact". The generated PDDL stays visible for anyone who wants it.
2. **An illegal choice is impossible to express, not reported afterwards.** Every slot
   the user fills in is a list restricted to what is legal at that point. Choices that
   are not legal stay visible, greyed, with the reason beside them, so the user learns
   the rule rather than wondering where the option went.
3. **Every failure is explained in terms of the mission, not the machinery.** "No plan
   found" is the least useful message a planning tool can produce. The tool says which
   fact could not be brought about, what would have produced it, and offers the two or
   three edits that would fix it.
4. **Every view is readable without training.** If a screen needs a briefing before a
   reviewer can use it, the screen is wrong. Views are judged by whether someone seeing
   them for the first time in a review meeting can follow the discussion.
5. **The tool never asks the user to maintain something the model does not use.** Hand
   arranged node positions and hand-drawn links that no generator reads are work the
   user does for nothing. Anything that can be computed is computed.

**How this is checked.** Each work item below carries a *non-specialist acceptance*
line: a task, phrased as a mission question, that someone with no planning background
must be able to complete unaided. These are not a substitute for unit tests; they are
the acceptance criteria that decide whether the item is done. Section 9 collects every
one of them into a walkthrough, and that walkthrough is a release gate. Section 9 also
states the single exception, which needs a named person to accept it in writing.

---

## 3. What exists today

The tool builds as `ame_authoring_tool` behind the `authoring` CMake preset, on Windows
with MSVC 2022 and on Linux with GCC or Clang. It has a headless `--self-test` mode
that renders frames off-screen and writes a PNG plus a JSON result, so an automated
agent can check the user interface without a display attached. Source lives in
[`subprojects/AME/src/authoring`](../../../subprojects/AME/src/authoring); its own test
suites run under the `authoring-release` and `authoring-debug` test presets.

| Capability | What works | Where |
|------------|-----------|-------|
| **Project format** | Versioned JSON project holding types, objects, facts, actions, scenarios, state groups and behaviour-tree bindings, with round-trip tests | `project_model.cpp` |
| **Authoring a domain** | Type hierarchy and objects in the sidebar; facts and actions as nodes; guided sentence editor for actions with type-aware dropdowns; undo and redo for adding and deleting | `app_shell.cpp`, `guided_editor_model.cpp`, `type_hierarchy_panel.cpp` |
| **Navigation** | Relation index over the whole model; relations panel with back and forward history; focused neighbourhood canvas with one- and two-step depth and a neighbour cap; whole-domain canvas retained with semantic zoom | `relation_index.cpp`, `neighbourhood_model.cpp`, `domain_graph_panel.cpp` |
| **Review views** | Fact-by-action matrix with CSV and Markdown export; object lifecycle view derived from user-declared groups of alternative facts | `fact_action_matrix.cpp`, `lifecycle_model.cpp` |
| **PDDL** | Generation with live preview and export; import of existing domain and problem files through a dedicated importer that preserves names and structure, including which facts the domain says must be observed rather than predicted | `pddl_generator.cpp`, `pddl_importer.cpp` |
| **Checking** | Four levels: continuous structural checks while editing, parse through `PddlParser`, grounding through `WorldModel`, and feasibility through `Planner` | `structural_validator.cpp`, `pddl_validator.cpp` |
| **Scenarios** | Scenario list with starting facts, goals and planning and execution expectations; simulated batch regression with progress, Stop, a results table and a JSON report | `scenario_runner.cpp` |
| **Contingency** | Reachability analysis run in process against the generated model, with a per-context results table. The exhaustive search and its pruning are `ame_core`'s `ContingencySearch`, shared with the contingency verifier | `contingency_analyser.cpp`, `ame/contingency_search.h` |
| **Command line** | `ame_mission_cli` runs, records or batch-checks a project's scenarios without a window, following the same conventions as the contingency verifier | `mission_cli_main.cpp`, `mission_commands.cpp` |
| **Previews** | Plan view as a causal graph with computed layers; compiled behaviour tree view parsed from the generated XML | `plan_graph_panel.cpp`, `bt_graph_panel.cpp` |
| **Failure explanation** | Backward chain from a failed goal to the first fact nothing produces, with buttons that apply the likely fixes | `failure_explainer.cpp` |
| **Assurance evidence** | A generated report of what has been checked about a model and what has not, from the File menu, in the review pack, and from the command line | `assurance_report.cpp` |
| **Behaviour-tree binding** | Per-action node type, reactive flag and subtree template with parameter placeholders, previewed against stand-in arguments | `app_shell.cpp` |
| **Simulation runs** | A scenario runs inside the tool against the real compiled tree, with stand-in action nodes; run controls, per-action run settings, the same runs from the command line, and a saved random seed | `simulation_engine.cpp`, `app_shell.cpp` |
| **Watching a run** | A timeline of what happened when, a panel of the facts as they stand, the compiled tree coloured by what each node is doing, and any earlier moment reachable by clicking the timeline | `app_shell.cpp`, `bt_graph_panel.cpp` |
| **Faults and replanning** | A run can fail one action attempt or set one fact at a tick; lifecycle alternatives remain consistent, failed steps replan from current state, and the abandoned and replacement plans are shown together | `simulation_engine.cpp`, `app_shell.cpp` |
| **Recorded runs and replay** | A simulated run can be saved in the runtime's three JSONL formats with a provenance manifest; recordings from this tool or a real system replay through the same timeline, facts and tree views without an open project | `run_record.cpp`, `app_shell.cpp` |
| **Comparing runs** | The current run or two saved runs can be compared by their first different tree tick, different actions and different final facts, with the result stated first in one sentence | `run_record.cpp`, `app_shell.cpp` |
| **Review packs** | One export writes a dated, indexed folder containing all scenario PDDL, both matrix formats, scenario results, a replayable run and a domain summary | `review_pack.cpp` |

**Known gaps carried into this plan.** Most of these are now closed; the table in section 11
says where each one went, and the items in section 5 record what was built.
Validation messages name the offending element but are not clickable. Importing a
domain replaces the project rather than merging. Problem export writes only the first
scenario. Scenario facts are typed as text rather than chosen from the grounded set.
Types cannot be renamed and action parameters cannot be reordered. Named presentation
groups and saved named views were both deferred. Workstream A picks these up; section 11
lists every unfinished item from the previous plan and says where each one went.

**The largest simulation gap has been closed.** Workstream B is complete: the compiled tree
runs in the tool, faults can trigger replacement plans, scenario batches check execution,
and runs can be saved, replayed and compared through the same runtime-compatible records.
The main remaining review gap is C3: generating one assurance report that ties the model to
its scenario, contingency and behaviour-tree evidence. The smaller authoring and file-handling
gaps listed above remain in workstream A.

**One thing the tool inherits from the core needs fixing before that work starts.** The
tree the tool shows is the tree the plan compiler produces, and that tree currently
carries a pair of helper nodes around every action, one reading a fact and one writing
one. They were written to get the pipeline working end to end. They are the wrong shape
both for the screens in workstream B and for a deployed system, for reasons set out in
B0 below, and B0 is where they are dealt with.

---

## 4. Scope

Three workstreams, in dependency order but overlapping in time.

- **A — Finish the authoring surface.** Close the gaps above so the tool stops asking
  users to work around it.
- **B — Simulation runs.** Let a user run a mission model inside the tool and watch it
  execute, in the way DevEnv shows a live system and in the way Groot showed a running
  behaviour tree, without depending on either.
- **C — Artefacts for review and evidence.** Make the tool produce the documents that
  reviews and assurance cases actually need, and keep designing screens the way the last
  round was designed: concept mockups first, reviewed, then built.

**Non-goals, unchanged from the previous plan and still deliberate.**

- Web or browser deployment. The tool is a local desktop application.
- ADL, conditional effects, numeric fluents or temporal PDDL. AME supports STRIPS with
  typing, and the tool targets exactly that subset.
- Replacing DevEnv. DevEnv keeps live monitoring of real ROS2 systems; the authoring
  tool simulates offline.
- Live ROS2 connection from the authoring tool. Simulation runs are in-process against
  the world model, with no transport involved.
- Multi-user collaboration and concurrent editing.
- Graph-first authoring, where a domain is built by wiring nodes together. Every
  published evaluation that measured it found it slower than typing, and it does not
  help the people the tool is for. The graph is for looking, not for writing.

### Two decisions taken while building workstream B

Both came from the same question: how the tool's own command line relates to AME's
existing command-line tooling, which is the contingency verifier described in
[`contingency_verifier.md`](../../../subprojects/AME/doc/guides/contingency_verifier.md).
Neither was foreseen when this plan was written, and both change items below.

**The headless work moves out of the graphical binary.** Running a mission without a
window began as a flag on `ame_authoring_tool` and grew into three. That binary links
SDL2 and OpenGL, so a build agent had to carry a display stack to run a computation that
never opens a window, and its flags had drifted from the verifier's: `--report` against
`--json`, no `--help`, a different idea of what the exit code means. The headless commands
therefore become their own target, `ame_mission_cli`, linking only the core and the
project-model library, with `run`, `record` and `batch` subcommands, `--json` for the
machine-readable report, and an exit code that states the verdict, as the verifier's does.
The graphical tool keeps `--self-test`, which genuinely needs a window. This is item B8.

**The contingency search becomes one algorithm.** The verifier already does exhaustive
safe-state reachability with monotonicity pruning, in an application file of some 1600
lines. A8 as written below asked for monotonicity pruning to be added to the authoring
tool's own analyser, which would have produced a second implementation of the same proof
with nothing checking that the two agree. Instead the search moves into `ame_core`, and
the verifier, A8 and the assurance report in C3 all call it. A8 keeps its user-facing
work — per-scenario declarations of what counts as a contingency and what counts as a
safe state, and reporting how much of the space was covered — and loses the algorithm.

---

## 5. Workstream A — Finish the authoring surface

### A1. Files and projects behave the way users expect

**This item is done.** The dialogs were already there; what was missing now exists. `File`
has a `Recent projects` submenu holding the last eight, most recent first, with anything that
has since been deleted dropped when the list is read, so the menu never offers a path that
cannot be opened. Closing with unsaved work asks first, offering to save, to close anyway, or
to keep working. A recovery copy is written beside the project every half minute while there
are unsaved changes and removed once the project is saved, so a tool that stops unexpectedly
has not taken the day's edits with it.

The list belongs to the person rather than to any project, so it is kept with the user's own
settings. A settings file this tool cannot read is ignored rather than fatal: the list is a
convenience, and it is written afresh on the next open.

**Build.** Native file dialogs for New, Open, Save, Save As, Import and Export, which are
done. A recent-projects list, a prompt on quitting with unsaved changes, and autosave of a
recovery copy alongside the project file, which are not.

**Non-specialist acceptance.** A user opens the tool, opens a project they saved
yesterday from the recent list, changes it, closes the tool, and is asked whether to
save. No file path is ever typed.

**Effort.** Small.

### A2. Undo covers everything, including typing

**This item is done.** `CommandStack::executeCoalescing` takes a key naming the field being
typed in, and folds a run of keystrokes in one field into a single undoable step. Typing five
letters leaves one entry on the stack, and one press of undo puts back the name the field
held before the typing started rather than removing one letter. Moving to another field, or
using undo, ends the run, so returning to a field later starts a new step. The Edit menu says
what would be undone — "Undo rename type" rather than "Undo" — using the label the stack
already carried.

**Build.** Extend the command stack to text edits and dropdown changes, coalescing a
run of keystrokes in one field into a single undoable step. Show the name of the next
undoable action in the Edit menu, so the menu says "Undo rename action" rather than
"Undo".

**Non-specialist acceptance.** A user renames an action, decides against it, presses
the undo shortcut once, and the old name is back.

**Effort.** Small to medium.

### A3. Problems point at the thing that is wrong

**This item is done.** The checkers' output is a list of rows rather than a block of text.
Each row is one sentence, and clicking it selects the element it names and opens the Domain
tab on it. A parser failure now reads as "The action 'move' could not be read back from the
generated PDDL", with the parser's own words kept behind a details toggle, because those
words describe positions in text the user never saw. The raw block is still one click away
for anyone who wants it, and "Copy diagnostics" is unchanged.

The wording is done in `problem_list.cpp` rather than in the user interface, so it is
testable without a window, and one of those tests asserts that the word "predicate" appears
nowhere in the list. The checkers keep saying "predicate" among themselves, because that is
what the generated PDDL calls it; the list a non-specialist works through cannot be the one
screen that does.

**Build.** Make every entry in the validation list select and reveal the element it
refers to, in whichever view is open. Replace parser error text with a sentence naming
the element and what is wrong with it, keeping the raw parser message available behind
a details toggle.

**Non-specialist acceptance.** A user is shown a project with three deliberate faults,
clicks the first problem in the list, lands on the offending action, fixes it, and
watches the entry disappear.

**Effort.** Small.

### A4. Scenario facts are chosen, not typed

**This item is done.** A scenario's starting facts and goals are built by choosing: pick the
fact, then pick each thing it involves from a list holding the objects of the right type.
Objects of the wrong type stay in the list, greyed, with the reason on hover — "base is a
location, and this has to be a sector" — so the rule is learned rather than hidden, which is
rule 2 of section 2. The Add button is disabled until the choice is a fact the project
actually has, and says what is missing when it is not.

Typing is kept, behind "Type it instead", and is held to exactly the same rules through the
same function. It accepts the three forms people write: `(at uav1 base)` as the generated
PDDL has it, `at uav1 base` as it is said aloud, and `at(uav1, base)` as most programming
languages write it. The third of those was a defect found by its own test, which is a good
argument for the rules living in `fact_chooser.cpp` where they can be tested rather than in
the drawing code where they cannot.

**Build.** Replace the free-text fact rows in the scenario editor with a chooser over
the grounded fact set: pick the fact, then pick each object from a list restricted by
the parameter type. Keep a text entry as an alternate path for users who prefer it, and
validate it against the same set.

**Non-specialist acceptance.** A user sets up a starting situation of six facts for a
domain they did not write, without knowing what a fluent is, and cannot produce a fact
that does not exist.

**Effort.** Medium.

### A5. Import merges instead of replacing

**This item is done.** Importing a domain into a project that already has one now shows what
the import would do before doing any of it: what would be added, what would be overwritten
and what is already the same, with the cost of each replacement stated — "losing 3 conditions
and 2 outcomes". Adding is never a choice, because it takes nothing away; replacing is, per
kind of thing. Importing into an empty project still just imports.

Two decisions worth recording. Replacing an action keeps the things the imported PDDL cannot
know about: its behaviour-tree binding, its run settings and where it sits on the canvas. All
three are this project's work, and losing them would mean rebinding every action after every
import. And imported elements are now laid out by what they have to do with each other, each
action in a column with the facts it uses beneath it, rather than in the two long rows that
said nothing about the domain. Every scenario is exported as its own problem file, into a
folder the user picks.

**Build.** On importing a domain into a non-empty project, show what would be added,
what would be replaced and what would conflict, and let the user choose per group.
Export every scenario as its own problem file rather than only the first. Lay imported
elements out by following the relationships between them, rather than the two long rows
of facts and actions that import produces today.

**Non-specialist acceptance.** A user imports a second domain file into an existing
project and can see, before committing, exactly which of their existing actions it
would overwrite.

**Effort.** Medium.

### A6. Named groups and saved views

**Partly done: saved views are built, presentation groups are not.** A saved view stores what
was in focus, how far out from it the view reached, which relationships were shown and which
view was open, under a name held in the project file, so it survives reopening and travels
with the project to whoever else opens it. Opening one by name puts the focus back where it
was. That is the half of this item the walkthrough exercises, and the half a review meeting
needs.

Presentation groups — a named set of facts and actions drawn as a labelled box that collapses
to one node — are stored in the project format and round-trip through it, but nothing draws
them yet. The drawing belongs to the whole-domain canvas, which is the view this plan says is
for looking rather than authoring, and it is the one piece of workstream A left unbuilt.

**Build.** Presentation groups, which are a named set of facts and actions drawn as a
labelled box that collapses to a single node on the whole-domain canvas. Saved views,
which store a focus element, a depth and a relationship filter under a name, held in
the project file so they survive reopening. Both were deferred from the previous plan;
both exist to turn the canvas into a set of small curated diagrams for review meetings,
which is how stakeholders will actually use it.

**Non-specialist acceptance.** A reviewer opens a project, picks the saved view called
"communications loss", and sees the same picture the author saw when they saved it.

**Effort.** Medium.

### A7. Editing operations that are still missing

**This item is done.** Of the seven, one turned out to exist already: the behaviour-tree view
has had a toggle on every node that holds others, alongside Collapse all and Expand all, so a
large tree could always be read a branch at a time. The other six are now built:

| Wanted | What it does now |
|--------|------------------|
| Rename a type | A rename button beside each type, reaching everything that names it: the things of that type, the parameters that ask for it, the types below it, and the state groups about it, in one undoable step. A rename that would clash says so before it is attempted |
| Reorder an action's parameters | Up and down beside each one. The conditions and outcomes name their parameters, so they follow the move untouched |
| Add a type from the canvas menu | The entry that did nothing now opens the same kind of box the other two use |
| Copy and paste | `Ctrl+C` and `Ctrl+V`, and Edit menu entries that name what would be pasted. The copy is given a name nothing else is using, and lands where it can be seen rather than on top of the original |
| Fit the canvas to its contents | `F` |
| Quick-add from the canvas | `A` |

Together these finish the keyboard-only path, which the previous plan claimed and did not
deliver.

**Build.** Seven small things a user reasonably expects and cannot currently do:
rename a type; reorder an action's parameters; add a type from the canvas menu, where
the entry exists today but does nothing; copy and paste facts and actions, rather than
only duplicate; fit the canvas to its contents with a key; open the quick-add box from
the canvas rather than only from the palette; and expand or collapse a subtree in the
behaviour-tree view, so a large tree can be read a branch at a time.

Together these also finish the keyboard-only path, which the previous plan claimed and
did not deliver: with them in place, a user can create, edit, navigate and check a
domain without reaching for the mouse.

**Non-specialist acceptance.** A user renames a type that is already used by four
actions and by objects in two scenarios, and nothing else in the project breaks.

**Effort.** Medium.

### A8. Contingency analysis the user controls

**This item is done.** A scenario can declare which facts represent a contingency and which
facts count as a safe state. Declaring neither infers both, as the tool always did, so a
project that has never been near this screen still gets an answer. Declaring the contingency
narrows what is varied, which is what makes a large domain checkable at all: the eight-fact
cap is gone, and a domain still too large to enumerate is refused with the numbers and the
advice to say which facts matter, rather than with "too many".

The search itself is `ame_core`'s, shared with the contingency verifier, per the decision in
section 4. The results now say how much of the space was covered and how — how many
combinations were planned for, how many followed from those without planning, and whether
anything could be carried between them at all, since a domain with conditions about facts
being false makes that unsound.

**Build.** Let the user say, per scenario, which facts represent a contingency and
which represent a safe state, instead of the tool inferring them from where they appear
in the model. Keep the inference as the starting suggestion. Say in the results how much
of the space was covered.

The search itself is not built here. Section 4 records the decision: the exhaustive
search and its monotonicity pruning move out of the contingency verifier into `ame_core`,
and this item calls that shared search rather than growing a second copy of it. Doing the
extraction first is therefore a prerequisite for this item.

**Non-specialist acceptance.** A user marks "communications lost" as a contingency and
"at the recovery point" as a safe state, runs the analysis, and reads whether the safe
state is reachable from the contingency.

**Effort.** Medium.

---

## 6. Workstream B — Simulation runs

This is the substantial new work. Today a user can establish that a plan exists. They
cannot see it play out, cannot see what happens when a step fails, and cannot show a
reviewer the mission behaving. Two existing pieces of the project show what that should
feel like:

- **DevEnv** ([`subprojects/AME/tools/devenv`](../../../subprojects/AME/tools/devenv))
  shows a live system: the behaviour tree with per-node status, a timeline of status
  changes, world-model facts as they change, tick controls, and a replay mode that
  loads recorded runs from the three JSONL files the runtime writes.
- **Groot** was the BehaviorTree.CPP tool that showed a tree lighting up as it ran.
  Groot2 is commercial and Groot 1 is unmaintained, which is why AME's observability
  stack was built to replace it (see
  [`subprojects/AME/doc/architecture/05-observability.md`](../../../subprojects/AME/doc/architecture/05-observability.md)).
  The live tree view described below is the replacement for what Groot did, built on
  AME's own event stream, with no dependency on either version.

The authoring tool should give the same experience against a simulated world rather
than a real one, at design time, before anything is deployed.

### B0. Each planned action carries its own state checks

**This item is done.** It was core work rather than authoring-tool work, and it is
recorded, along with the follow-ups a review of it raised, in
[`doc/todo/AME/TODO.md`](../../todo/AME/TODO.md) under "Planned-action contract:
follow-ups". The rest of this item is kept as written because it states why the
simulation-run screens needed the change, and because the walkthrough in section 9 still
checks it. One caveat for workstream B: an action with no registered implementation now
aborts the compile unless stub mode is switched on, which the authoring tool does.

**The problem.** The plan compiler surrounds every action with helper nodes: one
`CheckWorldPredicate` for each of the action's preconditions before it, and one
`SetWorldPredicate` for each add effect and each delete effect after it
([`plan_compiler.cpp`](../../../subprojects/AME/src/lib/plan_compiler.cpp), function
`emitActionUnit`). Those nodes were written to get planning and execution working from
end to end, and they have two problems.

The first is a presentation problem, and it is the smaller of the two. A three-step
mission arrives on screen as roughly twenty boxes, of which three are the mission and
the rest are bookkeeping. Their labels are the planning vocabulary that section 2 rule 1
keeps off the screen, and a reviewer watching a run has to be told which boxes to
ignore before the picture means anything, which is what section 2 rule 4 exists to
prevent.

The second matters more. Writing an effect as a node in the tree asserts that the world
changed because the action returned success. For a simulated run that is reasonable:
predicted state is the only state there is. For a deployed system it is not. Whether the
vehicle actually arrived is something that deployment establishes for itself, and the
compiled tree should not be asserting it on the action's behalf. The world model already
draws the distinction the two cases need — every fact is recorded as either believed,
meaning predicted from a plan effect, or confirmed, meaning observed (`FactAuthority` in
[`world_model.h`](../../../subprojects/AME/include/ame/world_model.h)) — and it already
has a check for the two disagreeing. What is missing is a place for an action node to
make that choice, because at present the tree makes it first.

**Build.**

- The compiler emits one node per plan step. The step's grounded preconditions and
  effects travel with that node instead of being spread around it, so the tree has one
  box per thing the mission does and nothing else.
- A planned-action base class in `ame_core` reads that information, checks the
  preconditions before the action's own work runs, and applies the effects after it
  succeeds. Concrete action nodes derive from the base class and implement only the
  work. Neither the check nor the write is a node on the tree any more.
- Simulation and validation use a stand-in node derived from that base class. It takes
  the number of ticks the action is configured to take, succeeds or fails as configured,
  and applies the declared effects as believed facts. This is what the tree does today,
  moved inside the node.
- The goal guard at the top of a compiled tree is the other place `CheckWorldPredicate`
  appears. It becomes a single condition node asking whether the mission's goal has been
  met, so no fact-level plumbing is left anywhere in generated output.
- `CheckWorldPredicate` and `SetWorldPredicate` then appear in no generated tree. No
  hand-written tree in the repository uses them either; they occur only in compiler
  output and in the tests that assert on that output, so they can be withdrawn from
  mission execution once the compiler and the executor are updated.

**What this deliberately does not do.** It does not decide how a deployed system
establishes the state after an action. That is the deployment's business: a real
deployment supplies its own action nodes and already confirms, in whatever way suits the
system it is commanding, that the thing it asked for happened. Nothing in this plan or in
`ame_core` changes that. What the work above gives those nodes is the declared
preconditions and effects to work from, and a compiled tree that has stopped writing the
world model over their heads.

**How the information reaches the node.** Put it on the emitted element, as attributes
listing the fact names for preconditions, add effects and delete effects. The compiled
XML then still describes itself, which matters because the authoring tool's tree view,
DevEnv and every recorded run read that XML, and a saved tree should stay meaningful
without the project that produced it. The alternative, giving each node a step
identifier that keys into a table stored alongside the tree, keeps the XML shorter but
makes a tree file useless on its own; take it only if the attribute lists turn out to be
unworkable in practice.

**What it touches.** The compiler, the behaviour-tree node headers, the executor
component and the ROS2 executor node, together with seven test binaries and four
architecture files that describe the current tree shape. The core TODO entry lists them
individually. Most of the effort is in those tests and documents rather than in the
compiler itself.

**Non-specialist acceptance.** A user opens the compiled tree for a three-step mission,
sees three boxes named after the three things the mission does, and can say what every
box on the screen is for.

**Effort.** Medium.

### B1. Simulation engine

**This item is done.** The engine is
[`simulation_engine.cpp`](../../../subprojects/AME/src/authoring/simulation_engine.cpp), a
Run tab holds the controls, and `ame_mission_cli` runs the same scenarios without a
window (item B8 moved those commands out of the graphical tool). The rest of this item is kept as written because it states what
the engine is for. Four things about the finished work are worth stating, because they were
decisions rather than transcription, and one of them is a finding.

- **Every action is bound to the stand-in, and nothing else changes.** The run builds the
  same action registry the tool builds for the behaviour-tree preview, except that each
  action's node type is the stand-in. The binding's reactive flag is kept, because it
  decides when preconditions are re-checked and so decides what the run does. A compiled
  tree for a run therefore has the shape a deployment would run, with one box per thing the
  mission does, and no node that talks to an external system is ever built.
- **The scenario's starting facts are recorded as observed; everything a run produces is
  predicted.** That is what the two authorities in the world model mean, and keeping the
  distinction makes a run honest about the case the domain cares about: an action whose
  preconditions the domain says must be observed will not proceed on a fact that only a
  plan effect produced. A run shows that refusal where a deployment would show it. There is
  a test for exactly this case.
- **The reports are written to a file, not to standard output.** The planner writes its own
  progress lines to standard output, so anything reading a run as JSON needs a channel it
  does not share.
- **The default duration is four ticks, not one.** This plan and the concept pack both said
  one tick. Building it showed why that does not work: a behaviour tree walks a sequence of
  actions that each finish immediately within a single tick, so a mission of one-tick
  actions starts and ends on tick one. The run is correct and the goals are met, but there
  is nothing to watch and no order visible on a timeline, which is what the screens are for.
  Nothing a real vehicle does finishes in one tick either. The default is now four ticks,
  which is one second at the run screen's default speed of four ticks a second, and the rule
  the default exists to serve is unchanged: a project that has never been near the run
  screen still runs without anybody configuring anything.

**Build.** An in-process runner that takes the compiled behaviour-tree XML the tool
already produces, builds it with a BehaviorTree.CPP factory, and ticks it against a
world model populated from the scenario's starting facts. Every action in the tree is
built as the stand-in node from B0, which checks the action's preconditions against the
world model, takes the configured number of ticks, and applies the action's effects as
believed facts. Real action nodes talk to external systems and are never built here. The
substitution follows the pattern the demo application already uses
([`subprojects/AME/src/apps/main.cpp`](../../../subprojects/AME/src/apps/main.cpp)),
with the difference that after B0 one stand-in covers every action instead of one being
written per action.

Each action gets simulation settings stored in the project beside its behaviour-tree
binding: how many ticks it takes, whether it succeeds, and optionally a probability of
failure with a fixed random seed so a run is reproducible. Defaults are four ticks and
always succeeds, so a user who never opens these settings still gets a working simulation
that can be watched. See the note above for why the default duration is not one tick.

Run control offers step one tick, run to completion, run at a chosen speed, pause and
reset. The whole engine is separable from the user interface and driven by a headless
entry point, so simulations can also run in the test suite and in continuous
integration.

**Non-specialist acceptance.** A user selects a scenario, presses Run, and watches the
mission complete, without configuring anything first.

**Effort.** Large.

### B2. Live tree view

**This item is done.** The behaviour tree has no tab of its own any more: the Run tab draws it,
sitting waiting when nothing is running and coloured as the run goes when something is.
Nodes take the four colours the rest of the tool uses, the node being ticked takes a bright
border, and a node that holds others reports what they are doing, so a finished branch reads
as finished at a glance. Steps are matched to nodes by the grounded signature the compiler
writes as the node's name, so the drawing and the run never have to agree on node ordering.

Clicking a node now opens its detail beside the tree. For an action, the detail names the
grounded action, the facts that must be true, observed, or false before it can run, and the
facts it makes true or false. The panel reads the action contract already stored on the
compiled XML node, so it explains the exact tree on screen.

**Build.** Extend the existing behaviour-tree view so that during a run each node is
coloured by its current status, the node being ticked is marked, and completed branches
are visibly finished. Because of B0 there is one node per action, so a user can click it
to see the action it came from, the facts that had to be true before it could run, and
the facts it changed. Nodes keep the plain-language names the guided editor gave them
rather than the generated identifiers.

**Non-specialist acceptance.** Someone watching over the author's shoulder can say
which part of the mission is happening now, and which parts already finished, without
being told how to read the picture.

**Effort.** Medium.

### B3. Facts and timeline

**This item is done.** The timeline is the primary run view, above the smaller tree. Each
action has its own row against a tick axis, so independent actions that run at the same time
appear as overlapping bars on separate rows. Fact changes are marked on the action that
caused them. The facts panel shows every grounded world-model fact, whether it is true at the
viewed tick, and when it last changed, with a name filter and a highlight on changes at that
tick.

Clicking the timeline moves the timeline playhead, facts, tree colours, goal state, and the
sentence that says what is happening to the selected tick. The engine reconstructs that view
from the scenario's starting facts, recorded fact changes, and action start and finish ticks;
it does not store a copy of the world at every tick. Looking back does not pause or move a live
run, and pressing Run or Step returns the view to the live tick.

**Build.** Two panels beside the tree, both fed from the same run.

- A **fact panel** listing the world-model facts, showing which are currently true,
  highlighting each change as it happens, and letting the user filter to one object or
  one fact.
- A **timeline** with one row per action, showing when it started and finished, and
  markers for fact changes. Clicking any point in the timeline moves the whole screen,
  tree and facts included, to that moment.

This is the same division of labour as DevEnv's execution and world-model tabs, applied
to a simulated run.

**Non-specialist acceptance.** A user is asked "when did the vehicle stop being at the
base?" and can answer it from the timeline.

**Effort.** Medium.

### B4. Fault injection and replanning

**This item is done.** Faults belong to the loaded run and survive Reset. The Run tab offers
exactly the two reviewed mechanisms: fail a selected action attempt once, or set a grounded
fact true or false at a selected tick. Setting an alternative state true clears the other
states in the project's declared lifecycle group. Before tick one, the screen names the
action expected to be under way, the precondition the fact change can remove, and the domain
actions that relate to that change. It also shows the saved random seed.

A failed tree is replanned from the world model as it stands. The replacement tree continues
the same run, while the timeline keeps the abandoned work. The Run tab states the reason and
shows the abandoned and replacement action lists together. Each planning attempt is counted
as a replan. If no replacement exists, the right-hand comparison uses the existing failure
explainer to identify a fact the domain cannot bring about.

**Build.** Let the user make a run go wrong on purpose, in the two ways that matter:
force a chosen action to fail on a chosen attempt, and change a fact part-way through
the run to represent something happening outside the mission's control, such as
communications being lost.

When a step fails, the tool does what the runtime does: replans from the world model as
it stands and compiles a new tree. The screen shows both the plan that was abandoned
and the plan that replaced it, with the reason for the change stated in a sentence. If
no new plan exists, the failure explainer from the existing tool takes over and says
which fact could not be brought about.

**Non-specialist acceptance.** A user makes one action fail, sees the mission recover
by a different route, and can say in their own words why the new plan is different.

**Effort.** Large.

### B5. Scenario simulation in batch

**This item is done.** Scenario expectations now cover goal completion, action-count bounds,
actions that must or must not run, the maximum acceptable replans, and a named run fault, while
the existing planning expectations keep their meaning. Older project files take defaults that
preserve their planning result and expect a feasible run to reach its goal.

The batch runner plans and simulates each scenario, writes a reason for every result, and uses
the same code from the graphical tool and from `ame_mission_cli`. In the graphical tool it runs one
scenario per frame. This was chosen over a worker thread because the existing application,
project model and planner are owned by the user-interface thread; yielding between scenarios
keeps the interface responsive and makes Stop immediate without adding shared ownership or
locking.

**Build.** Extend the existing regression runner so a scenario's expectations cover
execution as well as planning: whether the run reached the goal, how many actions ran,
which actions must or must not appear, how many replans were acceptable, and how the
run behaved under a named fault. Running the set produces a pass or fail per scenario
with the reason for each failure written out. Because a simulated run takes longer than
a planning check, the run reports progress as it goes and can be stopped part-way, which
the current planning-only batch run does not need to do and does not do.

**Non-specialist acceptance.** A reviewer runs the whole scenario set, sees eight
passes and one failure, and can read what the failing scenario expected and what it
actually did.

**Effort.** Medium.

### B6. Recorded runs, replayable in DevEnv

**This item is done.** Each simulation keeps the JSON emitted by `AmeBTLogger`, sends world
changes through `WmAuditLog`, and records the first plan and every replacement plan through
`PlanAuditLog`. Saving the run writes those three DevEnv files and `run.json`, which names the
project, scenario, seed and injected faults and carries `"simulated": true`. The optional
simulation tick added to each JSON object is ignored by DevEnv and lets the authoring tool
reconstruct its tick-based timeline exactly.

The Run tab opens a recording without requiring its project. A runtime recording with no
authoring manifest is treated as a real-system run and is labelled plainly as such; it is
never called simulated by inference. Both sources use the same timeline, facts and tree views.

**Build.** Every simulation run is recorded to the three JSONL files the runtime
already writes, using the same schemas: behaviour-tree events through `AmeBTLogger`
with a callback sink, fact changes through `WmAuditLog`, and planning episodes through
`PlanAuditLog`. Because DevEnv's replay loader
([`subprojects/AME/tools/devenv/models/replay.py`](../../../subprojects/AME/tools/devenv/models/replay.py))
reads exactly these three files, a run recorded in the authoring tool opens in DevEnv
with no conversion, and a run recorded from a real system opens in the authoring tool's
timeline the same way.

The tool gains a replay mode over any recorded run, its own or the runtime's, using the
same tree, fact and timeline views as a live simulation.

**Non-specialist acceptance.** A user saves a run, sends the folder to a colleague, and
the colleague opens it and steps through the same mission.

**Effort.** Medium.

### B7. Comparing runs

**This item is done.** The File menu compares the current in-memory run with a saved run, so a
user can check a domain change without first saving the new result. It can also compare two
saved folders. The Run tab leads with one sentence naming the first different tree tick, then
shows both run summaries, actions present on only one side, and facts whose final values differ.

**Build.** Put two recorded runs side by side and show where they diverge: the first
tick at which the trees differ, which actions differ, and which facts differ at the
end. The obvious uses are a run before and after a domain change, and a nominal run
against the same scenario with a fault injected.

**Non-specialist acceptance.** A user changes one action, re-runs a scenario, and the
comparison tells them in one line what their change did to the mission.

**Effort.** Medium.

### B8. One command line for everything a run can do

**This item is done.** `ame_mission_cli` is a separate target with `run`, `record` and
`batch` subcommands. It links the core and the project-model library and nothing else: `ldd`
reports no SDL and no OpenGL against the graphical tool's two, so a build agent needs no
display stack to run a mission. The work behind the commands lives in `mission_commands.cpp`
in the library, so the command line and the test suite drive the same code, and the tests
cover the built executable's exit codes as well, because that is the part a build agent
reads. The graphical tool now takes `--self-test` and nothing else.

One decision worth recording: the machine-readable report is written only when `--json` names
a file, rather than being printed when it is not. Standard output belongs to the summary a
person reads, and the planner writes progress lines there as well, so it is no place for a
document another program has to parse. This is what the verifier already did, and the earlier
`--report` flag on the graphical tool did not.

**Build.** A separate target, `ame_mission_cli`, holding every headless command: run one
scenario, record a run to a folder, and run the whole scenario set against its
expectations. It links the core and the project-model library and nothing else, so a build
agent needs no display stack to use it. Its conventions match the contingency verifier,
which is the tool AME already had: the project file is a positional argument, `--json`
names the machine-readable report, `--help` explains itself, and the exit code states the
verdict rather than merely reporting that the process ran. The graphical tool keeps
`--self-test` alone, because that is the only headless thing it does that needs a window.

Section 4 records why this exists. The point is not tidiness: the same three questions
about a mission model — is it feasible, does it run, does it stay safe under every
contingency — should be answerable from a script in three commands that look alike.

**Non-specialist acceptance.** This item has none, and needs none: its users are build
agents and the engineers who write their scripts. The walkthrough in section 9 is
unaffected.

**Effort.** Small to medium.

---

## 7. Workstream C — Artefacts

### C1. Concept mockups before code

**This item is done for the screens built so far.** The simulation pack was written, reviewed
and answered; the answers are in the table below and the build followed them. The practice
itself stands for whatever is drawn next: a pack before the code, ending in questions, and a
pack whose questions nobody answers has failed.

**Build.** Every screen in workstream B gets a concept mockup before it is implemented,
in the form the last round used: a single standalone HTML page under
[`doc/design/AME/`](../../design/AME/), self-contained with no external files, drawn
with the tool's own theme values, using real content from
`subprojects/AME/domains/mission_autonomy`, and ending with the specific questions the
review needs answered. That format worked. It got the wording of the guided editor
settled before any of it was built, and it found two real properties of the mission
domain while it was being drawn.

The pack for the simulation screens is written and is at
[`doc/design/AME/simulation_run_hmi_concepts.html`](../../design/AME/simulation_run_hmi_concepts.html).
It has six concepts, one for each item from B1 to B7: run controls and the live tree,
the fact panel and timeline, the fault-injection controls, the abandoned-plan and
new-plan comparison, the batch results table, and run records with the comparison of two
runs. It asks six questions, the first three being the ones this plan committed to
asking — what the run controls are called, whether the timeline or the tree is the
primary view, and how a change of plan is announced without using the word "replan".

**All six have been answered, and the answers are recorded in the pack beside the question
they settle.** They are repeated here because they decide what B2, B3 and B4 build.

| Question | Decision |
|----------|----------|
| What the run controls are called | The ordinary machine words: **Run, Pause, Step, Stop, Reset**. The people who use this are technical, just not software engineers, and those are the words they use. The mission-word proposal is dropped |
| Timeline or tree as the primary view | **The timeline.** The tree stays beside it, as the answer to "where are we" rather than as the thing a reviewer reads first |
| How a change of plan is announced | **"Replan" is allowed on the screen**, for the same reason as the control names. The plain sentence is still worth writing; it no longer has to avoid the word |
| A separate Run tab, or the behaviour-tree tab becoming it | **The behaviour-tree tab becomes the Run tab.** There is no separate tree tab, so the compiled tree is a thing that runs rather than a thing you inspect |
| Whether a simulated run and a real run look identical | **They look the same, and the data says which is which.** Every run this tool produces carries a `simulated` marker in its report and its record |
| Whether two fault mechanisms are enough | **Yes.** Failing an action and setting a fact part-way through are adequate. No third mechanism is planned, and B4 builds exactly these two |

All six decisions are built, including the `simulated` marker in both reports and records.

**Non-specialist acceptance.** A reviewer reads the pack in a browser with no tools
installed and can answer its questions without asking what anything means.

**Effort.** Medium per pack.

### C2. Exports that reviews can use

**This item is done.** `File > Export Review Pack...` writes one dated folder with an index,
the generated domain, one clearly named problem file for every scenario, the fact-by-action
matrix as CSV and Markdown, a Markdown scenario results table, a recorded run folder, and a
domain summary. The summary lists the types, objects, facts, actions and facts that no action
makes true. The existing individual PDDL, regression and matrix exports remain available.

**Build.** Consolidate exporting into one place, covering: the generated domain and
every scenario's problem file; the fact-by-action matrix as CSV and Markdown, which
already exists; the scenario results table; a recorded run folder; and a domain summary
listing types, objects, facts, actions and the facts nothing produces. Everything
written in one operation into one dated folder, so a review pack is one action rather
than nine.

**Non-specialist acceptance.** A user exports a review pack before a meeting and can
say what each file in it is from its name alone.

**Effort.** Small to medium.

### C3. Evidence for the assurance case

**This item is done.** `AssuranceReport::generate` writes Markdown a reviewer reads without
opening the tool: what the model contains, the facts nothing in the mission brings about and
which actions rely on them, what each action is bound to, the facts an action may act on only
once they have been observed, how every scenario behaved, and what stayed reachable under
each declared contingency. It is reachable three ways: a File menu entry, the review pack in
C2, and `ame_mission_cli evidence`.

**Its last section is the one that matters.** A report that answers only the flattering half
of "what does this cover" is not evidence, so the report always ends by saying what it does
not tell you: that nothing in it is evidence about the field, since every run is a simulation
of stand-in actions; that a scenario behaving as expected means the model matched what
somebody wrote down, not that what they wrote down was right; how many scenarios declare no
contingency, so nothing has been checked about them going wrong; and how many actions have
nothing bound to them. On the vehicle-autonomy project as it stands, that section reports 17
actions with nothing bound and the single scenario declaring no contingency — which is a
truer picture of that model's assurance position than any prose summary would have given.

**Build.** A generated report tying a mission model to the claims made about it: which
scenarios were run, which passed, which contingencies were checked and what was
reachable, which facts nothing produces, and which actions are bound to which
behaviour-tree nodes. The framework this feeds is described in
[`doc/plans/AME/autonomy_assurance_plan.md`](autonomy_assurance_plan.md); the point of
generating it from the tool is that the evidence and the model cannot drift apart.

**Non-specialist acceptance.** A safety reviewer reads the report without opening the
tool and can tell which parts of the mission have been checked and which have not.

**Effort.** Medium.

---

## 8. Sequence and milestones

| Milestone | Items | What it delivers |
|-----------|-------|------------------|
| **M1 — Concepts reviewed** | C1 (simulation pack) — done, answers in section 7 | The simulation screens are agreed on paper before code is written |
| **M2 — It runs** | B0 (done), B1 (done), B2 (done) | The compiled tree shows one box per mission step, and a scenario can be run inside the tool and watched |
| **M3 — It is legible** | B3 (done), A3 (done), A4 (done) | Facts and timing are visible; problems and scenarios are workable without planning knowledge |
| **M4 — It survives contact** | B4 (done), B5 (done) | Faults, replanning and batch expectations |
| **M5 — It is shareable** | B6 (done), B7 (done), C2 (done), B8 (done) | Recorded runs, comparison, a one-action review pack, and one command line for all of it |
| **M6 — Finished surface** | A1 (done), A2 (done), A5 (done), A6 (saved views done), A7 (done), A8 (done) | The remaining authoring gaps closed |
| **M7 — Evidence** | C3 (done) | Generated assurance material |

The ordering is deliberate. C1 comes first because the wording and layout decisions in
it are cheap now and expensive after the screens exist. B0 comes before any of the run
screens because it changes the shape of the tree those screens draw, and doing it
afterwards would mean building the live tree view twice. The rest of workstream A sits
late because those items are irritations rather than obstacles, and because a user who
can simulate a mission but has to type a file path is better off than one with polished
file handling and no way to see their mission run. A3 and A4 are pulled forward into M3
because they block the non-specialist acceptance for everything else: a user who cannot
build a starting situation cannot run a simulation.

**Dependencies.**

```
B0 ──> B1 ──> B2 ──> B3 ──> B4 ──> B5
        │       │      │            │
        │       │      └──> B6 ──> B7
        │       │             │
        │       └─────────────┴──> C2 ──> C3
        └──> A4 (scenario facts feed the run)

C1 precedes B2, B3 and B4
A1, A2, A3, A5, A6, A7, A8 are independent
```

---

## 9. The walkthrough

This is the acceptance test for the requirement in section 2. It collects the
non-specialist acceptance line from every work item in this plan, so that no item can be
called done without its own line having been checked. Steps 2 and 3 exercise parts of the
tool that already exist, and are included because the rest of the walkthrough depends on
them working.

**Who runs it, and what it decides.** It is run by someone who does not write software,
working alone, with no help and no documentation open. It is a release gate: a release
does not go ahead until every step covering an item in that release has been completed by
such a person. A step that needs help is a defect against the item in brackets, not
against the user, and the release waits for the fix. The one exception is stated in
section 10 and requires a named person to accept it in writing; a developer running the
steps themselves is not an acceptable substitute, because a developer cannot tell whether
a screen makes sense to someone who has never seen a planning model.

**The main session.** One sitting, in the order a real user meets these tasks.

| Step | Task | Item |
|------|------|------|
| 1 | Open the tool and open a mission project saved yesterday, from the recent list | A1 |
| 2 | Find the action that makes a sector count as searched, using the relations panel | Exists today |
| 3 | Add an action the domain did not previously have, using the guided editor | Exists today |
| 4 | Rename a type that four actions and two scenarios already use, and check nothing broke | A7 |
| 5 | Rename the new action, decide against it, and undo with one keystroke | A2 |
| 6 | Fix the three problems the tool reports, by clicking each one | A3 |
| 7 | Build a starting situation of six facts for a scenario | A4 |
| 8 | Open the compiled tree for the scenario and say what every box on it is for | B0 |
| 9 | Run the scenario and watch it complete | B1, B2 |
| 10 | Say when a particular fact stopped being true | B3 |
| 11 | Make one action fail, run again, and explain why the mission took a different route | B4 |
| 12 | Run the whole scenario set and say which one failed and why | B5 |
| 13 | Change one action, re-run, and say from the comparison what the change did | B7 |
| 14 | Export a review pack and name what each file in it is | C2 |

**Separate checks.** These do not fit one sitting, either because they need a second
person, a second machine, or a document read away from the tool. Each is still run by
someone who does not write software.

| Check | Task | Item |
|-------|------|------|
| 15 | Import a second domain into the project and see, before committing, which existing actions it would overwrite | A5 |
| 16 | Open a saved view by name and get the picture the author saved | A6 |
| 17 | Mark a fact as a contingency and another as a safe state, run the analysis, and read whether the safe state is reachable | A8 |
| 18 | Save a run, send the folder to a colleague, and have the colleague step through the same mission | B6 |
| 19 | Read a concept mockup pack in a browser and answer its questions without asking what anything means | C1 |
| 20 | Read the generated assurance report without opening the tool and say which parts of the mission have been checked | C3 |

---

## 10. Risks

| Risk | Likelihood | Impact | What we do about it |
|------|-----------|--------|--------------------|
| Simulated behaviour diverges from real execution, so a mission that works in the tool fails in the field | Medium | High | The simulation uses the real compiled tree, the real world model and the real planner. After B0 the substitution is one node per action and nothing else, and the simulated node is given the same preconditions and effects a deployed one would be; the difference is that the simulated node predicts the state that follows an action, where a deployed system establishes it. Each substitution is visible in the user interface with its settings shown, and the tool never claims a run proves field behaviour |
| B0 changes the compiled tree for everything that reads it, not just the authoring tool | Medium | Medium | The two node types being withdrawn appear only in generated output and in the tests that assert on it; no hand-written tree in the repository uses them, so the change is confined to the compiler, the executor and those tests. The architecture files and the user guide listed in B0 are updated in the same change, so no document is left describing the old tree shape |
| Simulation settings per action become a modelling burden of their own | Medium | Medium | Defaults that need no configuration: four ticks, always succeeds. Settings are only opened by users who want a specific fault, or an action whose duration matters to the mission |
| Fault injection grows into a scripting language | Medium | Medium | Two mechanisms only, fixed at the start: force an action to fail on a given attempt, and set a fact at a given tick. Anything more expressive is a separate decision, not a slow accumulation |
| The timeline and tree views drift apart from DevEnv's, so the two tools disagree | Low | Medium | Both read the same three JSONL schemas. B6 makes the files interchangeable in both directions, which is testable |
| Concept mockups become documentation nobody reads | Medium | Low | Each pack ends with specific questions and is reviewed before the corresponding build starts. A pack with no answered questions has failed and should not be repeated |
| The upstream node-editor library stays quiet | Medium | Low | It is MIT-licensed and stable, the fork maintained as part of ImGui Bundle is the more active line, and nothing in this plan needs new features from it |
| Non-specialist acceptance is signed off by developers | Medium | High | The walkthrough in section 9 is a release gate and is run by someone who does not write software. If nobody suitable is available, the release does not proceed on a developer's run of the same steps. It waits, or the person who owns the release accepts the gap in writing, naming which steps were not run and why. That waiver is the single exception, it is recorded with the release, and it does not carry over to the next one |

---

## 11. Where the history went

The previous plan listed sixty work items across seven phases, each with a tick box
and, in many cases, a note explaining what had been deferred or done differently. That
record was useful while the tool was being built and is preserved in the repository
history; the commit that introduced this document removes the file.

Everything the old plan recorded as finished is described in section 3 as a capability.
The rationale it referenced in passing lives in the research documents, which remain the
place to look for why the tool is shaped this way.

Everything it recorded as unfinished is listed below, so that deleting the file does not
quietly lose work. Each entry is either carried into an item of this plan or dropped on
purpose with the reason given. Nothing is left implicit, and no reader should have to
open the repository history to find out what happened to a deferral.

**Carried forward.** Every row below has since been built except the two marked as still
open, which are the only pieces of workstream A left. The items in sections 5 to 7 record what
was built and any decision worth keeping.

| Unfinished in the old plan | Now |
|----------------------------|-----|
| File open and save-as stubbed; hard-coded paths | A1, done: dialogs, a recent list, a prompt on quitting, and a recovery copy |
| Undo does not cover text edits | A2, done: a run of keystrokes in one field is one undoable step |
| Validation messages name an element but are not clickable | A3, done: every row selects and reveals what it names |
| Scenario facts typed as text rather than chosen | A4, done: chosen from lists restricted by type, with typing kept as an alternative |
| Domain import overwrites the project; no merge choice | A5, done: the import shows what it would overwrite and asks first |
| Problem export writes only the first scenario | A5, done: one file per scenario |
| Imported elements laid out as two long rows | A5, done: laid out by what they have to do with each other |
| Collapsible named groups | A6, **still open**: stored in the project format, but nothing draws them |
| Saved named views | A6, done: stored in the project and reopened by name |
| Types cannot be renamed | A7, done, reaching everywhere the name is used |
| Action parameters cannot be reordered | A7, done |
| The canvas "Add Type" menu entry does nothing | A7, done |
| No copy and paste, only duplicate | A7, done |
| No fit-canvas-to-contents key | A7, done: `F` |
| Quick-add reachable only from the palette, not the canvas or the keyboard | A7, done: `A` |
| Behaviour-tree subtrees cannot be expanded or collapsed | A7: this already worked, and was found to when A7 was built |
| Keyboard-only workflow claimed but incomplete | A7, done |
| Contingency and safe-state facts inferred, not declared per scenario | A8, done, with inference kept as the default |
| Contingency search limited to eight facts; no monotonicity pruning | A8, done: the cap is gone and the pruning is `ame_core`'s, shared with the verifier |
| Batch runs show no progress and cannot be stopped | B5, done: progress per scenario and a Stop button |

**Dropped, with the reason.**

| Dropped | Why |
|---------|-----|
| Light theme | One dark theme is the house style across DevEnv and this tool. It would be reopened by a user asking for it, not by the plan |
| Icon set for node types | Text labels read correctly in review. Icons would add a legend to learn, which works against the rule that a view should be readable without training |
| Drag from the palette onto the canvas | The canvas is not where authoring happens, which is a stated non-goal in section 4. Clicking a palette entry to select and edit is the right interaction, and the quick-add work in A7 covers creating things |
| Per-flow colour-coding in the plan view | The neighbourhood and matrix views established one colour language for relationships. A second, unrelated colour language in the plan view would teach users a rule that holds on one screen only |
| Layout presets and a docking arrangement menu | The tool moved to fixed tabs, so there is no arrangement to preset. A6's saved views cover the underlying need, which was returning to a particular picture |
| A custom animation layer for canvas navigation | The node editor already animates panning and zooming. Nothing was missing |
| Building the importer on the PDDL parser rather than its own reader | Settled during the build, not deferred. The parser discards names and structure the importer needs; the dedicated reader is tested for round-trip against the generator |
