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
| **PDDL** | Generation with live preview and export; import of existing domain and problem files through a dedicated importer that preserves names and structure | `pddl_generator.cpp`, `pddl_importer.cpp` |
| **Checking** | Four levels: continuous structural checks while editing, parse through `PddlParser`, grounding through `WorldModel`, and feasibility through `Planner` | `structural_validator.cpp`, `pddl_validator.cpp` |
| **Scenarios** | Scenario list with starting facts, goals and expected outcomes; batch regression run with a results table and a JSON report | `scenario_runner.cpp` |
| **Contingency** | Reachability analysis run in process against the generated model, with a per-context results table | `contingency_analyser.cpp` |
| **Previews** | Plan view as a causal graph with computed layers; compiled behaviour tree view parsed from the generated XML | `plan_graph_panel.cpp`, `bt_graph_panel.cpp` |
| **Failure explanation** | Backward chain from a failed goal to the first fact nothing produces, with buttons that apply the likely fixes | `failure_explainer.cpp` |
| **Behaviour-tree binding** | Per-action node type, reactive flag and subtree template with parameter placeholders, previewed against stand-in arguments | `app_shell.cpp` |

**Known gaps carried into this plan.** File open and save-as still use hard-coded paths
rather than a file dialog. Text edits are not undoable, only additions and deletions.
Validation messages name the offending element but are not clickable. Importing a
domain replaces the project rather than merging. Problem export writes only the first
scenario. Scenario facts are typed as text rather than chosen from the grounded set.
Types cannot be renamed and action parameters cannot be reordered. Named presentation
groups and saved named views were both deferred. Workstream A picks these up; section 11
lists every unfinished item from the previous plan and says where each one went.

**The largest gap is not in that list.** The tool can tell you that a plan exists and
show you the tree it compiled. It cannot show you that tree running. That is
workstream B, and it is the main new work in this plan.

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

---

## 5. Workstream A — Finish the authoring surface

### A1. Files and projects behave the way users expect

**Build.** Native file dialogs for New, Open, Save, Save As, Import and Export.
A recent-projects list. A prompt on quitting with unsaved changes. Autosave of a
recovery copy alongside the project file.

**Non-specialist acceptance.** A user opens the tool, opens a project they saved
yesterday from the recent list, changes it, closes the tool, and is asked whether to
save. No file path is ever typed.

**Effort.** Small.

### A2. Undo covers everything, including typing

**Build.** Extend the command stack to text edits and dropdown changes, coalescing a
run of keystrokes in one field into a single undoable step. Show the name of the next
undoable action in the Edit menu, so the menu says "Undo rename action" rather than
"Undo".

**Non-specialist acceptance.** A user renames an action, decides against it, presses
the undo shortcut once, and the old name is back.

**Effort.** Small to medium.

### A3. Problems point at the thing that is wrong

**Build.** Make every entry in the validation list select and reveal the element it
refers to, in whichever view is open. Replace parser error text with a sentence naming
the element and what is wrong with it, keeping the raw parser message available behind
a details toggle.

**Non-specialist acceptance.** A user is shown a project with three deliberate faults,
clicks the first problem in the list, lands on the offending action, fixes it, and
watches the entry disappear.

**Effort.** Small.

### A4. Scenario facts are chosen, not typed

**Build.** Replace the free-text fact rows in the scenario editor with a chooser over
the grounded fact set: pick the fact, then pick each object from a list restricted by
the parameter type. Keep a text entry as an alternate path for users who prefer it, and
validate it against the same set.

**Non-specialist acceptance.** A user sets up a starting situation of six facts for a
domain they did not write, without knowing what a fluent is, and cannot produce a fact
that does not exist.

**Effort.** Medium.

### A5. Import merges instead of replacing

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

**Build.** Let the user say, per scenario, which facts represent a contingency and
which represent a safe state, instead of the tool inferring them from where they appear
in the model. Keep the inference as the starting suggestion. Add monotonicity pruning to
the search so the analysis is not limited to eight facts at a time, and say in the
results how much of the space was covered.

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

### B1. Simulation engine

**Build.** An in-process runner that takes the compiled behaviour-tree XML the tool
already produces, builds it with a BehaviorTree.CPP factory, and ticks it against a
world model populated from the scenario's starting facts. `CheckWorldPredicate` and
`SetWorldPredicate` already work purely against the world model and are used unchanged.
Real action nodes talk to external systems, so simulation substitutes a stand-in node
for each action, following the pattern the demo application already uses
([`subprojects/AME/src/apps/main.cpp`](../../../subprojects/AME/src/apps/main.cpp)).

Each action gets simulation settings stored in the project beside its behaviour-tree
binding: how many ticks it takes, whether it succeeds, and optionally a probability of
failure with a fixed random seed so a run is reproducible. Defaults are one tick and
always succeeds, so a user who never opens these settings still gets a working
simulation.

Run control offers step one tick, run to completion, run at a chosen speed, pause and
reset. The whole engine is separable from the user interface and driven by a headless
entry point, so simulations can also run in the test suite and in continuous
integration.

**Non-specialist acceptance.** A user selects a scenario, presses Run, and watches the
mission complete, without configuring anything first.

**Effort.** Large.

### B2. Live tree view

**Build.** Extend the existing behaviour-tree view so that during a run each node is
coloured by its current status, the node being ticked is marked, and completed branches
are visibly finished. A user can click any node to see which action it came from and
which facts it checks or sets. Nodes keep the plain-language names the guided editor
gave them rather than the generated identifiers.

**Non-specialist acceptance.** Someone watching over the author's shoulder can say
which part of the mission is happening now, and which parts already finished, without
being told how to read the picture.

**Effort.** Medium.

### B3. Facts and timeline

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

**Build.** Put two recorded runs side by side and show where they diverge: the first
tick at which the trees differ, which actions differ, and which facts differ at the
end. The obvious uses are a run before and after a domain change, and a nominal run
against the same scenario with a fault injected.

**Non-specialist acceptance.** A user changes one action, re-runs a scenario, and the
comparison tells them in one line what their change did to the mission.

**Effort.** Medium.

---

## 7. Workstream C — Artefacts

### C1. Concept mockups before code

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
Those answers are needed before B2, B3 and B4 start.

**Non-specialist acceptance.** A reviewer reads the pack in a browser with no tools
installed and can answer its questions without asking what anything means.

**Effort.** Medium per pack.

### C2. Exports that reviews can use

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
| **M1 — Concepts reviewed** | C1 (simulation pack) | The simulation screens are agreed on paper before code is written |
| **M2 — It runs** | B1, B2 | A scenario can be run inside the tool and watched |
| **M3 — It is legible** | B3, A3, A4 | Facts and timing are visible; problems and scenarios are workable without planning knowledge |
| **M4 — It survives contact** | B4, B5 | Faults, replanning and batch expectations |
| **M5 — It is shareable** | B6, B7, C2 | Recorded runs, comparison, and a one-action review pack |
| **M6 — Finished surface** | A1, A2, A5, A6, A7, A8 | The remaining authoring gaps closed |
| **M7 — Evidence** | C3 | Generated assurance material |

The ordering is deliberate. C1 comes first because the wording and layout decisions in
it are cheap now and expensive after the screens exist. The rest of workstream A sits
late because those items are irritations rather than obstacles, and because a user who
can simulate a mission but has to type a file path is better off than one with polished
file handling and no way to see their mission run. A3 and A4 are pulled forward into M3
because they block the non-specialist acceptance for everything else: a user who cannot
build a starting situation cannot run a simulation.

**Dependencies.**

```
B1 ──> B2 ──> B3 ──> B4 ──> B5
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
| 8 | Run the scenario and watch it complete | B1, B2 |
| 9 | Say when a particular fact stopped being true | B3 |
| 10 | Make one action fail, run again, and explain why the mission took a different route | B4 |
| 11 | Run the whole scenario set and say which one failed and why | B5 |
| 12 | Change one action, re-run, and say from the comparison what the change did | B7 |
| 13 | Export a review pack and name what each file in it is | C2 |

**Separate checks.** These do not fit one sitting, either because they need a second
person, a second machine, or a document read away from the tool. Each is still run by
someone who does not write software.

| Check | Task | Item |
|-------|------|------|
| 14 | Import a second domain into the project and see, before committing, which existing actions it would overwrite | A5 |
| 15 | Open a saved view by name and get the picture the author saved | A6 |
| 16 | Mark a fact as a contingency and another as a safe state, run the analysis, and read whether the safe state is reachable | A8 |
| 17 | Save a run, send the folder to a colleague, and have the colleague step through the same mission | B6 |
| 18 | Read a concept mockup pack in a browser and answer its questions without asking what anything means | C1 |
| 19 | Read the generated assurance report without opening the tool and say which parts of the mission have been checked | C3 |

---

## 10. Risks

| Risk | Likelihood | Impact | What we do about it |
|------|-----------|--------|--------------------|
| Simulated behaviour diverges from real execution, so a mission that works in the tool fails in the field | Medium | High | The simulation uses the real compiled tree, the real world model and the real planner. Only the action nodes are substituted, and each substitution is visible in the user interface with its settings shown. The tool never claims a run proves field behaviour |
| Simulation settings per action become a modelling burden of their own | Medium | Medium | Defaults that need no configuration: one tick, always succeeds. Settings are only opened by users who want a specific fault |
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

**Carried forward.**

| Unfinished in the old plan | Now |
|----------------------------|-----|
| File open and save-as stubbed; hard-coded paths | A1 |
| Undo does not cover text edits | A2 |
| Validation messages name an element but are not clickable | A3 |
| Scenario facts typed as text rather than chosen | A4 |
| Domain import overwrites the project; no merge choice | A5 |
| Problem export writes only the first scenario | A5 |
| Imported elements laid out as two long rows | A5 |
| Collapsible named groups | A6 |
| Saved named views | A6 |
| Types cannot be renamed | A7 |
| Action parameters cannot be reordered | A7 |
| The canvas "Add Type" menu entry does nothing | A7 |
| No copy and paste, only duplicate | A7 |
| No fit-canvas-to-contents key | A7 |
| Quick-add reachable only from the palette, not the canvas or the keyboard | A7 |
| Behaviour-tree subtrees cannot be expanded or collapsed | A7 |
| Keyboard-only workflow claimed but incomplete | A7, which is what completes it |
| Contingency and safe-state facts inferred, not declared per scenario | A8 |
| Contingency search limited to eight facts; no monotonicity pruning | A8 |
| Batch runs show no progress and cannot be stopped | B5, where a simulated run makes it matter |

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
