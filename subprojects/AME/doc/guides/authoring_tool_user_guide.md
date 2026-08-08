# AME Authoring Tool User Guide

The AME authoring tool is a local graphical workbench for creating, importing, validating, and reviewing AME PDDL domains and mission scenarios.

It is intended for systems engineers, autonomy developers, and reviewers who need to work with AME mission models without hand-editing every PDDL file. The tool keeps a structured project model as its working format, generates AME-compatible PDDL, and uses the same parser, world model, planner, and plan compiler as the runtime stack.

---

## 1) What the tool is for

Use the authoring tool to:

1. Define PDDL types, objects, predicates, and action schemas graphically.
2. Import existing domain and problem `.pddl` files into a structured project.
3. Build mission scenarios from initial facts, goals, and expected outcomes.
4. Validate the model at structural, parser, grounding, and planning levels.
5. Preview generated PDDL, solved plans, and compiled Behavior Trees.
6. Run scenario regression checks and contingency reachability analysis.
7. Save and replay runtime-compatible runs, and compare two runs.
8. Export a dated review pack containing the model and its review evidence.

The tool is offline and local. It does not replace the AME DevEnv, ROS2 nodes, or Foxglove
runtime monitoring path. Use the authoring tool for model authoring, simulation and recorded
run review; use DevEnv and the deployed AME nodes for live execution monitoring. The two tools
read the same three recorded-run files.

---

## 2) Build and launch

The tool is gated behind `AME_BUILD_AUTHORING=ON`, and the `authoring` preset turns that on for
you. The preset configures into its own `build-authoring/` directory and has matching build and
test presets, `authoring-release` and `authoring-debug`.

From the repository root on Windows:

```bat
cmake --preset authoring
cmake --build --preset authoring-release --target ame_authoring_tool ame_mission_cli --parallel %NUMBER_OF_PROCESSORS%
build-authoring\subprojects\AME\src\Release\ame_authoring_tool.exe
```

From the repository root on Linux, using GCC or Clang:

```bash
cmake --preset authoring -DCMAKE_BUILD_TYPE=Release
cmake --build --preset authoring-release --target ame_authoring_tool ame_mission_cli --parallel $(nproc)
build-authoring/subprojects/AME/src/ame_authoring_tool
```

Run the tool's own test suites with the matching test preset:

```bash
ctest --preset authoring-release
```

The same `authoring` preset serves both platforms. Two details differ on Linux. The build type has
to be given explicitly, because the generators normally used there choose one configuration at
configure time rather than at build time. The executable also lands directly in
`build-authoring/subprojects/AME/src/` rather than in a `Release` subdirectory, for the same reason.

### Building AME on its own

You do not need the whole workspace. `subprojects/AME` is a complete CMake project with its own
presets, and it needs neither PCL nor PYRAMID on disk. Open that folder in VS Code, or configure it
from the command line:

```bash
cd subprojects/AME
cmake --preset default
cmake --build --preset release --parallel
build/src/ame_authoring_tool
```

Every third-party dependency is checked in under `subprojects/AME/external`, so this works with no
network access. The `default` preset sets `AME_REQUIRE_VENDORED_DEPENDENCIES=ON`, which means a
dependency missing from that directory is an error naming what is absent, rather than a silent
attempt to download it.

If you would rather configure by hand than use the preset, note that **two** options are needed,
not one:

```bash
cmake -S . -B build-authoring -DUNMANNED_BUILD_AME=ON -DAME_BUILD_AUTHORING=ON
```

`AME_BUILD_AUTHORING` is declared inside the AME subproject, and `UNMANNED_BUILD_AME` defaults to
off. Setting only `AME_BUILD_AUTHORING` means the AME subdirectory is never processed, so the
option is never declared and CMake discards the setting with nothing more than a
"Manually-specified variables were not used by the project" warning. The configure then succeeds
and produces no authoring tool at all.

The tool is supported equally on both platforms and contains no platform-specific code. SDL2,
Dear ImGui, and imgui-node-editor are cross-platform, and the Linux build compiles SDL2 from
source with X11 and OpenGL support. Building on Linux therefore needs the OpenGL and X11
development headers present (on Debian and Ubuntu these come from `libgl1-mesa-dev` and
`xorg-dev`); everything else is fetched by CMake.

The first configure downloads the GUI dependencies through CMake `FetchContent`, including Dear ImGui, imgui-node-editor, SDL2, stb, JetBrains Mono, and tinyfiledialogs.

### Headless self-test

The executable includes a self-test mode for automated smoke testing and screenshot capture:

```bat
build\subprojects\AME\src\Release\ame_authoring_tool.exe --self-test ame_authoring_self_test.png
```

The command creates a hidden window, drives the real application shell, writes a PNG screenshot, and prints a JSON result to stdout. A successful run exits with code `0`.

On a Linux machine without a display server, use SDL's offscreen backend:

```bash
SDL_VIDEODRIVER=offscreen build/subprojects/AME/src/ame_authoring_tool --self-test ame_authoring_self_test.png
```

---

## 3) Main screen

The application has four workflow tabs:

| Tab | Purpose |
|-----|---------|
| `Domain` | Main authoring surface for palette, types, objects, scenarios, properties, and the node graph |
| `PDDL` | Editable domain PDDL, the list of problems, grounding report, regression results, and contingency results |
| `Plan` | Read-only plan graph after a successful feasibility check |
| `Run` | The compiled Behavior Tree, and the controls that run a scenario against it |

The status bar shows the current project name, validation state, and last operation. The layout is saved to `ame_authoring_tool.ini` during normal interactive use.

The PDDL tab starts from the generated domain text. Edit it directly and select `Apply edited
PDDL to project` to round-trip it through the same importer used for PDDL files. Objects,
scenarios, and lifecycle groupings are retained when the edited domain is applied. Select
`Reload generated PDDL` to discard un-applied text edits.

### Domain views

The right side of the `Domain` tab has five views over the same project model:

| View | Purpose |
|------|---------|
| `Neighbourhood` | Shows the selected fact or action and only its nearby relationships |
| `Relations` | Lists everything that needs, makes true, or makes false the selection |
| `Matrix` | Shows all facts against all actions as `R`, `+`, and `-` marks |
| `Lifecycles` | Shows transitions within user-declared state groups for each object type |
| `Whole domain` | Retains the complete canvas for small domains and presentation use, and is where named groups are drawn |

Use the back and forward buttons to retrace selections made in the palette, relation lists,
or graph. The neighbourhood view has one-step and two-step depth settings and filters for the
three relationship kinds. It shows at most twenty neighbours; select the `+ n more` node to
open the complete list.

---

## 4) Project files

The native project format is JSON with the extension:

```text
*.ameproj.json
```

Use:

- `File > New` to start a clean model.
- `File > Open...` to load an existing project.
- `File > Recent projects` to reopen one of the last eight without typing a path. A project
  that has since been deleted is not offered.
- `File > Save` or `File > Save As...` to write the current structured model.

The project stores:

- type hierarchy,
- predicates,
- action schemas,
- user-declared lifecycle state groups,
- objects,
- scenarios,
- scenario expected outcomes,
- per-action Behavior Tree bindings,
- graph node positions used by the optional whole-domain view.

Action-to-action relationships are derived from action outcomes and requirements. They are
not stored or drawn by hand. Older version-1 files that contain `causalLinks` still load, but
that legacy field is ignored and is not written when the project is saved again.

Closing the tool with unsaved changes asks whether to save first. While there are unsaved
changes the tool also writes a recovery copy beside the project, named `<project>.recovery`,
and removes it once the project is saved. If the tool stops unexpectedly, open that file to
get the work back.

Renaming a type reaches everything that names it — the objects of that type, the parameters
that ask for it, the types below it — in one step, so one press of undo puts the old name
back everywhere. Use the `rename` button beside the type in the sidebar.

Typing is undoable: a run of keystrokes in one field is a single step, so undo puts back what
the field held before you started rather than removing one letter. The `Edit` menu names what
would be undone.

PDDL remains an import/export artefact. The structured project is the better format for ongoing graphical editing.

---

## 5) Author a domain

### Types and objects

Open the `Domain` tab. In the left sidebar:

1. Expand `Types`.
2. Add each type with a name and parent type.
3. Use `object` as the root type.
4. Expand `Objects`.
5. Add each named object and its type.

Example:

| Name | Parent/type |
|------|-------------|
| `location` | parent `object` |
| `sector` | parent `location` |
| `robot` | parent `object` |
| `uav1` | object type `robot` |
| `base` | object type `location` |
| `sector_a` | object type `sector` |

Types cannot be deleted while child types or objects still use them.

### Predicates

Create predicates from either:

- the `Palette` quick-add control, or
- right-clicking the graph canvas and selecting `Add Predicate`.

Select a predicate node to edit its name and parameters in the properties area.

Example predicate:

```pddl
(at ?r - robot ?l - location)
```

Use parameter names with the usual PDDL variable prefix, for example `?r`, `?from`, and `?to`.

### Actions

Create actions from either:

- the `Palette` quick-add control, or
- right-clicking the graph canvas and selecting `Add Action`.

Select an action node to edit:

- what the action involves,
- what must be true before it can happen,
- what becomes true or false afterwards,
- its Behavior Tree binding.

The editor presents these as the sentence groups `It involves`, `Before it can happen`, and
`Afterwards`. Fact names and action parameter names are selected from dropdowns. Choices that
do not fit the required type remain visible in grey with a short reason, such as `applies to
a sector`. This makes the type rule visible without allowing the illegal choice.

`Afterwards` has two parts, because an action can both make facts true and make them false:

| Part | Button | What it writes in the PDDL |
|------|--------|----------------------------|
| `These become true` | `+ add a fact it makes true` | The fact, as an add effect |
| `These become false` | `+ add a fact it makes false` | `(not FACT)`, as a delete effect |

Both work the same way: choose the fact from the dropdown, press the button, then choose
which of the action's involved names each slot refers to. An existing row says `becomes true`
in green or `becomes false` in red at its end, so a long list can be read at a glance.

Making a fact false is how an action stops something being the case — a vehicle that moves
away from the base makes `at base` false as well as making `at sector` true. An action that
only ever adds facts usually means a lifecycle has been half-modelled, and the `Lifecycles`
view is where that shows up.

`Before it can happen` also supports conditions beyond a simple list of true facts. The
buttons and dropdowns use these phrases:

| Screen wording | Meaning |
|----------------|---------|
| `must be false` | The selected fact must not hold |
| `Any one of these is enough` | At least one fact or nested group must hold |
| `For every` | The inner condition must hold for every thing of the selected type |
| `For at least one` | The inner condition must hold for one or more things of the selected type |
| `must be the same as` / `must be different from` | Restrict the two selected names to equal or unequal values |

Completed groups are summarised in one plain sentence. Select a group or fact to change it;
the dropdowns include only names and types that are legal in that position. Imported actions
using these forms open in the same editor rather than falling back to raw PDDL.

The generated PDDL and current checks remain visible below the sentences, and the PDDL tab
remains the editable text path for experienced authors.

Example action:

```pddl
(:action move
  :parameters (?r - robot ?from - location ?to - location)
  :precondition (at ?r ?from)
  :effect (and
    (at ?r ?to)
    (not (at ?r ?from))))
```

For scenario facts, use object names such as `uav1` and `sector_a`. The action editor uses action
parameter names such as `?r` and `?to` in its dropdowns.

### Derived relationships

Amber means an action needs a fact to be true, green means an action makes a fact true, and
red-orange means either that an action needs a fact to be false or makes it false. The label
beside the line distinguishes the two. The same colours and words are used in the relations
panel, both graph views, the matrix, and the failure report. Matrix mark `F` means “must be
false”; `R` remains the mark for a fact that must be true, and `A` marks one fact in a set of
alternatives.

The tool also derives that action A may enable action B when A makes a fact true that B needs
and their parameter types can describe at least one common grounded value. These relationships
are read-only because the PDDL action definitions are authoritative.

### Fact-by-action matrix export

Open the `Matrix` view and use `Export CSV` or `Export Markdown`. Both formats include the
complete table and preserve cells with more than one mark. Use these exports as review or
assurance evidence alongside the generated PDDL.

### Named groups on the whole-domain canvas

A large domain drawn all at once is hard to talk about in a meeting. A group is a named set
of facts and actions that the canvas draws as one labelled box, and which you can close so
that the box stands in for everything inside it.

To make one, open the `Whole domain` view, select the facts and actions you want on the
canvas, and choose `Group these`. The button is unavailable until the selection can be
grouped, and hovering over it says why: nothing is selected, or something in the selection
is already in another group. Anything can be in at most one group, because two boxes each
claiming to stand for the same fact would make closing them ambiguous.

The `Groups` list beside the button opens, closes, renames and removes each group. Closing a
group replaces its contents with a single box carrying the group's name and what it holds,
such as `3 facts and 2 actions`. Lines that reached one of its members now reach the box
instead; lines that would then be drawn twice on top of each other are drawn once; and a line
with both ends inside the closed group is not drawn at all, because it would start and finish
on the same box. Selecting the closed box and choosing `Open` puts the contents back. Every
one of these is a single step on the undo stack.

You never place or size a group's box. It is drawn around wherever its contents are, and a
group closed for the first time appears in the middle of what it held.

Two things to know. Groups change only how the domain is drawn: nothing about them reaches
the generated PDDL, and adding, closing or removing one cannot change what the planner does.
And membership is stored by name, so deleting a fact or an action takes it out of any group
it was in, and a group left holding nothing is removed with it — undoing the deletion brings
both back.

### Lifecycle groupings

Open `Lifecycles`, enter a grouping name, object type, and space-separated fact names, then
select `Add state grouping`. A transition appears when one action makes one group member false
and another member true. If no action makes that change, the view says so; an empty lifecycle
is valid information rather than a rendering error. Automatic grouping suggestion is not
implemented.

---

## 6) Bind actions to Behavior Tree nodes

Each action can define how a compiled plan should call runtime behavior:

- `BT node type`: emit a simple BT node with the selected type.
- `Subtree XML`: use a custom subtree template.
- `Reactive`: compile through reactive behavior so preconditions are rechecked while the action is running.

Subtree templates can use placeholders:

```xml
<InvokeService service="mobility" operation="move" robot="{param0}" target="{param2}" />
```

When the planner produces a grounded action such as `move(uav1, base, sector_a)`, placeholders are resolved by argument index.

If an action has no custom binding, the preview still uses the compiler defaults where possible, but production models should bind every action that will execute against a real integration.

---

## 7) Create scenarios

Scenarios represent PDDL problem instances for the current domain.

In the `Domain` tab, expand `Scenarios`:

1. Enter a scenario name and click `Add Scenario`.
2. Add initial-state facts. Choose the fact, then choose each thing it involves from the
   list, which holds only objects of the right type. Objects of the wrong type stay in the
   list, greyed, and say why on hover. Open `Type it instead` to type a fact if you prefer;
   `(at uav1 base)`, `at uav1 base` and `at(uav1, base)` are all accepted, and are checked
   against the same rules.
3. Add goal facts.
4. Optionally define the expected outcome.

An initial fact or goal is built by selecting a predicate and entering object arguments.

Example initial state:

```pddl
(at uav1 base)
```

Example goals:

```pddl
(searched sector_a)
(classified sector_a)
```

Expected outcomes are used by `Validate > Run All Scenarios`:

| Field | Meaning |
|-------|---------|
| `Should succeed` | Whether the planner is expected to find a plan |
| `Min plan steps` | Lower bound, or `0` for no bound |
| `Max plan steps` | Upper bound, or `0` for no bound |
| `Expected actions` | Action schemas that must appear in the plan |
| `Forbidden actions` | Action schemas that must not appear in the plan |

---

## 8) Validate and preview

The `Validate` menu provides the main review workflow.

### Validate Now

Runs structural checks, generates PDDL, parses it through AME's `PddlParser`, and grounds it through the AME world-model path.

Results appear in the `PDDL` tab:

- parser errors,
- structural errors and warnings,
- grounding statistics,
- warnings for predicates or action schemas with no ground instances.

The lower half of the `PDDL` tab lists every problem the checkers found, worst first, one
sentence each. Click a row to select the fact or action it names and open it on the `Domain`
tab. When a problem comes from the PDDL reader rather than from the project's structure, the
row says which element could not be read and keeps the reader's own words behind
`What the reader said`. Tick `Show the raw text instead` for the whole diagnostic block as
it used to appear.

### Check Feasibility

Runs validation and then calls the AME planner for the selected scenario.

On success, the `Plan` tab shows:

- plan status,
- step count,
- cost,
- expanded/generated search counts,
- solve time,
- read-only causal plan graph.

When no plan exists, the `Plan` tab names a failed goal and walks backwards through the domain
until it reaches the first fact that no action can make true. The report offers direct actions:

- add the missing fact to the scenario's starting facts,
- add an action that restores it,
- jump to the relevant action,
- mark the scenario as expected to fail.

Expected failure is useful for contingency scenarios where infeasibility is the behavior under
test. The right side of the report also lists every fact that no action produces.

### Plan & Preview

Runs validation, planning, and Behavior Tree compilation in one workflow. The tool switches to the `Plan` tab and populates both:

- `Plan`: solved action graph,
- `Run`: compiled Behavior Tree graph, waiting to be run.

Selecting a plan step highlights the corresponding action schema in the domain graph.

### Run All Scenarios

Plans and simulates every scenario in the project, then compares both results with its
expected outcome. The `PDDL` tab reports which scenario is running and how many have
finished. Use `Stop batch` to leave the remaining scenarios waiting. Each result states why
the observed planning and execution did or did not match the expectation.

Use `File > Export Regression Report...` to write the latest batch report as JSON.

### Saved views

`Save this view` on the `Domain` tab stores what is in focus, how far out the view reaches,
which relationships are shown and which view is open, under a name. Saved views live in the
project file, so they survive reopening and travel with the project. Pick one from
`Saved views` to put the picture back.

### Run Contingency Analysis

Runs an in-process contingency reachability analysis. The tool identifies context predicates from the model, enumerates context combinations, and checks whether the selected scenario remains solvable.

By default the tool works out for itself which facts are context — those that appear in
action preconditions and are changed by no action effect — and treats the scenario's goal as
the safe state. A scenario can say instead: name the facts that represent a contingency worth
checking, and the facts that count as having recovered. Declaring the contingency narrows what
is varied, which is what makes a large domain checkable.

The result says how much of the space was covered: how many combinations were planned for, how
many followed from those without planning, and whether anything could be carried between them
at all. In a domain with conditions about facts being false, nothing can, and every
combination is planned for.

Results appear in the `PDDL` tab as feasible, infeasible, or error rows. Context predicate nodes are highlighted in the domain graph after a report is generated.

---

## 8a) Run a scenario

The `Run` tab runs a scenario inside the tool. The run uses the project's generated PDDL,
the same world model, the same planner and the same plan compiler that the runtime uses.
The only substitution is the action nodes: every action is built as a stand-in that waits
for the number of ticks the project gives it, then succeeds or fails as the project says,
and records the action's declared effects. **A run is evidence about the mission model. It
is never evidence about how the system will behave in the field**, which is why the screen
and the status bar both say `SIMULATED` while a run is loaded.

The controls sit in one row and never move:

| Control | What it does |
|---------|--------------|
| `Run` | Starts the selected scenario, or resumes a paused one |
| `Pause` | Stops the run advancing without ending it |
| `Step` | Advances exactly one tick, pausing the run first |
| `Stop` | Ends the run where it stands, leaving what it did on screen |
| `Reset` | Plans, compiles and loads the same scenario from the beginning |
| `speed` | How many ticks a second a running scenario advances |
| `scenario` | Which scenario to run |

### Making a run go wrong

`Make this run go wrong` contains exactly two fault controls:

- Choose an action and an attempt number. That attempt fails once; later attempts use the
  action's normal settings.
- Choose a grounded fact, a value and a tick. The world model applies that change as an
  observed event outside the mission's control.

These settings belong to the loaded run, not to the PDDL domain, and Reset keeps them. When
a fact is made true and belongs to a declared lifecycle group, the other alternative facts
for the same objects are made false. This prevents the injected event from leaving two
alternative states true together.

Before the first tick, the panel describes what the configured fault is expected to do. It
names the action expected to be under way, the precondition that can be lost, and the domain
actions that can respond. It also shows the seed used for random action failures. The same
seed and settings produce the same random draws.

When a tree step fails, the engine replans from the world model as it stands and continues
with the replacement tree. The Run tab states why the replan happened and shows the plan that
was abandoned beside the plan that replaced it. The timeline retains actions from both plans
and the run counts its replans. If no replacement plan exists, the same panel uses the
failure explanation from the Plan tab to name a fact that the domain could not bring about.

Use `Save this named fault with the scenario` when the scenario batch should repeat these
run settings and check its execution expectations.

The timeline is the main view below the controls. It has one row per action and a tick axis.
Each bar gives the action's start and finish ticks. Actions that run at the same time have
bars that overlap on separate rows, which makes parallel work visible without having to read
the tree structure. A marker on a bar shows the tick when that action changed a world-model
fact.

The facts panel beside the timeline lists every grounded world-model fact. Each row says
whether the fact is true at the viewed tick and whether it was true from the start, has not
happened yet, or last changed on a particular tick. A row is highlighted when it changed at
the viewed tick. Enter an object name such as `uav1`, or a fact name such as `searched`, in
the filter to reduce the list.

Click a point on the timeline to inspect that moment. The playhead, facts, tree colours,
goals, and sentence that says what is happening all move to the selected tick. This does not
pause, rewind, or otherwise change the live run. Press `Run` or `Step` to return the screen to
the live tick.

The compiled Behavior Tree remains below the timeline to answer where the run has got to.
Before a run it is the tree the plan compiled, waiting. During a run each node takes the
colour of what it is doing — waiting, happening now, finished, or went wrong — and the node
being ticked takes a bright border. A node that holds others shows what they are doing, so a
finished branch reads as finished without opening it. Select an action node to see the
grounded action it came from, the facts that must be true, observed, or false before it can
run, and the facts it makes true or false.

Every action in the tree is drawn as the stand-in that is actually running, so there is no
screen you can be looking at without knowing the actions are simulated.

Nothing has to be set up before the first run. An action that nobody has configured takes
four ticks, which is one second at the default speed, and works. The duration is not one
tick on purpose: a behaviour tree walks a sequence of actions that each finish immediately
within a single tick, so one-tick actions would make a whole mission start and end on tick
one, with nothing to watch and no order visible. Give an action its own duration where the
mission depends on how long it takes.

### Per-action run settings

Select an action on the `Domain` tab. Under `In a simulated run`:

- `how long it takes, in ticks`: how many ticks the stand-in waits before finishing.
- `it works`: clear this to make the action fail every time it is reached.
- `chance it goes wrong`: the probability that an otherwise working action fails.

Random draws use the project's saved seed, so a run with a random element repeats exactly
until somebody changes the seed. Settings are stored in the project file beside the action's
behaviour-tree binding, and a project saved before runs existed takes the defaults.

### Runs from the command line

The same runs work without a window, which is how they are used in a test suite or in
continuous integration. They live in their own program, `ame_mission_cli`, rather than in the
graphical tool: it links no display libraries, so a build agent needs no display stack to run
a mission.

```bash
ame_mission_cli run    my-mission.ameproj.json --scenario nominal --json run.json
ame_mission_cli record my-mission.ameproj.json --scenario nominal --out runs/nominal
ame_mission_cli batch  my-mission.ameproj.json --json regression.json
```

| Command | What it does |
|---------|--------------|
| `run` | Simulates one named scenario and reports what happened |
| `record` | Simulates one named scenario and writes a folder of replay files |
| `batch` | Simulates every scenario and checks each against the planning and execution expectations the project records for it |

| Option | Meaning |
|--------|---------|
| `--scenario <name>` | Which scenario to run. Required by `run` and `record` |
| `--json <file>` | Where to write the machine-readable report. Without it, no report is written |
| `--out <folder>` | Where `record` writes the run. Required by `record` |
| `-h`, `--help` | Show the usage summary |

The exit code is `0` when the mission behaved as the project expects, and `1` when it did not
or the command could not be carried out. Standard output carries the summary a person reads,
which is why the report goes to the file named by `--json`: the planner writes progress lines
of its own to standard output, so it is no place for a document another program has to parse.

`record` requires `--scenario` because one folder holds one run. The scenario's saved fault
settings and the project's random seed are applied, so the recorded run is the one the
project describes and repeats exactly.

**One convention across the tools.** These conventions are the ones
[`contingency_verifier`](contingency_verifier.md) already used: the file being examined is a
positional argument, `--json` names the machine-readable report, `--help` explains itself,
and the exit code is the verdict. The three questions worth asking about a mission model —
whether it can be planned, whether it runs, and whether it stays safe under every contingency
— are therefore three commands that look alike:

```bash
ame_mission_cli      batch  my-mission.ameproj.json          --json runs.json
contingency_verifier domain.pddl problem.pddl                --json contingency.json
```

The graphical tool takes no command-line options other than `--self-test`, which needs a
window and so belongs to it.

### Save and replay a run

Use `File > Save Current Run...` after starting a simulation. Choose a new or empty folder.
The folder contains:

```text
ame_bt_events.jsonl
ame_wm_audit.jsonl
ame_plan_audit.jsonl
run.json
```

The first three files use the runtime observability schemas. `run.json` names the project,
scenario, random seed and injected faults, and says that the run was simulated. It also records
`timeBasis` as `simulated_tick_time` and gives `tickPeriodSeconds`. The period is the reciprocal
of the engine's ticks per second. Event timestamps therefore describe how long the mission
appeared to take on the Run tab: every event made by one tick has that tick's timestamp, and
the next tick advances by `tickPeriodSeconds`. They do not describe how quickly the processor
computed the simulation. The Run tab and status bar continue to show `SIMULATED` while this
recording is replayed.

Use `File > Open Recorded Run...` to replay any folder with the three JSONL files. The project
that produced the files does not have to be open. A recording from a deployed system normally
has no authoring `run.json`; the tool treats it as a real-system run and states `REAL SYSTEM`
above the replay and in the status bar. The timeline, facts, tree and goal views otherwise work
in the same way as they do for a simulation.

To open an authoring-tool recording in DevEnv without conversion:

1. Start DevEnv offline from the repository root:

   ```bash
   python -m subprojects.AME.tools.devenv --backend none
   ```

2. Open `Observability` and select the `JSONL Replay` tab.
3. Select `Load Directory...`.
4. Choose the folder containing `ame_bt_events.jsonl`, `ame_wm_audit.jsonl`, and
   `ame_plan_audit.jsonl`.
5. Use the `Time Scrubber` to move through the recorded events.

DevEnv ignores the authoring manifest and the optional simulation tick fields. It reads the
same core fields that it reads from a runtime recording. Its time scrubber uses the simulated
`ts_us` timeline, so each mission tick remains a visible step. Recordings made by a real system
keep their measured timestamps unchanged.

### Compare two runs

Use `File > Compare Current Run with Recorded Run...` to compare the run in memory with an
earlier saved folder. The current run does not have to be saved first. This is the shortest
path for checking a run before and after a domain change, or a nominal run against a faulted
run.

Use `File > Compare Two Recorded Runs...` when both results are already saved. The comparison
appears on the Run tab. Its first line states where the trees first differ so that a reviewer
can take the answer without reading the detail. Below it, the tool shows the two run summaries,
actions found on only one side, and facts with different final values.

---

## 9) Import and export PDDL

### Import an existing domain

The importer covers the same finite PDDL subset as `ame_core`: typed STRIPS, negative facts in
action conditions, nested `and` and `or` groups, finite `forall` and `exists` groups, equality
and inequality filters, `(either ...)` action-input types, domain constants, confirmed facts,
and top-level goal alternatives. An invalid condition message names the action that contains
it. Conditional effects, numeric expressions and temporal actions remain unsupported by both
the tool and the runtime.

Importing into a project that already has facts or actions shows what the import would do
before it does any of it: what would be added, what would be overwritten, and what is already
the same, with the cost of each replacement named. Anything new is always added. Tick the
kinds of thing you are willing to have overwritten — types, facts, actions, objects — and the
rest of your work is left alone. Replacing an action keeps its behaviour-tree binding, its run
settings and its place on the canvas, because the imported PDDL says nothing about any of
those.

Use:

```text
File > Import PDDL Domain...
```

This imports types, domain constants, facts, actions, their full condition trees, and an
initial graph layout from a domain `.pddl` file.

Current behavior: domain import replaces the current project model and clears undo history.

### Import an existing problem

Use:

```text
File > Import PDDL Problem...
```

This imports objects, initial-state facts, and goals as a scenario on the current project.
If the goal offers alternatives, the scenario shows them under `any one of these is enough`.
Each choice remains editable with the ordinary fact chooser, and `Add another acceptable
goal` creates another route to mission success.

Import the matching domain first, then import one or more problem files.

### Export generated PDDL

Use:

```text
File > Export Domain PDDL...
File > Export every scenario's problem file...
File > Export Problem PDDL...
```

Export refuses to write if structural errors are present.

`Export every scenario's problem file...` asks where to put them and writes one file per
scenario, named after the project and the scenario. `Export Problem PDDL...` remains for
writing a single scenario to a name you choose.

### Export assurance evidence

`File > Export Assurance Evidence...` writes a Markdown report a reviewer can read without
the tool: what the model contains, the facts nothing in the mission brings about, what each
action is bound to, the facts that must be observed before an action will act on them, how
every scenario behaved, and what stayed reachable under each declared contingency.

The report always ends with what it does **not** cover — that its runs are simulations rather
than evidence about the field, how many scenarios declare no contingency, and how many actions
have nothing bound to them. The same report goes into the review pack as
`07-assurance-evidence.md`, and `ame_mission_cli evidence <project> --json report.md` writes
it from a script.

### Export a review pack

Use `File > Export Review Pack...` and choose a parent folder. The tool creates a dated folder
whose files can be identified from their names. `00-index.md` explains each item. The pack
contains:

- the generated domain PDDL;
- one generated problem PDDL file for every scenario;
- the fact-by-action matrix as CSV and Markdown;
- a Markdown table of all scenario results;
- one replayable recorded-run folder;
- a domain summary listing types, objects, facts, actions and facts that no action produces.

If a run is currently loaded, the pack includes it without asking you to save it first.
Otherwise it runs the first scenario for the recorded-run part of the pack. The individual
domain, problem, regression-report and matrix export commands remain available.

---

## 10) Keyboard shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+Z` | Undo |
| `Ctrl+Y` | Redo |
| `Delete` | Delete selected graph nodes or links |
| `Ctrl+C` | Copy the selected fact or action |
| `Ctrl+V` | Paste it, under a name nothing else uses |
| `Ctrl+D` | Duplicate selection |
| `F` | Fit the canvas to everything on it |
| `A` | Quick-add a fact or an action |
| `Ctrl+Tab` | Cycle workflow tabs |
| `F5` | Plan & Preview |
| `F6` | Validate Now |
| `Esc` | Exit application |

---

## 11) Practical workflow

For a new model:

1. `File > New`.
2. Add types and objects.
3. Add predicates.
4. Add actions with preconditions and effects.
5. Add action BT bindings.
6. Add at least one scenario.
7. Run `Validate > Validate Now`.
8. Fix structural and parser errors.
9. Run `Validate > Check Feasibility`.
10. Run `Validate > Plan & Preview`.
11. Save the project.
12. Export a review pack when the model is ready for review.

For an existing PDDL model:

1. `File > Import PDDL Domain...`.
2. `File > Import PDDL Problem...`.
3. Save as `*.ameproj.json`.
4. Run `Validate > Validate Now`.
5. Run `Validate > Plan & Preview`.
6. Add expected outcomes.
7. Run `Validate > Run All Scenarios`.
8. Export a review pack if the model is being reviewed or baselined.

---

## 12) Current limitations

- Conditional effects, numeric fluents, temporal PDDL and durative actions are out of scope
  for the whole of AME, not just for this tool: `PddlParser` rejects them too.
- The plan view is a read-only preview, and the tree on the `Run` tab cannot be edited.
- Saved views are stored in the project and reopen by name. Named groups are drawn on the
  whole-domain canvas only; the neighbourhood view always shows individual facts and actions.
- The tool performs design-time validation only; live ROS2 execution monitoring remains in DevEnv/Foxglove.

---

## 13) Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `ame_authoring_tool` target is missing | Configure was run without authoring enabled | Run `cmake --preset authoring`. If configuring by hand, pass both `-DUNMANNED_BUILD_AME=ON` and `-DAME_BUILD_AUTHORING=ON` — see section 2 |
| Configure reports "Manually-specified variables were not used by the project: AME_BUILD_AUTHORING" | `UNMANNED_BUILD_AME` was off, so the option was never declared and the setting was discarded | Add `-DUNMANNED_BUILD_AME=ON`, or use the `authoring` preset, which sets both |
| Configure reports that a dependency is not checked in | Something was deleted from `subprojects/AME/external`, or a dependency was added to the build without being vendored | Restore the directory from version control, or run `subprojects/AME/scripts/vendor_dependencies.py --fetch` on a machine with network access and commit the result |
| Configure tries to download something | The build is not requiring the checked-in copies | Configure with `-DAME_REQUIRE_VENDORED_DEPENDENCIES=ON`, which AME's own presets already set, so that a missing copy fails immediately instead |
| Window opens but font differs | `JetBrainsMono-Regular.ttf` was not copied next to the executable | Rebuild the `ame_authoring_tool` target |
| Export is refused | Structural validation has errors | Open the `PDDL` tab, fix `ERR` entries, then export again |
| Planner returns no plan | Scenario goal is unreachable from the initial state, or action preconditions/effects are incomplete | Inspect generated PDDL, run validation, and check action schemas |
| Grounding report shows zero ground actions | Missing objects or parameter types do not match | Add objects for each action parameter type and revalidate |
| Imported problem fails | Problem does not match the imported domain | Import the matching domain first and check predicate/object names |
