# AME Authoring Tool User Guide

The AME authoring tool is a local graphical workbench for creating, importing, validating, and reviewing AME PDDL domains and mission scenarios.

It is intended for systems engineers, autonomy developers, and reviewers who need to work with AME mission models without hand-editing every PDDL file. The tool keeps a structured project model as its working format, generates AME-compatible STRIPS PDDL, and uses the same parser, world model, planner, and plan compiler as the runtime stack.

---

## 1) What the tool is for

Use the authoring tool to:

1. Define PDDL types, objects, predicates, and action schemas graphically.
2. Import existing domain and problem `.pddl` files into a structured project.
3. Build mission scenarios from initial facts, goals, and expected outcomes.
4. Validate the model at structural, parser, grounding, and planning levels.
5. Preview generated PDDL, solved plans, and compiled Behavior Trees.
6. Run scenario regression checks and contingency reachability analysis.
7. Export domain/problem PDDL and regression reports for review or CI use.

The tool is offline and local. It does not replace the AME DevEnv, ROS2 nodes, or Foxglove runtime monitoring path. Use the authoring tool for model authoring and design-time validation; use DevEnv and the deployed AME nodes for execution monitoring.

---

## 2) Build and launch

The tool is gated behind `AME_BUILD_AUTHORING=ON`, and the `authoring` preset turns that on for
you. The preset configures into its own `build-authoring/` directory and has matching build and
test presets, `authoring-release` and `authoring-debug`.

From the repository root on Windows:

```bat
cmake --preset authoring
cmake --build --preset authoring-release --target ame_authoring_tool --parallel %NUMBER_OF_PROCESSORS%
build-authoring\subprojects\AME\src\Release\ame_authoring_tool.exe
```

From the repository root on Linux, using GCC or Clang:

```bash
cmake --preset authoring -DCMAKE_BUILD_TYPE=Release
cmake --build --preset authoring-release --target ame_authoring_tool --parallel $(nproc)
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
| `PDDL` | Editable domain PDDL, validation output, grounding report, regression results, and contingency results |
| `Plan` | Read-only plan graph after a successful feasibility check |
| `BT` | Read-only Behavior Tree graph after a successful plan compile |

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
| `Whole domain` | Retains the complete canvas for small domains and presentation use |

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

An outcome that removes a fact is shown as `becomes false` on the same row. The generated PDDL
and current checks remain visible below the sentences, and the PDDL tab remains the editable
text path for experienced authors.

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

Amber means an action needs a fact, green means an action makes a fact true, and red-orange
means it makes a fact false. The same colours are used in the relations panel, both graph
views, the matrix, and the failure report.

The tool also derives that action A may enable action B when A makes a fact true that B needs
and their parameter types can describe at least one common grounded value. These relationships
are read-only because the PDDL action definitions are authoritative.

### Fact-by-action matrix export

Open the `Matrix` view and use `Export CSV` or `Export Markdown`. Both formats include the
complete table and preserve cells with more than one mark. Use these exports as review or
assurance evidence alongside the generated PDDL.

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
2. Add initial-state facts.
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
- `BT`: compiled Behavior Tree graph.

Selecting a plan step highlights the corresponding action schema in the domain graph.

### Run All Scenarios

Runs every scenario in the project and compares each result with its expected outcome. The regression table appears in the `PDDL` tab.

Use `File > Export Regression Report...` to write the latest batch report as JSON.

### Run Contingency Analysis

Runs an in-process contingency reachability analysis. The tool identifies context predicates from the model, enumerates context combinations, and checks whether the selected scenario remains solvable.

Context predicates are predicates that:

- appear in action preconditions, and
- are not changed by any action effect.

Results appear in the `PDDL` tab as feasible, infeasible, or error rows. Context predicate nodes are highlighted in the domain graph after a report is generated.

---

## 9) Import and export PDDL

### Import an existing domain

Use:

```text
File > Import PDDL Domain...
```

This imports types, predicates, actions, and an initial graph layout from a domain `.pddl` file.

Current behavior: domain import replaces the current project model and clears undo history.

### Import an existing problem

Use:

```text
File > Import PDDL Problem...
```

This imports objects, initial-state facts, and goals as a scenario on the current project.

Import the matching domain first, then import one or more problem files.

### Export generated PDDL

Use:

```text
File > Export Domain PDDL...
File > Export Problem PDDL...
```

Export refuses to write if structural errors are present.

Current behavior: problem export writes the first scenario in the project.

---

## 10) Keyboard shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+Z` | Undo |
| `Ctrl+Y` | Redo |
| `Delete` | Delete selected graph nodes or links |
| `Ctrl+D` | Duplicate selection |
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
12. Export domain/problem PDDL when ready for runtime use or review.

For an existing PDDL model:

1. `File > Import PDDL Domain...`.
2. `File > Import PDDL Problem...`.
3. Save as `*.ameproj.json`.
4. Run `Validate > Validate Now`.
5. Run `Validate > Plan & Preview`.
6. Add expected outcomes.
7. Run `Validate > Run All Scenarios`.
8. Export a regression report if the model is being reviewed or baselined.

---

## 12) Current limitations

- The generated and imported PDDL targets the AME-supported STRIPS + typing subset.
- ADL, conditional effects, numeric fluents, temporal PDDL, and durative actions are out of scope.
- Domain import replaces the current project rather than merging with it.
- Problem export currently exports the first scenario.
- Scenario facts are entered as predicate plus argument text, not as generated fluent checkboxes.
- The canvas `Add Type` context menu item is a placeholder; add types through the sidebar.
- Plan and BT views are read-only previews.
- Semantic zoom is available in the whole-domain canvas, but collapsible named groups are not yet implemented.
- Selection history is available during exploration, but saved named views are not yet stored.
- The tool performs design-time validation only; live ROS2 execution monitoring remains in DevEnv/Foxglove.

---

## 13) Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `ame_authoring_tool` target is missing | Configure was run without authoring enabled | Run `cmake --preset authoring`. If configuring by hand, pass both `-DUNMANNED_BUILD_AME=ON` and `-DAME_BUILD_AUTHORING=ON` — see section 2 |
| Configure reports "Manually-specified variables were not used by the project: AME_BUILD_AUTHORING" | `UNMANNED_BUILD_AME` was off, so the option was never declared and the setting was discarded | Add `-DUNMANNED_BUILD_AME=ON`, or use the `authoring` preset, which sets both |
| First configure fails while fetching dependencies | Network or Git access issue during `FetchContent` | Retry from a network-enabled developer environment |
| Window opens but font differs | `JetBrainsMono-Regular.ttf` was not copied next to the executable | Rebuild the `ame_authoring_tool` target |
| Export is refused | Structural validation has errors | Open the `PDDL` tab, fix `ERR` entries, then export again |
| Planner returns no plan | Scenario goal is unreachable from the initial state, or action preconditions/effects are incomplete | Inspect generated PDDL, run validation, and check action schemas |
| Grounding report shows zero ground actions | Missing objects or parameter types do not match | Add objects for each action parameter type and revalidate |
| Imported problem fails | Problem does not match the imported domain | Import the matching domain first and check predicate/object names |
