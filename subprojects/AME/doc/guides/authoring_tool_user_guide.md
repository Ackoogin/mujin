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

The tool is gated behind `AME_BUILD_AUTHORING=ON` and is included in the `authoring` configure preset.

From the repository root on Windows:

```bat
cmake --preset authoring
cmake --build build --config Release --target ame_authoring_tool --parallel %NUMBER_OF_PROCESSORS%
build\subprojects\AME\src\Release\ame_authoring_tool.exe
```

Equivalent explicit configure:

```bat
cmake --preset default -DAME_BUILD_AUTHORING=ON
cmake --build build --config Release --target ame_authoring_tool
```

The first configure downloads the GUI dependencies through CMake `FetchContent`, including Dear ImGui, imgui-node-editor, SDL2, stb, JetBrains Mono, and tinyfiledialogs.

### Headless self-test

The executable includes a self-test mode for automated smoke testing and screenshot capture:

```bat
build\subprojects\AME\src\Release\ame_authoring_tool.exe --self-test ame_authoring_self_test.png
```

The command creates a hidden window, drives the real application shell, writes a PNG screenshot, and prints a JSON result to stdout. A successful run exits with code `0`.

---

## 3) Main screen

The application has four workflow tabs:

| Tab | Purpose |
|-----|---------|
| `Domain` | Main authoring surface for palette, types, objects, scenarios, properties, and the node graph |
| `PDDL` | Generated domain PDDL, validation output, grounding report, regression results, and contingency results |
| `Plan` | Read-only plan graph after a successful feasibility check |
| `BT` | Read-only Behavior Tree graph after a successful plan compile |

The status bar shows the current project name, validation state, and last operation. The layout is saved to `ame_authoring_tool.ini` during normal interactive use.

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
- causal links,
- objects,
- scenarios,
- scenario expected outcomes,
- per-action Behavior Tree bindings,
- graph node positions.

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

- action name,
- typed parameters,
- preconditions,
- add effects,
- delete effects,
- Behavior Tree binding.

Example action:

```pddl
(:action move
  :parameters (?r - robot ?from - location ?to - location)
  :precondition (at ?r ?from)
  :effect (and
    (at ?r ?to)
    (not (at ?r ?from))))
```

Preconditions and effects are entered by choosing a predicate and providing argument names. For an action schema, use action parameter names such as `?r` and `?to`; for scenario facts, use object names such as `uav1` and `sector_a`.

### Causal links

Drag from an action add-effect output pin to another action precondition input pin to create an informational causal link. The tool accepts the link only when the predicate signatures are compatible.

Causal links help reviewers understand flow, but formal semantics still come from the PDDL precondition and effect lists.

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
- The tool performs design-time validation only; live ROS2 execution monitoring remains in DevEnv/Foxglove.

---

## 13) Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `ame_authoring_tool` target is missing | Configure was run without authoring enabled | Run `cmake --preset authoring` or configure with `-DAME_BUILD_AUTHORING=ON` |
| First configure fails while fetching dependencies | Network or Git access issue during `FetchContent` | Retry from a network-enabled developer environment |
| Window opens but font differs | `JetBrainsMono-Regular.ttf` was not copied next to the executable | Rebuild the `ame_authoring_tool` target |
| Export is refused | Structural validation has errors | Open the `PDDL` tab, fix `ERR` entries, then export again |
| Planner returns no plan | Scenario goal is unreachable from the initial state, or action preconditions/effects are incomplete | Inspect generated PDDL, run validation, and check action schemas |
| Grounding report shows zero ground actions | Missing objects or parameter types do not match | Add objects for each action parameter type and revalidate |
| Imported problem fails | Problem does not match the imported domain | Import the matching domain first and check predicate/object names |

