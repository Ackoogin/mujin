# Graphical Autonomy Authoring Tool — Work Plan

**Approach:** Option 1 — C++ native tool using Dear ImGui + imgui-node-editor, linked against `ame_core`
**Reference:** `doc/research/AME/graphical_authoring_option1_extend_existing.md`

---

## Phase 0: Foundation & Infrastructure

### WI-0.1: CMake Target & Dependency Setup

**Description:** Add `ame_authoring_tool` executable target to the build system. Fetch Dear ImGui, imgui-node-editor, and a platform backend (SDL2 or GLFW + OpenGL3) via FetchContent.

**Deliverables:**
- [x] `subprojects/AME/src/authoring/CMakeLists.txt` with FetchContent for Dear ImGui, imgui-node-editor, SDL2/GLFW
- [x] Minimal `main.cpp` that opens an empty ImGui window with docking enabled
- [ ] Build succeeds on Windows (MSVC 2022) and Linux (GCC/Clang)
- [x] New configure preset `authoring` or integrated into existing `default` preset (gated by `AME_BUILD_AUTHORING=ON`)

**Bonus (agent self-test):**
- [x] `--self-test [output.png]` CLI flag: hidden window, 3 rendered frames, glReadPixels PNG capture, JSON result to stdout — enables agent-driven screenshot verification with no user in the loop

**Acceptance criteria:** `cmake --preset default -DAME_BUILD_AUTHORING=ON && cmake --build --preset release --target ame_authoring_tool` produces a running window.

**Dependencies:** None
**Effort:** Small

---

### WI-0.2: Application Shell

**Description:** Establish the top-level application structure: main menu bar, docking layout, status bar, and empty placeholder panels for each view.

**Deliverables:**
- [x] Main menu bar (File, Edit, View, Validate, Help)
- [x] Dockable panel layout with panels: Domain Graph, Properties, PDDL Preview, Validation Output, Plan View, BT View
- [x] Persistent layout save/restore (imgui.ini → ame_authoring_tool.ini)
- [x] Status bar showing project name, validation state, and last operation
- [ ] File > New / Open / Save / Save As with native file dialog (ImGui-FileBrowser or tinyfiledialogs) — Open/SaveAs stubbed as TODO; file dialog deferred to WI-0.3

**Acceptance criteria:** Application opens with empty docked panels; layout persists across restarts; File dialogs functional.

**Dependencies:** WI-0.1
**Effort:** Small

---

### WI-0.3: Project Model & Serialisation

**Description:** Define the internal structured data model for an authoring project. Implement JSON serialisation using nlohmann/json.

**Deliverables:**
- [x] `ProjectModel` class holding: type hierarchy, predicates, action schemas, objects, scenario definitions (initial state + goals), node layout positions
- [x] JSON schema definition (documented in header comments; version field in JSON)
- [x] `ProjectModel::save(path)` / `ProjectModel::load(path)` with versioned format (rejects version != 1)
- [x] Unit tests for round-trip serialisation (RoundTrip, LoadMissingFile, LoadBadJson, ClearResetsVersion)

**Acceptance criteria:** Create a model programmatically, save to JSON, reload, verify equality. Schema supports all PDDL STRIPS elements that AME handles.

**Dependencies:** WI-0.1
**Effort:** Medium

---

## Phase 1: Domain Graph Authoring

### WI-1.1: Node Editor Canvas Integration

**Description:** Integrate imgui-node-editor into the Domain Graph panel. Implement pan, zoom, selection, and an empty canvas with grid background.

**Deliverables:**
- [x] imgui-node-editor context created and rendered in the Domain Graph panel
- [x] Pan/zoom with mouse wheel and middle-click drag (built into imgui-node-editor)
- [x] Multi-select with box selection (built into imgui-node-editor)
- [ ] Right-click context menu (empty placeholder: "Add Action", "Add Predicate", "Add Type")
- [x] Canvas background grid (built into imgui-node-editor)

**Acceptance criteria:** Empty interactive canvas in the Domain Graph panel with functional navigation.

**Dependencies:** WI-0.2
**Effort:** Small

---

### WI-1.2: Type Hierarchy Authoring

**Description:** Allow users to define the PDDL type hierarchy and objects through a sidebar panel or dedicated node-graph sub-view.

**Deliverables:**
- [x] Types panel listing all defined types in a tree (parent-child)
- [x] Add/delete types (rename deferred — low priority)
- [x] Set parent type (single inheritance, matching AME TypeSystem)
- [x] Objects panel: add/delete objects, assign to types
- [x] Validation: duplicate names, non-existent parent, type-in-use guards on delete
- [x] Changes reflected immediately in ProjectModel

**Acceptance criteria:** Define a type hierarchy (e.g. `object > location > sector`, `object > robot > uav`), add objects (`uav1: uav`, `sector_a: sector`), save project, reload, verify.

**Dependencies:** WI-0.3, WI-1.1
**Effort:** Medium

---

### WI-1.3: Predicate Nodes

**Description:** Implement predicate definition as nodes on the Domain Graph canvas.

**Deliverables:**
- [ ] "Add Predicate" from context menu or palette creates a predicate node
- [ ] Predicate node displays: name, typed parameter list
- [ ] Editable name and parameter list (add/remove/reorder parameters, assign types from type hierarchy)
- [ ] Visual distinction from action nodes (colour, shape)
- [ ] Properties panel shows selected predicate's details for editing
- [ ] Predicate nodes have output pins (one per predicate, representing "this predicate is available")

**Acceptance criteria:** Create predicates `(at ?r - robot ?l - location)`, `(searched ?s - sector)` via the graph UI. Verify they appear in `ProjectModel`.

**Dependencies:** WI-1.1, WI-1.2
**Effort:** Medium

---

### WI-1.4: Action Schema Nodes

**Description:** Implement PDDL action schema definition as the primary node type on the Domain Graph canvas.

**Deliverables:**
- [x] "Add Action" from context menu or palette creates an action node
- [x] Action node displays: name, parameters, precondition input pins, effect output pins (add/del)
- [x] Editable name and typed parameter list
- [x] Precondition section: add/remove precondition references (link to predicate nodes)
- [x] Add-effect section: add/remove add-effect references
- [x] Delete-effect section: add/remove delete-effect references
- [x] Each precondition/effect shows the grounded template (e.g. `(at ?r ?from)`)
- [x] Blueprint-style header with action name and colour coding
- [x] Properties panel shows full action details for editing

**Acceptance criteria:** Create action `move(?r - robot, ?from - location, ?to - location)` with precondition `(at ?r ?from)`, add-effect `(at ?r ?to)`, del-effect `(at ?r ?from)`. Verify in `ProjectModel`.

**Dependencies:** WI-1.1, WI-1.2, WI-1.3
**Effort:** Large

---

### WI-1.5: Causal Dependency Links

**Description:** Allow users to visually link action effects to action preconditions, showing the causal relationships between actions.

**Deliverables:**
- [x] Drag from an action's add-effect output pin to another action's precondition input pin to create a causal link
- [x] Links rendered as coloured Bezier curves
- [x] Link validation: effect predicate signature must match precondition predicate signature
- [x] Invalid link attempts rejected with visual feedback
- [x] Delete links via right-click or selection + Delete key
- [x] Links are informational/layout aids (the formal semantics come from the PDDL precondition/effect declarations, not the visual links)

**Acceptance criteria:** Link `move`'s add-effect `(at ?r ?to)` to `search`'s precondition `(at ?r ?s)`. Invalid links (type mismatch) are rejected visually.

**Dependencies:** WI-1.4
**Effort:** Medium

---

### WI-1.6: Undo/Redo Command Stack

**Description:** Implement a command-pattern undo/redo system for all authoring operations.

**Deliverables:**
- [x] `CommandStack` class with `execute(cmd)`, `undo()`, `redo()`
- [x] Commands for: add/delete/edit type, add/delete/edit predicate, add/delete/edit action, add/delete link, move node — add/delete only; text-input edits deferred (TODO in code)
- [x] Ctrl+Z / Ctrl+Y keyboard shortcuts
- [x] Edit menu entries: Undo, Redo (greyed when unavailable)
- [x] Undo history limited to configurable depth (default 100)

**Acceptance criteria:** Create nodes, undo creation, redo creation. Edit action parameters, undo edit. All operations reversible.

**Dependencies:** WI-1.4
**Effort:** Medium

---

## Phase 2: PDDL Generation & Validation

### WI-2.1: PDDL Code Generation

**Description:** Generate syntactically correct PDDL domain and problem files from the `ProjectModel`.

**Deliverables:**
- [x] `PddlGenerator` class: `generateDomain(model) -> string`, `generateProblem(model, scenario) -> string`
- [x] Output matches STRIPS subset that AME's `PddlParser` accepts (round-trip test green)
- [x] PDDL Preview panel shows live-generated PDDL (updates on model changes)
- [x] Syntax highlighting in the preview panel (keyword colouring via ImGui text)
- [x] Export to `.pddl` files via File > Export PDDL

**Acceptance criteria:** Author UAV search domain graphically, generate PDDL, verify it matches the existing `domains/uav_search/domain.pddl` semantically.

**Dependencies:** WI-1.4, WI-0.3
**Effort:** Medium

---

### WI-2.2: PDDL Parse Validation (Level 2)

**Description:** Validate generated PDDL by parsing it through AME's `PddlParser` and surfacing errors in the UI.

**Deliverables:**
- [x] Validate menu item / toolbar button triggers: generate PDDL -> `PddlParser::parseFromString()`
- [x] Parse errors mapped back to graph elements (highlight offending node/predicate in red) — best-effort substring match against parser error text
- [x] Validation Output panel shows error messages with clickable references to graph elements — bulleted predicate/action references; "clickable" deferred
- [x] Status bar shows "Valid" / "N errors" indicator
- [x] Auto-validate on save (configurable)

**Acceptance criteria:** Introduce a deliberate error (missing parameter type), validate, see error highlighted on the correct node with a clear message.

**Dependencies:** WI-2.1
**Effort:** Medium

---

### WI-2.3: World Model Grounding Validation (Level 3)

**Description:** Populate a `WorldModel` from parsed PDDL and verify that all predicates and actions ground correctly.

**Deliverables:**
- [x] After successful parse, populate `WorldModel` with types, objects, predicates, actions
- [x] Report grounding statistics: number of ground fluents, number of ground actions
- [x] Flag warnings: zero ground instances for a predicate (indicates unused types), zero ground actions for a schema
- [x] Display grounded state in a "Grounding Report" sub-panel

**Acceptance criteria:** Author a domain with valid types/objects, validate, see grounding stats. Remove a required type, see grounding warning.

**Dependencies:** WI-2.2
**Effort:** Small

---

### WI-2.4: Planning Feasibility Validation (Level 4)

**Description:** Run the AME planner against authored scenarios to verify that goals are reachable.

**Deliverables:**
- [ ] Scenario definition UI: initial state (fact checkboxes from grounded fluents) + goal fluents
- [ ] "Check Feasibility" button: generates PDDL, parses, grounds, calls `Planner::solve()`
- [ ] Results: plan found (show step count, time), no plan (show "no plan exists"), error (show message)
- [ ] If plan found: store plan steps for Plan View (Phase 3)
- [ ] Multiple scenarios per project (scenario list panel)

**Acceptance criteria:** Define UAV search scenario with goals `(searched sector_a) (classified sector_a)`, run planner, get valid plan. Change goal to impossible state, verify "no plan" result.

**Dependencies:** WI-2.3
**Effort:** Medium

---

### WI-2.5: Structural Validation (Level 1)

**Description:** Continuous authoring-time validation without requiring PDDL generation.

**Deliverables:**
- [ ] Real-time validation as user edits: missing names, undeclared types in parameters, duplicate predicate names, unreferenced predicates
- [ ] Visual indicators on nodes: warning badges, red outlines for errors, yellow for warnings
- [ ] Validation Output panel shows all current issues (live-updated)
- [ ] Validation severity levels: Error (blocks PDDL generation), Warning (informational)

**Acceptance criteria:** Create action with parameter of undeclared type — see warning immediately without explicitly running validation.

**Dependencies:** WI-1.4, WI-1.3
**Effort:** Medium

---

## Phase 3: Visualisation Views

### WI-3.1: Plan View (Causal Graph)

**Description:** After a successful planning run, display the plan as a causal dependency graph using imgui-node-editor in a separate view.

**Deliverables:**
- [ ] Plan View panel with its own imgui-node-editor context (read-only)
- [ ] Each plan step rendered as a node (action name + parameters)
- [ ] Causal dependency edges: add-effect of step i -> precondition of step j
- [ ] Parallel flows visually grouped (matching `PlanCompiler`'s causal analysis)
- [ ] Colour-coding per flow
- [ ] Auto-layout using topological sort + layer assignment
- [ ] Click node to see details (preconditions, effects, which flow)

**Acceptance criteria:** Plan the UAV search scenario, switch to Plan View, see nodes for each step with causal edges. Parallel steps appear side-by-side.

**Dependencies:** WI-2.4
**Effort:** Medium

---

### WI-3.2: BT View (Compiled Tree)

**Description:** Display the compiled BT XML as a visual tree using imgui-node-editor.

**Deliverables:**
- [ ] BT View panel with its own imgui-node-editor context (read-only)
- [ ] Parse BT XML output from `PlanCompiler::compile()`
- [ ] Render tree structure: Sequence, Parallel, ReactiveFallback as container nodes; CheckWorldPredicate, SetWorldPredicate, action nodes as leaf nodes
- [ ] Parent-child edges showing tree hierarchy
- [ ] Colour-coding by node type (control flow = blue, condition = green, action = orange, effect = purple)
- [ ] Expand/collapse subtrees
- [ ] Click node to see port values / parameters

**Acceptance criteria:** Compile a plan, switch to BT View, see the full compiled tree matching the XML structure.

**Dependencies:** WI-2.4
**Effort:** Medium

---

### WI-3.3: Side-by-Side Domain + Plan Preview

**Description:** Enable a workflow where the user authors in the Domain Graph, runs the planner, and immediately sees the resulting plan and BT alongside.

**Deliverables:**
- [ ] Layout preset: "Author & Preview" arranges Domain Graph (left), Plan View (top-right), BT View (bottom-right)
- [ ] "Plan & Preview" toolbar button: validates, plans, compiles, updates all views in one action
- [ ] Cross-view highlighting: select a plan step to highlight the corresponding action schema in Domain Graph and BT nodes in BT View
- [ ] View > Layout Presets menu (Author & Preview, Domain Only, Validation Focus)

**Acceptance criteria:** Use "Plan & Preview", see all three views populated. Click a plan step node, see the corresponding action highlighted in Domain Graph.

**Dependencies:** WI-3.1, WI-3.2
**Effort:** Small

---

## Phase 4: Scenario Management & Regression

### WI-4.1: Scenario Editor

**Description:** Full scenario authoring with initial state, goals, and expected outcomes.

**Deliverables:**
- [ ] Scenario list panel: create, rename, duplicate, delete scenarios
- [ ] Per-scenario: initial state editor (checkboxes for each grounded fluent)
- [ ] Per-scenario: goal editor (select goal fluents)
- [ ] Per-scenario: expected outcome metadata (plan length range, specific actions expected/forbidden, expected success/failure)
- [ ] Scenarios stored in `ProjectModel` and serialised to project JSON

**Acceptance criteria:** Create nominal scenario and off-nominal scenario (e.g. comms loss). Both stored and reloadable.

**Dependencies:** WI-2.4, WI-0.3
**Effort:** Medium

---

### WI-4.2: Scenario Regression Runner

**Description:** Batch-run all scenarios and compare results against expected outcomes.

**Deliverables:**
- [ ] "Run All Scenarios" button
- [ ] Progress indicator (scenario N of M)
- [ ] Results table: scenario name, status (pass/fail/error), plan length, time, deviation from expected
- [ ] Detailed failure report: which expectation failed and why
- [ ] Export results to JSON report file

**Acceptance criteria:** Define 3 scenarios with expectations, run batch, see pass/fail results. Deliberately break a domain element, rerun, see regressions flagged.

**Dependencies:** WI-4.1
**Effort:** Medium

---

### WI-4.3: PDDL Import (Round-Trip)

**Description:** Import existing `.pddl` domain and problem files into the structured `ProjectModel`.

**Deliverables:**
- [ ] File > Import PDDL Domain / Import PDDL Problem
- [ ] Parse via `PddlParser::parseFromString()`, extract types, predicates, actions, objects, initial state, goals
- [ ] Populate `ProjectModel` from parsed data
- [ ] Auto-layout imported nodes on canvas (topological layout algorithm)
- [ ] Handle import conflicts (merge vs overwrite if project already has content)

**Acceptance criteria:** Import `domains/uav_search/domain.pddl` + `problem.pddl`, see all elements appear on canvas, save project, verify round-trip.

**Dependencies:** WI-2.1, WI-1.4
**Effort:** Medium

---

### WI-4.4: Contingency Verifier Integration

**Description:** Integrate the existing `contingency_verifier` for safe-state reachability analysis from within the authoring tool.

**Deliverables:**
- [ ] "Run Contingency Analysis" menu item
- [ ] Configure contingency predicates and safe-state predicates in scenario editor
- [ ] Run verifier against generated PDDL
- [ ] Display results: reachable/unreachable states, pruning statistics, coverage percentage
- [ ] Highlight actions/predicates involved in unreachable contingency paths

**Acceptance criteria:** Run contingency analysis on a domain with fault predicates, see reachability report.

**Dependencies:** WI-4.1, WI-2.4
**Effort:** Medium

---

## Phase 5: Polish & Integration

### WI-5.1: Action Registry Authoring

**Description:** Allow users to define the BT node mappings for PDDL actions (the `ActionRegistry` configuration).

**Deliverables:**
- [ ] Per-action: assign BT node type or subtree XML template
- [ ] Per-action: set reactive flag
- [ ] Subtree template editor with `{param0}`, `{param1}` placeholder support
- [ ] Preview resolved BT fragment for a given grounding

**Acceptance criteria:** Define `move` action mapped to a `MoveToLocation` BT node type. Compile a plan using it, see the correct BT node in BT View.

**Dependencies:** WI-1.4, WI-3.2
**Effort:** Medium

---

### WI-5.2: Node Palette & Search

**Description:** Provide a searchable palette of available node types for quick authoring.

**Deliverables:**
- [ ] Draggable palette panel with categories: Actions, Predicates, Types
- [ ] Search/filter box
- [ ] Drag from palette to canvas to create node
- [ ] Double-click canvas for quick-add popup (type to search, Enter to create)

**Acceptance criteria:** Open palette, search "move", drag to canvas, node created with default parameters ready for editing.

**Dependencies:** WI-1.4
**Effort:** Small

---

### WI-5.3: Keyboard Shortcuts & Workflow Polish

**Description:** Full keyboard-driven workflow for power users.

**Deliverables:**
- [ ] Delete: remove selected nodes/links
- [ ] Ctrl+C / Ctrl+V: copy/paste nodes
- [ ] Ctrl+D: duplicate selection
- [ ] F: fit canvas to content
- [ ] Space (on canvas): quick-add popup
- [ ] Tab: cycle between panels
- [ ] F5: Plan & Preview
- [ ] F6: Validate

**Acceptance criteria:** Complete a full domain authoring workflow using only keyboard (no mouse required for node creation, editing, validation).

**Dependencies:** WI-1.6, WI-5.2
**Effort:** Small

---

### WI-5.4: Theming & Visual Polish

**Description:** Professional appearance matching the project's aesthetic.

**Deliverables:**
- [ ] Dark theme (default) matching DevEnv colour scheme
- [ ] Light theme option
- [ ] Consistent node colouring: actions (blue header), predicates (green header), types (grey header)
- [ ] Link colouring by type: precondition links (orange), effect links (purple), causal links (teal)
- [ ] Icon set for node types (optional, can use text labels initially)
- [ ] Smooth animations for canvas navigation

**Acceptance criteria:** Tool looks professional and visually consistent. Non-technical stakeholders find it legible.

**Dependencies:** WI-1.4
**Effort:** Small

---

## Summary & Milestones

| Milestone | Work Items | Delivers | Target Readiness |
|-----------|------------|----------|-----------------|
| **M0: Buildable shell** | WI-0.1, WI-0.2, WI-0.3 | Empty app opens, project save/load works | Foundation |
| **M1: Domain authoring** | WI-1.1 – WI-1.6 | Full graphical PDDL domain creation | Core authoring |
| **M2: Validated output** | WI-2.1 – WI-2.5 | PDDL generation + multi-level validation | Functional tool |
| **M3: Visual preview** | WI-3.1 – WI-3.3 | Plan & BT visualisation alongside authoring | Review-ready |
| **M4: Scenario management** | WI-4.1 – WI-4.4 | Regression packs, import, contingency | Engineering-complete |
| **M5: Production polish** | WI-5.1 – WI-5.4 | Action registry, palette, shortcuts, theme | Release-ready |

---

## Dependency Graph

```
WI-0.1 ──> WI-0.2 ──> WI-1.1 ──> WI-1.3 ──> WI-1.4 ──> WI-1.5
  │              │                    │           │           │
  └──> WI-0.3 ──┘                    │           │           └──> WI-1.6
         │                           │           │
         └───────────────────────────>└───────────┴──> WI-2.1 ──> WI-2.2 ──> WI-2.3 ──> WI-2.4
                                                                                           │
         WI-1.2 ──> WI-1.3                                                                 │
                                                         WI-3.1 <─────────────────────────┘
                                                         WI-3.2 <──────────────────────────┘
                                                         WI-3.3 <── WI-3.1 + WI-3.2

         WI-4.1 <── WI-2.4 + WI-0.3
         WI-4.2 <── WI-4.1
         WI-4.3 <── WI-2.1 + WI-1.4
         WI-4.4 <── WI-4.1 + WI-2.4

         WI-5.1 <── WI-1.4 + WI-3.2
         WI-5.2 <── WI-1.4
         WI-5.3 <── WI-1.6 + WI-5.2
         WI-5.4 <── WI-1.4
```

---

## Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| imgui-node-editor maintenance slows further | Medium | Low | Library is stable; fork if needed; imnodes as fallback |
| Dear ImGui docking branch instability | Low | Medium | Pin to known-good commit; docking is now merged to master |
| PDDL round-trip lossy for complex domains | Medium | Medium | Start with STRIPS subset (which is all AME supports); document limitations |
| Undo/redo complexity grows with features | Medium | Low | Command pattern isolates each operation; test coverage per command |
| Auto-layout quality for large domains | Medium | Low | Use simple layered layout initially; can integrate dagre/graphviz later |
| Platform backend (SDL2/GLFW) issues on Windows | Low | Low | Both are well-tested on Windows/MSVC; SDL2 preferred for broader input handling |

---

## Non-Goals (Explicitly Out of Scope)

- Web/browser deployment (that is Option 2/3 from the parent research)
- ADL, conditional effects, or temporal PDDL (AME only supports STRIPS + typing)
- Replacing the existing DevEnv (it continues for monitoring/observability)
- Live execution monitoring (that remains in DevEnv + Foxglove)
- Multi-user collaboration
- ROS2 integration from the authoring tool (authoring is offline; execution uses DevEnv/ROS2)
