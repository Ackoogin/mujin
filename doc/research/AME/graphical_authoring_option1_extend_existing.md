# Option 1: Extend Existing -- Graphical Autonomy Authoring Tool

**Purpose:** Detailed technical assessment of Option 1 (extend the existing system) from the [parent options document](graphical_autonomy_authoring_tool_options.md), with specific focus on C++ immediate-mode GUI node-editor libraries and their suitability for building a graphical autonomy authoring and validation tool within the AME pipeline.

**Status:** Research and recommendation. No implementation started.

---

## 1. Executive Summary

Option 1 ("extend existing") can be pursued through two distinct routes:

- **Route A -- C++ native tool** using Dear ImGui plus a node-editor extension (imgui-node-editor, imnodes, or similar), built as a new CMake target that links directly against `ame_core`.
- **Route B -- Python DevEnv extension** using Dear PyGui's built-in node editor API, extending the existing Python DevEnv tabs.

Route A is the stronger path. It gives native access to `WorldModel`, `PddlParser`, `Planner`, `PlanCompiler`, and `ActionRegistry` without serialisation boundaries, and the best candidate node-editor libraries (imgui-node-editor, imnodes) are mature, MIT-licensed, and purpose-built for exactly this kind of tool. The existing DevEnv can continue to serve its monitoring and observability role while the new C++ tool handles authoring.

---

## 2. What the Authoring Tool Must Integrate With

The authoring tool must read and write directly against these AME APIs:

| API | Header | Role in Authoring |
|-----|--------|-------------------|
| `PddlParser` | `include/ame/pddl_parser.h` | Import existing PDDL domains/problems |
| `WorldModel` | `include/ame/world_model.h` | Register predicates, objects, action schemas; inspect grounded state |
| `TypeSystem` | `include/ame/type_system.h` | Define type hierarchies |
| `ActionRegistry` | `include/ame/action_registry.h` | Map PDDL actions to BT node types or subtree templates |
| `Planner` | `include/ame/planner.h` | Validate plans are feasible from authored domains |
| `PlanCompiler` | `include/ame/plan_compiler.h` | Generate BT XML from plans; preview compiled trees |

All of these are C++ APIs in `ame_core`. A C++ tool links directly. A Python tool must go through the Python bindings (`_ame_py`) or over ROS2/PCL/Foxglove.

---

## 3. Candidate Node-Editor Libraries

### 3.1 imgui-node-editor (thedmd)

**Repository:** https://github.com/thedmd/imgui-node-editor
**Licence:** MIT
**Language:** C++ (Dear ImGui extension)
**Stars / Activity:** ~6.4k stars; maintained; used by production tools including the Blueprint visual scripting system

#### Features

- Full-featured node graph editor built on Dear ImGui
- Nodes with titled headers, typed input/output pins, and customisable bodies
- Bezier curve links between pins with flow markers
- Interactive: drag nodes, pan/zoom canvas, create/delete links, selection, context menus
- Grouping and comment nodes
- Navigation: smooth zoom, fit-to-content, node search
- Blueprint-style visual language support (header colours, flow/data pin distinction)
- JSON serialisation of node/link layout
- Renders entirely through Dear ImGui draw lists -- no extra GPU dependencies

#### Integration with AME

A natural fit. Each authoring concept maps to a node type:

| AME Concept | Node Representation |
|-------------|---------------------|
| PDDL Action schema | Node with parameter input pins, precondition/effect output pins |
| Predicate | Small node or pin type |
| Object / Type | Node or sidebar panel element |
| Plan step | Node in a compiled-plan view graph |
| BT node | Node in a behaviour-tree view graph |
| Causal dependency | Link between action effect pin and action precondition pin |

#### Concerns

- Requires Dear ImGui integration (not Dear PyGui -- these are different)
- Adds Dear ImGui as a dependency to the C++ build (lightweight, header-only, MIT)
- Canvas persistence must be handled by the application (JSON save/load)

#### Verdict

**Best overall candidate for a C++ native authoring tool.** Rich enough for a production-grade node editor, permissively licensed, well-maintained.

---

### 3.2 imnodes (Nelarius)

**Repository:** https://github.com/Nelarius/imnodes
**Licence:** MIT
**Language:** C++ (Dear ImGui extension)
**Stars / Activity:** ~2.2k stars; maintained; lean codebase (~2 files)

#### Features

- Minimalist node-graph editor for Dear ImGui
- Nodes, pins (input/output), and links
- Minimap
- Grid-snapping
- Simple API: `BeginNode()`, `BeginInputAttribute()`, `Link()`, etc.
- No built-in context menus, grouping, or Blueprint-style headers

#### Integration with AME

Works well for a simpler version of the same concept. Less visual polish than imgui-node-editor, but faster to integrate and easier to extend.

#### Concerns

- Less visually rich -- no node headers with colours, no flow markers
- No grouping or comment nodes
- Would need more custom code for a polished experience

#### Verdict

**Good second choice or prototyping option.** Simpler API, faster to stand up, but less capable for a full authoring workbench.

---

### 3.3 Dear PyGui Built-in Node Editor

**Library:** Part of Dear PyGui (`dearpygui.dearpygui`)
**Licence:** MIT
**Language:** Python
**API:** `dpg.add_node_editor()`, `dpg.add_node()`, `dpg.add_node_attribute()`, `dpg.add_node_link()`

#### Features

- Built into the existing DevEnv UI framework
- Nodes with input/output attributes
- Link creation/deletion with callbacks
- Pan and context menu support
- Integrates with all other Dear PyGui widgets (tables, plots, text inputs)

#### Integration with AME

Could be added as a new tab in the existing DevEnv. Backend integration goes through the existing `pcl_client.py` or `ros2_client.py` layers.

#### Concerns

- Dear PyGui's node editor is more basic than imgui-node-editor
- No Blueprint-style visual polish
- AME API access is indirect (through Python bindings or RPC)
- Dear PyGui is less actively maintained than Dear ImGui itself

#### Verdict

**Lowest-effort starting point, but limited ceiling.** Best for a quick proof-of-concept tab in the existing DevEnv, not for a full authoring tool.

---

### 3.4 Groot / Groot2 (BehaviorTree.CPP)

**Groot2:** https://www.behaviortree.dev/groot/ (commercial, proprietary)
**Groot (v1):** Was open-source but is now deprecated/unmaintained
**Licence:** Groot2 is commercial; Groot1 was Apache-2.0 but no longer supported

#### Features

- Purpose-built BT.CPP visual editor
- Drag-and-drop BT node composition
- Live monitoring of BT execution state
- XML import/export matching BT.CPP's format

#### Integration with AME

AME's observability stack (Layer 1-5) was explicitly designed to **replace Groot2** with open, composable, sink-based logging (see `doc/architecture/05-observability.md`). Groot2's commercial licence makes it unsuitable. Groot1 is unmaintained and does not support BT.CPP v4.

#### Verdict

**Not suitable.** Commercial licensing conflicts with project constraints, and the project has already moved away from it.

---

### 3.5 Qt Node Editor (paceholder/nodeeditor)

**Repository:** https://github.com/paceholder/nodeeditor
**Licence:** BSD-3-Clause
**Language:** C++ (Qt Widgets)
**Stars / Activity:** ~3.0k stars; intermittently maintained

#### Features

- Polished node graph editor using Qt Widgets
- Typed port connections with validation
- Data propagation model (nodes can compute outputs)
- Undo/redo support
- Built-in serialisation

#### Integration with AME

Would require adding Qt as a dependency. Qt itself is LGPL (dynamic linking) or commercial -- this conflicts with the project's licensing constraints. The node editor library (BSD-3-Clause) is fine, but its Qt dependency is not.

#### Concerns

- Qt LGPL/commercial licence is a blocker per project policy
- Heavyweight dependency for a single feature
- Qt-based tool would be architecturally separate from both ImGui and Dear PyGui

#### Verdict

**Not recommended due to Qt licensing.** If Qt were already in the stack, this would be a strong candidate.

---

### 3.6 ImNodeFlow (Fattorino)

**Repository:** https://github.com/Fattorino/ImNodeFlow
**Licence:** MIT
**Language:** C++ (Dear ImGui extension)
**Stars / Activity:** ~480 stars; moderate activity; v1.2.2 (Jun 2024)

#### Features

- Built-in typed input/output logic with connection filtering
- Template-based I/O system with type awareness
- Zoom support
- Modern C++ style (C++17)
- Automatic pin type validation

#### Integration with AME

The typed connection system maps naturally to PDDL type constraints -- a predicate output pin of type `(at ?r ?l)` could only connect to an action precondition input expecting that predicate signature.

#### Concerns

- Smallest community of the ImGui options (483 stars)
- Less battle-tested than imgui-node-editor or imnodes
- No undo/redo or serialisation

#### Verdict

**Interesting for its type-aware connections, but too young for a production tool.** Worth monitoring.

---

### 3.7 BTStudio (osu-uwrt)

**Repository:** https://github.com/osu-uwrt/btstudio
**Licence:** Open source
**Language:** TypeScript / React (web-based)
**Activity:** Updated March 2026

#### Features

- Open-source BehaviorTree editor targeting BT.CPP v4 XML format
- Workspace-based workflow
- Shared subtree libraries
- Automatic cross-file subtree updates

#### Integration with AME

Could serve as a reference implementation for BT.CPP v4 XML editing patterns, even though it cannot be directly embedded in a C++ desktop tool. Its approach to handling the `<TreeNodesModel>` section and subtree includes is instructive.

#### Concerns

- Web-only; would need Electron/CEF to embed
- Less mature than React Flow ecosystem

#### Verdict

**Useful reference, not a direct candidate.** Study its BT.CPP v4 XML handling for design inspiration.

---

### 3.8 Other Notable Libraries

| Library | Licence | Language | Notes |
|---------|---------|----------|-------|
| **ax-nodeeditor** | MIT | C++ | Fork of imgui-node-editor with added features |
| **imgui-knobs** / **imgui-toggle** | MIT | C++ | Complementary ImGui widgets for parameter editing |
| **QuickQanava** | BSD-3 | C++17/QML | Rich graph vis with Qt6 QML; too heavy a dependency |
| **Rete.js** | MIT | TypeScript | Plugin-based flow editor; web-only, smaller ecosystem than React Flow |

---

## 4. Comparison Matrix

| Criterion | imgui-node-editor | imnodes | ImNodeFlow | Dear PyGui Node Ed. | Qt nodeeditor | Groot2 |
|-----------|:-:|:-:|:-:|:-:|:-:|:-:|
| **Licence** | MIT | MIT | MIT | MIT | BSD-3 (but Qt LGPL) | Commercial |
| **Visual polish** | High | Medium | Medium | Low-Medium | High | High |
| **Native AME integration** | Direct (C++) | Direct (C++) | Direct (C++) | Indirect (Python) | Direct (C++) | Limited |
| **Node headers & styles** | Yes | No | Basic | Basic | Yes | Yes |
| **Grouping / comments** | Yes | No | No | No | No | No |
| **Pan / zoom** | Yes | Yes (minimap) | Yes | Yes | Yes | Yes |
| **Typed connections** | Manual | Manual | Built-in | Manual | Built-in | Yes |
| **Context menus** | Yes | Manual | Manual | Manual | Yes | Yes |
| **Undo / redo** | Manual | Manual | Manual | Manual | Built-in | Built-in |
| **Serialisation** | JSON (manual) | Manual | Manual | Manual | Built-in (JSON) | XML |
| **Maturity** | Production | Stable | Young | Stable | Stable | Production |
| **Maintenance** | Active | Maintenance | Moderate | Slowing | Active | Active |
| **New dependency weight** | Dear ImGui (light) | Dear ImGui (light) | Dear ImGui (light) | None (already used) | Qt (heavy) | N/A |
| **Licence risk** | None | None | None | None | LGPL concern | Commercial |

### Note on the PDDL Visual Editing Landscape

No mature open-source visual/node-based PDDL domain editor exists. Available PDDL tools are text-focused (VS Code PDDL plugin, editor.planning.domains, etc.). This means any solution requires custom development for the PDDL domain authoring component. No library provides PDDL editing out of the box. The authoring tool's value comes from the custom mapping between PDDL planning concepts and the node-graph visual metaphor.

---

## 5. Recommended Route: C++ Native Tool with imgui-node-editor

### 5.1 Architecture

```
┌─────────────────────────────────────────────────┐
│  ame_authoring_tool  (new CMake executable)      │
│                                                   │
│  ┌─────────────┐  ┌────────────────────────────┐ │
│  │ Dear ImGui   │  │ imgui-node-editor          │ │
│  │ (UI frame,   │  │ (node canvas, links,       │ │
│  │  menus,      │  │  pan/zoom, selection)       │ │
│  │  docking,    │  │                            │ │
│  │  file dialog)│  │  ┌──────────────────────┐  │ │
│  │              │  │  │ Domain Graph View     │  │ │
│  │              │  │  │ (actions, predicates, │  │ │
│  │              │  │  │  types, objects)       │  │ │
│  │              │  │  ├──────────────────────┤  │ │
│  │              │  │  │ Plan View             │  │ │
│  │              │  │  │ (causal graph,        │  │ │
│  │              │  │  │  parallel flows)      │  │ │
│  │              │  │  ├──────────────────────┤  │ │
│  │              │  │  │ BT View               │  │ │
│  │              │  │  │ (compiled tree        │  │ │
│  │              │  │  │  structure)            │  │ │
│  │              │  │  └──────────────────────┘  │ │
│  └─────────────┘  └────────────────────────────┘ │
│                                                   │
│  ┌───────────────────────────────────────────────┐│
│  │ ame_core (linked directly)                     ││
│  │  WorldModel · PddlParser · Planner            ││
│  │  PlanCompiler · ActionRegistry · TypeSystem    ││
│  └───────────────────────────────────────────────┘│
└─────────────────────────────────────────────────┘
```

### 5.2 CMake Integration

The new target fits naturally into the existing build system:

```cmake
# subprojects/AME/src/apps/CMakeLists.txt (or new authoring/ directory)
add_executable(ame_authoring_tool
    authoring/main.cpp
    authoring/domain_graph_view.cpp
    authoring/plan_view.cpp
    authoring/bt_view.cpp
    authoring/pddl_generator.cpp
    authoring/project_model.cpp
)

target_link_libraries(ame_authoring_tool PRIVATE
    ame_core
    imgui           # FetchContent
    imgui_node_editor  # FetchContent
    # Platform backend (SDL2+OpenGL3 or GLFW+OpenGL3)
)
```

Dear ImGui, imgui-node-editor, and a platform backend (SDL2 or GLFW, both permissively licensed) would be added via FetchContent, consistent with the project's existing dependency management.

### 5.3 View Modes

The tool would provide three complementary graph views:

#### Domain Graph View (primary authoring surface)
- Action nodes with parameter pins, precondition input pins, and effect output pins
- Predicate nodes showing type signature
- Type hierarchy displayed as a tree sidebar or node graph
- Object instances listed per type
- Links show which predicates feed which action preconditions/effects
- **Authoring workflow:** create/edit action schemas, predicates, types, and objects graphically

#### Plan View (validation and review)
- After running the planner, display the causal graph as a node graph
- Each plan step is a node; causal dependency links shown between them
- Parallel flows visually separated (matching PlanCompiler's causal analysis)
- Colour-coding for flow membership
- **Read-only view** generated from planner output

#### BT View (compiled tree visualisation)
- Display the compiled BT XML as a tree graph
- Sequence, Selector, Parallel nodes as container nodes
- CheckWorldPredicate, SetWorldPredicate, InvokeService as leaf nodes
- **Read-only view** generated from PlanCompiler output

### 5.4 Authoring-to-PDDL Pipeline

The authoring tool maintains a structured internal model and generates PDDL:

```
User edits graph ──> Internal Model (structs/JSON)
                          │
                          ├──> Generate PDDL domain string
                          ├──> Generate PDDL problem string
                          │
                          ├──> PddlParser::parseFromString() validates
                          ├──> WorldModel populated from parse
                          ├──> Planner::solve() checks feasibility
                          └──> PlanCompiler::compile() previews BT
```

The internal model is canonical. PDDL is a generated artefact, visible in an "Advanced" panel for expert inspection. Round-trip import is supported via `PddlParser::parseFromString()`.

### 5.5 Validation Pipeline

Validation at multiple levels, all invoked from the GUI:

| Level | Check | AME API Used |
|-------|-------|-------------|
| 1. Structural | Missing names, dangling references, type mismatches | Internal model validation |
| 2. PDDL parse | Generated PDDL parses without errors | `PddlParser::parseFromString()` |
| 3. Grounding | All predicates and actions ground correctly | `WorldModel` eager grounding |
| 4. Planning | A plan exists for the given goals | `Planner::solve()` |
| 5. Compilation | Plan compiles to valid BT XML | `PlanCompiler::compile()` |
| 6. Contingency | Safe-state reachability under faults | `contingency_verifier` integration |

---

## 6. Dear PyGui Fallback (Route B)

If the C++ route is deferred, the existing DevEnv can be extended with a node-editor tab using Dear PyGui's built-in API:

```python
with dpg.node_editor(callback=on_link_created, delink_callback=on_link_deleted):
    with dpg.node(label="move", pos=[100, 100]):
        with dpg.node_attribute(attribute_type=dpg.mvNode_Attr_Input):
            dpg.add_text("?robot: robot")
        with dpg.node_attribute(attribute_type=dpg.mvNode_Attr_Input):
            dpg.add_text("pre: (at ?r ?from)")
        with dpg.node_attribute(attribute_type=dpg.mvNode_Attr_Output):
            dpg.add_text("add: (at ?r ?to)")
        with dpg.node_attribute(attribute_type=dpg.mvNode_Attr_Output):
            dpg.add_text("del: (at ?r ?from)")
```

This gives a basic node-graph tab quickly, but with limited visual polish and no path to the richer features imgui-node-editor provides (grouping, Blueprint-style headers, smooth zoom, flow markers).

---

## 7. Effort Estimates

| Route | Scope | Effort | Result |
|-------|-------|--------|--------|
| **A1: C++ imgui-node-editor, domain graph only** | Domain authoring + PDDL generation + validation | Medium | Production-grade authoring tool |
| **A2: A1 + plan/BT views** | Add plan causal graph and compiled BT visualisation | Medium-High | Full authoring + validation + visualisation |
| **B: Dear PyGui node tab** | Basic node graph in existing DevEnv | Low | Quick proof-of-concept, limited ceiling |
| **B+: Dear PyGui node tab + forms** | Node graph + structured form editors | Low-Medium | Functional but not polished |

---

## 8. Recommended Delivery Sequence

### Phase 1: Proof of Concept (Route B)

Add a `DomainGraphTab` to the existing DevEnv using Dear PyGui's node editor. This validates the UX concept and identifies what graph features are actually needed.

- Node per PDDL action schema
- Predicate nodes
- Links for precondition/effect relationships
- PDDL generation from the graph model
- Parse validation via Python bindings or PCL client

### Phase 2: C++ Authoring Tool (Route A1)

Build `ame_authoring_tool` using Dear ImGui + imgui-node-editor:

- Domain Graph View with full action/predicate/type authoring
- Internal structured model with JSON persistence
- PDDL generation and round-trip import
- Integrated validation (parse, ground, plan)
- Export PDDL files for use with the existing pipeline

### Phase 3: Visualisation Views (Route A2)

Add Plan View and BT View:

- Causal graph visualisation from planner output
- Compiled BT tree visualisation
- Side-by-side domain authoring + plan preview

### Phase 4: Scenario Management

- Scenario templates (initial state + goals)
- Regression pack management
- Expected vs actual outcome comparison
- Integration with the contingency verifier

---

## 9. BT.CPP v4 XML Interoperability

The authoring tool must read and write BT.CPP v4 XML to remain interoperable with the existing execution pipeline. Key format points:

- Trees are wrapped in `<root BTCPP_format="4">` containing one or more `<BehaviorTree ID="...">` elements
- A `<TreeNodesModel>` section declares node types with their input/output ports
- BT.CPP provides `BT::writeTreeNodesModelXML(factory)` to auto-generate this model XML from registered C++ node classes
- Subtree support via `<include>` and `_autoremap="true"` for blackboard port remapping
- `PlanCompiler::compile()` already generates this XML format; the authoring tool's BT View would parse and display it

The Plan View and BT View would be read-only visualisations of output from `Planner` and `PlanCompiler`. The Domain Graph View is the primary authoring surface.

---

## 10. Dependencies and Licensing

All recommended dependencies are permissively licensed:

| Dependency | Licence | Purpose | Fetch Method |
|------------|---------|---------|-------------|
| **Dear ImGui** | MIT | UI framework | FetchContent |
| **imgui-node-editor** | MIT | Node graph editor | FetchContent |
| **SDL2** or **GLFW** | Zlib / BSD-3 | Platform/window backend | FetchContent or system |
| **ImGui-FileBrowser** | MIT | File open/save dialogs | FetchContent (optional) |
| **nlohmann/json** | MIT | Project model serialisation | FetchContent |
| **stb_image** | Public domain | Image loading (optional) | Already common |

No GPL, LGPL, AGPL, or commercial dependencies.

---

## 11. Comparison with Other Options

| Factor | Option 1 (C++ extend) | Option 2 (Hybrid web) | Option 3 (Browser-first) |
|--------|:-----:|:-----:|:-----:|
| Native AME API access | Direct | Indirect (API layer) | Indirect (API layer) |
| Single executable | Yes | Yes (embedded) | No (browser + server) |
| Visual polish ceiling | High (imgui-node-editor) | Highest (React Flow) | Highest (React Flow) |
| Additional runtime deps | Minimal (ImGui, SDL2) | Python + Node.js + browser | Python + Node.js |
| Build system alignment | CMake (same as AME) | CMake + npm | CMake + npm |
| Offline / air-gapped | Native | Native | Needs local server |
| Stakeholder accessibility | Developer-to-engineer | Engineer-to-stakeholder | Stakeholder-friendly |
| Validation latency | Zero (in-process) | RPC round-trip | RPC round-trip |
| Platform backends | Windows, Linux, macOS | Windows, Linux, macOS | Any browser |

---

## 12. Recommendation

**Build a C++ native authoring tool using Dear ImGui + imgui-node-editor**, linked directly against `ame_core`.

This is the right choice for Option 1 because:

1. **Zero-friction AME integration.** The tool calls `PddlParser`, `WorldModel`, `Planner`, and `PlanCompiler` directly -- no serialisation, no RPC, no bindings layer.

2. **imgui-node-editor is production-proven.** MIT-licensed, 6k+ stars, actively maintained, and purpose-built for exactly this kind of domain-specific visual editor.

3. **Single build system.** Adds a new CMake target using the same FetchContent pattern as BT.CPP and LAPKT. No second build toolchain.

4. **Clean separation from DevEnv.** The authoring tool focuses on domain/scenario creation; the existing DevEnv continues to handle monitoring, observability, and execution control. No need to merge concerns.

5. **Incremental path.** Start with domain authoring (Phase 2), add plan/BT visualisation later (Phase 3), add scenario management last (Phase 4). Each phase delivers standalone value.

If a quick proof-of-concept is needed before committing to the C++ route, add a Dear PyGui node editor tab to the existing DevEnv first (Phase 1). This validates the concept at low cost and informs the C++ tool's design.

---

## 13. Sources

- imgui-node-editor: https://github.com/thedmd/imgui-node-editor (MIT)
- imnodes: https://github.com/Nelarius/imnodes (MIT)
- Dear ImGui: https://github.com/ocornut/imgui (MIT)
- Dear PyGui node editor API: https://dearpygui.readthedocs.io/en/latest/documentation/node-editor.html
- Qt Node Editor (paceholder): https://github.com/paceholder/nodeeditor (BSD-3-Clause, requires Qt LGPL)
- Groot2: https://www.behaviortree.dev/groot/ (commercial)
- SDL2: https://github.com/libsdl-org/SDL (Zlib)
- GLFW: https://github.com/glfw/glfw (Zlib/libpng)
- AME architecture docs: `subprojects/AME/doc/architecture/`
- ImNodeFlow: https://github.com/Fattorino/ImNodeFlow (MIT)
- BTStudio: https://github.com/osu-uwrt/btstudio (BT.CPP v4 reference editor)
- Rete.js: https://retejs.org/ (MIT)
- QuickQanava: https://github.com/cneben/QuickQanava (BSD-3-Clause)
- BT.CPP v4 XML Schema: https://www.behaviortree.dev/docs/learn-the-basics/xml_format/
- VS Code PDDL Plugin: https://github.com/jan-dolejsi/vscode-pddl
- Planning.Domains Editor: https://editor.planning.domains/
- Parent options document: `doc/research/AME/graphical_autonomy_authoring_tool_options.md`
