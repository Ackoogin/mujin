# Architecture

PDDL planning + behaviour tree execution with a shared world model.

---

## Components

```
┌─────────────┐    ┌──────────┐    ┌──────────────┐    ┌──────────┐
│ WorldModel  │───▶│  LAPKT   │───▶│ Plan-to-BT   │───▶│  BT.CPP  │
│             │◀───│  Planner  │    │  Compiler    │    │ Executor │
│             │    └──────────┘    └──────────────┘    └────┬─────┘
│             │◀─────────────────────────────────────────────┘
│             │         (effects update world model)
│             │
│             │◀──── Perception (external fact updates)
└─────────────┘
```

1. **WorldModel** — single source of truth. Typed objects + boolean predicates stored as an eagerly-grounded bitset. Presents two projections: a `STRIPS_Problem` for LAPKT, and a read-only blackboard view for BT.CPP.

2. **LAPKT Planner** — stateless STRIPS solver. Receives a projected `STRIPS_Problem` + goal, returns an ordered plan. Currently BFS(f)/SIW; solver is swappable.

3. **Plan-to-BT Compiler** — converts a LAPKT plan into executable BT XML. Builds a causal graph from effect-precondition matching, extracts independent execution flows, and composes them into a Parallel/Sequence BT structure.

4. **BT.CPP Executor** — ticks the compiled tree. BT nodes read preconditions from and write effects to the WorldModel via dedicated node types.

5. **ActionRegistry** — maps PDDL action names to BT implementations (single nodes, sub-tree templates, or pre-authored sub-tree files). Configures reactive vs. non-reactive precondition checking per action.

6. **MissionExecutor** — top-level tick loop with replan-on-failure. On action failure, snapshots world model state, replans, recompiles, and swaps the tree.

---

## WorldModel

```cpp
class WorldModel {
public:
    // Object/type management
    ObjectId addObject(const std::string& name, const std::string& type);

    // Predicate management
    PredicateId registerPredicate(const std::string& name,
                                  const std::vector<std::string>& param_types);

    // State manipulation (authoritative)
    void setFact(PredicateId pred, const std::vector<ObjectId>& args, bool value);
    bool getFact(PredicateId pred, const std::vector<ObjectId>& args) const;

    // String-keyed access (used by BT nodes)
    void setFact(const std::string& key, bool value);
    bool getFact(const std::string& key) const;

    // Index-based access (used by LAPKT projection)
    void setFact(unsigned fluent_id, bool value);
    bool getFact(unsigned fluent_id) const;

    // PDDL projection
    void projectToSTRIPS(aptk::STRIPS_Problem& prob) const;
    aptk::State* currentStateAsSTRIPS() const;

    // Blackboard projection (one-way push, blackboard is read-only)
    void syncToBlackboard(BT::Blackboard::Ptr bb) const;

    // Change tracking
    uint64_t version() const;  // monotonic, increments on any state change

private:
    DynamicBitset facts_;                       // storage-efficient, not vector<bool>
    BiMap<unsigned, std::string> fluent_map_;    // fluent_id <-> "predicate(arg1,arg2)"
    TypeSystem types_;
};
```

**Eager grounding**: when an object is added, all predicates that can bind to its type are immediately grounded. Fluent indices are stable — once assigned, they never change. This makes `projectToSTRIPS()` a direct mapping and allows replanning without rebuilding the problem.

**Authoritative state**: the WorldModel owns all truth. The blackboard is a read-only view pushed via `syncToBlackboard()`. All mutations go through `SetWorldPredicate` BT nodes that call `setFact()` directly. LAPKT gets a snapshot projection.

**Error model**: on action failure, the BT does not apply PDDL effects. Ground truth comes from external perception systems calling `setFact()`. The WorldModel never trusts the plan's model of what happened.

---

## Domain Specification

Hybrid approach: PDDL files define the planning model (predicates, types, action schemas with preconditions/effects). A separate configuration maps PDDL action names to BT implementations via the ActionRegistry.

PDDL parsing uses LAPKT's FF-parser (`ff_to_aptk`). Fallback: minimal PDDL tokenizer for STRIPS-level domains.

This separation keeps the planning model and execution model independently replaceable.

---

## ActionRegistry

```cpp
class ActionRegistry {
public:
    void registerAction(const std::string& pddl_name,
                        const std::string& bt_node_type,
                        bool reactive = false);

    void registerActionSubTree(const std::string& pddl_name,
                               const std::string& subtree_xml_template,
                               bool reactive = false);

    void registerActionSubTreeFile(const std::string& pddl_name,
                                    const std::string& xml_path,
                                    bool reactive = false);

    ActionImpl resolve(const std::string& action_name,
                       const std::vector<std::string>& params) const;
};
```

Three levels of action implementation:
1. **Simple**: `move(robot, from, to)` → single `MoveAction` BT node
2. **Template**: `transport(robot, pkg, from, to)` → parameterized sub-tree XML with `{param0}`, `{param1}` substitution
3. **Pre-authored**: hand-crafted BT XML file for complex behaviours (e.g., reactive search with fallbacks)

The `reactive` flag controls whether the compiled action unit uses `ReactiveSequence` (preconditions re-checked every tick) or `Sequence` (checked once at start). Long-running actions like `search_sector` use reactive; atomic actions use non-reactive.

---

## Plan-to-BT Compiler

Converts a LAPKT plan into a BT:

1. **Causal graph** — for each pair (i, j) where i < j: if any add-effect of step i is a precondition of step j, add edge i→j.

2. **Flow extraction** — topological sort, group into independent causal chains. Steps with no cross-flow dependencies form separate flows.

3. **Action unit generation** — each plan step becomes:
   ```xml
   <Sequence|ReactiveSequence name="action_params">
       <CheckWorldPredicate predicate="pre1" expected="true"/>
       ...
       <resolved BT fragment from ActionRegistry>
       <SetWorldPredicate predicate="add1" value="true"/>
       <SetWorldPredicate predicate="del1" value="false"/>
   </Sequence|ReactiveSequence>
   ```

4. **Tree composition**:
   - Single flow → top-level `Sequence`
   - Multiple flows → `Parallel` node with `success_count = flow_count`
   - Shared actions (join points) → blackboard-flag guard pattern (first execution sets done-flag; duplicates check and skip)

5. **Output** — XML string loadable by `BT::BehaviorTreeFactory::createTreeFromText()`

Sequential fallback mode (all steps in one `Sequence`) available for debugging.

---

## BT Node Types

### CheckWorldPredicate (Condition)

Reads `predicate` port (string key), queries `WorldModel::getFact()`, returns SUCCESS/FAILURE.

### SetWorldPredicate (SyncAction)

Reads `predicate` + `value` ports, calls `WorldModel::setFact()`, returns SUCCESS.

### ReplanOnFailure (Decorator)

Wraps a sub-tree. On child FAILURE, signals replan via blackboard flag. MissionExecutor picks up the signal.

Both world-model nodes receive `WorldModel*` via a shared pointer in the root blackboard.

---

## Replanning

```cpp
class MissionExecutor {
public:
    MissionExecutor(WorldModel& wm, ActionRegistry& registry);
    void setGoal(const std::vector<unsigned>& goal_fluents);

    TickResult tick() {
        if (!current_tree_) replan();

        auto status = current_tree_->tickOnce();

        if (status == BT::NodeStatus::FAILURE) {
            replan();
            return TickResult::REPLANNING;
        }
        if (status == BT::NodeStatus::SUCCESS) {
            return TickResult::COMPLETE;
        }
        return TickResult::RUNNING;
    }

private:
    void replan() {
        aptk::STRIPS_Problem prob;
        world_model_.projectToSTRIPS(prob);
        auto plan = solver.solve(prob, world_model_.currentStateAsSTRIPS(), goal_);
        auto bb = BT::Blackboard::create();
        world_model_.syncToBlackboard(bb);
        current_tree_ = compiler_.compileAndCreate(plan, bb);
    }
};
```

On failure: halt tree, snapshot world model (which may have been updated by perception since the failure), replan from current state, recompile, swap tree, resume ticking.

---

## ROS2 Deployment

The core library is ROS-agnostic. ROS2 adapter nodes wrap it:

```
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  WorldModel  │  │   Planner    │  │  Executor    │
│    Node      │  │    Node      │  │    Node      │
│              │  │              │  │              │
│ - Owns state │  │ - Stateless  │  │ - BT.CPP     │
│ - ROS2 svcs: │  │ - ROS2 svc:  │  │ - Ticks tree │
│   get/set    │  │   plan(goal) │  │ - Calls WM   │
│   facts      │  │   → plan     │  │   services   │
│ - Publishes  │  │              │  │              │
│   /world_    │  │              │  │              │
│   state      │  │              │  │              │
└──────────────┘  └──────────────┘  └──────────────┘
```

- **WorldModel Node** is the natural service boundary. All other nodes are clients.
- **Planner Node** is stateless and swappable.
- **Executor Node** is replaceable (BT or otherwise) without touching world model or planner.
- **Perception Nodes** are independent writers calling `set_fact` on the WorldModel service.
- Single-node (in-process) and multi-node (distributed) from the same code.

---

## Auditability

Every world model state change is logged with timestamp and source (BT node name or `"perception"`). The version counter enables replay and post-hoc analysis. The causal graph and compiled BT XML form the mission audit trail. See `extensions.md` for the full observability plan.

---

## Source Tree

```
include/mujin/
    world_model.h
    type_system.h
    action_registry.h
    plan_compiler.h
    pddl_parser.h
    bt_nodes/
        check_world_predicate.h
        set_world_predicate.h
        replan_on_failure.h
src/
    world_model.cpp
    type_system.cpp
    action_registry.cpp
    plan_compiler.cpp
    pddl_parser.cpp
    bt_nodes/
        check_world_predicate.cpp
        set_world_predicate.cpp
    main.cpp
tests/
    CMakeLists.txt
    test_world_model.cpp
    test_action_registry.cpp
    test_plan_compiler.cpp
    test_pddl_parser.cpp
    test_integration.cpp
domains/
    uav_search/
        domain.pddl
        problem.pddl
```

---

## Dependencies

| Dependency | Source | Purpose |
|------------|--------|---------|
| BehaviorTree.CPP 4.6.2 | FetchContent | BT execution |
| LAPKT Devel2.0 (core) | FetchContent | STRIPS model + search |
| LAPKT FF-parser | FetchContent (libff_parser) | PDDL parsing |
| Google Test | FetchContent | Unit + integration testing |
