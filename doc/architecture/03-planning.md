# Planning: PDDL Parser, Planner, ActionRegistry

## Domain Specification

Hybrid approach: PDDL files define the **planning model** (predicates, types, action schemas with preconditions/effects). A separate configuration maps PDDL action names to **BT implementations** via the ActionRegistry.

This separation keeps the planning model and execution model independently replaceable.

## PDDL Parser

`PddlParser` (`include/mujin/pddl_parser.h`) loads PDDL domain and problem files into a WorldModel.

- Uses LAPKT's FF-parser (`ff_to_aptk`) as backend
- Supports STRIPS-level PDDL (`:typing`, `:strips`)
- Populates types, objects, predicates, initial facts, and goal fluents
- Example domain: `domains/uav_search/domain.pddl` + `problem.pddl`

## Planner (LAPKT Integration)

`Planner` (`include/mujin/planner.h`) is a stateless STRIPS solver.

1. `Planner::solve()` calls `WorldModel::projectToSTRIPS()` to build an `aptk::STRIPS_Problem`
2. Runs BRFS (Breadth-First Search) via LAPKT
3. Returns plan steps as indices into `WorldModel::groundActions()`
4. Records solve time in `PlanResult::solve_time_ms`

LAPKT is built from source as the `lapkt_core` static library (not its own CMake project). MSVC compatibility shims are in `cmake/compat/`.

## ActionRegistry

`ActionRegistry` (`include/mujin/action_registry.h`) bridges PDDL action names to BT.CPP implementations.

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
2. **Template**: `transport(robot, pkg, from, to)` → parameterised sub-tree XML with `{param0}`, `{param1}` substitution
3. **Pre-authored**: hand-crafted BT XML file for complex behaviours (e.g., reactive search with fallbacks)

The `reactive` flag controls whether the compiled action unit uses `ReactiveSequence` (preconditions re-checked every tick) or `Sequence` (checked once at start).

## Adding a New PDDL Action

1. Implement a BT node (subclass `BT::SyncActionNode` or `BT::StatefulActionNode`) in `include/mujin/bt_nodes/` + `src/bt_nodes/`
2. Add it to `mujin_core` in `src/CMakeLists.txt`
3. Register in `ActionRegistry` via `registerAction(pddl_name, bt_node_type)` or `registerActionSubTree(...)`
4. Register the BT node type with the BT.CPP factory: `factory.registerNodeType<MyNode>("MyNode")`
5. In ROS2 mode: also register on `ExecutorNode::factory()` in `combined_main.cpp`
