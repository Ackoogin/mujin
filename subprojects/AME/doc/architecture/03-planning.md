# Planning: PDDL Parser, Planner, ActionRegistry

## Domain Specification

Hybrid approach: PDDL files define the **planning model** (predicates, types, action schemas with preconditions/effects). A separate configuration maps PDDL action names to **BT implementations** via the ActionRegistry.

This separation keeps the planning model and execution model independently replaceable.

## PDDL Parser

`PddlParser` (`include/ame/pddl_parser.h`) loads PDDL domain and problem files into a WorldModel.

- Supports STRIPS-level PDDL (`:typing`, `:strips`)
- Populates types, objects, predicates, initial facts, and goal fluents
- Example domain: `subprojects/AME/domains/uav_search/domain.pddl` + `problem.pddl`
- Two entry points:
  - `PddlParser::parse(domain_path, problem_path, wm)` -- from file paths (deployment)
  - `PddlParser::parseFromString(domain_pddl, problem_pddl, wm)` -- from string content (service-driven loading)

## Domain Loading

Domains can be loaded in two ways:

1. **File-based (deployment):** Set `domain.pddl_file` and `domain.problem_file` parameters. The domain is parsed at configure time.
2. **Service-based (devenv/testing):** Call `~/load_domain` (`ame_ros2/srv/LoadDomain`) with PDDL content as strings. This allows the backend to push domain models without requiring file access.

Both WorldModelNode and PlannerNode expose `~/load_domain`. The WorldModelNode loads the **union domain** (superset of all predicates/objects). Each PlannerNode loads its **own domain** (subset relevant to its planning task). In a multi-planner setup:

- WorldModelNode: union domain containing all predicates from all planners
- PlannerNode A: logistics domain (move, load, unload actions)
- PlannerNode B: surveillance domain (fly, search, classify actions)

When a planner snapshots world state, it queries WorldModelNode for current facts and applies only the ones matching its own predicates. The `LoadDomain` service accepts a `domain_id` field for identification and returns the number of grounded fluents and actions after parsing.

## Planner (LAPKT Integration)

`Planner` (`include/ame/planner.h`) is a stateless STRIPS solver.

1. `Planner::solve()` calls `WorldModel::projectToSTRIPS()` to build an `aptk::STRIPS_Problem`
2. Runs BRFS (Breadth-First Search) via LAPKT
3. Returns plan steps as indices into `WorldModel::groundActions()`
4. Records solve time in `PlanResult::solve_time_ms`

LAPKT is built from source as the `lapkt_core` static library (not its own CMake project). MSVC compatibility shims are in `cmake/compat/`.

## ActionRegistry

`ActionRegistry` (`include/ame/action_registry.h`) bridges PDDL action names to BT.CPP implementations.

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

1. **Simple**: `move(robot, from, to)` -> single `MoveAction` BT node
2. **Template**: `transport(robot, pkg, from, to)` -> parameterised sub-tree XML with `{param0}`, `{param1}` substitution
3. **Pre-authored**: hand-crafted BT XML file for complex behaviours (e.g., reactive search with fallbacks)

The `reactive` flag controls whether the compiled action unit uses `ReactiveSequence` (preconditions re-checked every tick) or `Sequence` (checked once at start).

## Adding a New PDDL Action

1. Implement a BT node (subclass `BT::SyncActionNode` or `BT::StatefulActionNode`) in `include/ame/bt_nodes/` + `src/bt_nodes/`
2. Add it to `ame_core` in `src/CMakeLists.txt`
3. Register in `ActionRegistry` via `registerAction(pddl_name, bt_node_type)` or `registerActionSubTree(...)`
4. Register the BT node type with the BT.CPP factory: `factory.registerNodeType<MyNode>("MyNode")`
5. In ROS2 mode: also register on `ExecutorNode::factory()` in `combined_main.cpp`

