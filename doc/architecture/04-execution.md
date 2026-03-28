# Execution: PlanCompiler, BT Nodes, Replanning

## Plan-to-BT Compiler

`PlanCompiler` (`include/ame/plan_compiler.h`) converts a LAPKT plan into executable BT XML.

### Algorithm

1. **Causal graph** — for each pair (i, j) where i < j: if any add-effect of step i is a precondition of step j, add edge i→j. Also track delete-effect conflicts for mutex detection.

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
   Sequence vs. ReactiveSequence comes from the ActionRegistry's per-action reactive flag.

4. **Tree composition**:
   - Single flow → top-level `Sequence`
   - Multiple flows → `Parallel` node with `success_count = flow_count`
   - Shared actions (join points) → blackboard-flag guard pattern (first execution sets done-flag; duplicates check and skip)

5. **Output** — XML string loadable by `BT::BehaviorTreeFactory::createTreeFromText()`

**Sequential fallback mode** (all steps in one `Sequence`) is available for debugging.

## BT Node Types

All world-model nodes receive `WorldModel*` via a shared pointer in the root blackboard.

### CheckWorldPredicate (Condition)

Reads `predicate` port (string key), queries `WorldModel::getFact()`, returns SUCCESS/FAILURE.

### SetWorldPredicate (SyncAction)

Reads `predicate` + `value` ports, calls `WorldModel::setFact()`, returns SUCCESS.

### ReplanOnFailure (Decorator)

Wraps a sub-tree. On child FAILURE, signals replan via blackboard flag. MissionExecutor picks up the signal.

### ExecutePhaseAction (StatefulAction)

Orchestrates a full plan–compile–execute cycle for a sub-goal set, enabling hierarchical decomposition of complex missions. See [07-extensions.md](07-extensions.md).

### InvokeService (StatefulAction)

Async PYRAMID service invocation node. Maps PDDL actions to external service calls via the `IPyramidService` abstract interface with timeout handling and cancellation. See [07-extensions.md](07-extensions.md).

## MissionExecutor / Replanning

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
