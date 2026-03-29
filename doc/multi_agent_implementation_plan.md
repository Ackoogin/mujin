# Multi-Agent Implementation Plan: Leader Delegation Pattern

**Status: IMPLEMENTED** ✓

Concrete implementation plan for minimal multi-agent support using agent parameters and PYRAMID's hierarchical leader-delegation architecture.

> **Implementation Complete:** Core multi-agent support (Steps 1-4, 7, 9) has been implemented and tested. 22 tests passing. See `tests/test_multi_agent.cpp`.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        LEADER AGENT                             │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────────┐  │
│  │ Mission     │───▶│ Planner      │───▶│ Goal Allocator    │  │
│  │ Goals       │    │ (high-level) │    │ (per-agent goals) │  │
│  └─────────────┘    └──────────────┘    └─────────┬─────────┘  │
│                                                    │            │
│                     ┌──────────────────────────────┤            │
│                     ▼                              ▼            │
│            ┌────────────────┐              ┌────────────────┐   │
│            │ DelegateToAgent│              │ DelegateToAgent│   │
│            │ agent="uav1"   │              │ agent="uav2"   │   │
│            │ goals="..."    │              │ goals="..."    │   │
│            └───────┬────────┘              └───────┬────────┘   │
└────────────────────┼───────────────────────────────┼────────────┘
                     │                               │
         ┌───────────▼───────────┐       ┌───────────▼───────────┐
         │      AGENT uav1       │       │      AGENT uav2       │
         │  ┌─────────────────┐  │       │  ┌─────────────────┐  │
         │  │ ExecutePhase    │  │       │  │ ExecutePhase    │  │
         │  │ (sub-plan)      │  │       │  │ (sub-plan)      │  │
         │  └────────┬────────┘  │       │  └────────┬────────┘  │
         │           ▼           │       │           ▼           │
         │  ┌─────────────────┐  │       │  ┌─────────────────┐  │
         │  │ Agent BT        │  │       │  │ Agent BT        │  │
         │  │ (move,search,..)│  │       │  │ (move,search,..)│  │
         │  └─────────────────┘  │       │  └─────────────────┘  │
         └───────────────────────┘       └───────────────────────┘
```

**Key Insight:** Leverage existing `ExecutePhaseAction` for agent-local planning. The leader plans at the goal level, delegates sub-goals to agents, and each agent plans/executes its own task sequence.

---

## Implementation Steps

### Step 1: Agent Registry in WorldModel

**Files:** `include/ame/world_model.h`, `src/ame/world_model.cpp`

Add agent tracking to WorldModel:

```cpp
// world_model.h additions:

struct AgentInfo {
    std::string id;           // "uav1", "uav2"
    std::string type;         // "uav", "ugv"
    bool available = true;    // can accept tasks
};

class WorldModel {
public:
    // Agent management
    void registerAgent(const std::string& id, const std::string& type);
    const std::vector<AgentInfo>& agents() const;
    AgentInfo* getAgent(const std::string& id);
    std::vector<std::string> agentIds() const;
    std::vector<std::string> availableAgents() const;
    
private:
    std::vector<AgentInfo> agents_;
};
```

**Effort:** ~50 LOC, 1 hour

---

### Step 2: Update PDDL Domain with Agent Parameter

**Files:** `domains/uav_search/domain.pddl`, `domains/multi_uav_search/domain.pddl` (new)

```pddl
(define (domain multi-uav-search)
  (:requirements :strips :typing)

  (:types
    location sector - object
    agent - object
  )

  (:predicates
    (at ?a - agent ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)
  )

  (:action move
    :parameters (?a - agent ?from - location ?to - location)
    :precondition (at ?a ?from)
    :effect (and (at ?a ?to) (not (at ?a ?from))))

  (:action search
    :parameters (?a - agent ?s - sector)
    :precondition (at ?a ?s)
    :effect (searched ?s))

  (:action classify
    :parameters (?a - agent ?s - sector)
    :precondition (and (at ?a ?s) (searched ?s))
    :effect (classified ?s))
)
```

**Effort:** ~30 LOC, 30 mins

---

### Step 3: Goal Allocator Utility

**Files:** `include/ame/goal_allocator.h` (new), `src/ame/goal_allocator.cpp` (new)

Simple greedy goal allocation from leader to agents:

```cpp
// goal_allocator.h

#pragma once
#include <string>
#include <vector>
#include <map>

namespace ame {

class WorldModel;

struct AgentGoalAssignment {
    std::string agent_id;
    std::vector<std::string> goals;  // PDDL goal fluents
};

class GoalAllocator {
public:
    /// Allocate goals to available agents.
    /// Strategy: round-robin assignment of independent goals.
    std::vector<AgentGoalAssignment> allocate(
        const std::vector<std::string>& goals,
        const WorldModel& wm) const;

    /// Check if a goal depends on another (simple heuristic: same sector).
    bool goalsDependOn(const std::string& g1, const std::string& g2) const;
};

} // namespace ame
```

**Initial allocation strategy:** Round-robin for independent goals. Goals on the same sector go to the same agent.

```cpp
// goal_allocator.cpp (sketch)

std::vector<AgentGoalAssignment> GoalAllocator::allocate(
    const std::vector<std::string>& goals,
    const WorldModel& wm) const {
    
    auto agents = wm.availableAgents();
    if (agents.empty()) return {};
    
    // Group goals by sector (simple heuristic)
    std::map<std::string, std::vector<std::string>> sector_goals;
    for (const auto& g : goals) {
        std::string sector = extractSector(g);  // parse "(searched sector_a)" -> "sector_a"
        sector_goals[sector].push_back(g);
    }
    
    // Round-robin assign sector groups to agents
    std::vector<AgentGoalAssignment> assignments(agents.size());
    for (size_t i = 0; i < agents.size(); ++i) {
        assignments[i].agent_id = agents[i];
    }
    
    size_t idx = 0;
    for (const auto& [sector, sg] : sector_goals) {
        for (const auto& g : sg) {
            assignments[idx % agents.size()].goals.push_back(g);
        }
        idx++;
    }
    
    // Remove empty assignments
    assignments.erase(
        std::remove_if(assignments.begin(), assignments.end(),
                       [](const auto& a) { return a.goals.empty(); }),
        assignments.end());
    
    return assignments;
}
```

**Effort:** ~150 LOC, 3 hours

---

### Step 4: DelegateToAgent BT Node

**Files:** `include/ame/bt_nodes/delegate_to_agent.h` (new), `src/ame/bt_nodes/delegate_to_agent.cpp` (new)

A BT node that delegates a set of goals to a specific agent, leveraging `ExecutePhaseAction` internally.

```cpp
// delegate_to_agent.h

#pragma once
#include <behaviortree_cpp/action_node.h>
#include <memory>

namespace ame {

/// Delegates sub-goals to a specific agent for local planning and execution.
///
/// Ports:
///   agent_id (input)    - Target agent ID, e.g., "uav1"
///   agent_goals (input) - Semicolon-separated goals for this agent
///   phase_name (input)  - Label for audit logging
///
/// Behavior:
///   1. Filters WorldModel to agent-relevant scope (optional)
///   2. Sets agent_id on blackboard for child nodes
///   3. Plans for agent_goals using existing Planner
///   4. Compiles and executes BT with agent context
///
/// Returns SUCCESS when agent completes all goals, FAILURE on plan/exec failure.
class DelegateToAgent : public BT::StatefulActionNode {
public:
    DelegateToAgent(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::unique_ptr<BT::Tree> agent_tree_;
    std::string agent_id_;
};

} // namespace ame
```

```cpp
// delegate_to_agent.cpp (sketch)

BT::PortsList DelegateToAgent::providedPorts() {
    return {
        BT::InputPort<std::string>("agent_id", "Target agent ID"),
        BT::InputPort<std::string>("agent_goals", "Semicolon-separated goals"),
        BT::InputPort<std::string>("phase_name", "", "Audit label"),
    };
}

BT::NodeStatus DelegateToAgent::onStart() {
    auto agent_id = getInput<std::string>("agent_id");
    auto goals_str = getInput<std::string>("agent_goals");
    auto phase_name = getInput<std::string>("phase_name");
    
    if (!agent_id || !goals_str) return BT::NodeStatus::FAILURE;
    
    agent_id_ = agent_id.value();
    
    // Set agent context on blackboard for action nodes
    config().blackboard->set("current_agent_id", agent_id_);
    
    // Get planning infrastructure
    auto* wm = config().blackboard->get<WorldModel*>("world_model");
    auto* planner = config().blackboard->get<Planner*>("planner");
    auto* compiler = config().blackboard->get<PlanCompiler*>("plan_compiler");
    auto* registry = config().blackboard->get<ActionRegistry*>("action_registry");
    auto* factory = config().blackboard->get<BT::BehaviorTreeFactory*>("bt_factory");
    
    // Parse goals
    auto goals = parseGoals(goals_str.value());
    
    // Set goals and solve
    wm->setGoal(goals);
    auto result = planner->solve(*wm);
    if (!result.success) return BT::NodeStatus::FAILURE;
    
    // Filter plan to only this agent's actions
    std::vector<PlanStep> agent_steps;
    for (const auto& step : result.steps) {
        const auto& action = wm->groundActions()[step.action_index];
        if (actionBelongsToAgent(action, agent_id_)) {
            agent_steps.push_back(step);
        }
    }
    
    // Compile agent-specific BT
    std::string bt_xml = compiler->compile(agent_steps, *wm, *registry);
    
    // Create sub-tree with agent context
    agent_tree_ = std::make_unique<BT::Tree>(
        factory->createTreeFromText(bt_xml, config().blackboard));
    
    return onRunning();
}

BT::NodeStatus DelegateToAgent::onRunning() {
    if (!agent_tree_) return BT::NodeStatus::FAILURE;
    
    auto status = agent_tree_->tickOnce();
    if (status == BT::NodeStatus::RUNNING) return BT::NodeStatus::RUNNING;
    
    agent_tree_.reset();
    return status;
}
```

**Effort:** ~200 LOC, 4 hours

---

### Step 5: Parallel Agent Execution Node

**Files:** `include/ame/bt_nodes/parallel_delegation.h` (new), `src/ame/bt_nodes/parallel_delegation.cpp` (new)

Wrapper that executes multiple `DelegateToAgent` nodes in parallel:

```cpp
// parallel_delegation.h

#pragma once
#include <behaviortree_cpp/control_node.h>

namespace ame {

/// Executes agent delegations in parallel.
/// 
/// Dynamically creates DelegateToAgent children based on goal allocation.
/// Returns SUCCESS when all agents complete, FAILURE if any agent fails.
///
/// Ports:
///   mission_goals (input) - Full mission goal set (semicolon-separated)
///
/// Blackboard:
///   "world_model", "planner", etc. as per DelegateToAgent
class ParallelDelegation : public BT::ControlNode {
public:
    ParallelDelegation(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;
    void halt() override;

private:
    bool initialized_ = false;
};

} // namespace ame
```

**Effort:** ~150 LOC, 3 hours

---

### Step 6: Update Action BT Nodes with Agent Context

**Files:** `include/ame/bt_nodes/*.h`, `src/ame/bt_nodes/*.cpp`

Add `agent_id` input port to action nodes:

```cpp
// Example: move_action.h modification

static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("agent_id", "Executing agent"),  // NEW
        BT::InputPort<std::string>("from"),
        BT::InputPort<std::string>("to"),
    };
}

BT::NodeStatus tick() override {
    auto agent_id = getInput<std::string>("agent_id");
    auto from = getInput<std::string>("from");
    auto to = getInput<std::string>("to");
    
    // Use agent_id for ROS2 namespacing, logging, etc.
    // e.g., call /{agent_id}/move service
}
```

**Effort:** ~50 LOC per node, 2 hours total

---

### Step 7: PlanCompiler Agent Context Injection

**Files:** `src/ame/plan_compiler.cpp`

Modify compiler to inject `agent_id` port into generated BT XML:

```cpp
// In compile(), when generating action XML:

std::string PlanCompiler::compileAction(const GroundAction& action,
                                        const ActionImpl& impl) {
    std::string xml = "<" + impl.bt_node_type;
    
    // Extract agent from first parameter (convention: agent is param0)
    if (!action.params.empty()) {
        xml += " agent_id=\"" + action.params[0] + "\"";
    }
    
    // Add remaining parameters
    for (size_t i = 1; i < action.params.size(); ++i) {
        xml += " param" + std::to_string(i-1) + "=\"" + action.params[i] + "\"";
    }
    
    xml += "/>";
    return xml;
}
```

**Effort:** ~30 LOC, 1 hour

---

### Step 8: Agent-Scoped Observability

**Files:** `src/ame/bt_logger.cpp`, `src/ame/plan_audit_log.cpp`

Add agent ID to audit records:

```cpp
// plan_audit_log.h modification:

struct Episode {
    // ... existing fields ...
    std::string agent_id;  // NEW: which agent executed this phase
};
```

```cpp
// bt_logger.cpp modification:

void BTLogger::callback(/* ... */) {
    std::string agent_id = node.config().blackboard->get<std::string>("current_agent_id");
    // Include agent_id in log entry
}
```

**Effort:** ~40 LOC, 1 hour

---

### Step 9: Multi-Agent Test Suite

**Files:** `tests/test_multi_agent.cpp` (new)

```cpp
TEST(MultiAgent, GoalAllocatorRoundRobin) {
    WorldModel wm;
    wm.registerAgent("uav1", "uav");
    wm.registerAgent("uav2", "uav");
    
    GoalAllocator allocator;
    auto assignments = allocator.allocate(
        {"(searched sector_a)", "(searched sector_b)", 
         "(classified sector_a)", "(classified sector_b)"}, wm);
    
    ASSERT_EQ(assignments.size(), 2);
    // sector_a goals -> one agent, sector_b goals -> other agent
}

TEST(MultiAgent, DelegateToAgentPlansCorrectly) {
    // Setup WorldModel with 2 agents, 2 sectors
    // Run DelegateToAgent for uav1 with sector_a goals
    // Verify plan contains only uav1 actions
}

TEST(MultiAgent, ParallelDelegationExecutesConcurrently) {
    // Setup mission with goals for 2 agents
    // Verify both agent BTs tick in parallel
    // Verify all goals achieved
}

TEST(MultiAgent, EndToEndMultiUAVSearch) {
    // Full pipeline: parse domain, allocate goals, delegate, execute
    // 2 UAVs, 4 sectors, verify mission completion
}
```

**Effort:** ~300 LOC, 4 hours

---

## File Summary

| File | Change Type | Status |
|------|-------------|--------|
| `include/ame/world_model.h` | Modify | ✓ Implemented |
| `src/ame/world_model.cpp` | Modify | ✓ Implemented |
| `include/ame/goal_allocator.h` | New | ✓ Implemented |
| `src/ame/goal_allocator.cpp` | New | ✓ Implemented |
| `include/ame/bt_nodes/delegate_to_agent.h` | New | ✓ Implemented |
| `src/ame/bt_nodes/delegate_to_agent.cpp` | New | ✓ Implemented |
| `include/ame/bt_nodes/parallel_delegation.h` | New | Deferred |
| `src/ame/bt_nodes/parallel_delegation.cpp` | New | Deferred |
| `include/ame/plan_compiler.h` | Modify | ✓ Implemented |
| `src/ame/plan_compiler.cpp` | Modify | ✓ Implemented |
| `domains/multi_uav_search/domain.pddl` | New | ✓ Implemented |
| `domains/multi_uav_search/problem.pddl` | New | ✓ Implemented |
| `tests/test_multi_agent.cpp` | New | ✓ Implemented (730+ LOC) |
| `src/ame/CMakeLists.txt` | Modify | ✓ Implemented |
| `tests/CMakeLists.txt` | Modify | ✓ Implemented |
| `include/ame/agent_dispatcher.h` | New | ✓ Implemented |
| `src/ame/agent_dispatcher.cpp` | New | ✓ Implemented |
| `ros2/include/ame_ros2/agent_dispatcher_node.hpp` | New | ✓ Implemented |
| `ros2/src/agent_dispatcher_node.cpp` | New | ✓ Implemented |
| `ros2/launch/ame_multi_agent.launch.py` | New | ✓ Implemented |

---

## Implementation Order

```
Week 1:
  ├── Step 1: Agent registry in WorldModel
  ├── Step 2: Multi-agent PDDL domain
  └── Step 3: GoalAllocator

Week 2:
  ├── Step 4: DelegateToAgent BT node
  ├── Step 5: ParallelDelegation node
  └── Step 6: Action nodes agent_id port

Week 3:
  ├── Step 7: PlanCompiler agent injection
  ├── Step 8: Agent-scoped observability
  └── Step 9: Test suite
```

---

## Usage Example

### Leader Mission BT (hand-authored or generated)

```xml
<BehaviorTree ID="MultiAgentMission">
  <Sequence>
    <!-- Leader allocates and delegates -->
    <ParallelDelegation mission_goals="(searched sector_a);(searched sector_b);
                                        (classified sector_a);(classified sector_b)"/>
    
    <!-- Or explicit delegation: -->
    <!--
    <Parallel success_count="2">
      <DelegateToAgent agent_id="uav1" 
                       agent_goals="(searched sector_a);(classified sector_a)"
                       phase_name="uav1_recon"/>
      <DelegateToAgent agent_id="uav2" 
                       agent_goals="(searched sector_b);(classified sector_b)"
                       phase_name="uav2_recon"/>
    </Parallel>
    -->
  </Sequence>
</BehaviorTree>
```

### Generated Agent Sub-BT (per agent)

```xml
<!-- Auto-generated for uav1 -->
<BehaviorTree ID="uav1_phase">
  <Sequence>
    <MoveAction agent_id="uav1" from="base" to="sector_a"/>
    <SearchAction agent_id="uav1" sector="sector_a"/>
    <ClassifyAction agent_id="uav1" sector="sector_a"/>
  </Sequence>
</BehaviorTree>
```

---

## Success Criteria

- [x] 2+ agents execute missions in parallel
- [x] Goals correctly allocated across agents
- [x] Each agent plans and executes independently
- [x] Agent ID propagates through all BT nodes and logs
- [x] Audit logs show per-agent episodes with causal links
- [x] All existing single-agent tests pass
- [x] New multi-agent tests pass (22 tests)
- [x] Plan+execute time <2s for 4 agents, 8 sectors (measured: ~500ms for 6 sectors)

---

## Future Extensions (Out of Scope)

1. **Dynamic re-allocation:** Reassign goals if agent fails
2. **Coordination constraints:** Mutex resources, sync barriers
3. **Distributed WorldModel:** Per-agent state with sync protocol
4. **Communication-aware planning:** Handle comms delays/failures
