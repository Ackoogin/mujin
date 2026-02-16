# PDDL Planning + Behaviour Trees: Integration Architecture

## 1. The Core Problem

You need three things working together:

1. **A consistent world model** — a single source of truth about the state of the world, usable by both the PDDL planner and the behaviour tree executor.
2. **A PDDL planner** (LAPKT) that can plan against that world model.
3. **A behaviour tree executor** (BehaviorTree.CPP) that can execute plans, where the tree structure itself is **dynamically constructed** from the plan — including composition of reusable sub-trees.

The key insight is that these aren't separate concerns bolted together — the world model is the **bridge** between deliberation (planning) and reaction (BT execution), and the plan-to-BT compilation step is where the architecture's intelligence lives.

---

## 2. Prior Art: What PlanSys2 Gets Right (and Wrong)

The ROS2 PlanSys2 framework (Martín et al., AAMAS 2021) is the most mature implementation of this exact pattern. Their architecture:

- **Domain Expert** — holds the PDDL domain
- **Problem Expert** — holds the current knowledge base (instances, predicates, goals)
- **Planner** — generates plans (POPF/Fast Downward as plugins)
- **Executor** — converts plans to BTs and executes them

Their plan→BT algorithm:

1. Build a **causal graph** from the plan (pair action effects with subsequent action preconditions)
2. Identify **execution flows** — independent causal chains that can run in parallel
3. Generate a BT where flows are children of a **Parallel** node, and within each flow, causally-dependent actions are in **Sequence** nodes
4. Each action node is wrapped: `Sequence → [CheckPreconditions, ExecuteAction, ApplyEffects]`
5. Actions shared across flows use a **Singleton** pattern — the same BT node instance is referenced from multiple places

**What PlanSys2 gets right:**

- Causal analysis enabling automatic parallelisation
- The action-unit pattern (check→execute→effects)
- Singleton nodes preventing duplicate execution

**What's less suitable for your context:**

- Tightly coupled to ROS2 (services, actions, DDS)
- World model is distributed across ROS2 nodes, not a single in-process structure
- No support for hand-authored sub-trees being composed into planned trees
- Limited to STRIPS-level temporal model (later work by González-Santamarta 2024 adds PDDL 2.1 temporal support via STN conversion)

---

## 3. Proposed Architecture

### 3.1 The World Model

The world model must serve two masters simultaneously:

**For LAPKT:** A set of boolean fluents (propositions) and typed objects that can be mapped to a `STRIPS_Problem`. LAPKT's agnostic interface works with fluent indices and action indices.

**For BehaviorTree.CPP:** A key/value store accessible from BT nodes. BT.CPP v4 provides the `Blackboard` for this, with scoped and global access patterns.

The solution is a **single authoritative WorldModel class** that presents two projections:

```
┌─────────────────────────────────────────────────────┐
│                    WorldModel                        │
│                                                      │
│  Authoritative state: typed objects + properties     │
│                                                      │
│  ┌──────────────┐          ┌───────────────────┐     │
│  │ PDDL View    │          │ Blackboard View   │     │
│  │              │          │                   │     │
│  │ fluent_id ←──┼──────────┼── "at_robot_A"    │     │
│  │ action_id ←──┼──────────┼── BT ActionNode   │     │
│  │              │          │                   │     │
│  │ Provides:    │          │ Provides:         │     │
│  │ STRIPS_Problem│         │ BB::Ptr           │     │
│  │ State vector │          │ get/set by key    │     │
│  └──────────────┘          └───────────────────┘     │
└─────────────────────────────────────────────────────┘
```

```cpp
class WorldModel {
public:
    // --- Object/Type management ---
    ObjectId addObject(const std::string& name, const std::string& type);

    // --- Predicate management ---
    PredicateId registerPredicate(const std::string& name,
                                  const std::vector<std::string>& param_types);

    // --- State manipulation (authoritative) ---
    void setFact(PredicateId pred, const std::vector<ObjectId>& args, bool value);
    bool getFact(PredicateId pred, const std::vector<ObjectId>& args) const;

    // --- PDDL projection ---
    // Builds/updates an aptk::STRIPS_Problem from current state.
    // Fluent indices are stable across calls (only grows).
    void projectToSTRIPS(aptk::STRIPS_Problem& prob) const;
    aptk::State* currentStateAsSTRIPS() const;

    // --- Blackboard projection ---
    // Syncs world model → blackboard (one-way push).
    // Each grounded predicate becomes a bool entry: "at(robot1,zoneA)" → true
    void syncToBlackboard(BT::Blackboard::Ptr bb) const;

    // --- Blackboard write-back ---
    // After BT action execution, apply effects back to world model.
    void applyEffects(const ActionEffects& effects);

    // --- Change tracking ---
    uint64_t version() const;  // monotonic, increments on any state change

private:
    // Ground facts stored as a bitset or dense vector, indexed by fluent_id
    std::vector<bool> facts_;

    // Bidirectional mapping: fluent_id ↔ "predicate(arg1,arg2,...)"
    BiMap<unsigned, std::string> fluent_map_;

    // Type hierarchy and object registry
    TypeSystem types_;
};
```

**Key design decisions:**

- The WorldModel owns the truth. LAPKT gets a **snapshot** (projected STRIPS_Problem + State). The Blackboard gets a **sync** (pushed values). Neither LAPKT nor the Blackboard can independently mutate the world model.
- Fluent indices are **stable** — once a grounded predicate is assigned an index, it never changes. This means you can replan without rebuilding the entire STRIPS_Problem, just update the initial state vector.
- The blackboard sync uses a naming convention: predicate instances become string keys like `"at(robot1,zoneA)"` with bool values. BT condition nodes can query these directly.
- For richer data (numeric values, coordinates, sensor readings) that don't map to PDDL predicates, the blackboard can hold additional entries that the planner never sees but BT nodes use for execution.

### 3.2 Action Registry: Bridging PDDL Actions and BT Nodes

Each PDDL action schema needs a corresponding BT implementation. This is where the PYRAMID service model maps naturally — each PDDL action corresponds to a service invocation or a sub-tree of service invocations.

```cpp
class ActionRegistry {
public:
    // Register a simple BT node type for a PDDL action
    void registerAction(const std::string& pddl_action_name,
                        const std::string& bt_node_type);

    // Register a sub-tree template for a PDDL action
    // The sub-tree XML can contain {param0}, {param1}... placeholders
    // that get substituted with the action's grounded parameters.
    void registerActionSubTree(const std::string& pddl_action_name,
                               const std::string& subtree_xml_template);

    // Register a pre-authored sub-tree file
    void registerActionSubTreeFile(const std::string& pddl_action_name,
                                    const std::string& xml_path);

    // Look up how to instantiate a grounded action
    ActionImpl resolve(const std::string& action_name,
                       const std::vector<std::string>& params) const;
};
```

This allows three levels of action implementation:

1. **Simple action → single BT node**: `move(robot, from, to)` → `MoveAction` node with port bindings
2. **Complex action → sub-tree template**: `transport(robot, package, from, to)` → a Sequence of pickup, move, putdown with parameter substitution
3. **Pre-authored sub-tree**: Hand-crafted BT XML for complex behaviours (e.g., `search_area` with reactive obstacle avoidance)

### 3.3 Plan-to-BT Compiler

This is the most architecturally significant component. It takes a LAPKT plan (ordered list of grounded actions) and produces a BT that can be executed by BehaviorTree.CPP.

```
LAPKT Plan                    Causal Graph                 BT (XML or programmatic)
┌──────────┐                 ┌───────────┐                ┌──────────────────┐
│ 0: moveAB │────effects────▶│ 0 ──▶ 2   │    compile    │ Parallel         │
│ 1: moveCd │────effects────▶│ 1 ──▶ 3   │──────────────▶│ ├─ Sequence (f1) │
│ 2: pickup │                │ 2 ──▶ 4   │                │ │  ├─ moveAB     │
│ 3: pickup │                │ 3 ──▶ 4   │                │ │  ├─ pickup     │
│ 4: assemb │                └───────────┘                │ │  └─ assemble   │
└──────────┘                                              │ └─ Sequence (f2) │
                                                          │    ├─ moveCD     │
                                                          │    ├─ pickup     │
                                                          │    └─ [join f1]  │
                                                          └──────────────────┘
```

```cpp
class PlanToBTCompiler {
public:
    PlanToBTCompiler(const WorldModel& wm,
                     const ActionRegistry& registry,
                     BT::BehaviorTreeFactory& factory);

    // Compile a LAPKT plan into a BT.
    // Returns XML string that can be loaded by factory.createTreeFromText()
    std::string compile(const Plan& plan);

    // Or compile and directly create the tree
    BT::Tree compileAndCreate(const Plan& plan,
                               BT::Blackboard::Ptr blackboard);

private:
    // Step 1: Build causal graph from plan
    CausalGraph buildCausalGraph(const Plan& plan);

    // Step 2: Identify parallel execution flows
    std::vector<ExecutionFlow> extractFlows(const CausalGraph& graph);

    // Step 3: For each action, generate the "action unit" BT fragment
    std::string generateActionUnit(const GroundedAction& action);

    // Step 4: Compose flows into final tree with parallel/sequence structure
    std::string composeTree(const std::vector<ExecutionFlow>& flows);
};
```

#### The Action Unit Pattern

Each planned action becomes a sub-tree that handles the full lifecycle:

```xml
<!-- Action unit for: move(robot1, zoneA, zoneB) -->
<ReactiveSequence name="move_robot1_zoneA_zoneB">
    <!-- Gate: check preconditions against world model -->
    <CheckWorldPredicate predicate="at(robot1,zoneA)" expected="true"/>
    <CheckWorldPredicate predicate="path_clear(zoneA,zoneB)" expected="true"/>

    <!-- Execute: the actual behaviour (could be a sub-tree) -->
    <SubTree ID="MoveSubTree"
             robot="{robot1}" from="{zoneA}" to="{zoneB}"/>

    <!-- Effects: update world model on success -->
    <SetWorldPredicate predicate="at(robot1,zoneA)" value="false"/>
    <SetWorldPredicate predicate="at(robot1,zoneB)" value="true"/>
</ReactiveSequence>
```

The `ReactiveSequence` is important: if a precondition becomes false during execution (e.g., path blocked), the action is interrupted. This gives you reactive behaviour within a deliberative framework.

#### Sub-Tree Composition

This is where it gets powerful. The ActionRegistry can map PDDL actions to arbitrarily complex sub-trees:

```xml
<!-- Pre-authored sub-tree for "search_area" action -->
<BehaviorTree ID="SearchAreaSubTree">
    <Fallback>
        <!-- Try systematic sweep first -->
        <Sequence>
            <GenerateSweepPattern area="{target_area}"
                                  pattern="{@sweep_waypoints}"/>
            <ForEachWaypoint waypoints="{@sweep_waypoints}">
                <Sequence>
                    <MoveToWaypoint waypoint="{current_wp}"/>
                    <ActivateSensors/>
                    <CheckDetection target="{search_target}"
                                    result="{@detected}"/>
                </Sequence>
            </ForEachWaypoint>
        </Sequence>
        <!-- Fallback: request human guidance -->
        <RequestOperatorGuidance task="search" area="{target_area}"/>
    </Fallback>
</BehaviorTree>
```

The planner sees `search_area(uav1, sector3, target_alpha)` as an atomic action with known preconditions and effects. The BT executor expands it into a rich reactive behaviour with fallbacks, loops, and sensor integration.

### 3.4 Replanning Integration

The architecture must handle plan failure and replanning. LAPKT's replanning example shows how to update initial state and goals without re-parsing. Combined with BT.CPP's reactive nodes, this gives us:

```
┌──────────────────────────────────────────────────────────────┐
│                     Execution Loop                            │
│                                                               │
│  ┌─────────┐    ┌──────────┐    ┌──────────┐    ┌─────────┐ │
│  │ World   │───▶│ LAPKT    │───▶│ Plan→BT  │───▶│ BT.CPP  │ │
│  │ Model   │◀───│ Planner  │    │ Compiler │    │ Executor│ │
│  │         │    └──────────┘    └──────────┘    └────┬────┘ │
│  │         │◀────────────────────────────────────────┘      │
│  │         │         (effects update world model)            │
│  └────┬────┘                                                 │
│       │                                                      │
│       ▼                                                      │
│  ┌─────────────────┐                                         │
│  │ Monitor Node    │  (top-level BT decorator)               │
│  │                 │                                         │
│  │ On FAILURE:     │                                         │
│  │  1. Halt tree   │                                         │
│  │  2. Snapshot WM │                                         │
│  │  3. Replan      │                                         │
│  │  4. Recompile   │                                         │
│  │  5. Swap tree   │                                         │
│  └─────────────────┘                                         │
└──────────────────────────────────────────────────────────────┘
```

```cpp
// Top-level execution with replanning
class MissionExecutor {
public:
    MissionExecutor(WorldModel& wm, ActionRegistry& registry);

    void setGoal(const std::vector<unsigned>& goal_fluents);

    // Main loop — call repeatedly (e.g., at 10Hz)
    TickResult tick() {
        if (!current_tree_) {
            replan();
        }

        auto status = current_tree_->tickOnce();

        if (status == BT::NodeStatus::FAILURE) {
            // Action failed — world model may have been updated
            // by the failing action's partial effects or by
            // external perception updates
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

        auto init_state = world_model_.currentStateAsSTRIPS();
        // Configure search (e.g., BFS(f) or SIW)
        // ...
        auto plan = solver.solve(prob, *init_state, goal_fluents_);

        if (plan.empty()) {
            // No plan found — escalate
            handlePlanFailure();
            return;
        }

        auto bb = BT::Blackboard::create();
        world_model_.syncToBlackboard(bb);
        current_tree_ = compiler_.compileAndCreate(plan, bb);
    }
};
```

### 3.5 Dynamic Sub-Tree Injection

Beyond compiling plans into trees, you may want to:

1. **Inject hand-authored sub-trees** at specific points in a planned tree (already covered by ActionRegistry)
2. **Dynamically modify a running tree** based on perception or operator commands
3. **Compose trees hierarchically** — a high-level mission plan that calls sub-plans

BehaviorTree.CPP v4 supports this through:

- `factory.registerBehaviorTreeFromText()` — register XML at runtime
- `SubTree` nodes that reference registered trees by ID
- The factory maintains a registry of tree definitions; you can add new ones at any time

This means the compiler can:

```cpp
// Register all pre-authored sub-trees
factory.registerBehaviorTreeFromFile("search_area.xml");
factory.registerBehaviorTreeFromFile("engage_target.xml");
factory.registerBehaviorTreeFromFile("return_to_base.xml");

// Compile a plan that references them
// The generated XML uses <SubTree ID="SearchAreaSubTree" .../>
// which resolves to the pre-registered definition.
auto xml = compiler.compile(plan);
factory.registerBehaviorTreeFromText(xml);  // registers "MissionPlan"
auto tree = factory.createTree("MissionPlan", blackboard);
```

For **hierarchical planning**, the outer plan might contain abstract actions like `execute_phase(phase1)` whose BT implementation triggers a sub-planner:

```cpp
class ExecutePhaseAction : public BT::StatefulActionNode {
    BT::NodeStatus onStart() override {
        auto phase = getInput<std::string>("phase");
        // Get sub-goals for this phase
        auto sub_goals = mission_model_.getPhaseGoals(phase);
        // Plan for sub-goals
        auto sub_plan = planner_.solve(world_model_, sub_goals);
        // Compile to sub-tree
        sub_tree_ = compiler_.compileAndCreate(sub_plan,
                        config().blackboard);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        return sub_tree_->tickOnce();
    }
};
```

---

## 4. Concrete BT Node Types Needed

### 4.1 World Model Interface Nodes

```cpp
// Condition: check a predicate in the world model
class CheckWorldPredicate : public BT::ConditionNode {
    static PortsList providedPorts() {
        return { InputPort<std::string>("predicate"),
                 InputPort<bool>("expected", true) };
    }
    NodeStatus tick() override {
        auto pred = getInput<std::string>("predicate").value();
        auto expected = getInput<bool>("expected").value();
        return (world_model_->queryFact(pred) == expected)
            ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

// Action: set a predicate (apply PDDL effect)
class SetWorldPredicate : public BT::SyncActionNode {
    static PortsList providedPorts() {
        return { InputPort<std::string>("predicate"),
                 InputPort<bool>("value") };
    }
    NodeStatus tick() override {
        auto pred = getInput<std::string>("predicate").value();
        auto value = getInput<bool>("value").value();
        world_model_->setFact(pred, value);
        return NodeStatus::SUCCESS;
    }
};
```

### 4.2 Planner Integration Nodes

```cpp
// Decorator: wraps a sub-tree with automatic replanning on failure
class ReplanOnFailure : public BT::DecoratorNode {
    NodeStatus tick() override {
        auto status = child()->executeTick();
        if (status == NodeStatus::FAILURE) {
            // Trigger replan via blackboard signal
            config().blackboard->set("@replan_requested", true);
            return NodeStatus::FAILURE;  // propagate up to mission executor
        }
        return status;
    }
};

// Action: invoke LAPKT planner inline and inject result as sub-tree
class PlanAndExecute : public BT::StatefulActionNode {
    static PortsList providedPorts() {
        return { InputPort<std::string>("goal_predicates") };
    }
    // ... spawns planner, compiles result, executes as sub-tree
};
```

### 4.3 PYRAMID Service Nodes

These would map to your PYRAMID service interfaces:

```cpp
// Generic PYRAMID service invocation node
class InvokeService : public BT::StatefulActionNode {
    static PortsList providedPorts() {
        return { InputPort<std::string>("service_name"),
                 InputPort<std::string>("operation"),
                 BidirectionalPort<ServiceMessage>("request"),
                 OutputPort<ServiceMessage>("response") };
    }
    // Maps to PYRAMID pub/sub or RPC depending on service definition
};
```

---

## 5. Worked Example: UAV Search-and-Classify Mission

### PDDL Domain (simplified)

```pddl
(:action search_sector
  :parameters (?uav - vehicle ?sector - area)
  :precondition (and (at ?uav base) (not (searched ?sector)))
  :effect (and (searched ?sector) (at ?uav ?sector) (not (at ?uav base))))

(:action classify_contact
  :parameters (?uav - vehicle ?contact - entity ?sector - area)
  :precondition (and (at ?uav ?sector) (detected ?contact ?sector)
                     (not (classified ?contact)))
  :effect (classified ?contact))

(:action return_to_base
  :parameters (?uav - vehicle ?sector - area)
  :precondition (at ?uav ?sector)
  :effect (and (at ?uav base) (not (at ?uav ?sector))))
```

### Generated Plan

```
0: search_sector(uav1, sectorA)
1: search_sector(uav2, sectorB)      [parallel with 0]
2: classify_contact(uav1, contact1, sectorA)  [after 0]
3: return_to_base(uav1, sectorA)     [after 2]
4: return_to_base(uav2, sectorB)     [after 1]
```

### Compiled BT (conceptual)

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MissionPlan">
    <Parallel success_count="2" failure_count="1">
      <!-- Flow 1: UAV1 -->
      <Sequence>
        <ReactiveSequence name="search_sector_uav1_sectorA">
          <CheckWorldPredicate predicate="at(uav1,base)" expected="true"/>
          <SubTree ID="SearchSectorBehaviour"
                   uav="uav1" sector="sectorA"/>
          <SetWorldPredicate predicate="searched(sectorA)" value="true"/>
          <SetWorldPredicate predicate="at(uav1,base)" value="false"/>
          <SetWorldPredicate predicate="at(uav1,sectorA)" value="true"/>
        </ReactiveSequence>

        <ReactiveSequence name="classify_uav1_contact1">
          <CheckWorldPredicate predicate="at(uav1,sectorA)"/>
          <CheckWorldPredicate predicate="detected(contact1,sectorA)"/>
          <SubTree ID="ClassifyContactBehaviour"
                   uav="uav1" contact="contact1"/>
          <SetWorldPredicate predicate="classified(contact1)" value="true"/>
        </ReactiveSequence>

        <ReactiveSequence name="rtb_uav1">
          <SubTree ID="ReturnToBaseBehaviour" uav="uav1" from="sectorA"/>
          <SetWorldPredicate predicate="at(uav1,sectorA)" value="false"/>
          <SetWorldPredicate predicate="at(uav1,base)" value="true"/>
        </ReactiveSequence>
      </Sequence>

      <!-- Flow 2: UAV2 (independent, runs in parallel) -->
      <Sequence>
        <ReactiveSequence name="search_sector_uav2_sectorB">
          <!-- ... similar structure ... -->
          <SubTree ID="SearchSectorBehaviour"
                   uav="uav2" sector="sectorB"/>
          <!-- ... effects ... -->
        </ReactiveSequence>

        <ReactiveSequence name="rtb_uav2">
          <SubTree ID="ReturnToBaseBehaviour" uav="uav2" from="sectorB"/>
          <!-- ... effects ... -->
        </ReactiveSequence>
      </Sequence>
    </Parallel>
  </BehaviorTree>

  <!-- Pre-authored reactive sub-tree for search -->
  <BehaviorTree ID="SearchSectorBehaviour">
    <Fallback>
      <Sequence>
        <NavigateToSector uav="{uav}" sector="{sector}"/>
        <ExecuteSweepPattern uav="{uav}" sector="{sector}"/>
      </Sequence>
      <Sequence>
        <ReportSearchFailure uav="{uav}" sector="{sector}"/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
```

---

## 6. Implementation Roadmap

### Phase 1: World Model + LAPKT Integration

- Implement `WorldModel` with fluent registry and state vector
- Implement `projectToSTRIPS()` using LAPKT's agnostic interface (`prob.add_fluent()`, `prob.add_action()`, etc.)
- Verify round-trip: construct problem → solve → extract plan → verify effects
- Use LAPKT's BFS(f) or SIW for initial solver (good balance of speed and coverage)

### Phase 2: Basic Plan-to-BT Compilation

- Implement causal graph builder (match action effects to subsequent preconditions)
- Implement flow extraction (topological sort, identify independent chains)
- Generate BT XML strings from flows
- Integrate with `BT::BehaviorTreeFactory::createTreeFromText()`
- Test with simple domains (logistics, blocks world)

### Phase 3: Action Registry + Sub-Tree Composition

- Build the ActionRegistry with template parameter substitution
- Create the standard BT node types (CheckWorldPredicate, SetWorldPredicate, etc.)
- Author initial sub-tree library for your domain
- Test composition: planned tree referencing pre-authored sub-trees

### Phase 4: Replanning Loop

- Implement MissionExecutor with tick-based execution
- Add world model update from BT action effects
- Add perception-driven world model updates (external to BT)
- Test replan-on-failure with injected failures

### Phase 5: PYRAMID Integration

- Map PYRAMID service definitions to PDDL action schemas
- Generate BT action nodes from PYRAMID service contracts
- Integration with PYRAMID pub/sub and RPC mechanisms

---

## 7. Key Design Considerations

### Thread Safety

The world model will be accessed from the BT tick thread and potentially from perception/sensor update threads. Use a versioned snapshot approach: the BT reads from a consistent snapshot, perception writes to a pending buffer, and snapshots are swapped at defined sync points (e.g., between ticks).

### LAPKT Solver Selection

For real-time replanning in a UAV context:

- **SIW (Serialized Iterated Width)** — very fast for problems with low width, good for logistics-style domains
- **BFS(f)** — best general-purpose performance, novelty-based
- **Deadline Aware Search** — if you have a hard time budget for planning

### Blackboard Scoping

Use BT.CPP v4's global blackboard (`@` prefix) for world model state, and local blackboards for action-specific data. This prevents name collisions when the same sub-tree template is instantiated multiple times with different parameters.

### Plan Validation

Before executing a compiled BT, validate that all referenced sub-trees are registered in the factory and all ports are satisfiable. BT.CPP's factory can introspect registered node models for this.

### ROS2 Node Decomposition

The core library is ROS-agnostic (pure C++). For ROS2 deployment, thin adapter nodes wrap the C++ API. The architecture supports both single-node (in-process) and multi-node (distributed) configurations:

- **WorldModel Node** — owns authoritative state, exposes `get_fact`/`set_fact` as ROS2 services, publishes state changes on a topic. Perception nodes are independent clients that call `set_fact`.
- **Planner Node** — stateless service or action server. Receives a STRIPS_Problem snapshot + goal, returns a plan. Easily swappable for a different planner.
- **Executor Node** — owns the BT runtime (or alternative execution framework). Ticks the tree, calls WorldModel services for precondition checks and effect application.

The WorldModel service boundary is the natural split point. For development and testing, all components run in a single process with direct C++ API calls. For production, the same `WorldModel` class is wrapped in a ROS2 service node, and executor/planner become separate nodes communicating via services and topics.

This decomposition also supports the design goal that execution layers (BT or otherwise) are replaceable — the executor node can be swapped without touching the world model or planner nodes.

### Determinism and Auditability

For defence/safety contexts: log every world model state change with timestamp and source (which BT node or external update). The world model version counter enables replay and post-hoc analysis. Consider making the causal graph and compiled BT XML part of the mission audit trail.
