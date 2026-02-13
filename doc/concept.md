\# PDDL Planning + Behaviour Trees: Integration Architecture



\## 1. The Core Problem



You need three things working together:



1\. \*\*A consistent world model\*\* — a single source of truth about the state of the world, usable by both the PDDL planner and the behaviour tree executor.

2\. \*\*A PDDL planner\*\* (LAPKT) that can plan against that world model.

3\. \*\*A behaviour tree executor\*\* (BehaviorTree.CPP) that can execute plans, where the tree structure itself is \*\*dynamically constructed\*\* from the plan — including composition of reusable sub-trees.



The key insight is that these aren't separate concerns bolted together — the world model is the \*\*bridge\*\* between deliberation (planning) and reaction (BT execution), and the plan-to-BT compilation step is where the architecture's intelligence lives.



---



\## 2. Prior Art: What PlanSys2 Gets Right (and Wrong)



The ROS2 PlanSys2 framework (Martín et al., AAMAS 2021) is the most mature implementation of this exact pattern. Their architecture:



\- \*\*Domain Expert\*\* — holds the PDDL domain

\- \*\*Problem Expert\*\* — holds the current knowledge base (instances, predicates, goals)

\- \*\*Planner\*\* — generates plans (POPF/Fast Downward as plugins)

\- \*\*Executor\*\* — converts plans to BTs and executes them



Their plan→BT algorithm:

1\. Build a \*\*causal graph\*\* from the plan (pair action effects with subsequent action preconditions)

2\. Identify \*\*execution flows\*\* — independent causal chains that can run in parallel

3\. Generate a BT where flows are children of a \*\*Parallel\*\* node, and within each flow, causally-dependent actions are in \*\*Sequence\*\* nodes

4\. Each action node is wrapped: `Sequence → \[CheckPreconditions, ExecuteAction, ApplyEffects]`

5\. Actions shared across flows use a \*\*Singleton\*\* pattern — the same BT node instance is referenced from multiple places



\*\*What PlanSys2 gets right:\*\*

\- Causal analysis enabling automatic parallelisation

\- The action-unit pattern (check→execute→effects)

\- Singleton nodes preventing duplicate execution



\*\*What's less suitable for your context:\*\*

\- Tightly coupled to ROS2 (services, actions, DDS)

\- World model is distributed across ROS2 nodes, not a single in-process structure

\- No support for hand-authored sub-trees being composed into planned trees

\- Limited to STRIPS-level temporal model (later work by González-Santamarta 2024 adds PDDL 2.1 temporal support via STN conversion)



---



\## 3. Proposed Architecture



\### 3.1 The World Model



The world model must serve two masters simultaneously:



\*\*For LAPKT:\*\* A set of boolean fluents (propositions) and typed objects that can be mapped to a `STRIPS\_Problem`. LAPKT's agnostic interface works with fluent indices and action indices.



\*\*For BehaviorTree.CPP:\*\* A key/value store accessible from BT nodes. BT.CPP v4 provides the `Blackboard` for this, with scoped and global access patterns.



The solution is a \*\*single authoritative WorldModel class\*\* that presents two projections:



```

┌─────────────────────────────────────────────────────┐

│                    WorldModel                        │

│                                                      │

│  Authoritative state: typed objects + properties     │

│                                                      │

│  ┌──────────────┐          ┌───────────────────┐     │

│  │ PDDL View    │          │ Blackboard View   │     │

│  │              │          │                   │     │

│  │ fluent\_id ←──┼──────────┼── "at\_robot\_A"    │     │

│  │ action\_id ←──┼──────────┼── BT ActionNode   │     │

│  │              │          │                   │     │

│  │ Provides:    │          │ Provides:         │     │

│  │ STRIPS\_Problem│         │ BB::Ptr           │     │

│  │ State vector │          │ get/set by key    │     │

│  └──────────────┘          └───────────────────┘     │

└─────────────────────────────────────────────────────┘

```



```cpp

class WorldModel {

public:

&nbsp;   // --- Object/Type management ---

&nbsp;   ObjectId addObject(const std::string\& name, const std::string\& type);

&nbsp;   

&nbsp;   // --- Predicate management ---

&nbsp;   PredicateId registerPredicate(const std::string\& name, 

&nbsp;                                 const std::vector<std::string>\& param\_types);

&nbsp;   

&nbsp;   // --- State manipulation (authoritative) ---

&nbsp;   void setFact(PredicateId pred, const std::vector<ObjectId>\& args, bool value);

&nbsp;   bool getFact(PredicateId pred, const std::vector<ObjectId>\& args) const;

&nbsp;   

&nbsp;   // --- PDDL projection ---

&nbsp;   // Builds/updates an aptk::STRIPS\_Problem from current state.

&nbsp;   // Fluent indices are stable across calls (only grows).

&nbsp;   void projectToSTRIPS(aptk::STRIPS\_Problem\& prob) const;

&nbsp;   aptk::State\* currentStateAsSTRIPS() const;

&nbsp;   

&nbsp;   // --- Blackboard projection ---

&nbsp;   // Syncs world model → blackboard (one-way push).

&nbsp;   // Each grounded predicate becomes a bool entry: "at(robot1,zoneA)" → true

&nbsp;   void syncToBlackboard(BT::Blackboard::Ptr bb) const;

&nbsp;   

&nbsp;   // --- Blackboard write-back ---

&nbsp;   // After BT action execution, apply effects back to world model.

&nbsp;   void applyEffects(const ActionEffects\& effects);

&nbsp;   

&nbsp;   // --- Change tracking ---

&nbsp;   uint64\_t version() const;  // monotonic, increments on any state change

&nbsp;   

private:

&nbsp;   // Ground facts stored as a bitset or dense vector, indexed by fluent\_id

&nbsp;   std::vector<bool> facts\_;

&nbsp;   

&nbsp;   // Bidirectional mapping: fluent\_id ↔ "predicate(arg1,arg2,...)"

&nbsp;   BiMap<unsigned, std::string> fluent\_map\_;

&nbsp;   

&nbsp;   // Type hierarchy and object registry

&nbsp;   TypeSystem types\_;

};

```



\*\*Key design decisions:\*\*



\- The WorldModel owns the truth. LAPKT gets a \*\*snapshot\*\* (projected STRIPS\_Problem + State). The Blackboard gets a \*\*sync\*\* (pushed values). Neither LAPKT nor the Blackboard can independently mutate the world model.

\- Fluent indices are \*\*stable\*\* — once a grounded predicate is assigned an index, it never changes. This means you can replan without rebuilding the entire STRIPS\_Problem, just update the initial state vector.

\- The blackboard sync uses a naming convention: predicate instances become string keys like `"at(robot1,zoneA)"` with bool values. BT condition nodes can query these directly.

\- For richer data (numeric values, coordinates, sensor readings) that don't map to PDDL predicates, the blackboard can hold additional entries that the planner never sees but BT nodes use for execution.



\### 3.2 Action Registry: Bridging PDDL Actions and BT Nodes



Each PDDL action schema needs a corresponding BT implementation. This is where the PYRAMID service model maps naturally — each PDDL action corresponds to a service invocation or a sub-tree of service invocations.



```cpp

class ActionRegistry {

public:

&nbsp;   // Register a simple BT node type for a PDDL action

&nbsp;   void registerAction(const std::string\& pddl\_action\_name,

&nbsp;                       const std::string\& bt\_node\_type);

&nbsp;   

&nbsp;   // Register a sub-tree template for a PDDL action

&nbsp;   // The sub-tree XML can contain {param0}, {param1}... placeholders

&nbsp;   // that get substituted with the action's grounded parameters.

&nbsp;   void registerActionSubTree(const std::string\& pddl\_action\_name,

&nbsp;                              const std::string\& subtree\_xml\_template);

&nbsp;   

&nbsp;   // Register a pre-authored sub-tree file

&nbsp;   void registerActionSubTreeFile(const std::string\& pddl\_action\_name,

&nbsp;                                   const std::string\& xml\_path);

&nbsp;   

&nbsp;   // Look up how to instantiate a grounded action

&nbsp;   ActionImpl resolve(const std::string\& action\_name,

&nbsp;                      const std::vector<std::string>\& params) const;

};

```



This allows three levels of action implementation:



1\. \*\*Simple action → single BT node\*\*: `move(robot, from, to)` → `MoveAction` node with port bindings

2\. \*\*Complex action → sub-tree template\*\*: `transport(robot, package, from, to)` → a Sequence of pickup, move, putdown with parameter substitution

3\. \*\*Pre-authored sub-tree\*\*: Hand-crafted BT XML for complex behaviours (e.g., `search\_area` with reactive obstacle avoidance)



\### 3.3 Plan-to-BT Compiler



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

&nbsp;                                                         │    ├─ moveCD     │

&nbsp;                                                         │    ├─ pickup     │

&nbsp;                                                         │    └─ \[join f1]  │

&nbsp;                                                         └──────────────────┘

```



```cpp

class PlanToBTCompiler {

public:

&nbsp;   PlanToBTCompiler(const WorldModel\& wm, 

&nbsp;                    const ActionRegistry\& registry,

&nbsp;                    BT::BehaviorTreeFactory\& factory);

&nbsp;   

&nbsp;   // Compile a LAPKT plan into a BT.

&nbsp;   // Returns XML string that can be loaded by factory.createTreeFromText()

&nbsp;   std::string compile(const Plan\& plan);

&nbsp;   

&nbsp;   // Or compile and directly create the tree

&nbsp;   BT::Tree compileAndCreate(const Plan\& plan, 

&nbsp;                              BT::Blackboard::Ptr blackboard);



private:

&nbsp;   // Step 1: Build causal graph from plan

&nbsp;   CausalGraph buildCausalGraph(const Plan\& plan);

&nbsp;   

&nbsp;   // Step 2: Identify parallel execution flows

&nbsp;   std::vector<ExecutionFlow> extractFlows(const CausalGraph\& graph);

&nbsp;   

&nbsp;   // Step 3: For each action, generate the "action unit" BT fragment

&nbsp;   std::string generateActionUnit(const GroundedAction\& action);

&nbsp;   

&nbsp;   // Step 4: Compose flows into final tree with parallel/sequence structure

&nbsp;   std::string composeTree(const std::vector<ExecutionFlow>\& flows);

};

```



\#### The Action Unit Pattern



Each planned action becomes a sub-tree that handles the full lifecycle:



```xml

<!-- Action unit for: move(robot1, zoneA, zoneB) -->

<ReactiveSequence name="move\_robot1\_zoneA\_zoneB">

&nbsp;   <!-- Gate: check preconditions against world model -->

&nbsp;   <CheckWorldPredicate predicate="at(robot1,zoneA)" expected="true"/>

&nbsp;   <CheckWorldPredicate predicate="path\_clear(zoneA,zoneB)" expected="true"/>

&nbsp;   

&nbsp;   <!-- Execute: the actual behaviour (could be a sub-tree) -->

&nbsp;   <SubTree ID="MoveSubTree" 

&nbsp;            robot="{robot1}" from="{zoneA}" to="{zoneB}"/>

&nbsp;   

&nbsp;   <!-- Effects: update world model on success -->

&nbsp;   <SetWorldPredicate predicate="at(robot1,zoneA)" value="false"/>

&nbsp;   <SetWorldPredicate predicate="at(robot1,zoneB)" value="true"/>

</ReactiveSequence>

```



The `ReactiveSequence` is important: if a precondition becomes false during execution (e.g., path blocked), the action is interrupted. This gives you reactive behaviour within a deliberative framework.



\#### Sub-Tree Composition



This is where it gets powerful. The ActionRegistry can map PDDL actions to arbitrarily complex sub-trees:



```xml

<!-- Pre-authored sub-tree for "search\_area" action -->

<BehaviorTree ID="SearchAreaSubTree">

&nbsp;   <Fallback>

&nbsp;       <!-- Try systematic sweep first -->

&nbsp;       <Sequence>

&nbsp;           <GenerateSweepPattern area="{target\_area}" 

&nbsp;                                 pattern="{@sweep\_waypoints}"/>

&nbsp;           <ForEachWaypoint waypoints="{@sweep\_waypoints}">

&nbsp;               <Sequence>

&nbsp;                   <MoveToWaypoint waypoint="{current\_wp}"/>

&nbsp;                   <ActivateSensors/>

&nbsp;                   <CheckDetection target="{search\_target}" 

&nbsp;                                   result="{@detected}"/>

&nbsp;               </Sequence>

&nbsp;           </ForEachWaypoint>

&nbsp;       </Sequence>

&nbsp;       <!-- Fallback: request human guidance -->

&nbsp;       <RequestOperatorGuidance task="search" area="{target\_area}"/>

&nbsp;   </Fallback>

</BehaviorTree>

```



The planner sees `search\_area(uav1, sector3, target\_alpha)` as an atomic action with known preconditions and effects. The BT executor expands it into a rich reactive behaviour with fallbacks, loops, and sensor integration.



\### 3.4 Replanning Integration



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

&nbsp;   MissionExecutor(WorldModel\& wm, ActionRegistry\& registry);

&nbsp;   

&nbsp;   void setGoal(const std::vector<unsigned>\& goal\_fluents);

&nbsp;   

&nbsp;   // Main loop — call repeatedly (e.g., at 10Hz)

&nbsp;   TickResult tick() {

&nbsp;       if (!current\_tree\_) {

&nbsp;           replan();

&nbsp;       }

&nbsp;       

&nbsp;       auto status = current\_tree\_->tickOnce();

&nbsp;       

&nbsp;       if (status == BT::NodeStatus::FAILURE) {

&nbsp;           // Action failed — world model may have been updated

&nbsp;           // by the failing action's partial effects or by

&nbsp;           // external perception updates

&nbsp;           replan();

&nbsp;           return TickResult::REPLANNING;

&nbsp;       }

&nbsp;       

&nbsp;       if (status == BT::NodeStatus::SUCCESS) {

&nbsp;           return TickResult::COMPLETE;

&nbsp;       }

&nbsp;       

&nbsp;       return TickResult::RUNNING;

&nbsp;   }

&nbsp;   

private:

&nbsp;   void replan() {

&nbsp;       aptk::STRIPS\_Problem prob;

&nbsp;       world\_model\_.projectToSTRIPS(prob);

&nbsp;       

&nbsp;       auto init\_state = world\_model\_.currentStateAsSTRIPS();

&nbsp;       // Configure search (e.g., BFS(f) or SIW)

&nbsp;       // ...

&nbsp;       auto plan = solver.solve(prob, \*init\_state, goal\_fluents\_);

&nbsp;       

&nbsp;       if (plan.empty()) {

&nbsp;           // No plan found — escalate

&nbsp;           handlePlanFailure();

&nbsp;           return;

&nbsp;       }

&nbsp;       

&nbsp;       auto bb = BT::Blackboard::create();

&nbsp;       world\_model\_.syncToBlackboard(bb);

&nbsp;       current\_tree\_ = compiler\_.compileAndCreate(plan, bb);

&nbsp;   }

};

```



\### 3.5 Dynamic Sub-Tree Injection



Beyond compiling plans into trees, you may want to:



1\. \*\*Inject hand-authored sub-trees\*\* at specific points in a planned tree (already covered by ActionRegistry)

2\. \*\*Dynamically modify a running tree\*\* based on perception or operator commands

3\. \*\*Compose trees hierarchically\*\* — a high-level mission plan that calls sub-plans



BehaviorTree.CPP v4 supports this through:



\- `factory.registerBehaviorTreeFromText()` — register XML at runtime

\- `SubTree` nodes that reference registered trees by ID

\- The factory maintains a registry of tree definitions; you can add new ones at any time



This means the compiler can:



```cpp

// Register all pre-authored sub-trees

factory.registerBehaviorTreeFromFile("search\_area.xml");

factory.registerBehaviorTreeFromFile("engage\_target.xml");

factory.registerBehaviorTreeFromFile("return\_to\_base.xml");



// Compile a plan that references them

// The generated XML uses <SubTree ID="SearchAreaSubTree" .../>

// which resolves to the pre-registered definition.

auto xml = compiler.compile(plan);

factory.registerBehaviorTreeFromText(xml);  // registers "MissionPlan"

auto tree = factory.createTree("MissionPlan", blackboard);

```



For \*\*hierarchical planning\*\*, the outer plan might contain abstract actions like `execute\_phase(phase1)` whose BT implementation triggers a sub-planner:



```cpp

class ExecutePhaseAction : public BT::StatefulActionNode {

&nbsp;   BT::NodeStatus onStart() override {

&nbsp;       auto phase = getInput<std::string>("phase");

&nbsp;       // Get sub-goals for this phase

&nbsp;       auto sub\_goals = mission\_model\_.getPhaseGoals(phase);

&nbsp;       // Plan for sub-goals

&nbsp;       auto sub\_plan = planner\_.solve(world\_model\_, sub\_goals);

&nbsp;       // Compile to sub-tree

&nbsp;       sub\_tree\_ = compiler\_.compileAndCreate(sub\_plan, 

&nbsp;                       config().blackboard);

&nbsp;       return BT::NodeStatus::RUNNING;

&nbsp;   }

&nbsp;   

&nbsp;   BT::NodeStatus onRunning() override {

&nbsp;       return sub\_tree\_->tickOnce();

&nbsp;   }

};

```



---



\## 4. Concrete BT Node Types Needed



\### 4.1 World Model Interface Nodes



```cpp

// Condition: check a predicate in the world model

class CheckWorldPredicate : public BT::ConditionNode {

&nbsp;   static PortsList providedPorts() {

&nbsp;       return { InputPort<std::string>("predicate"),

&nbsp;                InputPort<bool>("expected", true) };

&nbsp;   }

&nbsp;   NodeStatus tick() override {

&nbsp;       auto pred = getInput<std::string>("predicate").value();

&nbsp;       auto expected = getInput<bool>("expected").value();

&nbsp;       return (world\_model\_->queryFact(pred) == expected) 

&nbsp;           ? NodeStatus::SUCCESS : NodeStatus::FAILURE;

&nbsp;   }

};



// Action: set a predicate (apply PDDL effect)

class SetWorldPredicate : public BT::SyncActionNode {

&nbsp;   static PortsList providedPorts() {

&nbsp;       return { InputPort<std::string>("predicate"),

&nbsp;                InputPort<bool>("value") };

&nbsp;   }

&nbsp;   NodeStatus tick() override {

&nbsp;       auto pred = getInput<std::string>("predicate").value();

&nbsp;       auto value = getInput<bool>("value").value();

&nbsp;       world\_model\_->setFact(pred, value);

&nbsp;       return NodeStatus::SUCCESS;

&nbsp;   }

};

```



\### 4.2 Planner Integration Nodes



```cpp

// Decorator: wraps a sub-tree with automatic replanning on failure

class ReplanOnFailure : public BT::DecoratorNode {

&nbsp;   NodeStatus tick() override {

&nbsp;       auto status = child()->executeTick();

&nbsp;       if (status == NodeStatus::FAILURE) {

&nbsp;           // Trigger replan via blackboard signal

&nbsp;           config().blackboard->set("@replan\_requested", true);

&nbsp;           return NodeStatus::FAILURE;  // propagate up to mission executor

&nbsp;       }

&nbsp;       return status;

&nbsp;   }

};



// Action: invoke LAPKT planner inline and inject result as sub-tree

class PlanAndExecute : public BT::StatefulActionNode {

&nbsp;   static PortsList providedPorts() {

&nbsp;       return { InputPort<std::string>("goal\_predicates") };

&nbsp;   }

&nbsp;   // ... spawns planner, compiles result, executes as sub-tree

};

```



\### 4.3 PYRAMID Service Nodes



These would map to your PYRAMID service interfaces:



```cpp

// Generic PYRAMID service invocation node

class InvokeService : public BT::StatefulActionNode {

&nbsp;   static PortsList providedPorts() {

&nbsp;       return { InputPort<std::string>("service\_name"),

&nbsp;                InputPort<std::string>("operation"),

&nbsp;                BidirectionalPort<ServiceMessage>("request"),

&nbsp;                OutputPort<ServiceMessage>("response") };

&nbsp;   }

&nbsp;   // Maps to PYRAMID pub/sub or RPC depending on service definition

};

```



---



\## 5. Worked Example: UAV Search-and-Classify Mission



\### PDDL Domain (simplified)



```

(:action search\_sector

&nbsp; :parameters (?uav - vehicle ?sector - area)

&nbsp; :precondition (and (at ?uav base) (not (searched ?sector)))

&nbsp; :effect (and (searched ?sector) (at ?uav ?sector) (not (at ?uav base))))



(:action classify\_contact

&nbsp; :parameters (?uav - vehicle ?contact - entity ?sector - area)  

&nbsp; :precondition (and (at ?uav ?sector) (detected ?contact ?sector)

&nbsp;                    (not (classified ?contact)))

&nbsp; :effect (classified ?contact))



(:action return\_to\_base

&nbsp; :parameters (?uav - vehicle ?sector - area)

&nbsp; :precondition (at ?uav ?sector)

&nbsp; :effect (and (at ?uav base) (not (at ?uav ?sector))))

```



\### Generated Plan

```

0: search\_sector(uav1, sectorA)

1: search\_sector(uav2, sectorB)      \[parallel with 0]

2: classify\_contact(uav1, contact1, sectorA)  \[after 0]

3: return\_to\_base(uav1, sectorA)     \[after 2]

4: return\_to\_base(uav2, sectorB)     \[after 1]

```



\### Compiled BT (conceptual)



```xml

<root BTCPP\_format="4">

&nbsp; <BehaviorTree ID="MissionPlan">

&nbsp;   <Parallel success\_count="2" failure\_count="1">

&nbsp;     <!-- Flow 1: UAV1 -->

&nbsp;     <Sequence>

&nbsp;       <ReactiveSequence name="search\_sector\_uav1\_sectorA">

&nbsp;         <CheckWorldPredicate predicate="at(uav1,base)" expected="true"/>

&nbsp;         <SubTree ID="SearchSectorBehaviour" 

&nbsp;                  uav="uav1" sector="sectorA"/>

&nbsp;         <SetWorldPredicate predicate="searched(sectorA)" value="true"/>

&nbsp;         <SetWorldPredicate predicate="at(uav1,base)" value="false"/>

&nbsp;         <SetWorldPredicate predicate="at(uav1,sectorA)" value="true"/>

&nbsp;       </ReactiveSequence>

&nbsp;       

&nbsp;       <ReactiveSequence name="classify\_uav1\_contact1">

&nbsp;         <CheckWorldPredicate predicate="at(uav1,sectorA)"/>

&nbsp;         <CheckWorldPredicate predicate="detected(contact1,sectorA)"/>

&nbsp;         <SubTree ID="ClassifyContactBehaviour"

&nbsp;                  uav="uav1" contact="contact1"/>

&nbsp;         <SetWorldPredicate predicate="classified(contact1)" value="true"/>

&nbsp;       </ReactiveSequence>

&nbsp;       

&nbsp;       <ReactiveSequence name="rtb\_uav1">

&nbsp;         <SubTree ID="ReturnToBaseBehaviour" uav="uav1" from="sectorA"/>

&nbsp;         <SetWorldPredicate predicate="at(uav1,sectorA)" value="false"/>

&nbsp;         <SetWorldPredicate predicate="at(uav1,base)" value="true"/>

&nbsp;       </ReactiveSequence>

&nbsp;     </Sequence>

&nbsp;     

&nbsp;     <!-- Flow 2: UAV2 (independent, runs in parallel) -->

&nbsp;     <Sequence>

&nbsp;       <ReactiveSequence name="search\_sector\_uav2\_sectorB">

&nbsp;         <!-- ... similar structure ... -->

&nbsp;         <SubTree ID="SearchSectorBehaviour" 

&nbsp;                  uav="uav2" sector="sectorB"/>

&nbsp;         <!-- ... effects ... -->

&nbsp;       </ReactiveSequence>

&nbsp;       

&nbsp;       <ReactiveSequence name="rtb\_uav2">

&nbsp;         <SubTree ID="ReturnToBaseBehaviour" uav="uav2" from="sectorB"/>

&nbsp;         <!-- ... effects ... -->

&nbsp;       </ReactiveSequence>

&nbsp;     </Sequence>

&nbsp;   </Parallel>

&nbsp; </BehaviorTree>

&nbsp; 

&nbsp; <!-- Pre-authored reactive sub-tree for search -->

&nbsp; <BehaviorTree ID="SearchSectorBehaviour">

&nbsp;   <Fallback>

&nbsp;     <Sequence>

&nbsp;       <NavigateToSector uav="{uav}" sector="{sector}"/>

&nbsp;       <ExecuteSweepPattern uav="{uav}" sector="{sector}"/>

&nbsp;     </Sequence>

&nbsp;     <Sequence>

&nbsp;       <ReportSearchFailure uav="{uav}" sector="{sector}"/>

&nbsp;     </Sequence>

&nbsp;   </Fallback>

&nbsp; </BehaviorTree>

</root>

```



---



\## 6. Implementation Roadmap



\### Phase 1: World Model + LAPKT Integration

\- Implement `WorldModel` with fluent registry and state vector

\- Implement `projectToSTRIPS()` using LAPKT's agnostic interface (`prob.add\_fluent()`, `prob.add\_action()`, etc.)

\- Verify round-trip: construct problem → solve → extract plan → verify effects

\- Use LAPKT's BFS(f) or SIW for initial solver (good balance of speed and coverage)



\### Phase 2: Basic Plan-to-BT Compilation  

\- Implement causal graph builder (match action effects to subsequent preconditions)

\- Implement flow extraction (topological sort, identify independent chains)

\- Generate BT XML strings from flows

\- Integrate with `BT::BehaviorTreeFactory::createTreeFromText()`

\- Test with simple domains (logistics, blocks world)



\### Phase 3: Action Registry + Sub-Tree Composition

\- Build the ActionRegistry with template parameter substitution

\- Create the standard BT node types (CheckWorldPredicate, SetWorldPredicate, etc.)

\- Author initial sub-tree library for your domain

\- Test composition: planned tree referencing pre-authored sub-trees



\### Phase 4: Replanning Loop

\- Implement MissionExecutor with tick-based execution

\- Add world model update from BT action effects

\- Add perception-driven world model updates (external to BT)

\- Test replan-on-failure with injected failures



\### Phase 5: PYRAMID Integration

\- Map PYRAMID service definitions to PDDL action schemas

\- Generate BT action nodes from PYRAMID service contracts

\- Integration with PYRAMID pub/sub and RPC mechanisms



---



\## 7. Key Design Considerations



\### Thread Safety

The world model will be accessed from the BT tick thread and potentially from perception/sensor update threads. Use a versioned snapshot approach: the BT reads from a consistent snapshot, perception writes to a pending buffer, and snapshots are swapped at defined sync points (e.g., between ticks).



\### LAPKT Solver Selection

For real-time replanning in a UAV context:

\- \*\*SIW (Serialized Iterated Width)\*\* — very fast for problems with low width, good for logistics-style domains

\- \*\*BFS(f)\*\* — best general-purpose performance, novelty-based

\- \*\*Deadline Aware Search\*\* — if you have a hard time budget for planning



\### Blackboard Scoping

Use BT.CPP v4's global blackboard (`@` prefix) for world model state, and local blackboards for action-specific data. This prevents name collisions when the same sub-tree template is instantiated multiple times with different parameters.



\### Plan Validation

Before executing a compiled BT, validate that all referenced sub-trees are registered in the factory and all ports are satisfiable. BT.CPP's factory can introspect registered node models for this.



\### Determinism and Auditability

For defence/safety contexts: log every world model state change with timestamp and source (which BT node or external update). The world model version counter enables replay and post-hoc analysis. Consider making the causal graph and compiled BT XML part of the mission audit trail.

