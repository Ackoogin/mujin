# Spatial Routing & Navigation: Options and Integration Strategy

**Purpose:** Evaluate approaches for modelling spatial/routing concerns (distance costs, waypoint generation, flight path planning) within the AME planning and execution pipeline.

**Status:** Options document. No implementation started.

**Key design principle:** The symbolic planner (LAPKT/STRIPS) operates over discrete predicates and should not be burdened with continuous geometric reasoning. Spatial concerns are handled by **external services** that respond to plan-generated navigation needs and report results back as WorldModel facts.

---

## 1. Current Baseline

AME treats spatial state as **discrete symbolic predicates**:

- `(at ?r ?l)` -- an agent is at a named location
- `(airspace-clear ?l)` -- a location is cleared for transit
- `(weather-ok ?l)` -- environmental envelope met at a location

The `move` action is a pure state transition: delete `(at ?r ?from)`, add `(at ?r ?to)`. There is no concept of distance, travel time, intermediate waypoints, or path feasibility. All locations are equidistant from the planner's perspective.

This works for small discrete domains but breaks down when:

- The plan should prefer closer locations (cost-aware planning)
- Move actions require actual flight plans with waypoints
- Path feasibility depends on obstacles, terrain, or airspace geometry
- Multi-agent coordination needs deconflicted routes

---

## 2. Design Philosophy: Separation of Concerns

The symbolic planner answers **what** to do and in **what order**. Spatial/routing services answer **how** to get there. This separation is deliberate:

| Concern | Owner | Mechanism |
|---------|-------|-----------|
| Goal selection and sequencing | PDDL planner (LAPKT) | STRIPS search over boolean state |
| Route feasibility | External path planner | Service call during BT execution |
| Waypoint generation | External path planner | Returns waypoint list via service response |
| Distance/cost estimation | Spatial oracle | Pre-populates WorldModel facts; planner uses as preconditions |
| Flight plan commitment | BT execution node | Sets `(route-planned ?from ?to)` fact on success |
| Geofence / airspace checks | Perception / external | Sets `(airspace-clear ?l)` facts with CONFIRMED authority |

The planner never computes geometry. It reasons about **whether spatial preconditions are met** and **what spatial outcomes are committed**.

---

## 3. Options

### Option A: Proximity Predicates (Symbolic Distance Encoding)

Encode distance relationships as grounded boolean facts that an external oracle maintains.

**WorldModel facts:**
```
(nearest ?r ?l)          -- ?l is the closest unvisited location to ?r
(reachable ?r ?l)        -- ?r can reach ?l given current fuel/battery
(adjacent ?l1 ?l2)       -- ?l1 and ?l2 are within direct transit range
(route-planned ?r ?l)    -- a valid flight plan exists for ?r to reach ?l
```

**PDDL domain extension:**
```pddl
(:predicates
  (nearest ?r - robot ?l - location)
  (reachable ?r - robot ?l - location)
  (route-planned ?r - robot ?from - location ?to - location)
)

(:action move
  :parameters (?r - robot ?from - location ?to - location)
  :precondition (and
    (at ?r ?from)
    (reachable ?r ?to)
    (route-planned ?r ?from ?to)
  )
  :effect (and
    (at ?r ?to)
    (not (at ?r ?from))
    (not (route-planned ?r ?from ?to))   ; consumed on use
  )
)
```

**How it works:**

1. Before planning, an external spatial oracle examines the map/environment and populates proximity/reachability facts into the WorldModel with `FactAuthority::CONFIRMED`
2. The planner uses these as hard preconditions -- it can only generate plans using reachable locations
3. A `(nearest ?r ?l)` predicate lets the planner prefer the closest target (by making it a precondition on a preferred action variant, or by having only one location marked nearest at a time)
4. During execution, a `PlanRoute` BT node calls the path planner service, and on success sets `(route-planned ?r ?from ?to)` as CONFIRMED

**Pros:**
- No changes to the planner or WorldModel infrastructure -- pure PDDL
- Clean separation: spatial oracle is a black box
- Works with existing `FactAuthority` system -- oracle facts are CONFIRMED
- Auditable: proximity decisions show up in WM audit log
- Gracefully degrades: if oracle is unavailable, facts stay BELIEVED or absent

**Cons:**
- Coarse: "nearest" is a single winner, not a ranking
- O(n^2) grounding for `(reachable ?r ?l)` with many locations
- Requires re-running oracle when state changes (agent moves, new obstacles)
- Cannot express "go to the 2nd-nearest if nearest is occupied"

**Assessment:** Good starting point. Low implementation cost. Sufficient for missions with <50 discrete locations.

---

### Option B: Two-Phase Plan-Then-Route

The PDDL planner produces an abstract plan with symbolic `move` actions. A second phase resolves each move into a concrete route before BT execution begins.

**Pipeline:**
```
PDDL Plan (abstract)
    v
Route Resolution Phase
  For each move(agent, from, to):
    Call path planner service
    Store waypoint list
    Estimate cost/duration
    If infeasible -> mark (not reachable ?r ?to), replan
    v
Annotated Plan (moves have waypoints + costs)
    v
PlanCompiler (generates BT with waypoint-following nodes)
```

**Implementation:**
```cpp
struct RouteInfo {
    std::vector<Waypoint> waypoints;
    double distance_m;
    double estimated_duration_s;
    bool feasible;
};

class RouteResolver {
    /// Resolve all move actions in a plan. Returns false if any are infeasible.
    bool resolveAll(const PlanResult& plan, const WorldModel& wm,
                    IPyramidService& path_planner,
                    std::vector<RouteInfo>& routes_out);
};
```

**BT structure for a resolved move:**
```xml
<ReactiveSequence name="move_uav1_base_sector_a">
    <CheckWorldPredicate predicate="(at uav1 base)" required_authority="confirmed"/>
    <CheckWorldPredicate predicate="(airspace-clear sector_a)" required_authority="confirmed"/>
    <FollowWaypoints agent="uav1" waypoints="{route_0_waypoints}"/>
    <SetWorldPredicate predicate="(at uav1 sector_a)" value="true"/>
    <SetWorldPredicate predicate="(at uav1 base)" value="false"/>
</ReactiveSequence>
```

**Pros:**
- Full route detail available before execution begins
- Can detect infeasibility early and replan at the symbolic level
- Duration estimates feed naturally into temporal planning (Extension 7)
- Waypoints are concrete -- execution is deterministic

**Cons:**
- Routes may become stale between resolution and execution (environment changes)
- Adds latency to the plan-compile cycle
- Requires a `FollowWaypoints` BT node (new node type)
- Two failure modes: planning failure vs. routing failure

**Assessment:** Natural fit for missions where routes are relatively stable (pre-surveyed environments). Pairs well with temporal planning (Extension 7) since route durations feed into STN scheduling.

---

### Option C: Lazy Route Resolution via BT Service Nodes

Routes are resolved **during execution**, not before. The `move` action maps to an `InvokeService` call that requests a route from the path planner, then a `FollowWaypoints` node executes it.

**ActionRegistry configuration:**
```cpp
registry.registerActionSubTree("move", R"xml(
    <Sequence>
        <InvokeService
            service_name="path_planner"
            operation="compute_route"
            param_names="?robot;?from;?to"
            param_values="{param0};{param1};{param2}"
            timeout_ms="3000"
            response_json="{route_response}"/>
        <SetWorldPredicate predicate="(route-planned {param0} {param1} {param2})" value="true"/>
        <FollowRoute agent="{param0}" route="{route_response}"/>
    </Sequence>
)xml");
```

**How it works:**
1. Planner produces abstract plan with `move` actions (no spatial detail)
2. PlanCompiler resolves `move` via ActionRegistry to the sub-tree above
3. During BT execution, `InvokeService` calls the path planner just-in-time
4. If the path planner returns infeasible, the action fails and triggers replanning
5. On success, `FollowRoute` executes the waypoints

**Pros:**
- Routes are always fresh -- computed just before execution
- Uses existing `InvokeService` infrastructure (Extension 4)
- No changes to the planner or PlanCompiler
- Failure handling is natural -- BT failure triggers replanning
- `(route-planned ...)` fact makes routing observable in WorldModel audit
- `required_authority` on precondition checks ensures perception confirmation

**Cons:**
- Execution pauses while waiting for route computation
- Cannot estimate total mission duration before execution starts
- If path planner is slow, BT tick blocks on RUNNING
- No global route deconfliction (each move is resolved independently)

**Assessment:** The most architecturally clean option. Aligns with the user's stated preference for external nodes that react to navigation needs and set facts to record completion. Recommended as the primary approach.

---

### Option D: Numeric Fluents with Cost-Aware Planning

Add numeric `(:functions)` to the PDDL domain and use a cost-aware planner to minimize total travel distance.

**PDDL domain extension:**
```pddl
(:functions
    (distance ?from - location ?to - location) - number
    (fuel ?r - robot) - number
    (fuel-rate ?r - robot) - number
)

(:action move
  :parameters (?r - robot ?from - location ?to - location)
  :precondition (and
    (at ?r ?from)
    (>= (fuel ?r) (* (distance ?from ?to) (fuel-rate ?r)))
  )
  :effect (and
    (at ?r ?to)
    (not (at ?r ?from))
    (decrease (fuel ?r) (* (distance ?from ?to) (fuel-rate ?r)))
  )
)

(:metric minimize (total-cost))
```

**Pros:**
- Planner-optimal: finds shortest/cheapest plan considering distances
- Fuel/battery constraints are handled by the planner, not by external checks
- Standard PDDL 2.1 -- supported by OPTIC, TFD, Aries (Extension 7 planners)

**Cons:**
- **Requires numeric fluent support in WorldModel** (Extension 7 prerequisite)
- **Requires a cost-aware planner** -- LAPKT BRFS is satisficing, not optimal
- Distance matrix must be pre-computed and loaded as PDDL `:init` facts
- O(n^2) distance entries for n locations
- Mixes spatial reasoning into the planner -- violates separation principle
- Much harder to maintain and debug

**Assessment:** Correct long-term approach for fuel/resource-constrained missions. Blocked on Extension 7 (temporal/numeric planning). Should not be the first step.

---

### Option E: Hybrid -- Symbolic + Lazy Resolution

Combines Option A (proximity predicates for plan guidance) with Option C (lazy route resolution for execution). This is the **recommended approach**.

**Planning phase:**
- Spatial oracle populates `(reachable ?r ?l)` and `(nearest ?r ?l)` facts
- Planner uses these to produce a spatially-informed but still abstract plan
- No routing detail in the plan itself

**Execution phase:**
- Each `move` action triggers a lazy `InvokeService` call to the path planner
- Path planner returns waypoints
- `FollowRoute` executes them
- On completion, `SetWorldPredicate` updates location facts with CONFIRMED authority
- Spatial oracle re-evaluates `(nearest ...)` based on new position

**The split:**

```
                         PLANNING                    EXECUTION
                    +-----------------+        +---------------------+
                    |                 |        |                     |
  Spatial Oracle -->|  (reachable)    |        |   InvokeService     |
  (pre-plan)        |  (nearest)      |--plan-->|   "path_planner"   |
                    |  (adjacent)     |        |        |            |
                    |                 |        |   FollowRoute       |
                    |  PDDL Planner   |        |        |            |
                    |  (LAPKT STRIPS) |        |   SetWorldPredicate |
                    |                 |        |   (at ?r ?to) CONF  |
                    +-----------------+        +---------------------+
```

**Pros:**
- Planner is spatially informed (avoids unreachable plans) but not burdened with geometry
- Routes are always fresh at execution time
- Uses all existing infrastructure: WorldModel facts, InvokeService, FactAuthority
- Graceful degradation: if spatial oracle is unavailable, all locations are assumed reachable (over-approximation -- safe for planning, execution will catch infeasibility)
- Clean audit trail: proximity decisions (CONFIRMED) + route outcomes (CONFIRMED) all in WM log
- Multi-agent deconfliction can be added at the service level without changing the planner

**Cons:**
- Two external dependencies (spatial oracle + path planner) to integrate
- Proximity facts may become stale during long plans (mitigated by replanning)
- Still no cost-optimal planning (satisficing STRIPS)

---

## 4. Recommendation

### Phase 1: Option E (Hybrid) -- Immediate

| Work Item | Component | Effort | Notes |
|-----------|-----------|--------|-------|
| Define proximity predicate schema | PDDL domain | Low | `(reachable)`, `(nearest)`, `(route-planned)` |
| Spatial oracle interface | `ISpatialOracle` | Low | `computeReachability(wm) -> fact updates` |
| Stub oracle for testing | `StubSpatialOracle` | Low | All locations reachable, nearest = first alphabetically |
| `FollowRoute` BT node | `bt_nodes/` | Medium | `StatefulActionNode`, reads waypoint data from blackboard |
| `move` sub-tree template | ActionRegistry | Low | `InvokeService` -> `SetWorldPredicate` -> `FollowRoute` |
| Update SACE domains | `subprojects/AME/domains/` | Low | Add proximity predicates to existing domains |
| Integration tests | `tests/` | Medium | End-to-end: plan with proximity constraints, execute with stub path planner |

### Phase 2: Cost-Aware Planning -- After Extension 7

Once numeric fluents and a cost-aware planner (OPTIC/Aries) are available:

- Add `(distance ?from ?to)` functions to domains
- Spatial oracle populates distance matrix as numeric facts
- Planner minimizes total distance or fuel consumption
- Route resolution remains lazy (Phase 1 infrastructure reused)

### Phase 3: Multi-Agent Route Deconfliction -- Future

- Path planner service becomes coordination-aware
- Receives all pending routes, returns deconflicted set
- `GoalAllocator` extended with spatial proximity heuristic (assign closest sectors)
- Shared airspace constraints as mutual-exclusion predicates

---

## 5. Spatial Oracle Interface

The spatial oracle is a service (local or external) that examines the current geometric environment and translates it into WorldModel predicates.

```cpp
class ISpatialOracle {
public:
    virtual ~ISpatialOracle() = default;

    /// Compute reachability from the current state. Sets facts in wm.
    /// Called before each planning episode.
    virtual void updateReachability(WorldModel& wm) = 0;

    /// Compute nearest unvisited target for each agent. Sets (nearest ...) facts.
    virtual void updateNearest(WorldModel& wm) = 0;
};
```

**When is the oracle called?**

- Before each `Planner::solve()` call (including replans)
- The `MissionExecutor` replan loop becomes:
  1. `oracle.updateReachability(wm)`
  2. `oracle.updateNearest(wm)`
  3. `planner.solve(wm)`
  4. `compiler.compile(plan)`

**Oracle fact authority:** All oracle-set facts use `FactAuthority::CONFIRMED` since they derive from geometric/sensor data, not plan predictions.

**Staleness:** Oracle facts include timestamps via `FactMetadata::timestamp_us`. A configurable TTL could mark stale reachability facts as BELIEVED, forcing re-evaluation before the planner trusts them. This pairs with the `required_authority="confirmed"` mechanism on `CheckWorldPredicate`.

---

## 6. FollowRoute BT Node

A new `StatefulActionNode` that executes a waypoint sequence:

```cpp
class FollowRoute : public BT::StatefulActionNode {
public:
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("agent"),
            BT::InputPort<std::string>("route"),       // serialised waypoints from InvokeService
            BT::OutputPort<std::string>("progress"),   // "3/12" style progress
        };
    }

    BT::NodeStatus onStart() override;    // parse route, send first waypoint command
    BT::NodeStatus onRunning() override;  // poll agent position, advance waypoints
    void onHalted() override;             // send stop command to agent
};
```

In simulation/test, this node can be stubbed to return SUCCESS immediately. In production, it interfaces with the flight controller via ROS2 topics or PYRAMID service calls.

---

## 7. Example: End-to-End Flow

**Scenario:** UAV must search three sectors. Sector B is closest, then A, then C.

**Before planning (oracle runs):**
```
WorldModel facts (CONFIRMED):
  (nearest uav1 sector_b)
  (reachable uav1 sector_a)
  (reachable uav1 sector_b)
  (reachable uav1 sector_c)
```

**Planner produces (STRIPS):**
```
1. move(uav1, base, sector_b)     -- nearest first
2. search(uav1, sector_b)
3. move(uav1, sector_b, sector_a)
4. search(uav1, sector_a)
5. move(uav1, sector_a, sector_c)
6. search(uav1, sector_c)
```

**Compiled BT (move actions expanded via ActionRegistry):**
```xml
<Sequence>
  <!-- Step 1: move to sector_b -->
  <ReactiveSequence>
    <CheckWorldPredicate predicate="(at uav1 base)" required_authority="confirmed"/>
    <CheckWorldPredicate predicate="(reachable uav1 sector_b)" required_authority="confirmed"/>
    <InvokeService service_name="path_planner" operation="compute_route"
                   request_json="agent=uav1;from=base;to=sector_b"
                   response_json="{route_0}"/>
    <SetWorldPredicate predicate="(route-planned uav1 base sector_b)" value="true"/>
    <FollowRoute agent="uav1" route="{route_0}"/>
    <SetWorldPredicate predicate="(at uav1 sector_b)" value="true"/>
    <SetWorldPredicate predicate="(at uav1 base)" value="false"/>
  </ReactiveSequence>
  <!-- Step 2: search sector_b -->
  <Sequence>
    <CheckWorldPredicate predicate="(at uav1 sector_b)" required_authority="confirmed"/>
    <SearchAction agent="uav1" sector="sector_b"/>
    <SetWorldPredicate predicate="(searched sector_b)" value="true"/>
  </Sequence>
  <!-- ... steps 3-6 follow same pattern -->
</Sequence>
```

**During execution:**
- Step 1: `InvokeService` calls path planner, gets waypoints. `FollowRoute` flies them. On arrival, `(at uav1 sector_b)` set as CONFIRMED.
- If path planner returns infeasible: `InvokeService` returns FAILURE, BT fails, `MissionExecutor` replans. Oracle re-runs, marks `(not (reachable uav1 sector_b))`, planner finds alternative.

---

## 8. Interaction with Existing Extensions

| Extension | Interaction |
|-----------|-------------|
| **Ext 3 (Perception)** | Perception confirms `(at ?r ?l)` via sensor data; spatial oracle uses perception for obstacle detection |
| **Ext 4 (PYRAMID)** | Path planner exposed as PYRAMID service; `InvokeService` already handles async calls |
| **Ext 5 (Thread Safety)** | Oracle updates use `enqueueMutation()` for batched fact updates between ticks |
| **Ext 6 (Hierarchical)** | Phase-level planning can have per-phase oracle calls; route resolution is per-phase |
| **Ext 7 (Temporal)** | Route duration estimates feed into STN scheduling; distance costs feed into numeric fluents |
| **SACE** | `(airspace-clear ?l)` already modelled; oracle adds `(reachable)` as additional safety gate |
| **FactAuthority** | Oracle facts = CONFIRMED; `required_authority="confirmed"` enforces perception-backed preconditions during execution |

---

## 9. Risk Assessment

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Path planner latency blocks BT ticks | Slow execution, timeout risk | Medium | Async via `InvokeService` (already non-blocking); configurable timeout |
| Spatial oracle staleness during long plans | Plan uses outdated proximity info | Medium | Replan triggers oracle refresh; TTL on proximity facts |
| Route infeasibility discovered late in execution | Wasted execution time | Low | Pre-plan feasibility check (Option B) as optional hardening step |
| O(n^2) reachability facts for large domains | Grounding explosion | Low | Oracle only grounds reachable pairs; or use `(adjacent)` transitivity |
| Multi-agent route conflicts | Collision risk, deadlock | Medium | Phase 3: coordination-aware path planner; mutual-exclusion predicates |
| Waypoint format coupling between path planner and FollowRoute | Brittle integration | Low | Standardise on a simple format (lat/lon/alt list); serialise as semicolon-separated string |

---

## References

- [Planning Wiki -- Action Costs](https://planning.wiki/ref/pddl/requirements#action-costs)
- [PDDL Numeric Fluents](https://planning.wiki/ref/pddl21/domain#functions)
- [BehaviorTree.CPP StatefulActionNode](https://www.behaviortree.dev/docs/3.8/tutorial-advanced/asynchronous_nodes/)
- [PX4 Path Planning Interface](https://docs.px4.io/main/en/computer_vision/path_planning_interface.html)
- [ROS2 Nav2 -- Behaviour Tree Navigator](https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html)

