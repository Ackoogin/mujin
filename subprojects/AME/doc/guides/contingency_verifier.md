# Contingency Verifier

Exhaustive safe-state reachability analysis for PDDL contingency domains.

---

## What it does

The contingency verifier accepts a PDDL domain and a template problem file, then proves whether every possible combination of system health states has a valid plan to reach the safety goal. It produces a reviewable report suitable for safety case evidence.

The tool:

1. Parses the domain and **automatically identifies context predicates** -- 0-ary predicates that gate actions but are never produced or consumed by any action.
2. Enumerates all 2^N combinations of these predicates.
3. For each combination, invokes the LAPKT BRFS planner.
4. Reports solvability, plan length, and recovery actions for every combination.
5. Exits with code 0 if all combinations are safe, 1 if any gaps exist.

---

## Context predicates

Context predicates represent **externally-determined facts** -- hardware health, sensor status, comms availability, weather -- that change due to real-world events outside the planner's control. In the real system these are always **CONFIRMED** by perception, never BELIEVED by the planner.

The tool identifies them automatically by checking two conditions:

- The predicate appears in at least one action's preconditions (it gates behaviour).
- The predicate never appears in any action's add or delete effects (the planner cannot change it).

Within any single planning cycle, these predicates are fixed context. The verifier exhausts every possible context snapshot the replanner might encounter after a perception update.

Examples from the vehicle autonomy domain:

| Context predicate | Real-world meaning |
|---|---|
| `(nav-operational)` | Primary navigation system healthy |
| `(engines-operational)` | Propulsion system healthy |
| `(comms-available)` | C2 data link active |
| `(gps-available)` | GPS signal available |
| `(inertial-nav-available)` | INS self-test passed |

---

## Usage

```bash
contingency_verifier <domain.pddl> <template_problem.pddl> [options]
```

**Options:**

| Flag | Description |
|------|-------------|
| `--no-prune` | Disable monotonicity pruning (solve every combination) |
| `--json <file>` | Write machine-readable report to JSON file |
| `--verbose` | Show full plan step signatures |
| `-h`, `--help` | Show help |

**Template problem file:** provides the structural initial state (position, flight phase, ODD predicates) and the safety goal. The tool toggles context predicates across all combinations while keeping the structural state fixed.

Run once per mission phase with different templates:

```bash
# Airborne: can every health state reach safe-state?
contingency_verifier domain.pddl problem_all_failed_airborne.pddl

# Airborne: which health states can return to base?
contingency_verifier domain.pddl problem_nav_fail_en_route.pddl --json rtb_report.json

# On-ground: can every health state abort safely?
contingency_verifier domain.pddl problem_nav_fail_pre_takeoff.pddl
```

**Exit code:** 0 = all combinations safe, 1 = design gaps found. Suitable for CI/CD gates.

---

## Build

The tool is built as part of the AME subproject:

```bash
cmake -B build -DUNMANNED_BUILD_AME=ON
cmake --build build --target contingency_verifier
```

The binary is at `build/subprojects/AME/src/contingency_verifier`.

---

## Monotone dominance pruning

The tool applies monotone dominance pruning by default to reduce the number of solver calls. The justification and validity conditions are printed in every report.

### Formal basis

Context predicates are:

1. **Not in any action's add effects** -- the planner cannot set them.
2. **Not in any action's delete effects** -- the planner cannot clear them.
3. **Appear only as positive preconditions** -- the STRIPS parser does not support negated preconditions.

Given these three conditions:

**Theorem.** If a goal is reachable from context S, it is also reachable from any context S' where S' has all facts of S plus additional context facts.

**Proof.** Any plan P valid in S is also valid in S'. Each action's preconditions are a conjunction of positive atoms. Since S ⊆ S', every precondition satisfied in S is also satisfied in S'. Effects are identical (context predicates are not in effects), so the plan produces the same state transitions and achieves the same goal.

**Corollaries:**

- **Safe propagation:** if context S is SAFE, all supersets of S are also SAFE.
- **Gap propagation:** if context S is a GAP, all subsets of S are also GAPS.

### When pruning is invalid

Pruning would be invalid if any of the three conditions were violated:

- If an action could **set** a context predicate (condition 1 violated), the planner could change the context mid-plan, and supersets might enable unwanted side effects.
- If an action could **clear** a context predicate (condition 2 violated), same issue.
- If a context predicate appeared as a **negated precondition** (condition 3 violated), additional capabilities could *disable* actions, breaking monotonicity.

The tool verifies all three conditions at startup and prints the verification result per predicate. Use `--no-prune` to disable pruning and solve every combination independently if the domain structure is uncertain.

### Performance impact

| Scenario | Without pruning | With pruning |
|---|---|---|
| All-off is solvable (e.g. emergency-land exists) | 2^N calls | **1 call** |
| Return-to-base (engines required) | 32 calls | **21 calls** |
| Full mission (NAV+ENG+COM required) | 32 calls | **29 calls** |

The algorithm processes combinations bottom-up (fewest capabilities first). When a minimal solvable state is found, all its supersets are immediately classified.

---

## Report output

### Text report (stdout)

The text report includes:

1. **Header** -- domain file, template problem, identified context predicates with initial values.
2. **Pruning justification** -- formal theorem, proof, and per-predicate condition verification.
3. **Combination table** -- every combination with result (SAFE / GAP / SAFE(implied) / GAP(implied)), plan length, solve time, and recovery actions used.
4. **Summary** -- total combinations, solver calls, pruning statistics, and any design gaps.

Implied results show which verified combination they were derived from, maintaining an audit trail.

### JSON report (`--json`)

Machine-readable output containing all fields from the text report plus per-combination flags, full plan signatures, and timing data. Suitable for downstream tooling, dashboards, or automated regression detection.

Structure:

```json
{
  "domain": "vehicle_autonomy/domain.pddl",
  "problem_template": "problem_all_failed_airborne.pddl",
  "total_combinations": 32,
  "solver_calls": 1,
  "pruned": 31,
  "safe_count": 32,
  "gap_count": 0,
  "health_variables": [...],
  "combinations": [
    {
      "combo": 0,
      "label": "nav-operational=off  engines-operational=off ...",
      "flags": {"nav-operational": false, ...},
      "result": "SAFE",
      "plan_length": 1,
      "solve_time_ms": 0.1,
      "actions": "emergency-land",
      "plan": "emergency-land(uav1,wp1)"
    },
    ...
  ]
}
```

---

## Contingency domain catalogue

### Vehicle autonomy (`domains/vehicle_autonomy/`)

Models the flight lifecycle of a single UAV. The mission is abstract (`execute-mission`); the domain focuses on getting the vehicle safely from ground to mission area and back.

**Actions (nominal):** `preflight-check`, `takeoff`, `fly`, `execute-mission`, `land`

**Actions (contingency):** `abort-on-ground`, `emergency-return-nav`, `emergency-return-gps`, `emergency-return-inertial`, `emergency-land`, `emergency-land-at-base`

**Redundancy ladder** (most capable to last resort):

```
nominal fly  >  emergency-return-nav  >  emergency-return-gps
             >  emergency-return-inertial  >  emergency-land
```

**Template problems:**

| File | Phase | Goal | Purpose |
|------|-------|------|---------|
| `problem_nominal.pddl` | Ground | Full mission + return | Nominal capability check |
| `problem_nav_fail_pre_takeoff.pddl` | Ground | Abort safely | Ground-abort verification |
| `problem_all_failed_airborne.pddl` | Airborne | Safe-state + landed | Universal safety proof |
| `problem_nav_fail_en_route.pddl` | Airborne | Safe-state + landed + at base | Return-to-base ladder |

### Mission autonomy (`domains/mission_autonomy/`)

Models multi-agent task execution with reallocation contingency. Vehicle-level concerns are abstracted away.

**Actions (nominal):** `move`, `search`, `classify`

**Actions (contingency):** `withdraw-agent`, `mark-task-failed`, `reallocate-task`, `search-degraded`

**Reallocation flow:**

```
sensor failure -> withdraw-agent -> mark-task-failed -> reallocate-task -> new agent executes
```

**Template problems:**

| File | Scenario | Expected |
|------|----------|----------|
| `problem_nominal.pddl` | Both agents healthy | Solvable |
| `problem_agent_lost.pddl` | uav2 unavailable | Solvable via reallocation |
| `problem_sensor_fail.pddl` | uav1 sensor down | Solvable via withdraw + reallocate |
| `problem_both_sensors_degraded.pddl` | Both sensors down | Unsolvable (design boundary) |
| `problem_comms_lost_no_realloc.pddl` | Comms lost, tasks pre-assigned | Solvable |
| `problem_comms_lost_realloc_needed.pddl` | Comms lost + agent down | Unsolvable (design boundary) |

---

## SACE integration

The contingency verifier addresses Gap 5 (Automated Scenario Generation) and partially Gap 2 (Reachability Analysis) from the SACE tooling analysis (`doc/assurance/AME/sace-pddl/13-tooling-analysis.md`).

It provides evidence for:

| SACE Stage | Evidence provided |
|---|---|
| Stage 2 (Hazardous Scenarios) | Systematic fault combination enumeration |
| Stage 5 (Design Assurance) | Degraded-mode configuration verification |
| Stage 7 (Out-of-Context) | Universal safe-state reachability proof |
| Stage 8 (Verification) | Scenario coverage metrics |

The tool consumes the same PDDL domain files used by the runtime execution planner, maintaining the shared-domain traceability principle described in the tooling analysis.

---

## Limitations

- **0-ary predicates only.** Parameterised context predicates (e.g. `(weather-ok ?l)`) are not enumerated. These can be handled by providing multiple template problems with different structural states.
- **STRIPS only.** The tool inherits the PDDL parser's STRIPS limitation. Negated preconditions are not supported; use explicit positive predicates (e.g. `(sensor-degraded ?a)` instead of `(not (sensor-operational ?a))`).
- **No timeout.** The LAPKT BRFS solver has no per-combination timeout. For large domains, individual combinations may take significant time. Use `--no-prune` with caution on large state spaces.
- **Sequential execution.** Combinations are solved sequentially. For large enumerations, consider running the tool overnight in CI/CD.
