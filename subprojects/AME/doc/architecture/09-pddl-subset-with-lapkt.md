# PDDL Subset Extensions Without LAPKT Changes

AME does not pass raw PDDL to LAPKT. `PddlParser` lowers a domain/problem into
`WorldModel`, `WorldModel::projectToSTRIPS()` builds an `aptk::STRIPS_Problem`,
and the planner runs LAPKT BRFS over grounded boolean fluents and grounded actions.
The BT compiler then turns the returned ground-action sequence into predicate
guards, action nodes, and add/delete predicate writes.

That means many PDDL features are AME parser/model/compiler decisions, not LAPKT
source-code restrictions. A feature is a good candidate only if it can be lowered
to the same finite STRIPS projection and the compiled BT can enforce the same
semantics during execution.

## Current Effective Planning Model

- finite typed objects
- boolean predicates grounded into fluents
- positive initial facts; omitted facts are false
- positive conjunctive goals
- grounded actions with positive conjunctive preconditions
- add effects and delete effects
- no conditional effects in the LAPKT projection
- unit-cost BRFS search from the projected world snapshot

## Using the Implemented Extensions (Authoring Guide)

The constructs below are **implemented and accepted by `PddlParser` today**
(added in the "Expand PDDL subset" change). This section is the plain-English
how-to for domain authors; the per-feature design notes further down explain how
each one is lowered to STRIPS. Everything here is compiled away before LAPKT, so
the planner and the compiled BT behave exactly as if you had hand-written the
expanded conjunctions.

All examples are minimal, self-contained domain/problem fragments. Use the same
`(:requirements :strips :typing)` line shown in the first example.

### Nested conjunctions

You can nest `(and ...)` to any depth in preconditions, effects, and goals — it
is flattened to the same flat conjunction of leaf atoms. Use this when it reads
more naturally to group related conditions.

```lisp
;; These two preconditions are identical to the planner:
:precondition (and (at ?r ?l) (and (fuelled ?r) (armed ?r)))
:precondition (and (at ?r ?l) (fuelled ?r) (armed ?r))
```

An empty `(and)` is accepted and means "no condition" (vacuously true) — handy
for an action that should always be applicable.

### Equality / inequality binding filters

`(= ?a ?b)` and `(not (= ?a ?b))` constrain *which parameter combinations* are
legal. They are **binding filters only** — they never become world facts and
never appear at runtime; their sole effect is that grounding produces fewer
action instances. One side may be a constant object name instead of a parameter.

```lisp
(:action move
  :parameters (?from - loc ?to - loc)
  ;; "?from must differ from ?to" -> drops self-moves like move(l1,l1)
  :precondition (and (at ?from) (not (= ?from ?to)))
  :effect (and (at ?to) (not (at ?from))))

(:action selfcheck
  :parameters (?a - loc ?b - loc)
  ;; "?a must equal ?b" -> keeps only selfcheck(l1,l1), selfcheck(l2,l2)
  :precondition (= ?a ?b)
  :effect (at ?a))
```

With objects `l1 l2`, `move` grounds to `move(l1,l2)` and `move(l2,l1)` only.
Equality is **precondition-only**: `(= ...)` in an effect or goal is rejected. An
operand that names neither a parameter nor a known object fails closed at
grounding.

### Disjunctive preconditions, `(or ...)`

An action may require *any one* of several alternatives. Under the hood each
disjunct becomes its own grounded action (one schema per branch, tagged
internally as `name#0`, `name#1`, …), but all branches resolve to the **same**
registered BT implementation — you author and register the action once.

```lisp
(:action engage
  :parameters (?o - target)
  ;; runs if the target is detected by radar OR by ESM
  :precondition (or (radar-track ?o) (esm-track ?o))
  :effect (engaged ?o))
```

`(or ...)` combines with `and`, negation, and equality; the whole precondition is
normalised to disjunctive normal form. The expansion is capped (limit 4096
clauses); a precondition that blows past the cap fails closed rather than
emitting a huge schema set. Keep alternatives modest.

### Disjunctive goals, `(or ...)`

A top-level `(or ...)` in the `:goal` means "reach **any** of these states." The
planner tries each alternative and returns the first that is solvable; the
compiled goal guard succeeds if any alternative holds.

```lisp
;; objects: l1 l2 l3 - loc ; init: (at l1) (linked l1 l2)
(:goal (or (at l2) (at l3)))
```

Here `l2` is reachable and `l3` is not — the goal is still solved (via `l2`) and
the audit log records which alternative was achieved. If *neither* alternative is
reachable the problem is reported unsolvable. Goals remain negation-free
(negative goals are not supported).

### Finite `forall` / `exists` preconditions

Quantifiers range over the **fixed object set** declared in the problem and are
expanded at parse time:

- `(forall (?x - T) phi)` becomes the conjunction of `phi` over every object of
  type `T` — "this must hold for *all* of them."
- `(exists (?x - T) phi)` becomes the disjunction over every object of type `T`
  — "this must hold for *at least one*" (and so splits into schemas like
  disjunctive preconditions).

```lisp
(:action sense
  :parameters (?r - robot)
  ;; applicable when the robot is at some sector
  :precondition (exists (?s - sector) (at ?r ?s))
  :effect (sensed ?r))

(:action sweep
  :parameters (?r - robot)
  ;; applicable only when every sector is clear
  :precondition (forall (?s - sector) (clear ?s))
  :effect (swept ?r))
```

Edge cases: an empty type domain makes a `forall` vacuously true (contributes
nothing) and an `exists` unsatisfiable (the action grounds to zero instances). A
quantifier variable that shadows an action parameter name (e.g. parameter `?s`
and `(forall (?s - sector) ...)`) is rejected. Quantifiers are precondition-only;
they are rejected in effects and goals. Because the expansion is over the object
set known at registration time, adding objects before registration enlarges it.

### `either` union parameter types

A parameter can range over more than one type with `(either t1 t2 ...)`. Grounding
enumerates the deduplicated union of objects of those types (including each type's
subtypes).

```lisp
(:types vehicle - object  uav ship - vehicle  truck - object)
...
(:action assign
  :parameters (?v - (either uav ship))   ;; grounds over uavs and ships, not trucks
  :precondition (tasked ?v)
  :effect (escorting ?v))
```

With objects `u1 - uav`, `s1 - ship`, `t1 - truck`, `assign` grounds to
`assign(u1)` and `assign(s1)` only. Overlapping unions (a type and its subtype)
are not double-counted. An unknown type name inside `either` is rejected. A
single-element `(either uav)` behaves identically to a bare `uav`. `either` is
allowed on action `:parameters`; predicate parameter declarations stay
single-typed.

### Quick reference

| Construct | Where allowed | Example |
|-----------|---------------|---------|
| Nested `(and ...)` | precondition, effect, goal | `(and A (and B C))` |
| `(= ?a ?b)` / `(not (= ?a ?b))` | precondition only | `(not (= ?from ?to))` |
| `(or ...)` | precondition, top-level goal | `(or (radar-track ?o) (esm-track ?o))` |
| `(forall (?x - T) phi)` | precondition only | `(forall (?s - sector) (clear ?s))` |
| `(exists (?x - T) phi)` | precondition only | `(exists (?s - sector) (at ?r ?s))` |
| `(either t1 t2)` | action parameter type | `(?v - (either uav ship))` |

Not accepted (rejected explicitly, not silently approximated): negative goals;
equality, quantifiers, or `either` in effects or goals; conditional effects;
numeric fluents; temporal/durative PDDL. See the per-feature plans below for the
lowering details and the **Implementation Rule** at the end for the invariant
every construct preserves.

## Reasonable Additions

These can be added in AME without changing LAPKT, provided parser, grounding,
planner projection, BT compilation, and tests are updated together.

| PDDL construct | Lowering strategy | Notes |
|----------------|------------------|-------|
| Explicit negative init facts, `(not (p ...))` | Treat as validation that the fluent is false | Useful import compatibility; should fail if paired with a positive fact. |
| Equality / inequality in action preconditions | Apply as binding filters during grounding | `(= ?a ?b)` and `(not (= ?a ?b))` do not need planner fluents. Ground fewer actions. |
| Nested conjunctions | Flatten during parse | Low risk; preserves current positive-conjunction model. |
| Negative preconditions | Track signed preconditions in AME; generate transient complement fluents only in the LAPKT projection | Avoids duplicating state in `WorldModel`; BT guards can check `expected="false"` directly. |
| Negative goals | Same transient-complement projection strategy as negative preconditions | Useful, but goal guards and verifiers must check false facts directly in AME. |
| Disjunctive preconditions | Split one action schema into multiple positive-precondition schemas | Can cause action blow-up; emitted BT action mapping must still resolve to the same implementation. |
| Disjunctive goals | Run one solve per disjunct, or add a synthetic goal-achievement action | Multiple solves are simpler and keep audit evidence clear. |
| Finite existential preconditions | Expand into a disjunction over objects, then split | Works only against the fixed object set known at grounding time. |
| Finite universal preconditions | Expand into a conjunction over objects | Safe for static object sets; can become large quickly. |
| Simple derived/static predicates | Materialize them before planning as ordinary fluents | Best for acyclic predicates over static facts. Dynamic derived predicates need careful re-materialization before each solve. |
| `either` type expressions | Expand to multiple typed alternatives during grounding | Avoids changing LAPKT; may require TypeSystem support for union-like parameter domains. |
| Bounded enumerated numeric state | Model as finite predicates, e.g. `(fuel-level uav high)` | Reasonable only for small discrete state spaces. Real arithmetic is not a good fit. |

### What These Mean In Plain English

Short explanations of each row above, for readers who do not live in PDDL. Each
one is "low risk" because it can be rewritten into the plain ingredients the
planner already understands — true/false facts, positive preconditions, add/delete
effects — without teaching LAPKT anything new.

- **Explicit negative init facts, `(not (p ...))`.** Some problem files spell out
  what is *false* at the start, e.g. `(not (jammed uav1))`. AME already treats any
  fact you don't mention as false, so these lines add no new information. We accept
  them purely as a consistency check: parsing confirms the fact really is false, and
  errors out if the same file also asserts it true (a contradiction).
- **Equality / inequality in preconditions.** Lets an action say "these two
  parameters must be the same object" `(= ?a ?b)` or "must be different"
  `(not (= ?a ?b))` — e.g. "fly from ?a to ?b where ?a ≠ ?b". The planner never sees
  this; during grounding we simply don't generate the action instances that break
  the rule. The effect is fewer grounded actions, not an extra fact to track.
- **Nested conjunctions.** PDDL lets you write `(and A (and B C))` instead of the
  flat `(and A B C)`. We flatten the nesting while reading the file. The planning
  model is unchanged — this is purely a parsing convenience.
- **Negative preconditions.** An action that may only run while some fact is
  *false*, e.g. "launch only while the platform is not safetied". Tracked as a
  separate signed precondition and compiled into a throwaway "complement" fact at
  the LAPKT boundary, so the real world state never has to store the negation. See
  the full **Negative Preconditions Plan** below.
- **Negative goals.** A goal that asks for a fact to end up *false*, e.g. "finish
  with the radar not emitting". Uses the same complement-fact trick as negative
  preconditions, with one catch: the goal checks and the runtime verifiers have to
  look for the false fact directly rather than waiting for a positive fact to appear.
- **Disjunctive preconditions, `(or A B)`.** An action that can run if *any one* of
  several conditions holds. We make a separate copy of the action for each
  alternative, each copy with an ordinary positive precondition, and point all
  copies at the same real behaviour. Watch out: many alternatives means many copies
  (action blow-up).
- **Disjunctive goals, `(or A B)`.** "Reach state A *or* state B." The simplest
  handling is to run the planner once per option and keep the first that succeeds.
  That keeps the audit trail honest about which goal was actually achieved.
- **Finite existential preconditions, `(exists (?x) ...)`.** "There is *some* object
  that satisfies this." Because the set of objects is fixed and known up front, we
  expand it into "satisfied for A, or B, or C ..." over those objects and then treat
  it like a disjunction.
- **Finite universal preconditions, `(forall (?x) ...)`.** "*Every* object satisfies
  this." Expanded into "satisfied for A, and B, and C ..." over the known objects.
  Safe, but the conjunction can get large quickly when there are many objects.
- **Simple derived / static predicates.** Predicates whose truth follows from a rule
  over other facts, rather than being asserted directly. If the rule only depends on
  things the plan never changes, we can compute these once before planning and store
  them as ordinary facts. Rules that change *as the plan runs* would need
  recomputing before every solve and are much riskier.
- **`either` type expressions.** A parameter that may be one of several types, e.g.
  "?v is either a uav or a ship". We expand it into separate single-type versions
  during grounding, so the planner still only ever sees plain typed parameters.
- **Bounded enumerated numeric state.** For small discrete quantities — fuel as
  low / medium / high, say — we model each level as its own true/false fact like
  `(fuel-level uav high)`. This only works for a handful of named values; genuine
  continuous arithmetic does not fit and belongs in the not-reasonable list below.

## High-Risk Additions

These are possible without editing LAPKT source only if AME compiles them away to
plain STRIPS, but they are easy to get wrong.

- Conditional effects: must either be compiled into action variants with explicit
  condition truth cases, or the LAPKT conditional-effect API must be verified and
  the BT compiler must learn equivalent conditional writes. Do not parse them as
  ordinary effects.
- Quantified effects: can be expanded over finite objects, but delete/add behavior
  must be deterministic and visible in the BT execution layer.
- Action costs and metrics: BRFS currently behaves as a unit-cost solver. Weighted
  planning needs a planner/search change or a separate backend, even if the PDDL
  can be parsed.
- Arbitrary ADL formulas: support only when each formula is normalized into a
  finite combination of the constructs above with clear blow-up bounds.

## Not Reasonable On The Current LAPKT Path

These should use a separate planner backend or a broader architecture change.

- durative actions and temporal constraints
- continuous numeric fluents and arithmetic effects
- PDDL preferences, trajectory constraints, and optimization metrics
- recursive derived predicates or axioms requiring fixpoint evaluation inside search
- object creation/destruction during planning
- probabilistic or nondeterministic effects

## Negative Preconditions Plan

Negative preconditions should be represented as first-class AME metadata and
compiled away only at the LAPKT boundary. Do not add persistent `(not-p ...)`
facts to `WorldModel`: AME already represents false facts natively, and
persistent complement facts would duplicate state, audit records, snapshots,
ROS/PCL messages, and every `setFact()` authority update path.

Recommended representation:

```cpp
struct GroundAction {
    std::string signature;
    unsigned schema_index;
    std::vector<std::string> args;
    std::vector<unsigned> preconditions;      // p must be true
    std::vector<unsigned> neg_preconditions;  // p must be false
    std::vector<unsigned> add_effects;
    std::vector<unsigned> del_effects;
};
```

Action schema internals should mirror this with positive and negative
precondition templates. Keep the existing `WorldModel::registerAction(...)`
signature as a compatibility overload that forwards an empty negative list to a
new overload.

### Parser

- Extend `PddlAction` with `neg_precondition_atoms`.
- Change `parsePrecondition()` to fill separate positive and negative template
  vectors.
- Accept only `(not ATOM)` as a negative precondition. Reject nested negation,
  `(not (and ...))`, `(not (or ...))`, and all existing unsupported ADL/numeric
  constructs.
- Keep goals positive initially unless negative-goal support is implemented in
  the same change.

### Grounding

- Resolve negative precondition templates to normal fluent IDs during
  `WorldModel::groundActionSchema()`.
- If a negated fluent template cannot be grounded, fail closed exactly like
  positive preconditions and effects.
- Do not create complement predicate schemas or complement fluent names in
  `WorldModel`.

### LAPKT Projection

`WorldModel::projectToSTRIPS()` should create complement fluents only in the
local `aptk::STRIPS_Problem`:

1. Add all normal AME fluents as today.
2. If any action has negative preconditions, reserve transient complement IDs:
   `not_id = wm.numFluents() + fluent_id`.
3. Add transient complement fluents with generated names such as
   `"(not (jammed uav1))"` for LAPKT diagnostics only.
4. Initial state includes `p` when the AME fact is true, otherwise includes the
   transient `not-p`.
5. Project action preconditions:
   - positive `p` -> require `p`
   - negative `not p` -> require transient `not-p`
6. Project action effects:
   - add `p` -> add `p`, delete transient `not-p`
   - delete `p` -> delete `p`, add transient `not-p`

`currentStateAsSTRIPS()` must apply the same initial-state rule when a projected
problem contains transient complement fluents.

### BT Compilation

`CheckWorldPredicate` already supports an `expected` input. Emit:

```xml
<CheckWorldPredicate predicate="(jammed uav1)" expected="false"/>
```

for negative preconditions. Keep effect emission unchanged: add effects set the
normal fluent true and delete effects set it false.

Update causal graph analysis before enabling parallel flows:

- `add(p)` supports later positive `p` and threatens later negative `not p`.
- `del(p)` supports later negative `not p` and threatens later positive `p`.
- A later `add(p)` conflicts with an earlier negative `not p` in a parallel flow.
- A later `del(p)` conflicts with an earlier positive `p` in a parallel flow.

When in doubt, preserve sequential order. A conservative BT is preferable to a
parallel tree that can violate a precondition at runtime.

### Verification And Tools

- Update `ForwardSimVerifier` to require `sim_get(id) == false` for
  `neg_preconditions`.
- Treat negative-precondition false checks as value checks at first. Do not
  require confirmed authority for false facts unless a later policy explicitly
  introduces confirmed-false semantics.
- Update Python bindings for `GroundAction` and any `registerAction` overloads.
- Update authoring importer/generator/model only if the authoring tool must
  round-trip negative preconditions. Otherwise the runtime parser can support
  them before the graphical model does.
- Update `contingency_verifier`: monotone pruning assumes positive
  preconditions only. Domains with negative preconditions must either disable
  pruning automatically with an explicit report message, or fail when pruning is
  requested.

### Test Plan

- Parser accepts `(not ATOM)` in action preconditions and rejects non-atomic
  negated formulas.
- Grounding resolves negative preconditions to existing fluent IDs.
- Planner applies an action with a negative precondition only when the fluent is
  false in the current projected state.
- Add effects and delete effects correctly update transient complement fluents
  across multi-step plans.
- BT compiler emits `expected="false"` checks for negative preconditions.
- Forward simulation rejects a proposed plan when a negative precondition is
  violated.
- Causal graph tests prove that add/delete interactions with negative
  preconditions are not incorrectly parallelized.
- Contingency verifier disables or rejects monotone pruning for domains with
  negative preconditions.

## Nested Conjunctions Plan

Goal: accept arbitrarily nested `(and ...)` inside preconditions, effects, and
goals, e.g. `(and (at ?r ?l) (and (fuelled ?r) (armed ?r)))`, treating it as the
flat conjunction of its leaf atoms.

This is the lowest-risk addition because the planning model does not change at
all: a nested `and` is just a different way of writing the same conjunction of
positive (and, with the negative-precondition work, negative) atoms.

### Current state

`parsePrecondition()` and `parseEffect()` in `pddl_parser.cpp` already recurse on
an `and` head, so a nested `and` in a precondition, effect, or goal is in practice
already flattened today. The remaining work is to make that behaviour explicit,
defended, and tested rather than incidental.

### Parser

- Keep the recursive `and` handling in `parsePrecondition()` and `parseEffect()`.
- Treat an empty `(and)` as an empty conjunction (no atoms), not an error.
- Do not special-case nesting depth; recursion already handles any depth.
- Negation nesting is governed by the negative-precondition rules, not here:
  `(and ... (not ATOM) ...)` is fine, `(not (and ...))` stays rejected.

### Grounding, Projection, BT Compilation, Verification

No changes. After flattening, the action carries the same positive/negative
template lists it would have had if the conjunction were written flat, so
grounding, the LAPKT projection, BT emission, and the verifiers all see identical
input.

### Authoring path

If the graphical authoring importer (`pddl_importer.cpp`) walks precondition and
effect S-expressions itself, give it the same recursive `and` flattening so a
round-tripped domain keeps nested conjunctions equivalent. Runtime support does
not depend on this.

### Test Plan

- Parser flattens `(and A (and B C))` in a precondition to the leaf atoms `A B C`.
- Same for effects and for the goal.
- Empty `(and)` is accepted and contributes no atoms.
- A nested conjunction produces the identical grounded action set (preconditions,
  add, delete) as the equivalent flat conjunction.

## Equality / Inequality Plan

Goal: accept `(= ?a ?b)` and `(not (= ?a ?b))` in action preconditions, including
the case where one side is a constant/object, e.g. `(not (= ?from ?to))`.

These are **binding filters**, not facts. They constrain which parameter
combinations are legal; they never become fluents and never appear at runtime.
The entire effect is that grounding produces fewer `GroundAction`s.

### Representation

Add an equality-constraint list to the action schema and to `PddlAction`:

```cpp
struct EqualityConstraint {
    std::string lhs;   // parameter name (e.g. "?a") or object name
    std::string rhs;
    bool must_equal;   // true for (= a b); false for (not (= a b))
};
```

`registerAction(...)` gains a further overload taking the constraint list, with
the existing overloads forwarding an empty list (same compatibility pattern used
for negative preconditions).

### Parser

- The precondition normalizer (`normalizePrecondition()`) intercepts the `=`
  head explicitly *before* the unsupported-head check, so `=` stays in
  `isUnsupportedExpressionHead()` and is still rejected in effects and goals
  (which do not run the normalizer).
- An `(= x y)` head becomes an `EqualityConstraint` with `must_equal = true`.
  Require exactly two arguments, each an atom.
- In the `not` branch, if the single child has an `=` head, emit an
  `EqualityConstraint` with `must_equal = false` instead of a negative
  precondition. `(not (= ...))` with the wrong arity is rejected.
- Equality is precondition-only: `=` in an effect or goal hits the unchanged
  unsupported-head check and is rejected.

### Grounding

- In `groundActionSchema()`, before building a `GroundAction` for a parameter
  combination, evaluate every constraint by mapping each side to its bound value
  (parameter name -> the chosen arg via the param index; otherwise the literal
  object name).
- If a side names neither a known parameter nor a known object, fail closed (same
  error style as an unresolved template).
- Skip the combination when any `must_equal` constraint has unequal sides, or any
  inequality constraint has equal sides. Otherwise ground as normal.

### Projection, BT Compilation, Verification

No changes. Constraints are fully resolved during grounding, so the projected
STRIPS problem, the emitted BT, and `ForwardSimVerifier` only ever see the
surviving ground actions. There is nothing to check at runtime.

### Test Plan

- `(= ?a ?b)` keeps only combinations where the two arguments are the same object.
- `(not (= ?a ?b))` removes self-pairs (e.g. drops `move(uav1,base,base)`).
- A constant operand (`(not (= ?to base))`) filters against the named object.
- An unknown operand fails closed at grounding.
- Equality in an effect or goal is rejected by the parser.

## Disjunctive Preconditions And Goals Plan

Goal: accept `(or A B ...)` in action preconditions and in goals.

LAPKT has no disjunction. Both cases are normalized into the existing
positive/negative conjunction machinery; only the *driver* changes for goals.

### Disjunctive preconditions

Strategy: convert each action's precondition to disjunctive normal form (DNF) and
emit **one schema per disjunct**, each a plain conjunction. All split schemas map
back to the same registered BT implementation.

#### Representation

- The split must not collide in the ground-action signature map. Two disjuncts of
  `deliver` over the same args would both produce `deliver(uav1)` and the second
  would be dropped by `ground_action_signatures_` dedup. Give each split a
  disjunct-tagged signature, e.g. `deliver#0(uav1)` / `deliver#1(uav1)`, while
  recording the **base name** `deliver` for BT resolution.
- Store the base/implementation name on `ActionSchemaInternal` (defaulting to the
  schema name) so the split schemas carry it through to their `GroundAction`s.

#### Parser

- Normalize the precondition S-expression to DNF: distribute `or` over `and`,
  with negative and equality leaves carried through unchanged.
- Bound the expansion: cap the number of disjuncts and fail closed past the cap
  rather than emitting an exponential schema set.
- Emit one `PddlAction` per DNF term, all sharing the base name and differing only
  by their conjunction and a disjunct index.

#### Grounding / Projection

- `groundActionSchema()` is unchanged except that the signature includes the
  disjunct tag and the `GroundAction` records the base name.
- Projection is unchanged: every split is an ordinary positive/negative-precondition
  action.

#### BT Compilation

- `PlanCompiler::actionName()` must strip the disjunct tag (`deliver#0` ->
  `deliver`) so registry lookup resolves the single shared implementation.
- The precondition checks emitted for a step come only from that disjunct's
  conjunction, which is exactly the branch the planner chose.

#### Verification

- `ForwardSimVerifier` treats each split as an ordinary action; no special case
  beyond resolving the base name where it reports signatures.

### Disjunctive goals

Strategy: treat `(or A B)` at goal top level as a set of **goal alternatives** and
solve one alternative at a time, accepting the first that succeeds (matches the
existing `goal_options` pattern in `contingency_verifier.cpp`).

#### Representation

- Add `goal_alternatives_` (a `vector<vector<unsigned>>`) to `WorldModel`, plus a
  `setGoalAlternatives(...)`. Keep `setGoal(...)` as the single-alternative case
  (one element).

#### Parser

- Detect `or` at the goal top level and produce one positive conjunction per
  disjunct via the existing goal parsing. Reject negation inside goal disjuncts
  unless negative goals are implemented in the same change.

#### Planner

- Solve per alternative and return the first success, recording which alternative
  was achieved for the audit log. Keep the per-alternative evidence so it is clear
  which goal the plan actually reached.

#### BT Compilation

- The goal guard becomes a `ReactiveFallback` whose check child is a `Fallback`
  over per-alternative `Sequence`s of `CheckWorldPredicate`. The tree succeeds when
  any alternative is satisfied; otherwise it runs the plan compiled for the chosen
  alternative.

### Test Plan

- A precondition `(or A B)` produces two ground actions whose signatures share a
  base name and whose BT units differ only in their precondition checks.
- The planner can satisfy the action via either branch; both resolve to the same
  registered implementation.
- DNF expansion past the cap fails closed.
- A goal `(or A B)` is solved when only A is reachable, when only B is reachable,
  and is reported unsolvable when neither is.
- The compiled goal guard returns success if any alternative holds.

## Finite Quantifier Expansion Plan

Goal: accept `(forall (?x - T) phi)` and `(exists (?x - T) phi)` in action
preconditions, expanded against the fixed object set known at registration time.

Because objects are added to the `WorldModel` before actions are registered in
`PddlParser::parseFromString`, `typeSystem().getObjectsOfType(T)` already knows
the full, finite object set when expansion runs. Quantifiers are therefore a
parse/normalization-time rewrite into constructs already covered above.

### Universal preconditions

`(forall (?x - T) phi)` becomes the **conjunction** of `phi[?x := o]` for every
`o` in `getObjectsOfType(T)`. Each instantiated `phi` is parsed with the normal
precondition path, so positive leaves extend the positive list and `(not ATOM)`
leaves extend the negative list. No schema split is needed; the action stays one
schema. Note the conjunction grows linearly with the object count.

### Existential preconditions

`(exists (?x - T) phi)` becomes the **disjunction** of `phi[?x := o]` for every
`o` in `getObjectsOfType(T)`, then feeds straight into the disjunctive-precondition
DNF/split path above. The same expansion cap applies.

### Parser

- The normalizer intercepts `forall` / `exists` heads explicitly before the
  unsupported-head check (they stay in `isUnsupportedExpressionHead`, so effects
  and goals still reject them). Parse the typed quantifier variable list and the
  single body.
- Substitute each object into the body with the existing `substituteTemplate`-style
  rewrite, guarding against a quantifier variable name colliding with an action
  parameter name.
- An empty type domain yields: universal -> empty conjunction (vacuously true,
  contributes nothing); existential -> empty disjunction (unsatisfiable), so the
  action grounds to zero instances. Make both explicit and tested.
- Expansion is precondition-only initially; reject quantifiers in effects and goals
  unless/until quantified effects are designed (they are in the high-risk list).

### Grounding / Projection / BT / Verification

No new mechanism: after expansion the action is an ordinary conjunction (universal)
or an ordinary disjunction-split (existential). All downstream stages reuse the
paths already described.

### Test Plan

- `(forall (?x - T) (clear ?x))` expands to a conjunction over all `T` objects.
- `(exists (?x - T) (at ?r ?x))` is satisfiable when the robot is at any `T` object.
- Empty `T`: universal action stays applicable; existential action grounds to none.
- A quantifier variable shadowing a parameter name is rejected.
- Adding an object before registration enlarges the expansion (fixed-set semantics).

## `either` Type Plan

Goal: accept union parameter types, `(?v - (either uav ship))`, meaning the
parameter ranges over objects of any of the listed types.

Strategy: expand the candidate object set for that parameter during grounding.
This avoids both a planner change and a schema split — each grounded instance is
still an ordinary action over a concrete object.

### Representation

- Allow an action parameter's type to be a list of type names rather than a single
  name. Encode it on `ActionSchemaInternal` (e.g. a `vector<vector<string>>`
  alongside `param_types`, or a small `ParamType` holding one-or-more type names).
- A single-type parameter is the one-element case, so existing schemas are
  unaffected.

### Parser

- Extend `parseTypedList()` so the token after `-` may be a compound
  `(either t1 t2 ...)` as well as a bare atom. Validate each listed type exists in
  the `TypeSystem`.
- Apply to action `:parameters`. Predicate parameter types stay single for now;
  reject `either` in predicate declarations unless a separate change adds it.

### Grounding

- In `generateCombinations()` / `groundActionSchema()`, for an `either` parameter
  enumerate the **deduplicated union** of `getObjectsOfType(t)` over the listed
  types (each already includes that type's subtype objects). Combine with the other
  parameters as usual.
- The grounded `GroundAction` is indistinguishable from one written with a single
  type, so signatures, templates, and effects need no special handling.

### Projection / BT Compilation / Verification

No changes. Every grounded instance is an ordinary action over a concrete object.

### Test Plan

- A parameter `(either uav ship)` grounds over both uav and ship objects, and over
  their subtypes.
- Overlapping unions (a type and its subtype) do not double-count objects.
- An unknown type inside `either` is rejected by the parser.
- A single-element `(either uav)` behaves identically to a bare `uav` type.

## Implementation Rule

For every new construct, preserve this invariant:

```
parsed PDDL semantics == WorldModel STRIPS projection == compiled BT execution semantics
```

If a construct cannot be represented as grounded boolean preconditions, add effects,
delete effects, and BT predicate checks/writes with matching behavior, it should not
be accepted by `PddlParser` on the LAPKT path. Reject it explicitly instead of
silently approximating it.

Recommended order:

1. Parser compatibility improvements: nested `and`, explicit negative init facts.
2. Grounding-only filters: equality and inequality between parameters.
3. Complement-fluent support: negative preconditions and negative goals.
4. Controlled normalization: disjunction and finite quantifier expansion.
5. Conditional effects only after projection and BT execution semantics are designed
   together.
