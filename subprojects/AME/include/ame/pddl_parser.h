#pragma once

#include <string>

namespace ame {

class WorldModel;

// STRIPS-level PDDL parser with a finite ADL subset that is compiled away
// before LAPKT (see doc/architecture/09-pddl-subset-with-lapkt.md).
// Parses domain.pddl and problem.pddl files into a WorldModel.
//
// Supports:
//   - :strips and :typing requirements; :types, :constants, :predicates,
//     :action, :objects, :init, and :goal sections.
//   - Action preconditions: positive atoms, arbitrarily nested `(and ...)`,
//     `(or ...)` (lowered to disjunctive normal form -- one schema per disjunct),
//     `(not ATOM)` negative preconditions (single negated atom only),
//     `(= a b)` / `(not (= a b))` equality binding filters, and finite
//     `(forall (?x - T) ...)` / `(exists (?x - T) ...)` quantifiers expanded
//     over the known object set.
//   - Action parameters with `(either t1 t2 ...)` union types.
//   - Effects: positive add effects, `(not atom)` delete effects, nested `and`.
//   - Goals: positive conjunctions and top-level `(or ...)` disjunctive goals
//     (any alternative counts as success). Goals remain negation-free.
//
// Does not support: negative goals, equality/quantifiers/`either` in effects or
// goals, conditional effects, numeric fluents, temporal PDDL, or durative
// actions. These are rejected explicitly rather than silently approximated.
class PddlParser {
public:
    // Parse a domain file and problem file, populating the given WorldModel.
    // Throws std::runtime_error on parse errors.
    static void parse(const std::string& domain_path,
                      const std::string& problem_path,
                      WorldModel& wm);

    // Parse from string content (for testing without files)
    static void parseFromString(const std::string& domain_pddl,
                                const std::string& problem_pddl,
                                WorldModel& wm);
};

} // namespace ame
