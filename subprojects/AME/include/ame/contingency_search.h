#pragma once

#include "ame/world_model.h"

#include <string>
#include <vector>

namespace ame {

/// \brief A grounded fact that gates actions but no plan can change.
///
/// These are the facts a domain treats as context: hardware health, sensor
/// status, whether the communications link is up, the weather. They appear in
/// at least one action's preconditions and in no action's effects, so within a
/// single planning cycle they are fixed, and a deployed system learns their
/// values by observation rather than by planning.
struct ContextFact {
    unsigned fluent_id = 0;
    std::string fluent_name;   // "(comms-available)"
    std::string short_name;    // "comms-available"
    bool initial_value = false;
};

/// \brief What the search concluded about one combination of context facts.
///
/// The two "implied" outcomes are conclusions drawn from another combination
/// rather than from a planner call. They are sound only because the context
/// facts appear as positive preconditions: see ContingencySearch::pruningIsSound.
enum class ContingencyOutcome {
    Reachable,
    Unreachable,
    ImpliedReachable,
    ImpliedUnreachable,
};

/// \brief One combination of context facts, and what the search found.
struct ContingencyCase {
    unsigned combination = 0;
    ContingencyOutcome outcome = ContingencyOutcome::Unreachable;
    unsigned plan_length = 0;
    double solve_time_ms = 0.0;
    std::string selected_goal;
    std::string initial_state;
    std::string end_state;
    std::string action_names;
    std::string full_plan;

    /// The combination this conclusion was drawn from, for the two implied
    /// outcomes. Meaningless unless has_implied_by is true.
    unsigned implied_by = 0;
    bool has_implied_by = false;

    bool reachable() const {
        return outcome == ContingencyOutcome::Reachable ||
               outcome == ContingencyOutcome::ImpliedReachable;
    }
};

/// \brief How the caller wants the search run.
struct ContingencySearchOptions {
    /// Draw conclusions about combinations that follow from ones already
    /// solved, instead of calling the planner for every combination. Ignored,
    /// with the report saying so, when the domain makes it unsound.
    bool prune = true;

    /// Acceptable end states, each an alternative. The first that can be
    /// reached is the one reported. Leave empty to use the problem's own goal.
    std::vector<std::vector<std::string>> goal_options;

    /// Vary only the facts of these predicates, rather than every fact the
    /// plan cannot change. Leave empty to vary all of them. This is how a user
    /// says which facts represent a contingency worth checking, when the
    /// domain contains context facts that are not one.
    std::vector<std::string> only_predicates;
};

/// \brief Everything one search found.
struct ContingencySearchReport {
    std::vector<ContextFact> context_facts;
    std::vector<std::vector<std::string>> goal_options;
    /// One entry per combination, indexed by the combination itself.
    std::vector<ContingencyCase> cases;

    unsigned solver_calls = 0;
    unsigned implied_reachable = 0;
    unsigned implied_unreachable = 0;
    double wall_time_ms = 0.0;

    bool pruning_requested = false;
    bool pruning_used = false;
    /// True when pruning was asked for and refused because the domain has
    /// negative preconditions, which break the reasoning it depends on.
    bool pruning_unsound_for_domain = false;

    size_t reachableCount() const;
    size_t unreachableCount() const;
    bool everyCombinationReachable() const;
};

/// \brief Exhaustive safe-state reachability over a domain's context facts.
///
/// The search enumerates every combination of the facts the plan cannot
/// change, and asks the planner whether an acceptable end state can still be
/// reached from each one. It is the evidence behind a claim that a mission
/// stays recoverable however its equipment is behaving.
///
/// This is shared deliberately. The contingency verifier runs it from the
/// command line over PDDL files, the authoring tool runs it over a project the
/// user is editing, and the generated assurance report quotes its results. One
/// algorithm means those three cannot disagree.
class ContingencySearch {
public:
    /// \brief The facts a plan cannot change: in some precondition, in no effect.
    static std::vector<ContextFact> identifyContextFacts(const WorldModel& wm);

    /// \brief Whether conclusions may be carried between combinations.
    ///
    /// Carrying a conclusion from one combination to another relies on adding
    /// a context fact never taking an action away. A negative precondition
    /// breaks that: a fact being true can stop an action applying. A domain
    /// with any negative precondition must therefore have every combination
    /// solved directly.
    static bool pruningIsSound(const WorldModel& wm);

    /// \brief The combination written out as "name=ON  other=off".
    static std::string combinationLabel(const std::vector<ContextFact>& facts,
                                        unsigned combination);

    /// \brief Run the search over a domain and a template problem.
    ///
    /// The template problem supplies the structural starting state, which is
    /// held fixed while the context facts are varied.
    static ContingencySearchReport run(const std::string& domain_pddl,
                                       const std::string& problem_pddl,
                                       const ContingencySearchOptions& options);
};

}  // namespace ame
