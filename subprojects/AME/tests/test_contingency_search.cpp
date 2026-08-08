#include <gtest/gtest.h>

#include "ame/contingency_search.h"
#include "ame/pddl_parser.h"

#include <algorithm>
#include <string>

namespace {

/// A domain with two context facts. The vehicle can reach the recovery point
/// by its primary route when the primary route is available, or by the backup
/// route when the backup is available. Neither route can be brought about by
/// planning, which is what makes them context.
const char* kDomain = R"pddl(
(define (domain recovery)
  (:requirements :strips :typing)
  (:types robot location - object)
  (:predicates
    (at ?r - robot ?l - location)
    (primary-open)
    (backup-open)
    (recovered ?r - robot)
  )
  (:action recover-by-primary
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (primary-open))
    :effect (and (recovered ?r) (at ?r ?to) (not (at ?r ?from)))
  )
  (:action recover-by-backup
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (backup-open))
    :effect (and (recovered ?r) (at ?r ?to) (not (at ?r ?from)))
  )
)
)pddl";

const char* kProblem = R"pddl(
(define (problem recovery-nominal)
  (:domain recovery)
  (:objects uav1 - robot field base - location)
  (:init (at uav1 field) (primary-open) (backup-open))
  (:goal (and (recovered uav1)))
)
)pddl";

/// The same domain, with a route that no context fact gates, so every
/// combination can reach the goal.
const char* kAlwaysRecoverableDomain = R"pddl(
(define (domain always-recovers)
  (:requirements :strips :typing)
  (:types robot location - object)
  (:predicates
    (at ?r - robot ?l - location)
    (primary-open)
    (recovered ?r - robot)
  )
  (:action recover-by-primary
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (primary-open))
    :effect (and (recovered ?r) (at ?r ?to) (not (at ?r ?from)))
  )
  (:action walk-back
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at ?r ?from)
    :effect (and (recovered ?r) (at ?r ?to) (not (at ?r ?from)))
  )
)
)pddl";

const char* kAlwaysRecoverableProblem = R"pddl(
(define (problem always-nominal)
  (:domain always-recovers)
  (:objects uav1 - robot field base - location)
  (:init (at uav1 field) (primary-open))
  (:goal (and (recovered uav1)))
)
)pddl";

const ame::ContextFact* findFact(const ame::ContingencySearchReport& report,
                                 const std::string& shortName) {
  const auto it = std::find_if(
      report.context_facts.begin(), report.context_facts.end(),
      [&shortName](const ame::ContextFact& fact) {
        return fact.short_name == shortName;
      });
  return it == report.context_facts.end() ? nullptr : &(*it);
}

/// The combination in which every named fact is true.
unsigned combinationWithAll(const ame::ContingencySearchReport& report) {
  return (1u << report.context_facts.size()) - 1u;
}

}  // namespace

TEST(ContingencySearch, FindsOnlyTheFactsNoActionCanChange) {
  ame::WorldModel wm;
  ame::PddlParser::parseFromString(kDomain, kProblem, wm);

  const auto facts = ame::ContingencySearch::identifyContextFacts(wm);

  ASSERT_EQ(facts.size(), 2U);
  const bool has_primary =
      std::any_of(facts.begin(), facts.end(), [](const ame::ContextFact& f) {
        return f.short_name == "primary-open";
      });
  const bool has_backup =
      std::any_of(facts.begin(), facts.end(), [](const ame::ContextFact& f) {
        return f.short_name == "backup-open";
      });
  EXPECT_TRUE(has_primary);
  EXPECT_TRUE(has_backup);

  // "recovered" and "at" are produced by actions, so they are not context.
  for (const auto& fact : facts) {
    EXPECT_EQ(fact.fluent_name.find("recovered"), std::string::npos);
    EXPECT_EQ(fact.fluent_name.find("(at "), std::string::npos);
  }
}

TEST(ContingencySearch, EveryCombinationIsAccountedFor) {
  ame::ContingencySearchOptions options;
  const ame::ContingencySearchReport report =
      ame::ContingencySearch::run(kDomain, kProblem, options);

  ASSERT_EQ(report.context_facts.size(), 2U);
  EXPECT_EQ(report.cases.size(), 4U);
  for (unsigned i = 0; i < report.cases.size(); ++i) {
    EXPECT_EQ(report.cases[i].combination, i);
  }
  EXPECT_EQ(report.reachableCount() + report.unreachableCount(),
            report.cases.size());
}

TEST(ContingencySearch, LosingEveryRouteIsReportedAsUnreachable) {
  ame::ContingencySearchOptions exhaustive;
  exhaustive.prune = false;
  const ame::ContingencySearchReport report =
      ame::ContingencySearch::run(kDomain, kProblem, exhaustive);

  // Combination 0 is every context fact false: no route is open.
  EXPECT_FALSE(report.cases[0].reachable());
  EXPECT_FALSE(report.everyCombinationReachable());

  // With both routes open the goal is reachable, and a plan is reported.
  // Pruning is off here on purpose: a combination settled by carrying a
  // conclusion from another one has no plan of its own, which the next test
  // covers.
  const unsigned all = combinationWithAll(report);
  EXPECT_TRUE(report.cases[all].reachable());
  EXPECT_GT(report.cases[all].plan_length, 0U);
  EXPECT_FALSE(report.cases[all].full_plan.empty());
}

TEST(ContingencySearch, ACarriedConclusionReportsNoPlanOfItsOwn) {
  const ame::ContingencySearchReport report =
      ame::ContingencySearch::run(kDomain, kProblem, {});

  // A combination nobody planned for is reported as reachable without a plan,
  // because the plan that settles it belongs to the combination it was carried
  // from. Anything reading these results has to say "implied by" rather than
  // invent a plan for it.
  const auto implied = std::find_if(
      report.cases.begin(), report.cases.end(),
      [](const ame::ContingencyCase& c) {
        return c.outcome == ame::ContingencyOutcome::ImpliedReachable;
      });
  ASSERT_NE(implied, report.cases.end());
  EXPECT_TRUE(implied->reachable());
  EXPECT_EQ(implied->plan_length, 0U);
  EXPECT_TRUE(implied->full_plan.empty());
  EXPECT_TRUE(implied->has_implied_by);
  EXPECT_TRUE(report.cases[implied->implied_by].reachable());
}

TEST(ContingencySearch, EitherRouteOnItsOwnIsEnough) {
  const ame::ContingencySearchReport report =
      ame::ContingencySearch::run(kDomain, kProblem, {});
  const ame::ContextFact* primary = findFact(report, "primary-open");
  const ame::ContextFact* backup = findFact(report, "backup-open");
  ASSERT_NE(primary, nullptr);
  ASSERT_NE(backup, nullptr);

  const auto indexOf = [&report](const std::string& shortName) {
    for (size_t i = 0; i < report.context_facts.size(); ++i) {
      if (report.context_facts[i].short_name == shortName) {
        return static_cast<unsigned>(i);
      }
    }
    return 0U;
  };
  const unsigned primary_only = 1u << indexOf("primary-open");
  const unsigned backup_only = 1u << indexOf("backup-open");

  EXPECT_TRUE(report.cases[primary_only].reachable());
  EXPECT_TRUE(report.cases[backup_only].reachable());
}

TEST(ContingencySearch, CarryingConclusionsAgreesWithSolvingEveryCombination) {
  ame::ContingencySearchOptions pruned;
  pruned.prune = true;
  ame::ContingencySearchOptions exhaustive;
  exhaustive.prune = false;

  const ame::ContingencySearchReport with_pruning =
      ame::ContingencySearch::run(kDomain, kProblem, pruned);
  const ame::ContingencySearchReport without_pruning =
      ame::ContingencySearch::run(kDomain, kProblem, exhaustive);

  ASSERT_EQ(with_pruning.cases.size(), without_pruning.cases.size());
  for (size_t i = 0; i < with_pruning.cases.size(); ++i) {
    EXPECT_EQ(with_pruning.cases[i].reachable(),
              without_pruning.cases[i].reachable())
        << "combination " << i << " disagrees";
  }

  // Pruning exists to save planner calls, so it must actually save some.
  EXPECT_LT(with_pruning.solver_calls, without_pruning.solver_calls);
  EXPECT_EQ(without_pruning.solver_calls, without_pruning.cases.size());
  EXPECT_TRUE(with_pruning.pruning_used);
  EXPECT_FALSE(without_pruning.pruning_used);
}

TEST(ContingencySearch, AConclusionCarriedOverNamesWhereItCameFrom) {
  const ame::ContingencySearchReport report =
      ame::ContingencySearch::run(kDomain, kProblem, {});

  const auto implied = std::find_if(
      report.cases.begin(), report.cases.end(),
      [](const ame::ContingencyCase& c) { return c.has_implied_by; });
  ASSERT_NE(implied, report.cases.end())
      << "expected at least one combination to be settled without planning";
  EXPECT_NE(implied->implied_by, implied->combination);
  EXPECT_LT(implied->implied_by, report.cases.size());
}

TEST(ContingencySearch, ADomainThatAlwaysRecoversHasNoGaps) {
  const ame::ContingencySearchReport report = ame::ContingencySearch::run(
      kAlwaysRecoverableDomain, kAlwaysRecoverableProblem, {});

  EXPECT_TRUE(report.everyCombinationReachable());
  EXPECT_EQ(report.unreachableCount(), 0U);
}

TEST(ContingencySearch, PruningIsSoundOnlyWithoutNegativePreconditions) {
  ame::WorldModel wm;
  ame::PddlParser::parseFromString(kDomain, kProblem, wm);
  EXPECT_TRUE(ame::ContingencySearch::pruningIsSound(wm));
}

TEST(ContingencySearch, TheLabelReadsAsTheFactsBeingOnOrOff) {
  const ame::ContingencySearchReport report =
      ame::ContingencySearch::run(kDomain, kProblem, {});

  const std::string none =
      ame::ContingencySearch::combinationLabel(report.context_facts, 0);
  const std::string all = ame::ContingencySearch::combinationLabel(
      report.context_facts, combinationWithAll(report));

  EXPECT_NE(none.find("=off"), std::string::npos);
  EXPECT_EQ(none.find("=ON"), std::string::npos);
  EXPECT_NE(all.find("=ON"), std::string::npos);
  EXPECT_EQ(all.find("=off"), std::string::npos);
}

TEST(ContingencySearch, AGoalTheCallerSuppliesOverridesTheProblemGoal) {
  ame::ContingencySearchOptions options;
  options.goal_options = {{"(at uav1 base)"}};
  // Solve every combination, so the one being checked reports the goal it
  // reached rather than carrying a conclusion from elsewhere.
  options.prune = false;

  const ame::ContingencySearchReport report =
      ame::ContingencySearch::run(kDomain, kProblem, options);

  ASSERT_FALSE(report.goal_options.empty());
  EXPECT_EQ(report.goal_options.front().front(), "(at uav1 base)");
  const unsigned all = combinationWithAll(report);
  EXPECT_TRUE(report.cases[all].reachable());
  EXPECT_NE(report.cases[all].selected_goal.find("(at uav1 base)"),
            std::string::npos);
}
