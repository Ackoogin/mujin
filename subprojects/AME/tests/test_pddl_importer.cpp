#include <gtest/gtest.h>

#include "pddl_generator.h"
#include "pddl_importer.h"

#include <string>

namespace {

const char* kUavSearchDomainPddl = R"pddl(
(define (domain uav-search)
  (:requirements :strips :typing)

  (:types
    location - object
    sector - location
    robot - object
  )

  (:predicates
    (at ?r - robot ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at ?r ?from)
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  (:action search
    :parameters (?r - robot ?s - sector)
    :precondition (at ?r ?s)
    :effect (searched ?s)
  )

  (:action classify
    :parameters (?r - robot ?s - sector)
    :precondition (and
      (at ?r ?s)
      (searched ?s)
    )
    :effect (classified ?s)
  )
)
)pddl";

const char* kUavSearchProblemPddl = R"pddl(
(define (problem uav-search-1)
  (:domain uav-search)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 base)
  )

  (:goal (and
    (searched sector_a)
    (classified sector_a)
  ))
)
)pddl";

const char* kDomainWithConstantsPddl = R"pddl(
(define (domain constants-demo)
  (:requirements :strips :typing)

  (:types
    mission priority - object
  )

  (:constants
    routine critical - priority
  )

  (:predicates
    (has-priority ?m - mission ?p - priority)
  )

  (:action mark-critical
    :parameters (?m - mission)
    :precondition (has-priority ?m critical)
    :effect (has-priority ?m routine)
  )
)
)pddl";

const char* kDomainWithConfirmedPredicatesPddl = R"pddl(
(define (domain strike)
  (:requirements :strips :typing)

  (:types
    target - object
    uav - object
  )

  (:predicates
    (airborne ?u - uav)
    (authorised ?t - target)
    (route-clear ?u - uav)
    (struck ?t - target)
  )

  (:confirmed-predicates authorised route-clear)

  (:action strike
    :parameters (?u - uav ?t - target)
    :precondition (and
      (airborne ?u)
      (authorised ?t)
      (route-clear ?u)
    )
    :effect (struck ?t)
  )
)
)pddl";

bool confirmedFlagOf(const ProjectModel& model, const std::string& name) {
  for (const auto& predicate : model.predicates) {
    if (predicate.name == name) {
      return predicate.confirmed;
    }
  }
  ADD_FAILURE() << "no predicate named " << name;
  return false;
}

} // namespace

TEST(PddlImporter, ImportsUavSearchDomain) {
  const PddlImportResult result = PddlImporter::importDomain(kUavSearchDomainPddl);

  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.model.projectName, "uav-search");
  EXPECT_GE(result.model.types.size(), 3U);
  EXPECT_EQ(result.model.predicates.size(), 3U);
  EXPECT_EQ(result.model.actions.size(), 3U);
  ASSERT_EQ(result.model.actions.size(), 3U);
  EXPECT_EQ(result.model.actions[0].name, "move");
  EXPECT_EQ(result.model.actions[0].addEffects.size(), 1U);
  EXPECT_EQ(result.model.actions[0].delEffects.size(), 1U);
}

TEST(PddlImporter, ImportsUavSearchProblem) {
  const PddlImportResult domain = PddlImporter::importDomain(kUavSearchDomainPddl);
  ASSERT_TRUE(domain.ok) << domain.error;

  const PddlImportResult problem =
      PddlImporter::importProblem(domain.model, kUavSearchProblemPddl);

  EXPECT_TRUE(problem.ok) << problem.error;
  EXPECT_TRUE(problem.error.empty());
  EXPECT_GE(problem.model.objects.size(), 4U);
  ASSERT_EQ(problem.model.scenarios.size(), 1U);
  EXPECT_EQ(problem.model.scenarios.back().name, "uav-search-1");
  EXPECT_EQ(problem.model.scenarios.back().initialState.size(), 1U);
  EXPECT_GE(problem.model.scenarios.back().goals.size(), 1U);
}

TEST(PddlImporter, ImportsDomainConstantsAsObjects) {
  const PddlImportResult result =
      PddlImporter::importDomain(kDomainWithConstantsPddl);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.model.objects.size(), 2U);
  EXPECT_EQ(result.model.objects[0].name, "routine");
  EXPECT_EQ(result.model.objects[0].type, "priority");
  EXPECT_EQ(result.model.objects[1].name, "critical");
  EXPECT_EQ(result.model.objects[1].type, "priority");
}

TEST(PddlImporter, RejectsMalformedPddl) {
  const PddlImportResult result = PddlImporter::importDomain("(garbage");

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST(PddlImporter, RoundTripDomain) {
  const PddlImportResult first = PddlImporter::importDomain(kUavSearchDomainPddl);
  ASSERT_TRUE(first.ok) << first.error;

  const std::string generated = PddlGenerator::generateDomain(first.model);
  const PddlImportResult second = PddlImporter::importDomain(generated);

  ASSERT_TRUE(second.ok) << second.error;
  EXPECT_EQ(second.model.types.size(), first.model.types.size());
  EXPECT_EQ(second.model.predicates.size(), first.model.predicates.size());
  EXPECT_EQ(second.model.actions.size(), first.model.actions.size());
}

TEST(PddlImporter, ImportsConfirmedPredicateDeclarations) {
  const PddlImportResult result =
      PddlImporter::importDomain(kDomainWithConfirmedPredicatesPddl);

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(confirmedFlagOf(result.model, "authorised"));
  EXPECT_TRUE(confirmedFlagOf(result.model, "route-clear"));
  EXPECT_FALSE(confirmedFlagOf(result.model, "airborne"));
  EXPECT_FALSE(confirmedFlagOf(result.model, "struck"));
}

// An import followed by a generate used to lose the declaration, which turned an
// action that had to wait for observed state into one that accepted predicted
// state, with nothing reported.
TEST(PddlImporter, RoundTripKeepsConfirmedPredicates) {
  const PddlImportResult first =
      PddlImporter::importDomain(kDomainWithConfirmedPredicatesPddl);
  ASSERT_TRUE(first.ok) << first.error;

  const std::string generated = PddlGenerator::generateDomain(first.model);
  EXPECT_NE(generated.find("(:confirmed-predicates"), std::string::npos)
      << generated;

  const PddlImportResult second = PddlImporter::importDomain(generated);
  ASSERT_TRUE(second.ok) << second.error;
  EXPECT_TRUE(confirmedFlagOf(second.model, "authorised"));
  EXPECT_TRUE(confirmedFlagOf(second.model, "route-clear"));
  EXPECT_FALSE(confirmedFlagOf(second.model, "airborne"));
  EXPECT_FALSE(confirmedFlagOf(second.model, "struck"));
}

TEST(PddlImporter, RejectsConfirmedPredicateThatIsNotDeclared) {
  const PddlImportResult result = PddlImporter::importDomain(R"pddl(
(define (domain typo)
  (:predicates (authorised ?t - target))
  (:confirmed-predicates authorized)
)
)pddl");

  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("authorized"), std::string::npos) << result.error;
}

// A domain with no declaration must not gain an empty section, which the core
// parser would accept but which would read as a decision nobody made.
TEST(PddlGeneratedDomain, OmitsConfirmedSectionWhenNoPredicateIsConfirmed) {
  const PddlImportResult imported =
      PddlImporter::importDomain(kUavSearchDomainPddl);
  ASSERT_TRUE(imported.ok) << imported.error;

  const std::string generated = PddlGenerator::generateDomain(imported.model);
  EXPECT_EQ(generated.find("(:confirmed-predicates"), std::string::npos)
      << generated;
}
