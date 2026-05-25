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
