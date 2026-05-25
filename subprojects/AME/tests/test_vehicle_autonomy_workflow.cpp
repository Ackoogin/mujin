// =========================================================================
// End-to-end workflow test for the vehicle_autonomy PDDL domain via the
// authoring tool's own subsystems (no GUI). Exercises the same code paths
// the UI triggers when the user clicks File > Import PDDL, then Validate
// Now, then Plan & Preview.
//
// Pipeline under test:
//   1. PddlImporter::importDomain  — parse domain.pddl into ProjectModel
//   2. PddlImporter::importProblem — append problem as a ScenarioDef
//   3. StructuralValidator::check  — quick syntactic-level sweep
//   4. PddlValidator::validate     — round-trip the regenerated PDDL
//                                    through ame::PddlParser
//   5. PddlGenerator::generateDomain — verify round-trip stays parseable
//   6. PddlValidator::validateAndBuildWorldModel + Planner::solve
//                                  — plan a feasible scenario end-to-end
//
// Source PDDL: subprojects/AME/domains/vehicle_autonomy/{domain,problem_*}.pddl
// =========================================================================

#include <gtest/gtest.h>

#include "pddl_generator.h"
#include "pddl_importer.h"
#include "pddl_validator.h"
#include "project_model.h"
#include "structural_validator.h"

#include <ame/planner.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef AME_DOMAINS_DIR
#error "AME_DOMAINS_DIR must be defined by CMake"
#endif

namespace {

std::string readFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) { return {}; }
  std::ostringstream ss; ss << f.rdbuf();
  return ss.str();
}

std::string vehiclePath(const char* leaf) {
  return std::string(AME_DOMAINS_DIR) + "/vehicle_autonomy/" + leaf;
}

}  // namespace

TEST(VehicleAutonomyWorkflow, DomainImportsCleanly) {
  const std::string pddl = readFile(vehiclePath("domain.pddl"));
  ASSERT_FALSE(pddl.empty()) << "couldn't read domain.pddl";

  PddlImportResult result = PddlImporter::importDomain(pddl);
  ASSERT_TRUE(result.ok) << "import failed: " << result.error;
  EXPECT_EQ(result.model.projectName, "vehicle-autonomy");

  // Domain has 4 types (location, base, waypoint, robot), >= 20 predicates,
  // >= 10 actions covering the nominal/contingency ladder.
  EXPECT_GE(result.model.types.size(), 4U);
  EXPECT_GE(result.model.predicates.size(), 20U);
  EXPECT_GE(result.model.actions.size(), 10U);
}

TEST(VehicleAutonomyWorkflow, NominalProblemImportsAndValidates) {
  const std::string domainPddl = readFile(vehiclePath("domain.pddl"));
  ASSERT_FALSE(domainPddl.empty());
  PddlImportResult domain = PddlImporter::importDomain(domainPddl);
  ASSERT_TRUE(domain.ok);

  const std::string problemPddl = readFile(vehiclePath("problem_nominal.pddl"));
  ASSERT_FALSE(problemPddl.empty()) << "couldn't read problem_nominal.pddl";

  PddlImportResult problem =
      PddlImporter::importProblem(domain.model, problemPddl, "nominal");
  ASSERT_TRUE(problem.ok) << "problem import failed: " << problem.error;

  // The imported problem appends a scenario plus objects.
  ASSERT_GE(problem.model.scenarios.size(), 1U);
  EXPECT_GE(problem.model.objects.size(), 3U);
  EXPECT_FALSE(problem.model.scenarios.back().initialState.empty());
  EXPECT_FALSE(problem.model.scenarios.back().goals.empty());

  // Structural validator should report 0 errors on a clean import.
  const StructuralReport sr = StructuralValidator::check(problem.model);
  EXPECT_EQ(sr.errorCount, 0U)
      << "first structural error: "
      << (sr.issues.empty() ? "n/a" : sr.issues.front().message);
}

TEST(VehicleAutonomyWorkflow, RoundTripsThroughGenerator) {
  // Import -> regenerate -> validate ensures every authoring path keeps the
  // domain semantically intact.
  const std::string domainPddl = readFile(vehiclePath("domain.pddl"));
  ASSERT_FALSE(domainPddl.empty());
  PddlImportResult domain = PddlImporter::importDomain(domainPddl);
  ASSERT_TRUE(domain.ok);

  const std::string regenerated = PddlGenerator::generateDomain(domain.model);
  EXPECT_NE(regenerated.find("(define (domain vehicle-autonomy)"),
            std::string::npos);
  EXPECT_NE(regenerated.find(":requirements :strips :typing"),
            std::string::npos);

  // The regenerated text must still parse through PddlValidator.
  const std::string problemPddl = readFile(vehiclePath("problem_nominal.pddl"));
  ASSERT_FALSE(problemPddl.empty());
  PddlImportResult combined =
      PddlImporter::importProblem(domain.model, problemPddl, "nominal");
  ASSERT_TRUE(combined.ok);

  const ValidationReport report = PddlValidator::validate(combined.model, "nominal");
  EXPECT_TRUE(report.ok)
      << "validator failed on round-tripped domain. errors: "
      << (report.errors.empty() ? "none" : report.errors.front().message);
}

TEST(VehicleAutonomyWorkflow, NominalScenarioIsFeasible) {
  // Import the nominal scenario and verify the planner finds a plan.
  const std::string domainPddl = readFile(vehiclePath("domain.pddl"));
  const std::string problemPddl = readFile(vehiclePath("problem_nominal.pddl"));
  ASSERT_FALSE(domainPddl.empty());
  ASSERT_FALSE(problemPddl.empty());

  PddlImportResult domain = PddlImporter::importDomain(domainPddl);
  ASSERT_TRUE(domain.ok);
  PddlImportResult full =
      PddlImporter::importProblem(domain.model, problemPddl, "nominal");
  ASSERT_TRUE(full.ok);

  // Plan via the same path AppShell::runFeasibilityCheck uses.
  ame::WorldModel wm;
  ValidationReport report =
      PddlValidator::validateAndBuildWorldModel(full.model, "nominal", wm);
  ASSERT_TRUE(report.ok)
      << "validation failed: "
      << (report.errors.empty() ? "n/a" : report.errors.front().message);

  ame::Planner planner;
  const ame::PlanResult plan = planner.solve(wm);

  EXPECT_TRUE(plan.success) << "no plan: " << plan.error_msg;
  EXPECT_GT(plan.steps.size(), 0U);
}

TEST(VehicleAutonomyWorkflow, MultipleContingencyProblemsImport) {
  // Spot-check that several contingency variants also import cleanly —
  // this is the regression-pack workflow the authoring tool exposes via
  // Validate > Run All Scenarios.
  const std::string domainPddl = readFile(vehiclePath("domain.pddl"));
  ASSERT_FALSE(domainPddl.empty());
  PddlImportResult domain = PddlImporter::importDomain(domainPddl);
  ASSERT_TRUE(domain.ok);

  ProjectModel model = domain.model;
  const std::vector<const char*> problems = {
    "problem_nominal.pddl",
    "problem_engine_fail_airborne.pddl",
    "problem_comms_lost_airborne.pddl",
    "problem_nav_fail_en_route.pddl",
  };
  for (const char* leaf : problems) {
    const std::string text = readFile(vehiclePath(leaf));
    ASSERT_FALSE(text.empty()) << "couldn't read " << leaf;
    PddlImportResult r =
        PddlImporter::importProblem(model, text, std::string(leaf));
    ASSERT_TRUE(r.ok) << leaf << " import failed: " << r.error;
    model = r.model;
  }
  EXPECT_GE(model.scenarios.size(), problems.size());
}
