#include <gtest/gtest.h>
#include "mujin/pddl_parser.h"
#include "mujin/world_model.h"

static const char* UAV_DOMAIN = R"(
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
    :effect (and (at ?r ?to) (not (at ?r ?from)))
  )
  (:action search
    :parameters (?r - robot ?s - sector)
    :precondition (at ?r ?s)
    :effect (searched ?s)
  )
  (:action classify
    :parameters (?r - robot ?s - sector)
    :precondition (and (at ?r ?s) (searched ?s))
    :effect (classified ?s)
  )
)
)";

static const char* UAV_PROBLEM = R"(
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
)";

TEST(PddlParser, ParseTypes) {
    mujin::WorldModel wm;
    mujin::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    auto& ts = wm.typeSystem();
    EXPECT_TRUE(ts.hasType("object"));
    EXPECT_TRUE(ts.hasType("location"));
    EXPECT_TRUE(ts.hasType("sector"));
    EXPECT_TRUE(ts.hasType("robot"));
    EXPECT_TRUE(ts.isSubtype("sector", "location"));
    EXPECT_TRUE(ts.isSubtype("location", "object"));
}

TEST(PddlParser, ParseObjects) {
    mujin::WorldModel wm;
    mujin::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    auto& ts = wm.typeSystem();
    EXPECT_TRUE(ts.hasObject("uav1"));
    EXPECT_TRUE(ts.hasObject("base"));
    EXPECT_TRUE(ts.hasObject("sector_a"));
    EXPECT_TRUE(ts.hasObject("sector_b"));
    EXPECT_EQ(ts.getObjectType("uav1"), "robot");
    EXPECT_EQ(ts.getObjectType("base"), "location");
    EXPECT_EQ(ts.getObjectType("sector_a"), "sector");
}

TEST(PddlParser, ParsePredicates) {
    mujin::WorldModel wm;
    mujin::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    // at(robot, location): 1 robot x 3 locations = 3
    // searched(sector): 2
    // classified(sector): 2
    // Total: 7
    EXPECT_EQ(wm.numFluents(), 7u);
}

TEST(PddlParser, ParseInitialState) {
    mujin::WorldModel wm;
    mujin::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
    EXPECT_FALSE(wm.getFact("(at uav1 sector_a)"));
    EXPECT_FALSE(wm.getFact("(searched sector_a)"));
}

TEST(PddlParser, ParseGoal) {
    mujin::WorldModel wm;
    mujin::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    auto& goals = wm.goalFluentIds();
    EXPECT_EQ(goals.size(), 2u);

    // Goals should be (searched sector_a) and (classified sector_a)
    bool has_searched = false, has_classified = false;
    for (auto id : goals) {
        if (wm.fluentName(id) == "(searched sector_a)") has_searched = true;
        if (wm.fluentName(id) == "(classified sector_a)") has_classified = true;
    }
    EXPECT_TRUE(has_searched);
    EXPECT_TRUE(has_classified);
}

TEST(PddlParser, ParseActions) {
    mujin::WorldModel wm;
    mujin::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    // move: 1 robot x 3 locations x 3 locations = 9
    // search: 1 robot x 2 sectors = 2
    // classify: 1 robot x 2 sectors = 2
    EXPECT_EQ(wm.numGroundActions(), 13u);
}

TEST(PddlParser, MatchesProgrammaticConstruction) {
    // Parse from PDDL
    mujin::WorldModel wm_parsed;
    mujin::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm_parsed);

    // Build programmatically (same domain)
    mujin::WorldModel wm_prog;
    auto& ts = wm_prog.typeSystem();
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("sector", "location");
    ts.addType("robot", "object");

    wm_prog.addObject("uav1", "robot");
    wm_prog.addObject("base", "location");
    wm_prog.addObject("sector_a", "sector");
    wm_prog.addObject("sector_b", "sector");

    wm_prog.registerPredicate("at", {"robot", "location"});
    wm_prog.registerPredicate("searched", {"sector"});
    wm_prog.registerPredicate("classified", {"sector"});

    wm_prog.registerAction("move",
        {"?r", "?from", "?to"}, {"robot", "location", "location"},
        {"(at ?r ?from)"}, {"(at ?r ?to)"}, {"(at ?r ?from)"});
    wm_prog.registerAction("search",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)"}, {"(searched ?s)"}, {});
    wm_prog.registerAction("classify",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)", "(searched ?s)"}, {"(classified ?s)"}, {});

    wm_prog.setFact("(at uav1 base)", true);
    wm_prog.setGoal({"(searched sector_a)", "(classified sector_a)"});

    // Compare
    EXPECT_EQ(wm_parsed.numFluents(), wm_prog.numFluents());
    EXPECT_EQ(wm_parsed.numGroundActions(), wm_prog.numGroundActions());

    // Verify same fluents exist (by name)
    for (unsigned i = 0; i < wm_parsed.numFluents(); ++i) {
        const auto& name = wm_parsed.fluentName(i);
        EXPECT_NO_THROW(wm_prog.fluentIndex(name)) << "Missing fluent: " << name;
    }

    // Verify same initial state
    for (unsigned i = 0; i < wm_parsed.numFluents(); ++i) {
        const auto& name = wm_parsed.fluentName(i);
        bool parsed_val = wm_parsed.getFact(i);
        bool prog_val = wm_prog.getFact(name);
        EXPECT_EQ(parsed_val, prog_val) << "Mismatch for " << name;
    }
}

TEST(PddlParser, ParseFromFile) {
    mujin::WorldModel wm;
    // Use the actual PDDL files
    std::string domain_path = std::string(PROJECT_SOURCE_DIR) + "/domains/uav_search/domain.pddl";
    std::string problem_path = std::string(PROJECT_SOURCE_DIR) + "/domains/uav_search/problem.pddl";
    mujin::PddlParser::parse(domain_path, problem_path, wm);

    EXPECT_EQ(wm.numFluents(), 7u);
    EXPECT_EQ(wm.numGroundActions(), 13u);
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
}
