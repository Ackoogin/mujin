#include <gtest/gtest.h>
#include "mujin/planner.h"
#include "mujin/world_model.h"

static mujin::WorldModel buildUAVDomain() {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("sector", "location");
    ts.addType("robot", "object");

    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.addObject("sector_a", "sector");
    wm.addObject("sector_b", "sector");

    wm.registerPredicate("at", {"robot", "location"});
    wm.registerPredicate("searched", {"sector"});
    wm.registerPredicate("classified", {"sector"});

    wm.registerAction("move",
        {"?r", "?from", "?to"}, {"robot", "location", "location"},
        {"(at ?r ?from)"}, {"(at ?r ?to)"}, {"(at ?r ?from)"});
    wm.registerAction("search",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)"}, {"(searched ?s)"}, {});
    wm.registerAction("classify",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)", "(searched ?s)"}, {"(classified ?s)"}, {});

    return wm;
}

TEST(Planner, SolvesUAVSearchProblem) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);
    EXPECT_GE(result.steps.size(), 3u);  // At least move + search + classify
}

TEST(Planner, PlanContainsRequiredActions) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);

    // Verify the plan contains a move to sector_a, a search, and a classify
    bool has_move = false, has_search = false, has_classify = false;
    for (auto& step : result.steps) {
        auto& ga = wm.groundActions()[step.action_index];
        if (ga.signature.find("move") != std::string::npos &&
            ga.signature.find("sector_a") != std::string::npos) {
            has_move = true;
        }
        if (ga.signature.find("search") != std::string::npos &&
            ga.signature.find("sector_a") != std::string::npos) {
            has_search = true;
        }
        if (ga.signature.find("classify") != std::string::npos &&
            ga.signature.find("sector_a") != std::string::npos) {
            has_classify = true;
        }
    }

    EXPECT_TRUE(has_move) << "Plan should contain move to sector_a";
    EXPECT_TRUE(has_search) << "Plan should contain search at sector_a";
    EXPECT_TRUE(has_classify) << "Plan should contain classify at sector_a";
}

TEST(Planner, UnsolvableProblemReturnsFalse) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    wm.addObject("a", "item");
    wm.registerPredicate("done", {"item"});
    // No actions registered — goal is unreachable
    wm.setGoal({"(done a)"});

    mujin::Planner planner;
    auto result = planner.solve(wm);

    EXPECT_FALSE(result.success);
}

TEST(Planner, AlreadySatisfiedGoal) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setFact("(searched sector_a)", true);
    wm.setFact("(classified sector_a)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.steps.size(), 0u);  // Goal already satisfied
}

TEST(Planner, ReplanAfterStateChange) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)"});

    mujin::Planner planner;
    auto result1 = planner.solve(wm);
    ASSERT_TRUE(result1.success);

    // Simulate: UAV has already moved to sector_a
    wm.setFact("(at uav1 base)", false);
    wm.setFact("(at uav1 sector_a)", true);

    // Replan — should now just need search (no move)
    auto result2 = planner.solve(wm);
    ASSERT_TRUE(result2.success);
    EXPECT_LT(result2.steps.size(), result1.steps.size());

    // The plan should contain search but not necessarily a move
    bool has_search = false;
    for (auto& step : result2.steps) {
        auto& ga = wm.groundActions()[step.action_index];
        if (ga.signature.find("search") != std::string::npos &&
            ga.signature.find("sector_a") != std::string::npos) {
            has_search = true;
        }
    }
    EXPECT_TRUE(has_search);
}
