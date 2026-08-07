#include <gtest/gtest.h>
#include "ame/planner.h"
#include "ame/world_model.h"

#include <stdexcept>

static ame::WorldModel buildUAVDomain() {
    ame::WorldModel wm;
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

    ame::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);
    EXPECT_GE(result.steps.size(), 3u);  // At least move + search + classify
}

TEST(Planner, PlanContainsRequiredActions) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    ame::Planner planner;
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
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    wm.addObject("a", "item");
    wm.registerPredicate("done", {"item"});
    // No actions registered -- goal is unreachable
    wm.setGoal({"(done a)"});

    ame::Planner planner;
    auto result = planner.solve(wm);

    EXPECT_FALSE(result.success);
}

TEST(Planner, EmptyGoalSetReturnsFalse) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);

    ame::Planner planner;
    auto result = planner.solve(wm, {});

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.error_msg.find("empty goal set"), std::string::npos);
}

TEST(Planner, AlreadySatisfiedGoal) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setFact("(searched sector_a)", true);
    wm.setFact("(classified sector_a)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    ame::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.steps.size(), 0u);  // Goal already satisfied
}

TEST(Planner, ReplanAfterStateChange) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)"});

    ame::Planner planner;
    auto result1 = planner.solve(wm);
    ASSERT_TRUE(result1.success);

    // Simulate: UAV has already moved to sector_a
    wm.setFact("(at uav1 base)", false);
    wm.setFact("(at uav1 sector_a)", true);

    // Replan -- should now just need search (no move)
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

// =========================================================================
// Negative preconditions through the LAPKT complement-fluent projection
// =========================================================================

// Domain where reaching the goal requires an action gated by a negative
// precondition (launch needs the platform to be NOT safe).
static ame::WorldModel buildNegPreDomain() {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");

    wm.registerPredicate("armed", {"robot"});
    wm.registerPredicate("safe", {"robot"});
    wm.registerPredicate("launched", {"robot"});

    // arm: requires safe and NOT armed -> sets armed
    wm.registerAction("arm",
        {"?r"}, {"robot"},
        /*pre*/{"(safe ?r)"}, /*neg_pre*/{"(armed ?r)"},
        /*add*/{"(armed ?r)"}, /*del*/{});
    // make_unsafe: requires safe -> clears safe
    wm.registerAction("make_unsafe",
        {"?r"}, {"robot"},
        /*pre*/{"(safe ?r)"}, /*neg_pre*/{},
        /*add*/{}, /*del*/{"(safe ?r)"});
    // launch: requires armed and NOT safe -> sets launched
    wm.registerAction("launch",
        {"?r"}, {"robot"},
        /*pre*/{"(armed ?r)"}, /*neg_pre*/{"(safe ?r)"},
        /*add*/{"(launched ?r)"}, /*del*/{});

    return wm;
}

TEST(PlannerNegPre, SolvesPlanRequiringNegativePrecondition) {
    auto wm = buildNegPreDomain();
    wm.setFact("(safe uav1)", true);
    wm.setGoal({"(launched uav1)"});

    ame::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);

    // Expected order: arm (while safe & not armed), make_unsafe, launch.
    int idx_arm = -1, idx_unsafe = -1, idx_launch = -1;
    for (int i = 0; i < static_cast<int>(result.steps.size()); ++i) {
        const auto& sig = wm.groundActions()[result.steps[i].action_index].signature;
        if (sig == "arm(uav1)") idx_arm = i;
        if (sig == "make_unsafe(uav1)") idx_unsafe = i;
        if (sig == "launch(uav1)") idx_launch = i;
    }
    ASSERT_GE(idx_arm, 0);
    ASSERT_GE(idx_unsafe, 0);
    ASSERT_GE(idx_launch, 0);
    EXPECT_LT(idx_arm, idx_launch);
    EXPECT_LT(idx_unsafe, idx_launch);
    // arm must run before the platform is made unsafe (arm needs safe true).
    EXPECT_LT(idx_arm, idx_unsafe);
}

TEST(PlannerNegPre, ActionBlockedWhenNegativeFactIsTrue) {
    // prepare requires (not armed); when armed is already true the only path to
    // the goal is blocked and no plan exists.
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.registerPredicate("armed", {"robot"});
    wm.registerPredicate("ready", {"robot"});
    wm.registerAction("prepare",
        {"?r"}, {"robot"},
        /*pre*/{}, /*neg_pre*/{"(armed ?r)"},
        /*add*/{"(ready ?r)"}, /*del*/{});

    wm.setFact("(armed uav1)", true);
    wm.setGoal({"(ready uav1)"});

    ame::Planner planner;
    auto blocked = planner.solve(wm);
    EXPECT_FALSE(blocked.success);

    // With armed false the same domain is solvable.
    ame::WorldModel wm2 = wm;
    wm2.setFact("(armed uav1)", false);
    auto ok = ame::Planner{}.solve(wm2);
    EXPECT_TRUE(ok.success);
    ASSERT_EQ(ok.steps.size(), 1u);
    EXPECT_EQ(wm2.groundActions()[ok.steps[0].action_index].signature, "prepare(uav1)");
}

TEST(PlannerNegPre, GroundingFailsClosedOnUnresolvableNegTemplate) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.registerPredicate("armed", {"robot"});
    // Negative precondition references an unknown predicate -> must throw.
    EXPECT_THROW(
        wm.registerAction("bad",
            {"?r"}, {"robot"},
            /*pre*/{}, /*neg_pre*/{"(nonexistent ?r)"},
            /*add*/{"(armed ?r)"}, /*del*/{}),
        std::runtime_error);
}

// =========================================================================
// Newly planned extensions: end-to-end planning
// =========================================================================

#include "ame/pddl_parser.h"

// Disjunctive precondition: a goal reachable through either branch is solvable,
// and the chosen ground action carries a base-name-recoverable signature.
TEST(PlannerExtensions, DisjunctivePreconditionSolvable) {
    const char* domain = R"(
(define (domain disj)
  (:requirements :strips :typing)
  (:types robot - object)
  (:predicates (keyed ?r - robot) (carded ?r - robot) (in ?r - robot))
  (:action enter
    :parameters (?r - robot)
    :precondition (or (keyed ?r) (carded ?r))
    :effect (in ?r)
  )
)
)";
    const char* problem = R"(
(define (problem disj-1)
  (:domain disj)
  (:objects uav1 - robot)
  (:init (carded uav1))
  (:goal (in uav1))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);

    auto r = ame::Planner{}.solve(wm);
    ASSERT_TRUE(r.success);
    ASSERT_EQ(r.steps.size(), 1u);
    const std::string sig = wm.groundActions()[r.steps[0].action_index].signature;
    // Disjunct-tagged signature; base name resolvable by stripping "#k".
    EXPECT_NE(sig.find("enter#"), std::string::npos);
}

// Disjunctive goal: solved when either alternative is reachable.
TEST(PlannerExtensions, DisjunctiveGoalSolvedViaEitherAlternative) {
    const char* domain = R"(
(define (domain dgoal)
  (:requirements :strips :typing)
  (:types loc - object)
  (:predicates (at ?x - loc) (linked ?a - loc ?b - loc))
  (:action move
    :parameters (?from - loc ?to - loc)
    :precondition (and (at ?from) (linked ?from ?to))
    :effect (and (at ?to) (not (at ?from)))
  )
)
)";
    const char* problem = R"(
(define (problem dgoal-1)
  (:domain dgoal)
  (:objects l1 l2 l3 - loc)
  (:init (at l1) (linked l1 l2))
  (:goal (or (at l2) (at l3)))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);
    ASSERT_EQ(wm.goalAlternatives().size(), 2u);

    auto r = ame::Planner{}.solve(wm);
    ASSERT_TRUE(r.success);  // l2 reachable, l3 not -> still solvable
    EXPECT_EQ(r.steps.size(), 1u);
}

TEST(PlannerExtensions, DisjunctiveGoalUnsolvableWhenNeitherReachable) {
    const char* domain = R"(
(define (domain dgoal2)
  (:requirements :strips :typing)
  (:types loc - object)
  (:predicates (at ?x - loc) (linked ?a - loc ?b - loc))
  (:action move
    :parameters (?from - loc ?to - loc)
    :precondition (and (at ?from) (linked ?from ?to))
    :effect (and (at ?to) (not (at ?from)))
  )
)
)";
    const char* problem = R"(
(define (problem dgoal2-1)
  (:domain dgoal2)
  (:objects l1 l2 l3 - loc)
  (:init (at l1))
  (:goal (or (at l2) (at l3)))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);
    auto r = ame::Planner{}.solve(wm);
    EXPECT_FALSE(r.success);
}

// Inequality binding filter: a tour that must move between distinct locations.
TEST(PlannerExtensions, InequalityPreventsSelfMove) {
    const char* domain = R"(
(define (domain ineq)
  (:requirements :strips :typing)
  (:types loc - object)
  (:predicates (at ?x - loc) (visited ?x - loc))
  (:action move
    :parameters (?from - loc ?to - loc)
    :precondition (and (at ?from) (not (= ?from ?to)))
    :effect (and (at ?to) (not (at ?from)) (visited ?to))
  )
)
)";
    const char* problem = R"(
(define (problem ineq-1)
  (:domain ineq)
  (:objects l1 l2 - loc)
  (:init (at l1))
  (:goal (visited l2))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);
    // No move(lX,lX) ground actions exist.
    for (const auto& ga : wm.groundActions()) {
        EXPECT_NE(ga.signature, "move(l1,l1)");
        EXPECT_NE(ga.signature, "move(l2,l2)");
    }
    auto r = ame::Planner{}.solve(wm);
    ASSERT_TRUE(r.success);
}

// Existential precondition solved through one instantiation.
TEST(PlannerExtensions, ExistentialPreconditionSolvable) {
    const char* domain = R"(
(define (domain exq)
  (:requirements :strips :typing)
  (:types robot sector - object)
  (:predicates (at ?r - robot ?s - sector) (sensed ?r - robot))
  (:action sense
    :parameters (?r - robot)
    :precondition (exists (?s - sector) (at ?r ?s))
    :effect (sensed ?r)
  )
)
)";
    const char* problem = R"(
(define (problem exq-1)
  (:domain exq)
  (:objects uav1 - robot sa sb - sector)
  (:init (at uav1 sb))
  (:goal (sensed uav1))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);
    auto r = ame::Planner{}.solve(wm);
    ASSERT_TRUE(r.success);
}

// either-typed parameter: action applies to objects of any listed type.
TEST(PlannerExtensions, EitherTypeParameterSolvable) {
    const char* domain = R"(
(define (domain eith)
  (:requirements :strips :typing)
  (:types uav ship truck - object)
  (:predicates (idle ?v - object) (tasked ?v - object))
  (:action task
    :parameters (?v - (either uav ship))
    :precondition (idle ?v)
    :effect (tasked ?v)
  )
)
)";
    const char* problem = R"(
(define (problem eith-1)
  (:domain eith)
  (:objects s1 - ship t1 - truck)
  (:init (idle s1) (idle t1))
  (:goal (tasked s1))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);
    // truck cannot be tasked
    for (const auto& ga : wm.groundActions()) {
        EXPECT_NE(ga.signature, "task(t1)");
    }
    auto r = ame::Planner{}.solve(wm);
    ASSERT_TRUE(r.success);
}
