#include <gtest/gtest.h>
#include "ame/pddl_parser.h"
#include "ame/world_model.h"

#include <stdexcept>
#include <string>

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

static const char* SIMPLE_PROBLEM = R"(
(define (problem unsupported-constructs-problem)
  (:domain unsupported-constructs)
  (:objects obj1 - object)
  (:init
    (p obj1)
  )
  (:goal (q obj1))
)
)";

static std::string makeDomainWithAction(const std::string& precondition,
                                        const std::string& effect) {
    return std::string(R"(
(define (domain unsupported-constructs)
  (:requirements :strips :typing)
  (:predicates
    (p ?o - object)
    (q ?o - object)
  )
  (:action test
    :parameters (?o - object)
    :precondition )") + precondition + R"(
    :effect )" + effect + R"(
  )
)
)";
}

TEST(PddlParser, ParseTypes) {
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    auto& ts = wm.typeSystem();
    EXPECT_TRUE(ts.hasType("object"));
    EXPECT_TRUE(ts.hasType("location"));
    EXPECT_TRUE(ts.hasType("sector"));
    EXPECT_TRUE(ts.hasType("robot"));
    EXPECT_TRUE(ts.isSubtype("sector", "location"));
    EXPECT_TRUE(ts.isSubtype("location", "object"));
}

TEST(PddlParser, ParseObjects) {
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

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
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    // at(robot, location): 1 robot x 3 locations = 3
    // searched(sector): 2
    // classified(sector): 2
    // Total: 7
    EXPECT_EQ(wm.numFluents(), 7u);
}

TEST(PddlParser, ParseInitialState) {
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
    EXPECT_FALSE(wm.getFact("(at uav1 sector_a)"));
    EXPECT_FALSE(wm.getFact("(searched sector_a)"));
}

TEST(PddlParser, ParseGoal) {
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

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
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    // move: 1 robot x 3 locations x 3 locations = 9
    // search: 1 robot x 2 sectors = 2
    // classify: 1 robot x 2 sectors = 2
    EXPECT_EQ(wm.numGroundActions(), 13u);
}

TEST(PddlParser, MatchesProgrammaticConstruction) {
    // Parse from PDDL
    ame::WorldModel wm_parsed;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm_parsed);

    // Build programmatically (same domain)
    ame::WorldModel wm_prog;
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
    ame::WorldModel wm;
    // Use the actual PDDL files
    std::string domain_path = std::string(PROJECT_SOURCE_DIR) + "/domains/uav_search/domain.pddl";
    std::string problem_path = std::string(PROJECT_SOURCE_DIR) + "/domains/uav_search/problem.pddl";
    ame::PddlParser::parse(domain_path, problem_path, wm);

    EXPECT_EQ(wm.numFluents(), 7u);
    EXPECT_EQ(wm.numGroundActions(), 13u);
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
}

TEST(PddlParser, ParsesConjunctionsAndDeleteEffects) {
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction(
        "(and (p ?o))",
        "(and (q ?o) (not (p ?o)))");

    ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm);

    ASSERT_EQ(wm.numGroundActions(), 1u);
    const auto& action = wm.groundActions().front();
    ASSERT_EQ(action.preconditions.size(), 1u);
    ASSERT_EQ(action.add_effects.size(), 1u);
    ASSERT_EQ(action.del_effects.size(), 1u);
    EXPECT_EQ(wm.fluentName(action.preconditions[0]), "(p obj1)");
    EXPECT_EQ(wm.fluentName(action.add_effects[0]), "(q obj1)");
    EXPECT_EQ(wm.fluentName(action.del_effects[0]), "(p obj1)");
}

TEST(PddlParser, AcceptsNegativePrecondition) {
    // Negative preconditions are now supported: (not (p ?o)) is a negative
    // precondition that grounds to a neg_precondition fluent id.
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction("(not (p ?o))", "(q ?o)");

    ASSERT_NO_THROW(
        ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm));

    bool found = false;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature == "test(obj1)") {
            found = true;
            EXPECT_TRUE(ga.preconditions.empty());
            ASSERT_EQ(ga.neg_preconditions.size(), 1u);
            EXPECT_EQ(wm.fluentName(ga.neg_preconditions[0]), "(p obj1)");
        }
    }
    EXPECT_TRUE(found);
}

TEST(PddlParser, AcceptsOrPrecondition) {
    // Disjunctive precondition splits into one schema per disjunct (DNF).
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction("(or (p ?o) (q ?o))", "(q ?o)");

    ASSERT_NO_THROW(
        ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm));

    // Two disjuncts over the single object obj1 -> two ground actions whose
    // names share the base "test" with disjunct tags.
    int with_p = 0, with_q = 0;
    for (const auto& ga : wm.groundActions()) {
        ASSERT_NE(ga.signature.find("test#"), std::string::npos);
        ASSERT_EQ(ga.preconditions.size(), 1u);
        const std::string pre = wm.fluentName(ga.preconditions[0]);
        if (pre == "(p obj1)") ++with_p;
        if (pre == "(q obj1)") ++with_q;
    }
    EXPECT_EQ(with_p, 1);
    EXPECT_EQ(with_q, 1);
}

TEST(PddlParser, AcceptsForallPrecondition) {
    // Universal precondition expands to a conjunction over all objects of the
    // type (here just obj1).
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction(
        "(forall (?x - object) (p ?x))",
        "(q ?o)");

    ASSERT_NO_THROW(
        ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm));

    bool found = false;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature == "test(obj1)") {
            found = true;
            ASSERT_EQ(ga.preconditions.size(), 1u);
            EXPECT_EQ(wm.fluentName(ga.preconditions[0]), "(p obj1)");
        }
    }
    EXPECT_TRUE(found);
}

TEST(PddlParser, RejectsWhenEffect) {
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction(
        "(p ?o)",
        "(when (p ?o) (q ?o))");

    EXPECT_THROW(
        ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm),
        std::runtime_error);
}

TEST(PddlParser, RejectsUnsupportedRequirements) {
    ame::WorldModel wm;
    const std::string domain = R"(
(define (domain unsupported-requirements)
  (:requirements :strips :typing :fluents)
  (:predicates
    (p ?o - object)
  )
)
)";

    EXPECT_THROW(
        ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm),
        std::runtime_error);
}

TEST(PddlParser, ParsesDomainConstants) {
    ame::WorldModel wm;
    const std::string domain = R"(
(define (domain constants-domain)
  (:requirements :strips :typing)
  (:types
    location - object
    robot - object
  )
  (:constants
    base target - location
  )
  (:predicates
    (at ?r - robot ?l - location)
    (visited ?l - location)
  )
  (:action visit
    :parameters (?r - robot ?l - location)
    :precondition (at ?r ?l)
    :effect (visited ?l)
  )
)
)";
    const std::string problem = R"(
(define (problem constants-problem)
  (:domain constants-domain)
  (:objects
    uav1 - robot
  )
  (:init
    (at uav1 base)
  )
  (:goal (visited target))
)
)";

    ame::PddlParser::parseFromString(domain, problem, wm);

    auto& ts = wm.typeSystem();
    EXPECT_TRUE(ts.hasObject("base"));
    EXPECT_TRUE(ts.hasObject("target"));
    EXPECT_EQ(ts.getObjectType("base"), "location");
    EXPECT_EQ(ts.getObjectType("target"), "location");
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
    EXPECT_NO_THROW(wm.fluentIndex("(visited target)"));
}

// =========================================================================
// Negative preconditions: (not ATOM) in action :precondition
// =========================================================================

static const char* NEG_DOMAIN = R"(
(define (domain neg-pre)
  (:requirements :strips :typing)
  (:types robot - object)
  (:predicates
    (armed ?r - robot)
    (safe ?r - robot)
    (launched ?r - robot)
  )
  (:action arm
    :parameters (?r - robot)
    :precondition (and (safe ?r) (not (armed ?r)))
    :effect (armed ?r)
  )
  (:action launch
    :parameters (?r - robot)
    :precondition (and (armed ?r) (not (safe ?r)))
    :effect (launched ?r)
  )
)
)";

static const char* NEG_PROBLEM = R"(
(define (problem neg-pre-1)
  (:domain neg-pre)
  (:objects uav1 - robot)
  (:init (safe uav1))
  (:goal (armed uav1))
)
)";

TEST(PddlParserNegPre, AcceptsNegatedAtomInPrecondition) {
    ame::WorldModel wm;
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(NEG_DOMAIN, NEG_PROBLEM, wm));

    // arm(uav1) requires (safe uav1) true and (armed uav1) false.
    bool found_arm = false;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature == "arm(uav1)") {
            found_arm = true;
            ASSERT_EQ(ga.preconditions.size(), 1u);
            ASSERT_EQ(ga.neg_preconditions.size(), 1u);
            EXPECT_EQ(wm.fluentName(ga.preconditions[0]), "(safe uav1)");
            EXPECT_EQ(wm.fluentName(ga.neg_preconditions[0]), "(armed uav1)");
        }
    }
    EXPECT_TRUE(found_arm);
}

TEST(PddlParserNegPre, RejectsNestedNegation) {
    const char* domain = R"(
(define (domain bad-neg)
  (:requirements :strips :typing)
  (:types robot - object)
  (:predicates (armed ?r - robot))
  (:action a
    :parameters (?r - robot)
    :precondition (not (not (armed ?r)))
    :effect (armed ?r)
  )
)
)";
    const char* problem = R"(
(define (problem bad-neg-1)
  (:domain bad-neg)
  (:objects uav1 - robot)
  (:init)
  (:goal (armed uav1))
)
)";
    ame::WorldModel wm;
    EXPECT_THROW(ame::PddlParser::parseFromString(domain, problem, wm),
                 std::runtime_error);
}

TEST(PddlParserNegPre, RejectsNegatedConjunction) {
    const char* domain = R"(
(define (domain bad-neg2)
  (:requirements :strips :typing)
  (:types robot - object)
  (:predicates (armed ?r - robot) (safe ?r - robot))
  (:action a
    :parameters (?r - robot)
    :precondition (not (and (armed ?r) (safe ?r)))
    :effect (armed ?r)
  )
)
)";
    const char* problem = R"(
(define (problem bad-neg2-1)
  (:domain bad-neg2)
  (:objects uav1 - robot)
  (:init)
  (:goal (armed uav1))
)
)";
    ame::WorldModel wm;
    EXPECT_THROW(ame::PddlParser::parseFromString(domain, problem, wm),
                 std::runtime_error);
}

TEST(PddlParserNegPre, RejectsNegatedGoal) {
    const char* domain = R"(
(define (domain goal-neg)
  (:requirements :strips :typing)
  (:types robot - object)
  (:predicates (armed ?r - robot))
  (:action a
    :parameters (?r - robot)
    :precondition (armed ?r)
    :effect (armed ?r)
  )
)
)";
    const char* problem = R"(
(define (problem goal-neg-1)
  (:domain goal-neg)
  (:objects uav1 - robot)
  (:init)
  (:goal (not (armed uav1)))
)
)";
    ame::WorldModel wm;
    EXPECT_THROW(ame::PddlParser::parseFromString(domain, problem, wm),
                 std::runtime_error);
}

// =========================================================================
// Nested conjunctions
// =========================================================================

TEST(PddlParser, FlattensNestedConjunction) {
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction("(and (p ?o) (and (q ?o)))", "(q ?o)");
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm));

    bool found = false;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature == "test(obj1)") {
            found = true;
            ASSERT_EQ(ga.preconditions.size(), 2u);
            std::vector<std::string> names = {wm.fluentName(ga.preconditions[0]),
                                              wm.fluentName(ga.preconditions[1])};
            EXPECT_NE(std::find(names.begin(), names.end(), "(p obj1)"), names.end());
            EXPECT_NE(std::find(names.begin(), names.end(), "(q obj1)"), names.end());
        }
    }
    EXPECT_TRUE(found);
}

TEST(PddlParser, AcceptsEmptyConjunction) {
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction("(and)", "(q ?o)");
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm));
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature == "test(obj1)") {
            EXPECT_TRUE(ga.preconditions.empty());
            EXPECT_TRUE(ga.neg_preconditions.empty());
        }
    }
}

// =========================================================================
// Equality / inequality binding filters
// =========================================================================

static const char* EQ_DOMAIN = R"(
(define (domain eq-dom)
  (:requirements :strips :typing)
  (:types loc - object)
  (:predicates (at ?x - loc))
  (:action move
    :parameters (?from - loc ?to - loc)
    :precondition (and (at ?from) (not (= ?from ?to)))
    :effect (and (at ?to) (not (at ?from)))
  )
  (:action selfcheck
    :parameters (?a - loc ?b - loc)
    :precondition (= ?a ?b)
    :effect (at ?a)
  )
)
)";

static const char* EQ_PROBLEM = R"(
(define (problem eq-1)
  (:domain eq-dom)
  (:objects l1 l2 - loc)
  (:init (at l1))
  (:goal (at l2))
)
)";

TEST(PddlParser, InequalityRemovesSelfPairs) {
    ame::WorldModel wm;
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(EQ_DOMAIN, EQ_PROBLEM, wm));

    int move_count = 0;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature.rfind("move(", 0) == 0) {
            ++move_count;
            // never from==to
            EXPECT_NE(ga.signature, "move(l1,l1)");
            EXPECT_NE(ga.signature, "move(l2,l2)");
        }
    }
    EXPECT_EQ(move_count, 2);  // move(l1,l2), move(l2,l1)
}

TEST(PddlParser, EqualityKeepsOnlyMatchingPairs) {
    ame::WorldModel wm;
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(EQ_DOMAIN, EQ_PROBLEM, wm));

    int self_count = 0;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature.rfind("selfcheck(", 0) == 0) {
            ++self_count;
            EXPECT_TRUE(ga.signature == "selfcheck(l1,l1)" ||
                        ga.signature == "selfcheck(l2,l2)");
        }
    }
    EXPECT_EQ(self_count, 2);
}

TEST(PddlParser, EqualityRejectedInEffect) {
    const char* domain = R"(
(define (domain eq-bad)
  (:requirements :strips :typing)
  (:types loc - object)
  (:predicates (at ?x - loc))
  (:action a
    :parameters (?x - loc ?y - loc)
    :precondition (at ?x)
    :effect (= ?x ?y)
  )
)
)";
    ame::WorldModel wm;
    EXPECT_THROW(ame::PddlParser::parseFromString(domain, EQ_PROBLEM, wm),
                 std::runtime_error);
}

// =========================================================================
// Disjunctive preconditions
// =========================================================================

TEST(PddlParser, DisjunctivePreconditionSplitsIntoSchemas) {
    ame::WorldModel wm;
    const std::string domain = makeDomainWithAction("(or (p ?o) (q ?o))", "(q ?o)");
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(domain, SIMPLE_PROBLEM, wm));

    int count = 0;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature.rfind("test#", 0) == 0) ++count;
    }
    EXPECT_EQ(count, 2);
}

// =========================================================================
// Finite quantifiers
// =========================================================================

static const char* QUANT_DOMAIN = R"(
(define (domain quant-dom)
  (:requirements :strips :typing)
  (:types robot sector - object)
  (:predicates (at ?r - robot ?s - sector) (sensed ?r - robot) (clear ?s - sector))
  (:action sense
    :parameters (?r - robot)
    :precondition (exists (?s - sector) (at ?r ?s))
    :effect (sensed ?r)
  )
  (:action sweep
    :parameters (?r - robot)
    :precondition (forall (?s - sector) (clear ?s))
    :effect (sensed ?r)
  )
)
)";

static const char* QUANT_PROBLEM = R"(
(define (problem quant-1)
  (:domain quant-dom)
  (:objects uav1 - robot sa sb - sector)
  (:init (at uav1 sa) (clear sa) (clear sb))
  (:goal (sensed uav1))
)
)";

TEST(PddlParser, ExistentialExpandsToDisjunctionSplit) {
    ame::WorldModel wm;
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(QUANT_DOMAIN, QUANT_PROBLEM, wm));

    int sense_count = 0;
    bool saw_sa = false, saw_sb = false;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature.rfind("sense#", 0) == 0) {
            ++sense_count;
            ASSERT_EQ(ga.preconditions.size(), 1u);
            const std::string pre = wm.fluentName(ga.preconditions[0]);
            if (pre == "(at uav1 sa)") saw_sa = true;
            if (pre == "(at uav1 sb)") saw_sb = true;
        }
    }
    EXPECT_EQ(sense_count, 2);
    EXPECT_TRUE(saw_sa);
    EXPECT_TRUE(saw_sb);
}

TEST(PddlParser, UniversalExpandsToConjunction) {
    ame::WorldModel wm;
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(QUANT_DOMAIN, QUANT_PROBLEM, wm));

    bool found = false;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature == "sweep(uav1)") {
            found = true;
            ASSERT_EQ(ga.preconditions.size(), 2u);  // (clear sa) AND (clear sb)
        }
    }
    EXPECT_TRUE(found);
}

TEST(PddlParser, QuantifierShadowingParameterRejected) {
    const char* domain = R"(
(define (domain shadow-dom)
  (:requirements :strips :typing)
  (:types sector - object)
  (:predicates (clear ?s - sector))
  (:action a
    :parameters (?s - sector)
    :precondition (forall (?s - sector) (clear ?s))
    :effect (clear ?s)
  )
)
)";
    const char* problem = R"(
(define (problem shadow-1)
  (:domain shadow-dom)
  (:objects sa - sector)
  (:init)
  (:goal (clear sa))
)
)";
    ame::WorldModel wm;
    EXPECT_THROW(ame::PddlParser::parseFromString(domain, problem, wm),
                 std::runtime_error);
}

// =========================================================================
// either types
// =========================================================================

static const char* EITHER_DOMAIN = R"(
(define (domain either-dom)
  (:requirements :strips :typing)
  (:types vehicle - object uav - vehicle ship - vehicle truck - object)
  (:predicates (tasked ?v - object))
  (:action assign
    :parameters (?v - (either uav ship))
    :precondition (tasked ?v)
    :effect (tasked ?v)
  )
)
)";

static const char* EITHER_PROBLEM = R"(
(define (problem either-1)
  (:domain either-dom)
  (:objects u1 - uav s1 - ship t1 - truck)
  (:init (tasked u1))
  (:goal (tasked s1))
)
)";

TEST(PddlParser, EitherTypeGroundsOverUnionOnly) {
    ame::WorldModel wm;
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(EITHER_DOMAIN, EITHER_PROBLEM, wm));

    bool saw_u1 = false, saw_s1 = false, saw_t1 = false;
    int count = 0;
    for (const auto& ga : wm.groundActions()) {
        if (ga.signature.rfind("assign(", 0) == 0) {
            ++count;
            if (ga.signature == "assign(u1)") saw_u1 = true;
            if (ga.signature == "assign(s1)") saw_s1 = true;
            if (ga.signature == "assign(t1)") saw_t1 = true;
        }
    }
    EXPECT_EQ(count, 2);
    EXPECT_TRUE(saw_u1);
    EXPECT_TRUE(saw_s1);
    EXPECT_FALSE(saw_t1);
}

TEST(PddlParser, EitherUnknownTypeRejected) {
    const char* domain = R"(
(define (domain either-bad)
  (:requirements :strips :typing)
  (:types uav - object)
  (:predicates (tasked ?v - object))
  (:action assign
    :parameters (?v - (either uav nope))
    :precondition (tasked ?v)
    :effect (tasked ?v)
  )
)
)";
    const char* problem = R"(
(define (problem either-bad-1)
  (:domain either-bad)
  (:objects u1 - uav)
  (:init (tasked u1))
  (:goal (tasked u1))
)
)";
    ame::WorldModel wm;
    EXPECT_THROW(ame::PddlParser::parseFromString(domain, problem, wm),
                 std::runtime_error);
}

// =========================================================================
// Disjunctive goals
// =========================================================================

TEST(PddlParser, DisjunctiveGoalSetsAlternatives) {
    const char* domain = R"(
(define (domain goal-or)
  (:requirements :strips :typing)
  (:types loc - object)
  (:predicates (at ?x - loc))
  (:action go
    :parameters (?to - loc)
    :precondition (at ?to)
    :effect (at ?to)
  )
)
)";
    const char* problem = R"(
(define (problem goal-or-1)
  (:domain goal-or)
  (:objects l1 l2 - loc)
  (:init (at l1))
  (:goal (or (at l1) (at l2)))
)
)";
    ame::WorldModel wm;
    ASSERT_NO_THROW(ame::PddlParser::parseFromString(domain, problem, wm));
    ASSERT_EQ(wm.goalAlternatives().size(), 2u);
    // primary mirrors the first alternative
    EXPECT_EQ(wm.goalFluentIds(), wm.goalAlternatives()[0]);
}

// =========================================================================
// Session re-grounding
// =========================================================================

// A WorldModel is a session. A host that plans repeatedly grounds a new
// problem into the same session and legitimately re-declares the same
// objects each time; that must be idempotent, not fatal.
TEST(PddlParser, ReparseIntoLiveSessionReDeclaresObjectsIdempotently) {
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);
    const unsigned fluents = wm.numFluents();
    const unsigned actions = wm.numGroundActions();

    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    EXPECT_TRUE(wm.typeSystem().hasObject("uav1"));
    EXPECT_EQ(wm.typeSystem().getObjectType("uav1"), "robot");
    // Re-grounding must not accumulate: a growing object set would silently
    // multiply the grounded action space on every re-tasking.
    EXPECT_EQ(wm.numFluents(), fluents);
    EXPECT_EQ(wm.numGroundActions(), actions);
}

// The diagnostic the duplicate check exists for -- one name meaning two
// different things -- must still be refused, naming both types.
TEST(PddlParser, ObjectRedeclaredWithDifferentTypeIsRejected) {
    const char* conflicting = R"(
(define (problem uav-search-2)
  (:domain uav-search)
  (:objects
    uav1 - sector
  )
  (:init)
  (:goal (searched sector_a))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(UAV_DOMAIN, UAV_PROBLEM, wm);

    try {
        ame::PddlParser::parseFromString(UAV_DOMAIN, conflicting, wm);
        FAIL() << "conflicting re-declaration was accepted";
    } catch (const std::runtime_error& err) {
        const std::string message = err.what();
        EXPECT_NE(message.find("uav1"), std::string::npos);
        EXPECT_NE(message.find("robot"), std::string::npos);
        EXPECT_NE(message.find("sector"), std::string::npos);
    }
}
