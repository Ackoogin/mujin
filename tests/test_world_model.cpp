#include <gtest/gtest.h>
#include "mujin/type_system.h"
#include "mujin/world_model.h"

// =============================================================================
// TypeSystem tests
// =============================================================================

TEST(TypeSystem, AddAndQueryTypes) {
    mujin::TypeSystem ts;
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("robot", "object");

    EXPECT_TRUE(ts.hasType("object"));
    EXPECT_TRUE(ts.hasType("location"));
    EXPECT_TRUE(ts.hasType("robot"));
    EXPECT_FALSE(ts.hasType("nonexistent"));
}

TEST(TypeSystem, SubtypeRelationship) {
    mujin::TypeSystem ts;
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("waypoint", "location");

    EXPECT_TRUE(ts.isSubtype("waypoint", "object"));
    EXPECT_TRUE(ts.isSubtype("waypoint", "location"));
    EXPECT_TRUE(ts.isSubtype("location", "object"));
    EXPECT_TRUE(ts.isSubtype("object", "object"));  // reflexive
    EXPECT_FALSE(ts.isSubtype("object", "location"));
}

TEST(TypeSystem, ObjectRegistration) {
    mujin::TypeSystem ts;
    ts.addType("location");
    ts.addType("robot");

    ts.addObject("base", "location");
    ts.addObject("sector_a", "location");
    ts.addObject("uav1", "robot");

    EXPECT_TRUE(ts.hasObject("base"));
    EXPECT_TRUE(ts.hasObject("uav1"));
    EXPECT_FALSE(ts.hasObject("nonexistent"));

    EXPECT_EQ(ts.getObjectType("base"), "location");
    EXPECT_EQ(ts.getObjectType("uav1"), "robot");
}

TEST(TypeSystem, GetObjectsOfType) {
    mujin::TypeSystem ts;
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("robot", "object");

    ts.addObject("base", "location");
    ts.addObject("sector_a", "location");
    ts.addObject("uav1", "robot");

    auto locations = ts.getObjectsOfType("location");
    EXPECT_EQ(locations.size(), 2u);

    auto robots = ts.getObjectsOfType("robot");
    EXPECT_EQ(robots.size(), 1u);

    // "object" is parent of both, should get all 3
    auto all_objects = ts.getObjectsOfType("object");
    EXPECT_EQ(all_objects.size(), 3u);
}

TEST(TypeSystem, UnknownTypeThrows) {
    mujin::TypeSystem ts;
    EXPECT_THROW(ts.addObject("x", "unknown_type"), std::runtime_error);
}

// =============================================================================
// WorldModel tests
// =============================================================================

TEST(WorldModel, EmptyModel) {
    mujin::WorldModel wm;
    EXPECT_EQ(wm.numFluents(), 0u);
    EXPECT_EQ(wm.version(), 0u);
}

TEST(WorldModel, RegisterPredicateZeroAry) {
    mujin::WorldModel wm;
    wm.registerPredicate("emergency", {});
    EXPECT_EQ(wm.numFluents(), 1u);
    EXPECT_EQ(wm.fluentName(0), "(emergency)");
}

TEST(WorldModel, EagerGroundingOnPredicateRegistration) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");

    wm.addObject("base", "location");
    wm.addObject("sector_a", "location");
    wm.addObject("uav1", "robot");

    // "at" predicate with (robot, location) -> 1 robot x 2 locations = 2 fluents
    wm.registerPredicate("at", {"robot", "location"});
    EXPECT_EQ(wm.numFluents(), 2u);
}

TEST(WorldModel, EagerGroundingOnObjectAddition) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");

    wm.addObject("uav1", "robot");
    wm.registerPredicate("at", {"robot", "location"});

    // No locations yet, so no fluents for "at"
    EXPECT_EQ(wm.numFluents(), 0u);

    // Adding a location triggers eager grounding
    wm.addObject("base", "location");
    EXPECT_EQ(wm.numFluents(), 1u);  // at(uav1, base)

    wm.addObject("sector_a", "location");
    EXPECT_EQ(wm.numFluents(), 2u);  // + at(uav1, sector_a)
}

TEST(WorldModel, SetGetFactByStringKey) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});

    std::string key = "(at uav1 base)";

    // Initially false
    EXPECT_FALSE(wm.getFact(key));

    // Set to true
    wm.setFact(key, true);
    EXPECT_TRUE(wm.getFact(key));

    // Set to false
    wm.setFact(key, false);
    EXPECT_FALSE(wm.getFact(key));
}

TEST(WorldModel, SetGetFactByIndex) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});

    unsigned idx = wm.fluentIndex("(at uav1 base)");

    EXPECT_FALSE(wm.getFact(idx));

    wm.setFact(idx, true);
    EXPECT_TRUE(wm.getFact(idx));

    wm.setFact(idx, false);
    EXPECT_FALSE(wm.getFact(idx));
}

TEST(WorldModel, VersionIncrements) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});

    EXPECT_EQ(wm.version(), 0u);

    wm.setFact("(at uav1 base)", true);
    EXPECT_EQ(wm.version(), 1u);

    // Setting to same value should NOT increment
    wm.setFact("(at uav1 base)", true);
    EXPECT_EQ(wm.version(), 1u);

    wm.setFact("(at uav1 base)", false);
    EXPECT_EQ(wm.version(), 2u);
}

TEST(WorldModel, FluentIndexBiMap) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.addObject("sector_a", "location");
    wm.registerPredicate("at", {"robot", "location"});

    EXPECT_EQ(wm.numFluents(), 2u);

    // Verify round-trip
    for (unsigned i = 0; i < wm.numFluents(); ++i) {
        const std::string& name = wm.fluentName(i);
        EXPECT_EQ(wm.fluentIndex(name), i);
    }
}

TEST(WorldModel, UnknownFluentThrows) {
    mujin::WorldModel wm;
    EXPECT_THROW(wm.getFact("(nonexistent)"), std::runtime_error);
    EXPECT_THROW(wm.setFact("(nonexistent)", true), std::runtime_error);
    EXPECT_THROW(wm.fluentIndex("(nonexistent)"), std::runtime_error);
}

TEST(WorldModel, MultiplePredicates) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");
    ts.addType("sector", "location");

    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.addObject("sector_a", "sector");
    wm.addObject("sector_b", "sector");

    // at(robot, location) -> 1 robot x 3 locations (base, sector_a, sector_b) = 3
    wm.registerPredicate("at", {"robot", "location"});
    EXPECT_EQ(wm.numFluents(), 3u);

    // searched(sector) -> 2 sectors = 2
    wm.registerPredicate("searched", {"sector"});
    EXPECT_EQ(wm.numFluents(), 5u);

    // classified(sector) -> 2 sectors = 2
    wm.registerPredicate("classified", {"sector"});
    EXPECT_EQ(wm.numFluents(), 7u);

    // Set and verify independent facts
    wm.setFact("(at uav1 base)", true);
    wm.setFact("(searched sector_a)", true);

    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
    EXPECT_FALSE(wm.getFact("(at uav1 sector_a)"));
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
    EXPECT_FALSE(wm.getFact("(searched sector_b)"));
    EXPECT_FALSE(wm.getFact("(classified sector_a)"));
}

TEST(WorldModel, LargeFluentCount) {
    // Test that the bitset scales beyond 64 fluents
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    for (int i = 0; i < 100; ++i) {
        wm.addObject("loc" + std::to_string(i), "location");
    }
    // connected(location, location) -> 100 x 100 = 10000 fluents
    wm.registerPredicate("connected", {"location", "location"});
    EXPECT_EQ(wm.numFluents(), 10000u);

    // Set some facts and verify
    wm.setFact("(connected loc0 loc99)", true);
    wm.setFact("(connected loc99 loc0)", true);
    EXPECT_TRUE(wm.getFact("(connected loc0 loc99)"));
    EXPECT_TRUE(wm.getFact("(connected loc99 loc0)"));
    EXPECT_FALSE(wm.getFact("(connected loc0 loc0)"));
}
