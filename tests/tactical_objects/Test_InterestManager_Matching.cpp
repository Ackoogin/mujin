#include <gtest/gtest.h>
#include <interest/InterestManager.h>
#include <store/ObjectStore.h>
#include <store/ObjectComponents.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

// ---------------------------------------------------------------------------
// matchesInterest — per-dimension tests
// ---------------------------------------------------------------------------

TEST(InterestManagerMatching, AffiliationMatchesCorrectly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  MilClassComponent mc;
  mc.profile.affiliation = Affiliation::Hostile;
  store.milclass().set(id, mc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;

  // Matching affiliation
  {
    InterestCriteria crit;
    crit.affiliation = Affiliation::Hostile;
    EXPECT_TRUE(mgr.matchesInterest(crit, *rec, store));
  }

  // Non-matching affiliation
  {
    InterestCriteria crit;
    crit.affiliation = Affiliation::Friendly;
    EXPECT_FALSE(mgr.matchesInterest(crit, *rec, store));
  }
}

TEST(InterestManagerMatching, ObjectTypeFilterMatchesCorrectly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);
  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;

  InterestCriteria crit_match;
  crit_match.object_type = ObjectType::Platform;
  EXPECT_TRUE(mgr.matchesInterest(crit_match, *rec, store));

  InterestCriteria crit_no;
  crit_no.object_type = ObjectType::Person;
  EXPECT_FALSE(mgr.matchesInterest(crit_no, *rec, store));
}

TEST(InterestManagerMatching, AreaFilterMatchesCorrectly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  KinematicsComponent kc;
  kc.position = {51.0, 0.0, 0.0};  // inside bbox
  store.kinematics().set(id, kc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;

  // Entity inside bbox
  {
    InterestCriteria crit;
    crit.area = BoundingBox{50.0, 52.0, -1.0, 1.0};
    EXPECT_TRUE(mgr.matchesInterest(crit, *rec, store));
  }

  // Entity outside bbox
  {
    InterestCriteria crit;
    crit.area = BoundingBox{10.0, 20.0, 10.0, 20.0};
    EXPECT_FALSE(mgr.matchesInterest(crit, *rec, store));
  }
}

TEST(InterestManagerMatching, BehaviorPatternFilterMatchesCorrectly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  BehaviorComponent bc;
  bc.behavior_pattern = "patrol";
  store.behaviors().set(id, bc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;

  InterestCriteria crit_match;
  crit_match.behavior_pattern = std::string("patrol");
  EXPECT_TRUE(mgr.matchesInterest(crit_match, *rec, store));

  InterestCriteria crit_no;
  crit_no.behavior_pattern = std::string("attack");
  EXPECT_FALSE(mgr.matchesInterest(crit_no, *rec, store));
}

TEST(InterestManagerMatching, MinimumConfidenceFilterMatchesCorrectly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  QualityComponent qc;
  qc.confidence = 0.8;
  store.quality().set(id, qc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;

  InterestCriteria crit_pass;
  crit_pass.minimum_confidence = 0.5;
  EXPECT_TRUE(mgr.matchesInterest(crit_pass, *rec, store));

  InterestCriteria crit_fail;
  crit_fail.minimum_confidence = 0.9;
  EXPECT_FALSE(mgr.matchesInterest(crit_fail, *rec, store));
}

TEST(InterestManagerMatching, TimeWindowFilterMatchesCorrectly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  LifecycleComponent lc;
  lc.last_update_timestamp = 1500.0;
  store.lifecycle().set(id, lc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;

  // timestamp within window
  {
    InterestCriteria crit;
    crit.time_window_start = 1000.0;
    crit.time_window_end   = 2000.0;
    EXPECT_TRUE(mgr.matchesInterest(crit, *rec, store));
  }

  // timestamp outside window (too late)
  {
    InterestCriteria crit;
    crit.time_window_start = 2000.0;
    crit.time_window_end   = 3000.0;
    EXPECT_FALSE(mgr.matchesInterest(crit, *rec, store));
  }
}

TEST(InterestManagerMatching, NoFilterMatchesEverything) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);
  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;
  InterestCriteria crit;  // all fields unset

  EXPECT_TRUE(mgr.matchesInterest(crit, *rec, store));
}

// ---------------------------------------------------------------------------
// matchingInterests — integration test
// ---------------------------------------------------------------------------

TEST(InterestManagerMatching, MatchingInterestsReturnsCorrectIds) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  MilClassComponent mc;
  mc.profile.affiliation = Affiliation::Hostile;
  store.milclass().set(id, mc);

  KinematicsComponent kc;
  kc.position = {51.0, 0.0, 0.0};
  store.kinematics().set(id, kc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  InterestManager mgr;

  // Interest 1: Hostile — should match
  InterestCriteria crit1;
  crit1.affiliation = Affiliation::Hostile;
  auto i1 = mgr.registerInterest(crit1);

  // Interest 2: Friendly — should not match
  InterestCriteria crit2;
  crit2.affiliation = Affiliation::Friendly;
  auto i2 = mgr.registerInterest(crit2);

  // Interest 3: Hostile + area match — should match
  InterestCriteria crit3;
  crit3.affiliation = Affiliation::Hostile;
  crit3.area = BoundingBox{50.0, 52.0, -1.0, 1.0};
  auto i3 = mgr.registerInterest(crit3);

  // Interest 4: cancelled — should never appear
  InterestCriteria crit4;
  auto i4 = mgr.registerInterest(crit4);
  mgr.cancelInterest(i4);

  (void)i2;

  auto matching = mgr.matchingInterests(*rec, store);
  ASSERT_EQ(matching.size(), 2u);
  // Should contain i1 and i3
  bool has_i1 = false, has_i3 = false;
  for (auto& mid : matching) {
    if (mid == i1) has_i1 = true;
    if (mid == i3) has_i3 = true;
  }
  EXPECT_TRUE(has_i1);
  EXPECT_TRUE(has_i3);
}
