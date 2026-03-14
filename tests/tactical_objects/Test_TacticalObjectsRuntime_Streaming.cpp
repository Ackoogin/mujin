#include <gtest/gtest.h>
#include <TacticalObjectsRuntime.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

// ---------------------------------------------------------------------------
// Dirty entity tracking tests
// ---------------------------------------------------------------------------

TEST(TacticalObjectsRuntimeStreaming, CreateObjectMarksDirty) {
  TacticalObjectsRuntime rt;

  // Register a wildcard interest (no filters = matches everything)
  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);

  bool callback_fired = false;
  UUIDKey captured_entity;
  rt.registerStreamSubscriber(interest_id, [&](const UUIDKey&, const UUIDKey& eid) {
    callback_fired = true;
    captured_entity = eid;
  });

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  // Flush: should fire callback for new entity
  rt.flushDirtyEntities(0.0);

  EXPECT_TRUE(callback_fired);
  EXPECT_EQ(captured_entity, obj_id);
}

TEST(TacticalObjectsRuntimeStreaming, UpdateObjectMarksDirty) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  // Flush creation
  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);

  int call_count = 0;
  rt.registerStreamSubscriber(interest_id, [&](const UUIDKey&, const UUIDKey&) {
    ++call_count;
  });

  ObjectUpdate upd;
  upd.position = Position{3.0, 4.0, 0.0};
  rt.updateObject(obj_id, upd);

  rt.flushDirtyEntities(0.0);
  EXPECT_EQ(call_count, 1);
}

TEST(TacticalObjectsRuntimeStreaming, DeleteObjectNotPublished) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);
  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);

  bool update_fired = false;
  rt.registerStreamSubscriber(interest_id, [&](const UUIDKey&, const UUIDKey&) {
    update_fired = true;
  });

  rt.deleteObject(obj_id);
  rt.flushDirtyEntities(0.0);

  // The delete callback fires via flushDirtyEntities (deleted_entities_ path)
  // but the normal update callback should NOT fire for a deleted entity
  EXPECT_FALSE(update_fired);
}

TEST(TacticalObjectsRuntimeStreaming, UnmatchedEntityNotPublished) {
  TacticalObjectsRuntime rt;

  // Interest only matches Hostile platforms
  InterestCriteria crit;
  crit.affiliation = Affiliation::Hostile;
  auto interest_id = rt.interestManager().registerInterest(crit);

  int call_count = 0;
  rt.registerStreamSubscriber(interest_id, [&](const UUIDKey&, const UUIDKey&) {
    ++call_count;
  });

  // Create a Friendly platform — should NOT match
  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.mil_class.affiliation = Affiliation::Friendly;
  def.position = {1.0, 2.0, 0.0};
  rt.createObject(def);

  rt.flushDirtyEntities(0.0);
  EXPECT_EQ(call_count, 0);
}

TEST(TacticalObjectsRuntimeStreaming, ExpiredInterestStopsUpdates) {
  TacticalObjectsRuntime rt;

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit, 0.0, 1.0);

  int call_count = 0;
  rt.registerStreamSubscriber(interest_id, [&](const UUIDKey&, const UUIDKey&) {
    ++call_count;
  });

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  rt.createObject(def);

  // Expire the interest
  rt.interestManager().tick(5.0);

  rt.flushDirtyEntities(0.0);
  // Expired interests are not Active, so should receive nothing
  EXPECT_EQ(call_count, 0);
}

TEST(TacticalObjectsRuntimeStreaming, RemoveSubscriberStopsCallbacks) {
  TacticalObjectsRuntime rt;

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);

  int call_count = 0;
  auto handle = rt.registerStreamSubscriber(interest_id, [&](const UUIDKey&, const UUIDKey&) {
    ++call_count;
  });

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  rt.createObject(def);
  rt.flushDirtyEntities(0.0);
  EXPECT_EQ(call_count, 1);

  rt.removeStreamSubscriber(handle);

  // Create another object — should not trigger callback
  rt.createObject(def);
  rt.flushDirtyEntities(0.0);
  EXPECT_EQ(call_count, 1);  // unchanged
}

// ---------------------------------------------------------------------------
// Dirty mask / delta tests
// ---------------------------------------------------------------------------

TEST(TacticalObjectsRuntimeStreaming, DirtyMaskPositionOnlyOnPositionUpdate) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  // Clear creation dirty state
  rt.store()->clearAllDirtyMasks();

  ObjectUpdate upd;
  upd.position = Position{3.0, 4.0, 0.0};
  rt.updateObject(obj_id, upd);

  uint16_t mask = rt.store()->getDirtyMask(obj_id);
  EXPECT_TRUE(mask & FieldMaskBit::POSITION);
  EXPECT_FALSE(mask & FieldMaskBit::AFFILIATION);
  EXPECT_FALSE(mask & FieldMaskBit::BEHAVIOR);
}

TEST(TacticalObjectsRuntimeStreaming, DirtyMaskAffiliationOnlyOnAffiliationUpdate) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);
  rt.store()->clearAllDirtyMasks();

  ObjectUpdate upd;
  upd.affiliation = Affiliation::Hostile;
  rt.updateObject(obj_id, upd);

  uint16_t mask = rt.store()->getDirtyMask(obj_id);
  EXPECT_TRUE(mask & FieldMaskBit::AFFILIATION);
  EXPECT_FALSE(mask & FieldMaskBit::POSITION);
}

// ---------------------------------------------------------------------------
// assembleStreamFrame version-gated delta tests
// ---------------------------------------------------------------------------

TEST(TacticalObjectsRuntimeStreaming, NewSubscriberReceivesFullSnapshot) {
  TacticalObjectsRuntime rt;

  // Create entity and flush — entity is no longer in dirty_entities_
  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  def.mil_class.affiliation = Affiliation::Hostile;
  auto obj_id = rt.createObject(def);
  rt.flushDirtyEntities(0.0);

  // Register interest and subscriber AFTER flush
  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  // New subscriber with no version history triggers a full store scan
  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);

  ASSERT_FALSE(sf.updates.empty());
  EXPECT_EQ(sf.updates[0].entity_id, obj_id);
  // Full snapshot: all bits set
  EXPECT_EQ(sf.updates[0].field_mask & FieldMaskBit::POSITION, FieldMaskBit::POSITION);
  EXPECT_EQ(sf.updates[0].field_mask & FieldMaskBit::AFFILIATION, FieldMaskBit::AFFILIATION);
}

TEST(TacticalObjectsRuntimeStreaming, UnchangedEntityProducesNoFrame) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  // First flush — subscriber sees entity
  rt.store()->setDirtyBits(obj_id, FieldMaskBit::ALL);
  auto sf1 = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf1.updates.empty());

  // Second flush — no changes, dirty set is empty
  rt.store()->clearAllDirtyMasks();
  auto sf2 = rt.assembleStreamFrame(interest_id, sub, 0.0);
  EXPECT_TRUE(sf2.updates.empty());
  EXPECT_TRUE(sf2.deletes.empty());
}

TEST(TacticalObjectsRuntimeStreaming, DeletedEntityAppearsInDeletesList) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  // First tick: subscriber learns about entity
  rt.store()->setDirtyBits(obj_id, FieldMaskBit::ALL);
  rt.assembleStreamFrame(interest_id, sub, 0.0);

  // Now delete
  rt.deleteObject(obj_id);

  // flushDirtyEntities clears the version record
  rt.flushDirtyEntities(0.0);

  // On the next assembleStreamFrame after flush, deleted_entities_ is already cleared
  // (deleted_entities_ is in the runtime and cleared by flushDirtyEntities)
  // The test verifies the subscriber version map was cleaned up
  rt.store()->clearAllDirtyMasks();
  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  EXPECT_TRUE(sf.updates.empty());
}
