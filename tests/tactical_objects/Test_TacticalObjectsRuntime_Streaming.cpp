#include <gtest/gtest.h>
#include <TacticalObjectsRuntime.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

// ---------------------------------------------------------------------------
// Dirty entity tracking tests
// ---------------------------------------------------------------------------

///< REQ_TACTICAL_OBJECTS_039: Domain events via entity-update callbacks (EventBus-equivalent).
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

  // First assemble — subscriber sees entity (no flush, dirty_entities_ still has obj_id)
  auto sf1 = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf1.updates.empty());

  // Second assemble — same entity, same version, hits "version unchanged" continue
  auto sf2 = rt.assembleStreamFrame(interest_id, sub, 0.0);
  EXPECT_TRUE(sf2.updates.empty());

  rt.flushDirtyEntities(0.0);
  rt.store()->clearAllDirtyMasks();
  auto sf3 = rt.assembleStreamFrame(interest_id, sub, 0.0);
  EXPECT_TRUE(sf3.updates.empty());
  EXPECT_TRUE(sf3.deletes.empty());
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

  // Assemble BEFORE flush to hit deleted_entities_ path
  auto sf_del = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_EQ(sf_del.deletes.size(), 1u);
  EXPECT_EQ(sf_del.deletes[0], obj_id);

  rt.flushDirtyEntities(0.0);

  rt.store()->clearAllDirtyMasks();
  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  EXPECT_TRUE(sf.updates.empty());
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathFirstTimeEntityUsesFullMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_a = rt.createObject(def);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  rt.assembleStreamFrame(interest_id, sub, 0.0);

  auto obj_b = rt.createObject(def);

  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf.updates.empty());
  // Mask should include all components that actually exist on the entity,
  // not FieldMaskBit::ALL (which causes encode/decode mismatch for absent components).
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::POSITION);
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::OBJECT_TYPE);
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathWithPositionMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  def.mil_class.affiliation = Affiliation::Hostile;
  auto obj_id = rt.createObject(def);

  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  auto sf1 = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf1.updates.empty());

  ObjectUpdate upd;
  upd.position = Position{3.0, 4.0, 0.0};
  rt.updateObject(obj_id, upd);

  auto sf2 = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf2.updates.empty());
  EXPECT_EQ(sf2.updates[0].field_mask & FieldMaskBit::POSITION, FieldMaskBit::POSITION);
  ASSERT_TRUE(sf2.updates[0].position.has_value());
  EXPECT_DOUBLE_EQ(sf2.updates[0].position->lat, 3.0);
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathWithVelocityMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  rt.assembleStreamFrame(interest_id, sub, 0.0);

  ObjectUpdate upd;
  upd.velocity = Velocity{1.0, 2.0, 3.0};
  rt.updateObject(obj_id, upd);

  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf.updates.empty());
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::VELOCITY);
  ASSERT_TRUE(sf.updates[0].velocity.has_value());
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathWithBehaviorMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  rt.assembleStreamFrame(interest_id, sub, 0.0);

  rt.setBehavior(obj_id, "patrol", "moving");

  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf.updates.empty());
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::BEHAVIOR);
  ASSERT_TRUE(sf.updates[0].behavior.has_value());
  EXPECT_EQ(sf.updates[0].behavior->behavior_pattern, "patrol");
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathWithIdentityMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  def.identity_name = "initial";
  auto obj_id = rt.createObject(def);

  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  rt.assembleStreamFrame(interest_id, sub, 0.0);

  ObjectUpdate upd;
  upd.identity_name = "Eagle-1";
  rt.updateObject(obj_id, upd);

  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf.updates.empty());
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::IDENTITY_NAME);
  ASSERT_TRUE(sf.updates[0].identity_name.has_value());
  EXPECT_EQ(*sf.updates[0].identity_name, "Eagle-1");
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathWithObjectTypeMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Unit;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  rt.assembleStreamFrame(interest_id, sub, 0.0);

  ObjectUpdate upd;
  upd.position = Position{1.0, 2.0, 0.0};
  rt.updateObject(obj_id, upd);

  rt.store()->clearAllDirtyMasks();
  rt.store()->setDirtyBits(obj_id, FieldMaskBit::OBJECT_TYPE);

  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf.updates.empty());
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::OBJECT_TYPE);
  ASSERT_TRUE(sf.updates[0].object_type.has_value());
  EXPECT_EQ(*sf.updates[0].object_type, ObjectType::Unit);
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathWithAffiliationAndMilClassMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  def.mil_class.affiliation = Affiliation::Friendly;
  auto obj_id = rt.createObject(def);

  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  rt.assembleStreamFrame(interest_id, sub, 0.0);

  ObjectUpdate upd;
  upd.affiliation = Affiliation::Hostile;
  rt.updateObject(obj_id, upd);

  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf.updates.empty());
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::AFFILIATION);
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::MIL_CLASS);
  ASSERT_TRUE(sf.updates[0].affiliation.has_value());
  EXPECT_EQ(*sf.updates[0].affiliation, Affiliation::Hostile);
}

TEST(TacticalObjectsRuntimeStreaming, DeltaPathWithConfidenceAndLifecycleMask) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  QualityComponent qc;
  qc.confidence = 0.9;
  rt.store()->quality().set(obj_id, qc);

  rt.flushDirtyEntities(0.0);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  SubscriptionHandle sub = rt.registerStreamSubscriber(interest_id, [](const UUIDKey&, const UUIDKey&) {});

  rt.assembleStreamFrame(interest_id, sub, 0.0);

  ObjectUpdate upd;
  upd.position = Position{1.0, 2.0, 0.0};
  rt.updateObject(obj_id, upd);

  rt.store()->clearAllDirtyMasks();
  rt.store()->setDirtyBits(obj_id, FieldMaskBit::CONFIDENCE | FieldMaskBit::LIFECYCLE_STATUS);

  auto sf = rt.assembleStreamFrame(interest_id, sub, 0.0);
  ASSERT_FALSE(sf.updates.empty());
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::CONFIDENCE);
  EXPECT_TRUE(sf.updates[0].field_mask & FieldMaskBit::LIFECYCLE_STATUS);
  ASSERT_TRUE(sf.updates[0].confidence.has_value());
  EXPECT_DOUBLE_EQ(*sf.updates[0].confidence, 0.9);
  ASSERT_TRUE(sf.updates[0].lifecycle_status.has_value());
}

///< REQ_TACTICAL_OBJECTS_058: Requirement achievability via interest/callback flow.
///< REQ_TACTICAL_OBJECTS_059: Tactical object solution via streaming frame assembly.
TEST(TacticalObjectsRuntimeStreaming, FlushDirtyEntitiesWithSubscribers) {
  TacticalObjectsRuntime rt;

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = {1.0, 2.0, 0.0};
  auto obj_id = rt.createObject(def);

  InterestCriteria crit;
  auto interest_id = rt.interestManager().registerInterest(crit);
  int callback_count = 0;
  rt.registerStreamSubscriber(interest_id, [&](const UUIDKey&, const UUIDKey&) { ++callback_count; });

  rt.flushDirtyEntities(0.0);
  EXPECT_EQ(callback_count, 1);
}
