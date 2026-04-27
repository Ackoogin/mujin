#include <gtest/gtest.h>
#include <store/ObjectStore.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

///< REQ_TACTICAL_OBJECTS_001: ObjectStore allocates a UUID for each new object.
TEST(ObjectStore, CreateObjectReturnsNonNullUUID) {
  ObjectStore store;
  auto id = store.createObject(ObjectType::Platform);
  ASSERT_FALSE(id.isNull());
  ASSERT_EQ(store.objectCount(), 1u);
}

///< REQ_TACTICAL_OBJECTS_002: Dense iteration count matches sparse lookup count.
TEST(ObjectStore, DenseAndSparseLookupConsistent) {
  ObjectStore store;
  std::vector<UUIDKey> ids;
  for (int i = 0; i < 100; ++i) {
    ids.push_back(store.createObject(ObjectType::Platform));
  }
  ASSERT_EQ(store.objectCount(), 100u);

  size_t dense_count = 0;
  store.forEachObject([&](const EntityRecord&) { ++dense_count; });
  ASSERT_EQ(dense_count, 100u);

  for (auto& id : ids) {
    ASSERT_NE(store.getRecord(id), nullptr);
  }
}

///< REQ_TACTICAL_OBJECTS_003: Entity exists without all components populated.
TEST(ObjectStore, SparseComponentsAbsentByDefault) {
  ObjectStore store;
  auto id = store.createObject(ObjectType::Platform);
  ASSERT_FALSE(store.kinematics().has(id));
  ASSERT_FALSE(store.milclass().has(id));
  ASSERT_EQ(store.kinematics().get(id), nullptr);
}

///< REQ_TACTICAL_OBJECTS_004: Version increments on mutation.
TEST(ObjectStore, VersionIncrementsOnUpdate) {
  ObjectStore store;
  auto id = store.createObject(ObjectType::Platform);
  const auto* r0 = store.getRecord(id);
  ASSERT_NE(r0, nullptr);
  uint64_t v0 = r0->version;

  KinematicsComponent kc;
  kc.position.lat = 51.5;
  store.kinematics().set(id, kc);
  store.bumpVersion(id);

  const auto* r1 = store.getRecord(id);
  uint64_t v1 = r1->version;
  ASSERT_GT(v1, v0);

  MilClassComponent mc;
  mc.profile.affiliation = Affiliation::Friendly;
  store.milclass().set(id, mc);
  store.bumpVersion(id);

  const auto* r2 = store.getRecord(id);
  ASSERT_GT(r2->version, v1);
}

///< REQ_TACTICAL_OBJECTS_003: Deleted objects are removed from all tables.
TEST(ObjectStore, DeleteRemovesFromAllTables) {
  ObjectStore store;
  auto id = store.createObject(ObjectType::Platform);

  KinematicsComponent kc;
  kc.position.lat = 10.0;
  store.kinematics().set(id, kc);
  ASSERT_TRUE(store.kinematics().has(id));

  store.deleteObject(id);
  ASSERT_EQ(store.getRecord(id), nullptr);
  ASSERT_FALSE(store.kinematics().has(id));

  size_t count = 0;
  store.forEachObject([&](const EntityRecord&) { ++count; });
  ASSERT_EQ(count, 0u);
}

///< REQ_TACTICAL_OBJECTS_003: Deleting a non-last element swaps it with the last.
TEST(ObjectStore, DeleteMiddleElementSwapsWithLast) {
  ObjectStore store;
  auto id0 = store.createObject(ObjectType::Platform);
  auto id1 = store.createObject(ObjectType::Person);
  auto id2 = store.createObject(ObjectType::Unit);

  // Add kinematics to all so removeFromAllComponents is exercised
  KinematicsComponent kc;
  kc.position.lat = 10.0;
  store.kinematics().set(id0, kc);
  store.kinematics().set(id1, kc);
  store.kinematics().set(id2, kc);

  // Delete the middle element (id1, index 1) -- triggers the idx != last swap
  ASSERT_TRUE(store.deleteObject(id1));
  ASSERT_EQ(store.objectCount(), 2u);

  // Remaining objects must still be retrievable
  ASSERT_NE(store.getRecord(id0), nullptr);
  ASSERT_NE(store.getRecord(id2), nullptr);
  ASSERT_EQ(store.getRecord(id1), nullptr);
}

///< REQ_TACTICAL_OBJECTS_045: Sparse objects don't allocate unused component entries.
TEST(ObjectStore, SparseObjectsDontAllocateUnusedComponents) {
  ObjectStore store;
  for (int i = 0; i < 50; ++i) {
    store.createObject(ObjectType::Platform);
  }
  ASSERT_EQ(store.milclass().size(), 0u);
  ASSERT_EQ(store.kinematics().size(), 0u);

  auto id = store.createObject(ObjectType::Platform);
  MilClassComponent mc;
  mc.profile.affiliation = Affiliation::Hostile;
  store.milclass().set(id, mc);
  ASSERT_EQ(store.milclass().size(), 1u);
  ASSERT_EQ(store.kinematics().size(), 0u);
}

///< Coverage: ObjectStore dirty mask API (setDirtyBits, clearDirtyMask, clearAllDirtyMasks).
TEST(ObjectStore, DirtyMaskSetClearAndGet) {
  ObjectStore store;
  auto id = store.createObject(ObjectType::Platform);
  ASSERT_EQ(store.getDirtyMask(id), 0u);

  store.setDirtyBits(id, 0x0001u);
  store.setDirtyBits(id, 0x0002u);
  ASSERT_EQ(store.getDirtyMask(id), 0x0003u);

  store.clearDirtyMask(id);
  ASSERT_EQ(store.getDirtyMask(id), 0u);

  store.setDirtyBits(id, 0x0010u);
  store.clearAllDirtyMasks();
  ASSERT_EQ(store.getDirtyMask(id), 0u);
}
