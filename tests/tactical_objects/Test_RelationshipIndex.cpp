#include <gtest/gtest.h>
#include <relationship/RelationshipIndex.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static UUIDKey makeKey() { return UUIDKey{UUIDHelper::generateV4()}; }

///< REQ_TACTICAL_OBJECTS_049: Relationship records are stored as first-class linked entities.
TEST(RelationshipIndex, StoreAndRetrieveRelationship) {
  RelationshipIndex idx;
  auto subj = makeKey();
  auto obj = makeKey();
  auto rel_id = makeKey();

  RelationshipRecord rec;
  rec.relationship_id = rel_id;
  rec.subject_id = subj;
  rec.object_id = obj;
  rec.type = RelationshipType::MemberOfUnit;
  rec.confidence = 0.95;
  rec.source = "intel-report";
  idx.insert(rec);

  auto by_subj = idx.bySubject(subj);
  ASSERT_EQ(by_subj.size(), 1u);
  ASSERT_EQ(by_subj[0].object_id, obj);

  auto by_obj = idx.byObject(obj);
  ASSERT_EQ(by_obj.size(), 1u);
  ASSERT_EQ(by_obj[0].subject_id, subj);
}

///< REQ_TACTICAL_OBJECTS_050: Hierarchical relationships are queryable.
TEST(RelationshipIndex, HierarchicalRelationshipQuery) {
  RelationshipIndex idx;
  auto unit = makeKey();
  auto person1 = makeKey();
  auto person2 = makeKey();
  auto platform = makeKey();
  auto equipment = makeKey();

  RelationshipRecord r1;
  r1.relationship_id = makeKey();
  r1.subject_id = person1;
  r1.object_id = unit;
  r1.type = RelationshipType::MemberOfUnit;
  idx.insert(r1);

  RelationshipRecord r2;
  r2.relationship_id = makeKey();
  r2.subject_id = person2;
  r2.object_id = unit;
  r2.type = RelationshipType::MemberOfUnit;
  idx.insert(r2);

  RelationshipRecord r3;
  r3.relationship_id = makeKey();
  r3.subject_id = equipment;
  r3.object_id = platform;
  r3.type = RelationshipType::EquipmentOnPlatform;
  idx.insert(r3);

  auto members = idx.byType(RelationshipType::MemberOfUnit);
  ASSERT_EQ(members.size(), 2u);

  auto equip = idx.bySubjectAndType(equipment, RelationshipType::EquipmentOnPlatform);
  ASSERT_EQ(equip.size(), 1u);
  ASSERT_EQ(equip[0].object_id, platform);
}

///< REQ_TACTICAL_OBJECTS_051: Tactical relationships are queryable.
TEST(RelationshipIndex, TacticalRelationshipQuery) {
  RelationshipIndex idx;
  auto escort = makeKey();
  auto vip = makeKey();
  auto attacker = makeKey();
  auto target = makeKey();

  RelationshipRecord r1;
  r1.relationship_id = makeKey();
  r1.subject_id = escort;
  r1.object_id = vip;
  r1.type = RelationshipType::Escorting;
  idx.insert(r1);

  RelationshipRecord r2;
  r2.relationship_id = makeKey();
  r2.subject_id = attacker;
  r2.object_id = target;
  r2.type = RelationshipType::Engaging;
  idx.insert(r2);

  auto escorting = idx.byType(RelationshipType::Escorting);
  ASSERT_EQ(escorting.size(), 1u);
  ASSERT_EQ(escorting[0].subject_id, escort);

  auto engaging = idx.byType(RelationshipType::Engaging);
  ASSERT_EQ(engaging.size(), 1u);
  ASSERT_EQ(engaging[0].subject_id, attacker);

  auto protecting = idx.byType(RelationshipType::Protecting);
  ASSERT_EQ(protecting.size(), 0u);
}

///< REQ_TACTICAL_OBJECTS_052: Relationship confidence and provenance are retained.
TEST(RelationshipIndex, ConfidenceAndProvenanceRetained) {
  RelationshipIndex idx;
  auto rel_id = makeKey();

  RelationshipRecord rec;
  rec.relationship_id = rel_id;
  rec.subject_id = makeKey();
  rec.object_id = makeKey();
  rec.type = RelationshipType::Supporting;
  rec.confidence = 0.65;
  rec.source = "sigint-analysis";
  rec.established_at = 1700000000.0;
  idx.insert(rec);

  const auto* got = idx.get(rel_id);
  ASSERT_NE(got, nullptr);
  ASSERT_DOUBLE_EQ(got->confidence, 0.65);
  ASSERT_EQ(got->source, "sigint-analysis");
  ASSERT_DOUBLE_EQ(got->established_at, 1700000000.0);
}

///< REQ_TACTICAL_OBJECTS_053: Relationship removal cleans all indexes.
TEST(RelationshipIndex, RemoveRelationshipCleansAllIndexes) {
  RelationshipIndex idx;
  auto subj = makeKey();
  auto obj  = makeKey();
  auto rel_id = makeKey();

  RelationshipRecord rec;
  rec.relationship_id = rel_id;
  rec.subject_id = subj;
  rec.object_id  = obj;
  rec.type = RelationshipType::Escorting;
  idx.insert(rec);

  // Verify present before removal
  ASSERT_NE(idx.get(rel_id), nullptr);
  ASSERT_EQ(idx.bySubject(subj).size(), 1u);

  // remove() returns true and cleans subject/object indexes
  ASSERT_TRUE(idx.remove(rel_id));
  ASSERT_EQ(idx.get(rel_id), nullptr);
  ASSERT_EQ(idx.bySubject(subj).size(), 0u);
  ASSERT_EQ(idx.byObject(obj).size(), 0u);

  // remove() returns false for unknown ID
  ASSERT_FALSE(idx.remove(makeKey()));
}

///< REQ_TACTICAL_OBJECTS_053: byEntity combines subject and object indexes.
TEST(RelationshipIndex, ByEntityCombinesSubjectAndObject) {
  RelationshipIndex idx;
  auto entityA = makeKey();
  auto entityB = makeKey();
  auto entityC = makeKey();

  // entityA is subject of r1, object of r2
  RelationshipRecord r1;
  r1.relationship_id = makeKey();
  r1.subject_id = entityA;
  r1.object_id  = entityB;
  r1.type = RelationshipType::Tracking;
  idx.insert(r1);

  RelationshipRecord r2;
  r2.relationship_id = makeKey();
  r2.subject_id = entityC;
  r2.object_id  = entityA;
  r2.type = RelationshipType::Protecting;
  idx.insert(r2);

  auto all = idx.byEntity(entityA);
  ASSERT_EQ(all.size(), 2u);
}
