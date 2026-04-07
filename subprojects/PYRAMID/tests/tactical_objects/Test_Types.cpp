#include <gtest/gtest.h>
#include <TacticalObjectsTypes.h>
#include <uuid/UUIDHelper.h>

#include <set>
#include <unordered_set>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

///< REQ_TACTICAL_OBJECTS_001: Enums for object type, affiliation, battle dimension, etc.
TEST(TacticalTypes, ObjectTypeEnumCoversAllValues) {
  std::set<int> values;
  values.insert(static_cast<int>(ObjectType::Platform));
  values.insert(static_cast<int>(ObjectType::Person));
  values.insert(static_cast<int>(ObjectType::Equipment));
  values.insert(static_cast<int>(ObjectType::Unit));
  values.insert(static_cast<int>(ObjectType::Formation));
  values.insert(static_cast<int>(ObjectType::Installation));
  values.insert(static_cast<int>(ObjectType::Feature));
  values.insert(static_cast<int>(ObjectType::Route));
  values.insert(static_cast<int>(ObjectType::Point));
  values.insert(static_cast<int>(ObjectType::Area));
  values.insert(static_cast<int>(ObjectType::Zone));
  ASSERT_EQ(values.size(), 11u);
}

///< REQ_TACTICAL_OBJECTS_001: UUIDKey wraps pyramid::core::uuid::UUID for use in unordered containers.
TEST(TacticalTypes, UUIDKeyHashAndEquality) {
  auto a = UUIDHelper::generateV4();
  auto b = UUIDHelper::generateV4();
  UUIDKey ka{a};
  UUIDKey kb{b};

  std::unordered_set<UUIDKey> s;
  s.insert(ka);
  s.insert(kb);
  ASSERT_EQ(s.size(), 2u);

  UUIDKey ka2{a};
  ASSERT_EQ(ka, ka2);
  ASSERT_EQ(std::hash<UUIDKey>{}(ka), std::hash<UUIDKey>{}(ka2));

  s.insert(ka2);
  ASSERT_EQ(s.size(), 2u);
}

///< REQ_TACTICAL_OBJECTS_002: SourceRef, Position, MilClassProfile structs exist and are default-constructible.
TEST(TacticalTypes, CoreStructsDefaultConstruct) {
  SourceRef sr;
  Position pos;
  MilClassProfile mcp;
  Observation obs;
  ObjectDefinition od;
  ASSERT_DOUBLE_EQ(obs.confidence, 0.0);
  ASSERT_DOUBLE_EQ(pos.lat, 0.0);
  ASSERT_DOUBLE_EQ(pos.lon, 0.0);
  ASSERT_EQ(mcp.affiliation, Affiliation::Unknown);
}
