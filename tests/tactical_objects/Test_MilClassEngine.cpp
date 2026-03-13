#include <gtest/gtest.h>
#include <milclass/MilClassEngine.h>
#include <store/ObjectStore.h>

using namespace tactical_objects;

///< REQ_TACTICAL_OBJECTS_014: MilClassEngine stores all 2525B fields.
TEST(MilClassEngine, StoreAndRetrieveAllFields) {
  auto store = std::make_shared<ObjectStore>();
  MilClassEngine engine(store);

  auto id = store->createObject(ObjectType::Platform);

  MilClassProfile profile;
  profile.battle_dim = BattleDimension::Air;
  profile.affiliation = Affiliation::Hostile;
  profile.role = "fighter";
  profile.status = MilStatus::Present;
  profile.echelon = Echelon::Brigade;
  profile.mobility = Mobility::None;
  profile.hq = false;
  profile.task_force = true;
  profile.feint_dummy = false;
  profile.installation = false;

  engine.setProfile(id, profile);
  auto got = engine.getProfile(id);
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->battle_dim, BattleDimension::Air);
  ASSERT_EQ(got->affiliation, Affiliation::Hostile);
  ASSERT_EQ(got->role, "fighter");
  ASSERT_EQ(got->status, MilStatus::Present);
  ASSERT_TRUE(got->task_force);
  ASSERT_FALSE(got->hq);
}

///< REQ_TACTICAL_OBJECTS_015: Source SIDC is preserved.
TEST(MilClassEngine, PreserveSourceSIDC) {
  auto store = std::make_shared<ObjectStore>();
  MilClassEngine engine(store);

  auto id = store->createObject(ObjectType::Platform);

  MilClassProfile profile;
  profile.source_sidc = "SHGPUCFRMS-----";
  engine.setProfile(id, profile);

  auto got = engine.getProfile(id);
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->source_sidc, "SHGPUCFRMS-----");
}

///< REQ_TACTICAL_OBJECTS_016: Derived symbol key is deterministic.
TEST(MilClassEngine, DerivedSymbolKeyDeterministic) {
  auto store = std::make_shared<ObjectStore>();
  MilClassEngine engine(store);

  auto id_a = store->createObject(ObjectType::Platform);
  auto id_b = store->createObject(ObjectType::Platform);

  MilClassProfile profile;
  profile.battle_dim = BattleDimension::Ground;
  profile.affiliation = Affiliation::Friendly;
  profile.role = "armor";
  profile.status = MilStatus::Present;
  profile.echelon = Echelon::Battalion;
  profile.mobility = Mobility::Tracked;
  profile.hq = true;

  engine.setProfile(id_a, profile);
  engine.setProfile(id_b, profile);

  auto key_a = engine.deriveSymbolKey(id_a);
  auto key_b = engine.deriveSymbolKey(id_b);
  ASSERT_FALSE(key_a.empty());
  ASSERT_EQ(key_a, key_b);

  profile.affiliation = Affiliation::Hostile;
  engine.setProfile(id_b, profile);
  auto key_b2 = engine.deriveSymbolKey(id_b);
  ASSERT_NE(key_a, key_b2);
}
