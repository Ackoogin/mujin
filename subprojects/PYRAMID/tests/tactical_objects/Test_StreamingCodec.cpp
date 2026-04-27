#include <gtest/gtest.h>
#include <StreamingCodec.h>
#include <store/ObjectStore.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static UUIDKey makeKey() {
  return UUIDKey{UUIDHelper::generateV4()};
}

static ObjectStore makeStoreWithEntity(UUIDKey id,
                                       Position pos = {1.0, 2.0, 3.0},
                                       Affiliation aff = Affiliation::Hostile) {
  ObjectStore store;
  // We can't call createObject with an existing key, so create then use the returned key
  (void)id;  // id is used as entity_id in encode calls; the store is populated separately
  return store;
}

// ---------------------------------------------------------------------------
// Enum ordinal round-trips
// ---------------------------------------------------------------------------

TEST(StreamingCodec, AffiliationOrdinalRoundTrip) {
  // AssumedFriend is the highest ordinal (=9); ordinals 0-9 are all consecutive.
  for (int i = 0; i <= static_cast<int>(Affiliation::AssumedFriend); ++i) {
    auto a = static_cast<Affiliation>(i);
    EXPECT_EQ(a, StreamingCodec::ordinalToAffiliation(StreamingCodec::affiliationToOrdinal(a)));
  }
}

TEST(StreamingCodec, ObjectTypeOrdinalRoundTrip) {
  for (int i = 0; i <= static_cast<int>(ObjectType::Zone); ++i) {
    auto t = static_cast<ObjectType>(i);
    EXPECT_EQ(t, StreamingCodec::ordinalToObjectType(StreamingCodec::objectTypeToOrdinal(t)));
  }
}

TEST(StreamingCodec, LifecycleStatusOrdinalRoundTrip) {
  for (int i = 0; i <= static_cast<int>(LifecycleStatus::Retired); ++i) {
    auto s = static_cast<LifecycleStatus>(i);
    EXPECT_EQ(s, StreamingCodec::ordinalToLifecycleStatus(StreamingCodec::lifecycleStatusToOrdinal(s)));
  }
}

TEST(StreamingCodec, BattleDimensionOrdinalRoundTrip) {
  for (int i = 0; i <= static_cast<int>(BattleDimension::SOF); ++i) {
    auto d = static_cast<BattleDimension>(i);
    EXPECT_EQ(d, StreamingCodec::ordinalToBattleDimension(StreamingCodec::battleDimensionToOrdinal(d)));
  }
}

TEST(StreamingCodec, MilStatusOrdinalRoundTrip) {
  for (int i = 0; i <= static_cast<int>(MilStatus::Known); ++i) {
    auto s = static_cast<MilStatus>(i);
    EXPECT_EQ(s, StreamingCodec::ordinalToMilStatus(StreamingCodec::milStatusToOrdinal(s)));
  }
}

TEST(StreamingCodec, EchelonOrdinalRoundTrip) {
  for (int i = 0; i <= static_cast<int>(Echelon::ArmyGroup); ++i) {
    auto e = static_cast<Echelon>(i);
    EXPECT_EQ(e, StreamingCodec::ordinalToEchelon(StreamingCodec::echelonToOrdinal(e)));
  }
}

TEST(StreamingCodec, MobilityOrdinalRoundTrip) {
  for (int i = 0; i <= static_cast<int>(Mobility::Amphibious); ++i) {
    auto m = static_cast<Mobility>(i);
    EXPECT_EQ(m, StreamingCodec::ordinalToMobility(StreamingCodec::mobilityToOrdinal(m)));
  }
}

// ---------------------------------------------------------------------------
// Single frame encode/decode round-trips
// ---------------------------------------------------------------------------

TEST(StreamingCodec, PositionOnlyUpdateRoundTrip) {
  EntityUpdateFrame frame;
  frame.message_type = STREAM_MSG_ENTITY_UPDATE;
  frame.entity_id  = makeKey();
  frame.version    = 42;
  frame.timestamp  = 1234.5;
  frame.field_mask = FieldMaskBit::POSITION;
  frame.position   = Position{51.5, -0.1, 100.0};

  auto encoded = StreamingCodec::encodeEntityUpdateFrame(frame);
  // position-only update: 1(type)+16(uuid)+8(ver)+8(ts)+2(mask)+24(pos) = 59 bytes
  EXPECT_LE(encoded.size(), 60u);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.message_type, STREAM_MSG_ENTITY_UPDATE);
  EXPECT_EQ(decoded.entity_id, frame.entity_id);
  EXPECT_EQ(decoded.version, 42u);
  EXPECT_DOUBLE_EQ(decoded.timestamp, 1234.5);
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::POSITION);
  ASSERT_TRUE(decoded.position.has_value());
  EXPECT_DOUBLE_EQ(decoded.position->lat, 51.5);
  EXPECT_DOUBLE_EQ(decoded.position->lon, -0.1);
  EXPECT_DOUBLE_EQ(decoded.position->alt, 100.0);
}

TEST(StreamingCodec, DeleteFrameRoundTrip) {
  EntityUpdateFrame frame;
  frame.message_type = STREAM_MSG_ENTITY_DELETE;
  frame.entity_id  = makeKey();
  frame.version    = 7;
  frame.timestamp  = 99.0;
  frame.field_mask = 0;

  auto encoded = StreamingCodec::encodeEntityUpdateFrame(frame);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.message_type, STREAM_MSG_ENTITY_DELETE);
  EXPECT_EQ(decoded.entity_id, frame.entity_id);
  EXPECT_EQ(decoded.version, 7u);
}

TEST(StreamingCodec, FullFieldMaskRoundTrip) {
  EntityUpdateFrame frame;
  frame.message_type = STREAM_MSG_ENTITY_UPDATE;
  frame.entity_id  = makeKey();
  frame.version    = 100;
  frame.timestamp  = 0.0;
  frame.field_mask = FieldMaskBit::ALL;
  frame.position   = Position{10.0, 20.0, 30.0};
  frame.velocity   = Velocity{1.0, 2.0, 3.0};
  frame.affiliation = Affiliation::Hostile;
  frame.object_type = ObjectType::Platform;
  frame.confidence  = 0.95;
  frame.lifecycle_status = LifecycleStatus::Active;

  MilClassProfile mp;
  mp.battle_dim  = BattleDimension::Air;
  mp.affiliation = Affiliation::Hostile;
  mp.status      = MilStatus::Present;
  mp.echelon     = Echelon::Company;
  mp.mobility    = Mobility::None;
  mp.hq          = true;
  mp.role        = "fighter";
  mp.source_sidc = "SFAPC----------";
  frame.mil_class = mp;

  BehaviorComponent bc;
  bc.behavior_pattern  = "patrol";
  bc.operational_state = "moving";
  frame.behavior = bc;

  frame.identity_name = "Eagle-1";

  auto encoded = StreamingCodec::encodeEntityUpdateFrame(frame);
  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());

  EXPECT_EQ(decoded.message_type, STREAM_MSG_ENTITY_UPDATE);
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::ALL);
  ASSERT_TRUE(decoded.position.has_value());
  EXPECT_DOUBLE_EQ(decoded.position->lat, 10.0);
  ASSERT_TRUE(decoded.velocity.has_value());
  EXPECT_DOUBLE_EQ(decoded.velocity->north, 1.0);
  ASSERT_TRUE(decoded.affiliation.has_value());
  EXPECT_EQ(*decoded.affiliation, Affiliation::Hostile);
  ASSERT_TRUE(decoded.object_type.has_value());
  EXPECT_EQ(*decoded.object_type, ObjectType::Platform);
  ASSERT_TRUE(decoded.confidence.has_value());
  EXPECT_DOUBLE_EQ(*decoded.confidence, 0.95);
  ASSERT_TRUE(decoded.lifecycle_status.has_value());
  EXPECT_EQ(*decoded.lifecycle_status, LifecycleStatus::Active);
  ASSERT_TRUE(decoded.mil_class.has_value());
  EXPECT_EQ(decoded.mil_class->battle_dim, BattleDimension::Air);
  EXPECT_TRUE(decoded.mil_class->hq);
  EXPECT_EQ(decoded.mil_class->role, "fighter");
  EXPECT_EQ(decoded.mil_class->source_sidc, "SFAPC----------");
  ASSERT_TRUE(decoded.behavior.has_value());
  EXPECT_EQ(decoded.behavior->behavior_pattern, "patrol");
  ASSERT_TRUE(decoded.identity_name.has_value());
  EXPECT_EQ(*decoded.identity_name, "Eagle-1");
}

TEST(StreamingCodec, AffiliationOnlyUpdateRoundTrip) {
  EntityUpdateFrame frame;
  frame.message_type = STREAM_MSG_ENTITY_UPDATE;
  frame.entity_id  = makeKey();
  frame.version    = 5;
  frame.timestamp  = 0.0;
  frame.field_mask = FieldMaskBit::AFFILIATION;
  frame.affiliation = Affiliation::Neutral;

  auto encoded = StreamingCodec::encodeEntityUpdateFrame(frame);
  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());

  EXPECT_EQ(decoded.field_mask, FieldMaskBit::AFFILIATION);
  ASSERT_TRUE(decoded.affiliation.has_value());
  EXPECT_EQ(*decoded.affiliation, Affiliation::Neutral);
  EXPECT_FALSE(decoded.position.has_value());
  EXPECT_FALSE(decoded.velocity.has_value());
}

// ---------------------------------------------------------------------------
// Fuzz-style truncation safety tests
// ---------------------------------------------------------------------------

TEST(StreamingCodec, TruncatedBufferReturnsError) {
  EntityUpdateFrame frame;
  frame.message_type = STREAM_MSG_ENTITY_UPDATE;
  frame.entity_id  = makeKey();
  frame.version    = 1;
  frame.timestamp  = 0.0;
  frame.field_mask = FieldMaskBit::POSITION | FieldMaskBit::VELOCITY;
  frame.position   = Position{1.0, 2.0, 3.0};
  frame.velocity   = Velocity{0.1, 0.2, 0.3};

  auto encoded = StreamingCodec::encodeEntityUpdateFrame(frame);

  // Try truncations at every byte -- none should crash; error frames have message_type==0
  for (size_t trunc = 0; trunc < encoded.size(); ++trunc) {
    auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), trunc);
    // Either a valid partial decode or an error sentinel
    EXPECT_TRUE(decoded.message_type == 0 ||
                decoded.message_type == STREAM_MSG_ENTITY_UPDATE ||
                decoded.message_type == STREAM_MSG_ENTITY_DELETE);
  }
}

TEST(StreamingCodec, EmptyBufferReturnsError) {
  auto decoded = StreamingCodec::decodeEntityUpdate(nullptr, 0);
  EXPECT_EQ(decoded.message_type, 0u);
}

// ---------------------------------------------------------------------------
// Batch frame encode/decode
// ---------------------------------------------------------------------------

TEST(StreamingCodec, BatchFrameEmpty) {
  std::vector<EntityUpdateFrame> frames;
  auto encoded = StreamingCodec::encodeBatchFrame(frames, 0.0);
  EXPECT_FALSE(encoded.empty());

  auto decoded = StreamingCodec::decodeBatchFrame(encoded.data(), encoded.size());
  EXPECT_TRUE(decoded.empty());
}

TEST(StreamingCodec, BatchFrameSingleEntity) {
  EntityUpdateFrame f;
  f.message_type = STREAM_MSG_ENTITY_UPDATE;
  f.entity_id  = makeKey();
  f.version    = 1;
  f.timestamp  = 1.0;
  f.field_mask = FieldMaskBit::POSITION;
  f.position   = Position{1.0, 2.0, 3.0};

  auto encoded = StreamingCodec::encodeBatchFrame({f}, 1.0);
  auto decoded = StreamingCodec::decodeBatchFrame(encoded.data(), encoded.size());

  ASSERT_EQ(decoded.size(), 1u);
  EXPECT_EQ(decoded[0].entity_id, f.entity_id);
  ASSERT_TRUE(decoded[0].position.has_value());
  EXPECT_DOUBLE_EQ(decoded[0].position->lat, 1.0);
}

TEST(StreamingCodec, BatchFrameHundredEntities) {
  std::vector<EntityUpdateFrame> frames;
  frames.reserve(100);
  for (int i = 0; i < 100; ++i) {
    EntityUpdateFrame f;
    f.message_type = STREAM_MSG_ENTITY_UPDATE;
    f.entity_id  = makeKey();
    f.version    = static_cast<uint64_t>(i + 1);
    f.timestamp  = static_cast<double>(i);
    f.field_mask = FieldMaskBit::POSITION;
    f.position   = Position{static_cast<double>(i), 0.0, 0.0};
    frames.push_back(f);
  }

  auto encoded = StreamingCodec::encodeBatchFrame(frames, 0.0);
  auto decoded = StreamingCodec::decodeBatchFrame(encoded.data(), encoded.size());

  ASSERT_EQ(decoded.size(), 100u);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(decoded[i].version, static_cast<uint64_t>(i + 1));
    ASSERT_TRUE(decoded[i].position.has_value());
    EXPECT_DOUBLE_EQ(decoded[i].position->lat, static_cast<double>(i));
  }
}

TEST(StreamingCodec, BatchFrameTruncatedSafe) {
  EntityUpdateFrame f;
  f.message_type = STREAM_MSG_ENTITY_UPDATE;
  f.entity_id  = makeKey();
  f.version    = 1;
  f.timestamp  = 0.0;
  f.field_mask = FieldMaskBit::POSITION;
  f.position   = Position{1.0, 2.0, 3.0};

  auto encoded = StreamingCodec::encodeBatchFrame({f}, 0.0);

  for (size_t trunc = 0; trunc < encoded.size(); ++trunc) {
    // Must not crash
    auto decoded = StreamingCodec::decodeBatchFrame(encoded.data(), trunc);
    // Result may be empty or partial -- that's fine
    (void)decoded;
  }
}

// ---------------------------------------------------------------------------
// encodeEntityUpdate (store-backed) round-trip
// ---------------------------------------------------------------------------

TEST(StreamingCodec, EncodeFromStorePositionOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  KinematicsComponent kc;
  kc.position = {51.0, -0.5, 200.0};
  store.kinematics().set(id, kc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::POSITION, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.message_type, STREAM_MSG_ENTITY_UPDATE);
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::POSITION);
  ASSERT_TRUE(decoded.position.has_value());
  EXPECT_DOUBLE_EQ(decoded.position->lat, 51.0);
}

TEST(StreamingCodec, EncodeFromStoreVelocityOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  KinematicsComponent kc;
  kc.velocity = {1.0, 2.0, 3.0};
  store.kinematics().set(id, kc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::VELOCITY, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::VELOCITY);
  ASSERT_TRUE(decoded.velocity.has_value());
  EXPECT_DOUBLE_EQ(decoded.velocity->north, 1.0);
}

TEST(StreamingCodec, EncodeFromStoreAffiliationOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  MilClassComponent mc;
  mc.profile.affiliation = Affiliation::Hostile;
  store.milclass().set(id, mc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::AFFILIATION, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::AFFILIATION);
  ASSERT_TRUE(decoded.affiliation.has_value());
  EXPECT_EQ(*decoded.affiliation, Affiliation::Hostile);
}

TEST(StreamingCodec, EncodeFromStoreObjectTypeOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Unit);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::OBJECT_TYPE, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::OBJECT_TYPE);
  ASSERT_TRUE(decoded.object_type.has_value());
  EXPECT_EQ(*decoded.object_type, ObjectType::Unit);
}

TEST(StreamingCodec, EncodeFromStoreConfidenceOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  QualityComponent qc;
  qc.confidence = 0.85;
  store.quality().set(id, qc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::CONFIDENCE, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::CONFIDENCE);
  ASSERT_TRUE(decoded.confidence.has_value());
  EXPECT_DOUBLE_EQ(*decoded.confidence, 0.85);
}

TEST(StreamingCodec, EncodeFromStoreLifecycleStatusOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::LIFECYCLE_STATUS, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::LIFECYCLE_STATUS);
  ASSERT_TRUE(decoded.lifecycle_status.has_value());
  EXPECT_EQ(*decoded.lifecycle_status, LifecycleStatus::Active);
}

TEST(StreamingCodec, EncodeFromStoreMilClassOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  MilClassComponent mc;
  mc.profile.battle_dim = BattleDimension::Air;
  mc.profile.role = "fighter";
  store.milclass().set(id, mc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::MIL_CLASS, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::MIL_CLASS);
  ASSERT_TRUE(decoded.mil_class.has_value());
  EXPECT_EQ(decoded.mil_class->battle_dim, BattleDimension::Air);
  EXPECT_EQ(decoded.mil_class->role, "fighter");
}

TEST(StreamingCodec, EncodeFromStoreBehaviorOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  BehaviorComponent bc;
  bc.behavior_pattern = "patrol";
  bc.operational_state = "moving";
  store.behaviors().set(id, bc);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::BEHAVIOR, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::BEHAVIOR);
  ASSERT_TRUE(decoded.behavior.has_value());
  EXPECT_EQ(decoded.behavior->behavior_pattern, "patrol");
}

TEST(StreamingCodec, EncodeFromStoreIdentityNameOnly) {
  ObjectStore store;
  UUIDKey id = store.createObject(ObjectType::Platform);

  IdentityComponent ic;
  ic.name = "Eagle-1";
  store.identities().set(id, ic);

  const auto* rec = store.getRecord(id);
  ASSERT_NE(rec, nullptr);

  auto encoded = StreamingCodec::encodeEntityUpdate(
      id, rec->version, 0.0, FieldMaskBit::IDENTITY_NAME, store);

  auto decoded = StreamingCodec::decodeEntityUpdate(encoded.data(), encoded.size());
  EXPECT_EQ(decoded.field_mask, FieldMaskBit::IDENTITY_NAME);
  ASSERT_TRUE(decoded.identity_name.has_value());
  EXPECT_EQ(*decoded.identity_name, "Eagle-1");
}
