#include <gtest/gtest.h>
#include <query/QueryEngine.h>
#include <milclass/MilClassEngine.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

class QueryEngineTest : public ::testing::Test {
protected:
  std::shared_ptr<ObjectStore> store = std::make_shared<ObjectStore>();
  std::shared_ptr<SpatialIndex> spatial = std::make_shared<SpatialIndex>(1.0);
  QueryEngine qe{store, spatial};
  MilClassEngine milclass{store};

  UUIDKey addEntity(ObjectType type, double lat, double lon,
                    Affiliation aff = Affiliation::Unknown,
                    const std::string& src_sys = "",
                    const std::string& src_eid = "",
                    double timestamp = 0.0) {
    auto id = store->createObject(type);
    KinematicsComponent kc;
    kc.position.lat = lat;
    kc.position.lon = lon;
    kc.timestamp = timestamp;
    store->kinematics().set(id, kc);
    spatial->insert(id, kc.position);

    if (aff != Affiliation::Unknown) {
      MilClassProfile p;
      p.affiliation = aff;
      milclass.setProfile(id, p);
    }

    if (!src_sys.empty()) {
      CorrelationComponent cc;
      SourceRef sr;
      sr.source_system = src_sys;
      sr.source_entity_id = src_eid;
      cc.source_refs.push_back(sr);
      store->correlation().set(id, cc);
    }

    if (timestamp > 0.0) {
      QualityComponent qc;
      qc.freshness_timestamp = timestamp;
      store->quality().set(id, qc);
    }

    return id;
  }
};

///< REQ_TACTICAL_OBJECTS_025: Query by UUID returns the exact entity.
TEST_F(QueryEngineTest, QueryByUUID) {
  auto id = addEntity(ObjectType::Platform, 51.0, 0.0);
  addEntity(ObjectType::Platform, 52.0, 1.0);

  QueryRequest req;
  req.by_uuid = id;
  auto resp = qe.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
  ASSERT_EQ(resp.entries[0].id, id);
}

///< REQ_TACTICAL_OBJECTS_010: Query by source system identifier.
TEST_F(QueryEngineTest, QueryBySourceSystem) {
  addEntity(ObjectType::Platform, 51.0, 0.0, Affiliation::Unknown, "radar", "T01");
  addEntity(ObjectType::Platform, 52.0, 1.0, Affiliation::Unknown, "elint", "T02");

  QueryRequest req;
  req.by_source_system = std::string("radar");
  auto resp = qe.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
}

///< REQ_TACTICAL_OBJECTS_011: Query by type.
TEST_F(QueryEngineTest, QueryByObjectType) {
  addEntity(ObjectType::Platform, 51.0, 0.0);
  addEntity(ObjectType::Person, 52.0, 1.0);

  QueryRequest req;
  req.by_type = ObjectType::Person;
  auto resp = qe.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
  ASSERT_EQ(resp.entries[0].record.type, ObjectType::Person);
}

///< REQ_TACTICAL_OBJECTS_012: Query by affiliation.
TEST_F(QueryEngineTest, QueryByAffiliation) {
  addEntity(ObjectType::Platform, 51.0, 0.0, Affiliation::Friendly);
  addEntity(ObjectType::Platform, 52.0, 1.0, Affiliation::Hostile);

  QueryRequest req;
  req.by_affiliation = Affiliation::Hostile;
  auto resp = qe.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
}

///< REQ_TACTICAL_OBJECTS_013: Compound query combines type + region.
TEST_F(QueryEngineTest, CompoundQueryTypeAndRegion) {
  addEntity(ObjectType::Platform, 51.0, 0.0);
  addEntity(ObjectType::Person, 51.1, 0.1);
  addEntity(ObjectType::Platform, 90.0, 90.0);

  QueryRequest req;
  req.by_type = ObjectType::Platform;
  req.by_region = BoundingBox{50.0, 52.0, -1.0, 1.0};
  auto resp = qe.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
}

///< REQ_TACTICAL_OBJECTS_044: Freshness filter excludes stale entities.
TEST_F(QueryEngineTest, FreshnessFilterExcludesStale) {
  addEntity(ObjectType::Platform, 51.0, 0.0, Affiliation::Unknown, "", "", 100.0);
  addEntity(ObjectType::Platform, 52.0, 1.0, Affiliation::Unknown, "", "", 200.0);

  QueryRequest req;
  req.max_age_seconds = 50.0;
  req.current_time = 210.0;
  auto resp = qe.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
}
