#include <gtest/gtest.h>
#include <zone/ZoneEngine.h>
#include <store/ObjectStore.h>
#include <spatial/SpatialIndex.h>
#include <geo/GeoMath.h>
#include <TacticalObjectsRuntime.h>
#include <uuid/UUIDHelper.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

// ---------------------------------------------------------------------------
// Fixture: pre-seeds entities and zones for throughput measurement
// ---------------------------------------------------------------------------
class ZonePerformanceTest : public ::testing::Test {
protected:
  std::shared_ptr<ObjectStore> store  = std::make_shared<ObjectStore>();
  std::shared_ptr<SpatialIndex> spatial = std::make_shared<SpatialIndex>(1.0);
  ZoneEngine engine{store, spatial};

  std::mt19937 rng{42};

  // Latitude/longitude helpers ------------------------------------------
  double randLat() {
    std::uniform_real_distribution<double> d(-60.0, 70.0);
    return d(rng);
  }
  double randLon() {
    std::uniform_real_distribution<double> d(-170.0, 170.0);
    return d(rng);
  }

  // Zone factories ------------------------------------------------------
  UUIDKey makeCircleZone(double lat, double lon, double radius_m) {
    ZoneDefinition def;
    def.zone_type = ZoneType::AOI;
    def.geometry.geometry_type = ZoneType::AOI;
    def.geometry.center = Position{lat, lon, 0};
    def.geometry.radius_m = radius_m;
    return engine.upsertZone(def);
  }

  UUIDKey makeSquareZone(double lat, double lon, double half_deg) {
    ZoneDefinition def;
    def.zone_type = ZoneType::NoGoArea;
    def.geometry.geometry_type = ZoneType::NoGoArea;
    def.geometry.vertices = {
      Position{lat - half_deg, lon - half_deg, 0},
      Position{lat - half_deg, lon + half_deg, 0},
      Position{lat + half_deg, lon + half_deg, 0},
      Position{lat + half_deg, lon - half_deg, 0}
    };
    return engine.upsertZone(def);
  }

  // Timer ---------------------------------------------------------------
  using Clock = std::chrono::high_resolution_clock;
  static double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }
};

// ========================================================================
// 1.  Thousands of entities tested against a single zone
// ========================================================================
TEST_F(ZonePerformanceTest, ThousandEntitiesAgainstOnePolygonZone) {
  auto zone_id = makeSquareZone(51.5, -0.1, 2.0);

  constexpr int N = 5000;
  std::vector<Position> positions;
  positions.reserve(N);
  for (int i = 0; i < N; ++i)
    positions.push_back(Position{randLat(), randLon(), 0});

  auto t0 = Clock::now();
  int inside_count = 0;
  for (auto& p : positions)
    inside_count += engine.isInside(p, zone_id) ? 1 : 0;
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] " << N << " isInside(polygon) checks: "
            << ms << " ms  (" << (ms / N * 1000.0) << " us/check, "
            << inside_count << " inside)\n";
  SUCCEED();
}

TEST_F(ZonePerformanceTest, ThousandEntitiesAgainstOneCircleZone) {
  auto zone_id = makeCircleZone(51.5, -0.1, 50000.0);

  constexpr int N = 5000;
  std::vector<Position> positions;
  positions.reserve(N);
  for (int i = 0; i < N; ++i)
    positions.push_back(Position{randLat(), randLon(), 0});

  auto t0 = Clock::now();
  int inside_count = 0;
  for (auto& p : positions)
    inside_count += engine.isInside(p, zone_id) ? 1 : 0;
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] " << N << " isInside(circle/haversine) checks: "
            << ms << " ms  (" << (ms / N * 1000.0) << " us/check, "
            << inside_count << " inside)\n";
  SUCCEED();
}

// ========================================================================
// 2.  Single entity tested against thousands of zones
// ========================================================================
TEST_F(ZonePerformanceTest, OneEntityAgainstThousandPolygonZones) {
  constexpr int Z = 2000;
  std::vector<UUIDKey> zone_ids;
  zone_ids.reserve(Z);
  for (int i = 0; i < Z; ++i)
    zone_ids.push_back(makeSquareZone(randLat(), randLon(), 0.5));

  Position probe{51.5, -0.1, 0};

  auto t0 = Clock::now();
  int inside_count = 0;
  for (auto& zid : zone_ids)
    inside_count += engine.isInside(probe, zid) ? 1 : 0;
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] 1 entity vs " << Z << " polygon zones: "
            << ms << " ms  (" << (ms / Z * 1000.0) << " us/zone, "
            << inside_count << " hits)\n";
  SUCCEED();
}

TEST_F(ZonePerformanceTest, OneEntityAgainstThousandCircleZones) {
  constexpr int Z = 2000;
  std::vector<UUIDKey> zone_ids;
  zone_ids.reserve(Z);
  for (int i = 0; i < Z; ++i)
    zone_ids.push_back(makeCircleZone(randLat(), randLon(), 10000.0));

  Position probe{51.5, -0.1, 0};

  auto t0 = Clock::now();
  int inside_count = 0;
  for (auto& zid : zone_ids)
    inside_count += engine.isInside(probe, zid) ? 1 : 0;
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] 1 entity vs " << Z << " circle zones: "
            << ms << " ms  (" << (ms / Z * 1000.0) << " us/zone, "
            << inside_count << " hits)\n";
  SUCCEED();
}

// ========================================================================
// 3.  N x M cross-product: many entities against many zones
// ========================================================================
TEST_F(ZonePerformanceTest, ManyEntitiesManyZonesCrossProduct) {
  constexpr int NUM_ENTITIES = 1000;
  constexpr int NUM_ZONES    = 500;

  std::vector<Position> entities;
  entities.reserve(NUM_ENTITIES);
  for (int i = 0; i < NUM_ENTITIES; ++i)
    entities.push_back(Position{randLat(), randLon(), 0});

  std::vector<UUIDKey> zone_ids;
  zone_ids.reserve(NUM_ZONES);
  for (int i = 0; i < NUM_ZONES; ++i) {
    if (i % 2 == 0)
      zone_ids.push_back(makeSquareZone(randLat(), randLon(), 1.0));
    else
      zone_ids.push_back(makeCircleZone(randLat(), randLon(), 50000.0));
  }

  auto t0 = Clock::now();
  long long total_checks = 0;
  int inside_count = 0;
  for (auto& pos : entities) {
    for (auto& zid : zone_ids) {
      inside_count += engine.isInside(pos, zid) ? 1 : 0;
      ++total_checks;
    }
  }
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] " << NUM_ENTITIES << " entities x " << NUM_ZONES
            << " zones = " << total_checks << " checks: "
            << ms << " ms  (" << (ms / total_checks * 1e6) << " ns/check, "
            << inside_count << " hits)\n";
  SUCCEED();
}

// ========================================================================
// 4.  Boundary transition tracking at scale
// ========================================================================
TEST_F(ZonePerformanceTest, TransitionTrackingThousandEntities) {
  constexpr int NUM_ENTITIES = 2000;
  constexpr int NUM_ZONES    = 50;
  constexpr int STEPS        = 5;

  std::vector<UUIDKey> obj_ids;
  obj_ids.reserve(NUM_ENTITIES);
  for (int i = 0; i < NUM_ENTITIES; ++i)
    obj_ids.push_back(UUIDKey{UUIDHelper::generateV4()});

  std::vector<UUIDKey> zone_ids;
  zone_ids.reserve(NUM_ZONES);
  for (int i = 0; i < NUM_ZONES; ++i)
    zone_ids.push_back(makeSquareZone(randLat(), randLon(), 2.0));

  auto t0 = Clock::now();
  long long total_evals = 0;
  int entering = 0, leaving = 0;
  for (int step = 0; step < STEPS; ++step) {
    double time = static_cast<double>(step) * 10.0;
    for (auto& oid : obj_ids) {
      Position pos{randLat(), randLon(), 0};
      for (auto& zid : zone_ids) {
        auto rel = engine.evaluateTransition(oid, zid, pos, time);
        entering += (rel == ZoneRelationship::Entering) ? 1 : 0;
        leaving  += (rel == ZoneRelationship::Leaving)  ? 1 : 0;
        ++total_evals;
      }
    }
  }
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] " << total_evals << " transition evaluations ("
            << NUM_ENTITIES << " entities x " << NUM_ZONES << " zones x "
            << STEPS << " steps): " << ms << " ms  ("
            << (ms / total_evals * 1e6) << " ns/eval, "
            << entering << " enter, " << leaving << " leave)\n";
  SUCCEED();
}

// ========================================================================
// 5.  Full-stack via TacticalObjectsRuntime (create + zone-check)
// ========================================================================
TEST_F(ZonePerformanceTest, RuntimeCreateAndZoneCheckAtScale) {
  TacticalObjectsRuntime rt;

  constexpr int NUM_ENTITIES = 3000;
  constexpr int NUM_ZONES    = 200;

  // Create zones
  std::vector<UUIDKey> zone_ids;
  zone_ids.reserve(NUM_ZONES);
  for (int i = 0; i < NUM_ZONES; ++i) {
    ZoneDefinition def;
    if (i % 3 == 0) {
      def.zone_type = ZoneType::AOI;
      def.geometry.center = Position{randLat(), randLon(), 0};
      def.geometry.radius_m = 30000.0;
    } else {
      def.zone_type = ZoneType::RestrictedArea;
      double lat = randLat(), lon = randLon();
      def.geometry.geometry_type = ZoneType::RestrictedArea;
      def.geometry.vertices = {
        Position{lat - 0.5, lon - 0.5, 0},
        Position{lat - 0.5, lon + 0.5, 0},
        Position{lat + 0.5, lon + 0.5, 0},
        Position{lat + 0.5, lon - 0.5, 0}
      };
    }
    zone_ids.push_back(rt.createZone(def));
  }

  // Create entities
  std::vector<UUIDKey> entity_ids;
  entity_ids.reserve(NUM_ENTITIES);
  auto t_create0 = Clock::now();
  for (int i = 0; i < NUM_ENTITIES; ++i) {
    ObjectDefinition od;
    od.type = ObjectType::Platform;
    od.position = Position{randLat(), randLon(), 0};
    od.affiliation = Affiliation::Unknown;
    entity_ids.push_back(rt.createObject(od));
  }
  auto t_create1 = Clock::now();

  std::cout << "[PERF] Created " << NUM_ENTITIES << " entities in "
            << elapsedMs(t_create0, t_create1) << " ms\n";

  // Zone containment sweep: every entity against every zone
  auto t_zone0 = Clock::now();
  long long checks = 0;
  int hits = 0;
  for (auto& eid : entity_ids) {
    const auto* kc = rt.store()->kinematics().get(eid);
    if (!kc) continue;
    for (auto& zid : zone_ids) {
      hits += rt.isInsideZone(kc->position, zid) ? 1 : 0;
      ++checks;
    }
  }
  auto t_zone1 = Clock::now();

  double ms = elapsedMs(t_zone0, t_zone1);
  std::cout << "[PERF] " << checks << " runtime zone checks ("
            << NUM_ENTITIES << " x " << NUM_ZONES << "): "
            << ms << " ms  (" << (ms / checks * 1e6) << " ns/check, "
            << hits << " hits)\n";
  SUCCEED();
}

// ========================================================================
// 6.  Haversine micro-benchmark (raw geo::haversineMeters throughput)
// ========================================================================
TEST_F(ZonePerformanceTest, HaversineMicroBenchmark) {
  constexpr int N = 100000;
  std::vector<std::pair<Position, Position>> pairs;
  pairs.reserve(N);
  for (int i = 0; i < N; ++i)
    pairs.push_back({Position{randLat(), randLon(), 0},
                     Position{randLat(), randLon(), 0}});

  auto t0 = Clock::now();
  double sum = 0;
  for (auto& [a, b] : pairs)
    sum += geo::haversineMeters(a, b);
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] " << N << " haversine calls: "
            << ms << " ms  (" << (ms / N * 1e6) << " ns/call, "
            << "sum=" << sum << ")\n";
  SUCCEED();
}

// ========================================================================
// 7.  Spatial index query at scale
// ========================================================================
TEST_F(ZonePerformanceTest, SpatialIndexQueryAtScale) {
  constexpr int N = 10000;
  SpatialIndex idx(1.0);

  std::vector<UUIDKey> ids;
  ids.reserve(N);
  for (int i = 0; i < N; ++i) {
    auto id = UUIDKey{UUIDHelper::generateV4()};
    idx.insert(id, Position{randLat(), randLon(), 0});
    ids.push_back(id);
  }

  constexpr int Q = 1000;
  auto t0 = Clock::now();
  long long total_results = 0;
  for (int q = 0; q < Q; ++q) {
    double clat = randLat(), clon = randLon();
    auto results = idx.queryRegion(clat - 5.0, clat + 5.0, clon - 5.0, clon + 5.0);
    total_results += static_cast<long long>(results.size());
  }
  auto t1 = Clock::now();

  double ms = elapsedMs(t0, t1);
  std::cout << "[PERF] " << Q << " spatial queries over " << N << " entities: "
            << ms << " ms  (" << (ms / Q * 1000.0) << " us/query, "
            << "avg results=" << (total_results / Q) << ")\n";
  SUCCEED();
}
