#include <gtest/gtest.h>
#include <spatial/SpatialIndex.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static constexpr double DEG = 3.14159265358979323846 / 180.0;

static UUIDKey makeKey() {
  return UUIDKey{UUIDHelper::generateV4()};
}

///< REQ_TACTICAL_OBJECTS_022: SpatialIndex returns overlapping candidates.
TEST(SpatialIndex, InsertAndQueryByRegion) {
  SpatialIndex idx(DEG);
  auto a = makeKey();
  auto b = makeKey();
  auto c = makeKey();

  idx.insert(a, Position{51.5 * DEG, -0.1 * DEG, 0});
  idx.insert(b, Position{48.8 * DEG,  2.3 * DEG, 0});
  idx.insert(c, Position{51.4 * DEG, -0.2 * DEG, 0});

  auto result = idx.queryRegion(51.0 * DEG, 52.0 * DEG, -1.0 * DEG, 0.0);
  ASSERT_EQ(result.size(), 2u);

  bool found_a = false, found_c = false;
  for (auto& r : result) {
    if (r == a) found_a = true;
    if (r == c) found_c = true;
  }
  ASSERT_TRUE(found_a);
  ASSERT_TRUE(found_c);
}

///< REQ_TACTICAL_OBJECTS_043: Moving one object updates only that object's index.
TEST(SpatialIndex, UpdatePositionReindexes) {
  SpatialIndex idx(DEG);
  auto a = makeKey();
  auto b = makeKey();

  idx.insert(a, Position{10.0 * DEG, 20.0 * DEG, 0});
  idx.insert(b, Position{10.0 * DEG, 20.0 * DEG, 0});

  idx.update(a, Position{10.0 * DEG, 20.0 * DEG, 0}, Position{50.0 * DEG, 60.0 * DEG, 0});

  auto old_region = idx.queryRegion( 9.0 * DEG, 11.0 * DEG, 19.0 * DEG, 21.0 * DEG);
  ASSERT_EQ(old_region.size(), 1u);
  ASSERT_EQ(old_region[0], b);

  auto new_region = idx.queryRegion(49.0 * DEG, 51.0 * DEG, 59.0 * DEG, 61.0 * DEG);
  ASSERT_EQ(new_region.size(), 1u);
  ASSERT_EQ(new_region[0], a);
}

TEST(SpatialIndex, RemoveFromIndex) {
  SpatialIndex idx(DEG);
  auto a = makeKey();
  idx.insert(a, Position{10.0 * DEG, 20.0 * DEG, 0});

  idx.remove(a, Position{10.0 * DEG, 20.0 * DEG, 0});
  auto result = idx.queryRegion(9.0 * DEG, 11.0 * DEG, 19.0 * DEG, 21.0 * DEG);
  ASSERT_TRUE(result.empty());
}

///< REQ_TACTICAL_OBJECTS_042: Regional query over 10k objects returns bounded candidate set.
TEST(SpatialIndex, TenThousandObjectsLocalQuery) {
  SpatialIndex idx(DEG);

  for (int i = 0; i < 10000; ++i) {
    double lat = (-90.0 + (static_cast<double>(i) / 10000.0) * 180.0) * DEG;
    double lon = (-180.0 + (static_cast<double>(i % 100) / 100.0) * 360.0) * DEG;
    idx.insert(makeKey(), Position{lat, lon, 0});
  }

  auto result = idx.queryRegion(0.0, 2.0 * DEG, 0.0, 2.0 * DEG);
  ASSERT_LT(result.size(), 200u);
}
