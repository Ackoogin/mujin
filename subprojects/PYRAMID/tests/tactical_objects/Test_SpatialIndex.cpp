#include <gtest/gtest.h>
#include <spatial/SpatialIndex.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static UUIDKey makeKey() {
  return UUIDKey{UUIDHelper::generateV4()};
}

///< REQ_TACTICAL_OBJECTS_022: SpatialIndex returns overlapping candidates.
TEST(SpatialIndex, InsertAndQueryByRegion) {
  SpatialIndex idx(1.0);
  auto a = makeKey();
  auto b = makeKey();
  auto c = makeKey();

  idx.insert(a, Position{51.5, -0.1, 0});
  idx.insert(b, Position{48.8, 2.3, 0});
  idx.insert(c, Position{51.4, -0.2, 0});

  auto result = idx.queryRegion(51.0, 52.0, -1.0, 0.0);
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
  SpatialIndex idx(1.0);
  auto a = makeKey();
  auto b = makeKey();

  idx.insert(a, Position{10.0, 20.0, 0});
  idx.insert(b, Position{10.0, 20.0, 0});

  idx.update(a, Position{10.0, 20.0, 0}, Position{50.0, 60.0, 0});

  auto old_region = idx.queryRegion(9.0, 11.0, 19.0, 21.0);
  ASSERT_EQ(old_region.size(), 1u);
  ASSERT_EQ(old_region[0], b);

  auto new_region = idx.queryRegion(49.0, 51.0, 59.0, 61.0);
  ASSERT_EQ(new_region.size(), 1u);
  ASSERT_EQ(new_region[0], a);
}

TEST(SpatialIndex, RemoveFromIndex) {
  SpatialIndex idx(1.0);
  auto a = makeKey();
  idx.insert(a, Position{10.0, 20.0, 0});

  idx.remove(a, Position{10.0, 20.0, 0});
  auto result = idx.queryRegion(9.0, 11.0, 19.0, 21.0);
  ASSERT_TRUE(result.empty());
}

///< REQ_TACTICAL_OBJECTS_042: Regional query over 10k objects returns bounded candidate set.
TEST(SpatialIndex, TenThousandObjectsLocalQuery) {
  SpatialIndex idx(1.0);

  for (int i = 0; i < 10000; ++i) {
    double lat = -90.0 + (static_cast<double>(i) / 10000.0) * 180.0;
    double lon = -180.0 + (static_cast<double>(i % 100) / 100.0) * 360.0;
    idx.insert(makeKey(), Position{lat, lon, 0});
  }

  auto result = idx.queryRegion(0.0, 2.0, 0.0, 2.0);
  ASSERT_LT(result.size(), 200u);
}
