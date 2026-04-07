#include <gtest/gtest.h>
#include <history/TacticalHistory.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static UUIDKey makeKey() { return UUIDKey{UUIDHelper::generateV4()}; }

///< REQ_TACTICAL_OBJECTS_048: As-of query returns the correct historical object state.
TEST(TacticalHistory, QueryStateAtTime) {
  TacticalHistory history;
  auto obj_id = makeKey();

  for (int i = 1; i <= 5; ++i) {
    HistoricalSnapshot snap;
    snap.object_id = obj_id;
    snap.timestamp = i * 100.0;
    snap.version = static_cast<uint64_t>(i);
    snap.position = Position{51.0 + i * 0.01, -0.1, 0};
    snap.confidence = 0.5 + i * 0.05;
    history.recordSnapshot(snap);
  }

  auto at_250 = history.queryAtTime(obj_id, 250.0);
  ASSERT_TRUE(at_250.has_value());
  ASSERT_EQ(at_250->version, 2u);
  ASSERT_DOUBLE_EQ(at_250->timestamp, 200.0);

  auto at_500 = history.queryAtTime(obj_id, 500.0);
  ASSERT_TRUE(at_500.has_value());
  ASSERT_EQ(at_500->version, 5u);

  auto at_50 = history.queryAtTime(obj_id, 50.0);
  ASSERT_FALSE(at_50.has_value());
}

///< REQ_TACTICAL_OBJECTS_048: totalSnapshots counts all stored snapshots.
TEST(TacticalHistory, TotalSnapshotsCount) {
  TacticalHistory history;
  auto obj_a = makeKey();
  auto obj_b = makeKey();

  for (int i = 0; i < 3; ++i) {
    HistoricalSnapshot s;
    s.object_id = obj_a;
    s.timestamp = i * 100.0;
    s.version = i + 1;
    history.recordSnapshot(s);
  }
  for (int i = 0; i < 5; ++i) {
    HistoricalSnapshot s;
    s.object_id = obj_b;
    s.timestamp = i * 50.0;
    s.version = i + 1;
    history.recordSnapshot(s);
  }

  ASSERT_EQ(history.totalSnapshots(), 8u);
}

///< REQ_TACTICAL_OBJECTS_048: Interval query returns retained states across a time window.
TEST(TacticalHistory, QueryStatesOverInterval) {
  TacticalHistory history;
  auto obj_id = makeKey();

  for (int i = 1; i <= 10; ++i) {
    HistoricalSnapshot snap;
    snap.object_id = obj_id;
    snap.timestamp = i * 100.0;
    snap.version = static_cast<uint64_t>(i);
    snap.position = Position{51.0, -0.1, 0};
    history.recordSnapshot(snap);
  }

  auto interval = history.queryInterval(obj_id, 300.0, 700.0);
  ASSERT_EQ(interval.size(), 5u);
  ASSERT_DOUBLE_EQ(interval.front().timestamp, 300.0);
  ASSERT_DOUBLE_EQ(interval.back().timestamp, 700.0);

  for (size_t i = 1; i < interval.size(); ++i) {
    ASSERT_GT(interval[i].timestamp, interval[i - 1].timestamp);
  }
}
