// Round-trip test for the generated ROS2 marshalling codec:
// domain_model -> ROS2 message -> serialized bytes -> ROS2 message -> domain_model.
#include "pyramid_ros2_codec.hpp"

#include <gtest/gtest.h>

namespace dm = pyramid::domain_model;
namespace codec = pyramid::ros2_codec;

TEST(Ros2Codec, EntityOptionalAndStrings) {
  dm::common::Entity e;
  e.update_time = 1234.5;     // tl::optional<double>
  e.id = "id-42";
  e.source = "src-7";
  auto back = codec::fromBinaryEntity(codec::toBinary(e));
  ASSERT_TRUE(back.update_time.has_value());
  EXPECT_DOUBLE_EQ(*back.update_time, 1234.5);
  EXPECT_EQ(back.id, "id-42");
  EXPECT_EQ(back.source, "src-7");

  dm::common::Entity empty;   // update_time absent
  auto back2 = codec::fromBinaryEntity(codec::toBinary(empty));
  EXPECT_FALSE(back2.update_time.has_value());
  EXPECT_EQ(back2.id, "");
}

TEST(Ros2Codec, GeodeticPositionCollapsedAngles) {
  dm::common::GeodeticPosition g;
  g.latitude = 51.5;
  g.longitude = -0.12;
  auto back = codec::fromBinaryGeodeticPosition(codec::toBinary(g));
  EXPECT_DOUBLE_EQ(back.latitude, 51.5);
  EXPECT_DOUBLE_EQ(back.longitude, -0.12);
}

TEST(Ros2Codec, QueryRepeatedAndOptional) {
  dm::common::Query q;
  q.id = {"a", "b", "c"};
  q.one_shot = true;
  auto back = codec::fromBinaryQuery(codec::toBinary(q));
  ASSERT_EQ(back.id.size(), 3u);
  EXPECT_EQ(back.id[1], "b");
  ASSERT_TRUE(back.one_shot.has_value());
  EXPECT_TRUE(*back.one_shot);
}

TEST(Ros2Codec, AchievementEnumsAndOptional) {
  dm::common::Achievement a;
  a.id = "ach-1";
  a.status = dm::common::Progress::InProgress;
  a.quality = 88.0;
  a.achieveability = dm::common::Feasibility::Feasible;
  auto back = codec::fromBinaryAchievement(codec::toBinary(a));
  EXPECT_EQ(back.id, "ach-1");
  EXPECT_EQ(back.status, dm::common::Progress::InProgress);
  ASSERT_TRUE(back.quality.has_value());
  EXPECT_DOUBLE_EQ(*back.quality, 88.0);
  EXPECT_EQ(back.achieveability, dm::common::Feasibility::Feasible);
}

TEST(Ros2Codec, RequirementNestedMessage) {
  dm::common::Requirement req;
  req.id = "req-1";
  req.status.id = "inner-ach";
  req.status.status = dm::common::Progress::Completed;
  auto back = codec::fromBinaryRequirement(codec::toBinary(req));
  EXPECT_EQ(back.id, "req-1");
  EXPECT_EQ(back.status.id, "inner-ach");
  EXPECT_EQ(back.status.status, dm::common::Progress::Completed);
}

TEST(Ros2Codec, CapabilityRepeatedMessage) {
  dm::common::Capability c;
  c.availability = true;
  c.name = "cap";
  dm::common::Contraint k1; k1.name = "x"; k1.value = 1;
  dm::common::Contraint k2; k2.name = "y"; k2.value = 2;
  c.contraint = {k1, k2};
  auto back = codec::fromBinaryCapability(codec::toBinary(c));
  EXPECT_TRUE(back.availability);
  EXPECT_EQ(back.name, "cap");
  ASSERT_EQ(back.contraint.size(), 2u);
  EXPECT_EQ(back.contraint[1].name, "y");
  EXPECT_EQ(back.contraint[1].value, 2);
}

TEST(Ros2Codec, ObjectMatchBaseInlined) {
  dm::tactical::ObjectMatch m;
  m.id = "om-1";
  m.matching_object_id = "obj-9";
  auto back = codec::fromBinaryObjectMatch(codec::toBinary(m));
  EXPECT_EQ(back.id, "om-1");
  EXPECT_EQ(back.matching_object_id, "obj-9");
}

TEST(Ros2Codec, PlanningGoalOneof) {
  dm::autonomy::PlanningGoal g;
  g.name = "goal";
  g.expression = std::string("(at robot home)");  // oneof member set
  auto back = codec::fromBinaryPlanningGoal(codec::toBinary(g));
  EXPECT_EQ(back.name, "goal");
  ASSERT_TRUE(back.expression.has_value());
  EXPECT_EQ(*back.expression, "(at robot home)");
  EXPECT_FALSE(back.requirement.has_value());
}
