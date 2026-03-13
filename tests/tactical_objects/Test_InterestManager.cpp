#include <gtest/gtest.h>
#include <interest/InterestManager.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

///< REQ_TACTICAL_OBJECTS_046: Compound interest requirement is accepted and persisted.
TEST(InterestManager, RegisterCompoundInterestRequirement) {
  InterestManager mgr;

  InterestCriteria crit;
  crit.affiliation = Affiliation::Hostile;
  crit.object_type = ObjectType::Platform;
  crit.area = BoundingBox{50.0, 52.0, -1.0, 1.0};
  crit.time_window_start = 1000.0;
  crit.time_window_end = 2000.0;
  crit.minimum_confidence = 0.7;

  auto id = mgr.registerInterest(crit, 1000.0, 5000.0);
  ASSERT_FALSE(id.isNull());

  const auto* rec = mgr.get(id);
  ASSERT_NE(rec, nullptr);
  ASSERT_EQ(rec->status, InterestStatus::Active);
  ASSERT_TRUE(rec->criteria.affiliation.has_value());
  ASSERT_EQ(*rec->criteria.affiliation, Affiliation::Hostile);
  ASSERT_TRUE(rec->criteria.object_type.has_value());
  ASSERT_EQ(*rec->criteria.object_type, ObjectType::Platform);
  ASSERT_TRUE(rec->criteria.area.has_value());
  ASSERT_DOUBLE_EQ(rec->criteria.minimum_confidence, 0.7);
}

///< REQ_TACTICAL_OBJECTS_047: Active interests support cancellation, expiry, and supersession.
TEST(InterestManager, CancelExpireAndSupersedeInterest) {
  InterestManager mgr;

  InterestCriteria crit;
  crit.affiliation = Affiliation::Hostile;

  auto id1 = mgr.registerInterest(crit, 1000.0, 1500.0);
  auto id2 = mgr.registerInterest(crit, 1000.0, 1200.0);
  auto id3 = mgr.registerInterest(crit, 1000.0, 3000.0);

  ASSERT_TRUE(mgr.cancelInterest(id1));

  mgr.tick(1300.0);

  InterestCriteria new_crit;
  new_crit.object_type = ObjectType::Equipment;
  auto id4 = mgr.supersedeInterest(id3, new_crit, 1300.0, 5000.0);

  ASSERT_EQ(mgr.get(id1)->status, InterestStatus::Cancelled);
  ASSERT_EQ(mgr.get(id2)->status, InterestStatus::Expired);
  ASSERT_EQ(mgr.get(id3)->status, InterestStatus::Superseded);
  ASSERT_EQ(mgr.get(id4)->status, InterestStatus::Active);

  auto active = mgr.activeInterests();
  ASSERT_EQ(active.size(), 1u);
  ASSERT_EQ(active[0].interest_id, id4);
}

///< REQ_TACTICAL_OBJECTS_060: Derived evidence requirement is component-agnostic.
TEST(InterestManager, DerivedEvidenceRequirementIsComponentAgnostic) {
  InterestManager mgr;

  InterestCriteria crit;
  crit.affiliation = Affiliation::Hostile;
  crit.object_type = ObjectType::Platform;
  crit.area = BoundingBox{50.0, 52.0, -1.0, 1.0};

  auto interest_id = mgr.registerInterest(crit, 1000.0, 5000.0);
  auto der = mgr.deriveEvidenceRequirement(interest_id);

  ASSERT_FALSE(der.requirement_id.isNull());
  ASSERT_EQ(der.source_interest_id, interest_id);
  ASSERT_FALSE(der.evidence_description.empty());
  ASSERT_EQ(der.evidence_description.find("SensorDataFusion"), std::string::npos);
  ASSERT_EQ(der.evidence_description.find("DataFusion"), std::string::npos);
  ASSERT_EQ(der.evidence_description.find("Geography"), std::string::npos);
}

///< REQ_TACTICAL_OBJECTS_062: Measurement criteria captured and applied during quality assessment.
TEST(InterestManager, MeasurementCriteriaAcceptance) {
  InterestManager mgr;

  InterestCriteria crit;
  crit.object_type = ObjectType::Platform;
  auto interest_id = mgr.registerInterest(crit, 1000.0);

  MeasurementCriterion mc;
  mc.criterion_id = UUIDKey{UUIDHelper::generateV4()};
  mc.interest_id = interest_id;
  mc.field_name = "confidence";
  mc.threshold = 0.9;
  mgr.addMeasurementCriterion(mc);

  ASSERT_FALSE(mgr.meetsCriteria(interest_id, 0.5));
  ASSERT_TRUE(mgr.meetsCriteria(interest_id, 0.95));
}

///< REQ_TACTICAL_OBJECTS_062: getMeasurementCriteria returns empty for unknown interest ID.
TEST(InterestManager, GetMeasurementCriteriaEmptyForUnknownId) {
  InterestManager mgr;
  UUIDKey bogus_id{UUIDHelper::generateV4()};
  auto criteria = mgr.getMeasurementCriteria(bogus_id);
  ASSERT_TRUE(criteria.empty());
}

///< REQ_TACTICAL_OBJECTS_063: Progress is reported against an active interest requirement.
TEST(InterestManager, ProgressReportingAgainstRequirement) {
  InterestManager mgr;

  InterestCriteria crit;
  crit.affiliation = Affiliation::Hostile;
  crit.object_type = ObjectType::Platform;
  auto interest_id = mgr.registerInterest(crit, 1000.0);

  std::vector<bool> criteria_met = {true, false, true};
  auto report = mgr.reportProgress(interest_id, criteria_met);

  ASSERT_EQ(report.interest_id, interest_id);
  ASSERT_EQ(report.total_criteria, 3u);
  ASSERT_EQ(report.satisfied_criteria, 2u);
  ASSERT_EQ(report.gaps.size(), 1u);
}
