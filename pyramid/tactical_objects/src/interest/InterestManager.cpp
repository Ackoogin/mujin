#include <interest/InterestManager.h>
#include <uuid/UUIDHelper.h>

namespace tactical_objects {

using pyramid::core::uuid::UUIDHelper;

UUIDKey InterestManager::registerInterest(const InterestCriteria& criteria,
                                          double now, double expires_at) {
  InterestRecord rec;
  rec.interest_id = UUIDKey{UUIDHelper::generateV4()};
  rec.criteria = criteria;
  rec.status = InterestStatus::Active;
  rec.registered_at = now;
  rec.expires_at = expires_at;
  auto id = rec.interest_id;
  records_[id] = std::move(rec);
  return id;
}

bool InterestManager::cancelInterest(const UUIDKey& interest_id) {
  auto it = records_.find(interest_id);
  if (it == records_.end()) return false;
  it->second.status = InterestStatus::Cancelled;
  return true;
}

UUIDKey InterestManager::supersedeInterest(const UUIDKey& old_id,
                                           const InterestCriteria& new_criteria,
                                           double now, double expires_at) {
  auto it = records_.find(old_id);
  if (it != records_.end()) {
    it->second.status = InterestStatus::Superseded;
  }
  auto new_id = registerInterest(new_criteria, now, expires_at);
  if (it != records_.end()) {
    it->second.superseded_by = new_id;
  }
  return new_id;
}

void InterestManager::tick(double now) {
  for (auto& pair : records_) {
    auto& rec = pair.second;
    if (rec.status == InterestStatus::Active &&
        rec.expires_at > 0.0 && now >= rec.expires_at) {
      rec.status = InterestStatus::Expired;
    }
  }
}

const InterestRecord* InterestManager::get(const UUIDKey& interest_id) const {
  auto it = records_.find(interest_id);
  return (it != records_.end()) ? &it->second : nullptr;
}

std::vector<InterestRecord> InterestManager::activeInterests() const {
  std::vector<InterestRecord> result;
  for (auto& pair : records_) {
    if (pair.second.status == InterestStatus::Active) {
      result.push_back(pair.second);
    }
  }
  return result;
}

DerivedEvidenceRequirement InterestManager::deriveEvidenceRequirement(
    const UUIDKey& interest_id) {
  DerivedEvidenceRequirement der;
  der.requirement_id = UUIDKey{UUIDHelper::generateV4()};
  der.source_interest_id = interest_id;

  auto it = records_.find(interest_id);
  if (it != records_.end()) {
    der.criteria = it->second.criteria;

    std::string desc = "Evidence required:";
    if (it->second.criteria.object_type) {
      desc += " object_type filter active";
    }
    if (it->second.criteria.affiliation) {
      desc += " affiliation filter active";
    }
    if (it->second.criteria.area) {
      desc += " spatial area filter active";
    }
    der.evidence_description = desc;

    it->second.derived_requirements.push_back(der);
  }

  return der;
}

void InterestManager::addMeasurementCriterion(const MeasurementCriterion& mc) {
  criteria_[mc.interest_id].push_back(mc);
}

std::vector<MeasurementCriterion> InterestManager::getMeasurementCriteria(
    const UUIDKey& interest_id) const {
  auto it = criteria_.find(interest_id);
  if (it != criteria_.end()) return it->second;
  return {};
}

bool InterestManager::meetsCriteria(const UUIDKey& interest_id, double confidence) const {
  auto mcs = getMeasurementCriteria(interest_id);
  for (auto& mc : mcs) {
    if (mc.field_name == "confidence" && confidence < mc.threshold) {
      return false;
    }
  }
  return true;
}

InterestProgressReport InterestManager::reportProgress(
    const UUIDKey& interest_id,
    const std::vector<bool>& criteria_met) const {
  InterestProgressReport report;
  report.interest_id = interest_id;
  report.total_criteria = criteria_met.size();
  report.satisfied_criteria = 0;
  for (size_t i = 0; i < criteria_met.size(); ++i) {
    if (criteria_met[i]) {
      ++report.satisfied_criteria;
    } else {
      report.gaps.push_back("criterion_" + std::to_string(i));
    }
  }
  return report;
}

} // namespace tactical_objects
