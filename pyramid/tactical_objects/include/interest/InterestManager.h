#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectStore.h>
#include <tl/optional.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace tactical_objects {

/// \brief Distinguishes passive read-current-data from active-find requests.
enum class QueryMode {
  ReadCurrent,  ///< Return only what is currently known.
  ActiveFind    ///< Actively seek entities matching the criteria.
};

enum class InterestStatus {
  Active,
  Cancelled,
  Expired,
  Superseded
};

struct InterestCriteria {
  tl::optional<QueryMode> query_mode;          ///< ReadCurrent (default) or ActiveFind.
  tl::optional<Affiliation> affiliation;
  tl::optional<ObjectType> object_type;
  tl::optional<BattleDimension> battle_dimension; ///< e.g. SeaSurface, Air.
  tl::optional<BoundingBox> area;
  tl::optional<std::string> behavior_pattern;
  double time_window_start = 0.0;
  double time_window_end = 0.0;
  double minimum_confidence = 0.0;
};

struct DerivedEvidenceRequirement {
  UUIDKey requirement_id;
  UUIDKey source_interest_id;
  std::string evidence_description;
  InterestCriteria criteria;
};

struct InterestProgressReport {
  UUIDKey interest_id;
  size_t total_criteria = 0;
  size_t satisfied_criteria = 0;
  std::vector<std::string> gaps;
};

struct InterestRecord {
  UUIDKey interest_id;
  InterestCriteria criteria;
  InterestStatus status = InterestStatus::Active;
  double registered_at = 0.0;
  double expires_at = 0.0;
  tl::optional<UUIDKey> superseded_by;
  std::vector<DerivedEvidenceRequirement> derived_requirements;
};

struct MeasurementCriterion {
  UUIDKey criterion_id;
  UUIDKey interest_id;
  std::string field_name;
  double threshold = 0.0;
};

/// \brief Captures how the runtime intends to satisfy a tactical object requirement.
struct ObjectSolution {
  UUIDKey solution_id;
  UUIDKey source_interest_id;
  tl::optional<ObjectType> intended_object_type;
  tl::optional<BattleDimension> intended_battle_dimension;
  tl::optional<BoundingBox> area_of_interest;
  double timing_start          = 0.0;
  double timing_end            = 0.0;
  double predicted_quality     = 0.0;  ///< 0..1
  double predicted_completeness = 0.0; ///< 0..1
  std::vector<DerivedEvidenceRequirement> evidence_requirements;
};

/// \brief Manages interest requirements, derived evidence requests, and progress tracking.
class InterestManager {
public:
  /// \brief Register a compound interest requirement.
  UUIDKey registerInterest(const InterestCriteria& criteria, double now = 0.0,
                           double expires_at = 0.0);

  /// \brief Cancel an active interest.
  bool cancelInterest(const UUIDKey& interest_id);

  /// \brief Supersede an interest with a replacement.
  UUIDKey supersedeInterest(const UUIDKey& old_id, const InterestCriteria& new_criteria,
                            double now = 0.0, double expires_at = 0.0);

  /// \brief Advance time and expire any interests past their deadline.
  void tick(double now);

  /// \brief Get an interest record.
  const InterestRecord* get(const UUIDKey& interest_id) const;

  /// \brief All active interest records.
  std::vector<InterestRecord> activeInterests() const;

  /// \brief Derive a component-agnostic evidence requirement from an interest.
  DerivedEvidenceRequirement deriveEvidenceRequirement(const UUIDKey& interest_id);

  /// \brief Determine an object solution for satisfying an interest requirement.
  ObjectSolution determineSolution(const UUIDKey& interest_id);

  /// \brief Register a measurement criterion against an interest.
  void addMeasurementCriterion(const MeasurementCriterion& mc);

  /// \brief Get measurement criteria for an interest.
  std::vector<MeasurementCriterion> getMeasurementCriteria(const UUIDKey& interest_id) const;

  /// \brief Evaluate whether a confidence value meets the measurement criteria.
  bool meetsCriteria(const UUIDKey& interest_id, double confidence) const;

  /// \brief Report progress on an interest requirement.
  InterestProgressReport reportProgress(const UUIDKey& interest_id,
                                        const std::vector<bool>& criteria_met) const;

  /// \brief Test whether a single entity matches a given criteria set.
  bool matchesInterest(const InterestCriteria& criteria, const EntityRecord& rec,
                       const ObjectStore& store) const;

  /// \brief Return IDs of all active interests whose criteria match the entity.
  std::vector<UUIDKey> matchingInterests(const EntityRecord& rec,
                                         const ObjectStore& store) const;

  size_t count() const { return records_.size(); }

private:
  std::unordered_map<UUIDKey, InterestRecord> records_;
  std::unordered_map<UUIDKey, std::vector<MeasurementCriterion>> criteria_;
};

} // namespace tactical_objects
