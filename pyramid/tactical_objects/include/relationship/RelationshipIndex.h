#pragma once

#include <TacticalObjectsTypes.h>
#include <tl/optional.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace tactical_objects {

struct RelationshipRecord {
  UUIDKey relationship_id;
  UUIDKey subject_id;
  UUIDKey object_id;
  RelationshipType type = RelationshipType::Tactical;
  double confidence = 1.0;
  std::string source;
  double established_at = 0.0;
};

/// \brief First-class relationship storage with subject/object reverse indexes.
class RelationshipIndex {
public:
  /// \brief Insert a relationship and index it by subject and object.
  void insert(const RelationshipRecord& record);

  /// \brief Remove a relationship by its UUID.
  bool remove(const UUIDKey& relationship_id);

  /// \brief Retrieve a relationship by its UUID.
  const RelationshipRecord* get(const UUIDKey& relationship_id) const;

  /// \brief All relationships where the given entity is the subject.
  std::vector<RelationshipRecord> bySubject(const UUIDKey& subject) const;

  /// \brief All relationships where the given entity is the object.
  std::vector<RelationshipRecord> byObject(const UUIDKey& object) const;

  /// \brief All relationships involving the given entity (subject or object).
  std::vector<RelationshipRecord> byEntity(const UUIDKey& entity) const;

  /// \brief Filter relationships by type.
  std::vector<RelationshipRecord> byType(RelationshipType type) const;

  /// \brief Filter relationships by subject and type.
  std::vector<RelationshipRecord> bySubjectAndType(const UUIDKey& subject,
                                                    RelationshipType type) const;

  size_t count() const { return records_.size(); }

private:
  std::unordered_map<UUIDKey, RelationshipRecord> records_;
  std::unordered_map<UUIDKey, std::vector<UUIDKey>> subject_index_;
  std::unordered_map<UUIDKey, std::vector<UUIDKey>> object_index_;

  void removeFromVec(std::vector<UUIDKey>& vec, const UUIDKey& id);
};

} // namespace tactical_objects
