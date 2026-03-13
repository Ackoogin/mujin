#include <relationship/RelationshipIndex.h>

#include <algorithm>

namespace tactical_objects {

void RelationshipIndex::insert(const RelationshipRecord& record) {
  records_[record.relationship_id] = record;
  subject_index_[record.subject_id].push_back(record.relationship_id);
  object_index_[record.object_id].push_back(record.relationship_id);
}

bool RelationshipIndex::remove(const UUIDKey& relationship_id) {
  auto it = records_.find(relationship_id);
  if (it == records_.end()) return false;

  auto& rec = it->second;
  auto sit = subject_index_.find(rec.subject_id);
  if (sit != subject_index_.end()) {
    removeFromVec(sit->second, relationship_id);
    if (sit->second.empty()) subject_index_.erase(sit);
  }
  auto oit = object_index_.find(rec.object_id);
  if (oit != object_index_.end()) {
    removeFromVec(oit->second, relationship_id);
    if (oit->second.empty()) object_index_.erase(oit);
  }

  records_.erase(it);
  return true;
}

const RelationshipRecord* RelationshipIndex::get(const UUIDKey& relationship_id) const {
  auto it = records_.find(relationship_id);
  return (it != records_.end()) ? &it->second : nullptr;
}

std::vector<RelationshipRecord> RelationshipIndex::bySubject(const UUIDKey& subject) const {
  std::vector<RelationshipRecord> result;
  auto it = subject_index_.find(subject);
  if (it != subject_index_.end()) {
    for (auto& rid : it->second) {
      auto rit = records_.find(rid);
      if (rit != records_.end()) result.push_back(rit->second);
    }
  }
  return result;
}

std::vector<RelationshipRecord> RelationshipIndex::byObject(const UUIDKey& object) const {
  std::vector<RelationshipRecord> result;
  auto it = object_index_.find(object);
  if (it != object_index_.end()) {
    for (auto& rid : it->second) {
      auto rit = records_.find(rid);
      if (rit != records_.end()) result.push_back(rit->second);
    }
  }
  return result;
}

std::vector<RelationshipRecord> RelationshipIndex::byEntity(const UUIDKey& entity) const {
  auto subj = bySubject(entity);
  auto obj = byObject(entity);
  subj.insert(subj.end(), obj.begin(), obj.end());
  return subj;
}

std::vector<RelationshipRecord> RelationshipIndex::byType(RelationshipType type) const {
  std::vector<RelationshipRecord> result;
  for (auto& pair : records_) {
    if (pair.second.type == type) {
      result.push_back(pair.second);
    }
  }
  return result;
}

std::vector<RelationshipRecord> RelationshipIndex::bySubjectAndType(
    const UUIDKey& subject, RelationshipType type) const {
  std::vector<RelationshipRecord> result;
  auto it = subject_index_.find(subject);
  if (it != subject_index_.end()) {
    for (auto& rid : it->second) {
      auto rit = records_.find(rid);
      if (rit != records_.end() && rit->second.type == type) {
        result.push_back(rit->second);
      }
    }
  }
  return result;
}

void RelationshipIndex::removeFromVec(std::vector<UUIDKey>& vec, const UUIDKey& id) {
  vec.erase(std::remove(vec.begin(), vec.end(), id), vec.end());
}

} // namespace tactical_objects
