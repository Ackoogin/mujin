#include <store/ObjectStore.h>
#include <uuid/UUIDHelper.h>

namespace tactical_objects {

UUIDKey ObjectStore::createObject(ObjectType type) {
  auto uuid = pyramid::core::uuid::UUIDHelper::generateV4();
  UUIDKey key{uuid};

  EntityRecord rec;
  rec.id = key;
  rec.type = type;
  rec.version = 1;

  size_t idx = dense_.size();
  dense_.push_back(rec);
  sparse_[key] = idx;

  LifecycleComponent lc;
  lc.status = LifecycleStatus::Active;
  lc.version = 1;
  lifecycle_.set(key, lc);

  return key;
}

const EntityRecord* ObjectStore::getRecord(const UUIDKey& id) const {
  auto it = sparse_.find(id);
  if (it == sparse_.end()) return nullptr;
  return &dense_[it->second];
}

bool ObjectStore::deleteObject(const UUIDKey& id) {
  auto it = sparse_.find(id);
  if (it == sparse_.end()) return false;

  size_t idx = it->second;
  size_t last = dense_.size() - 1;

  if (idx != last) {
    dense_[idx] = dense_[last];
    sparse_[dense_[idx].id] = idx;
  }
  dense_.pop_back();
  sparse_.erase(it);

  removeFromAllComponents(id);
  return true;
}

size_t ObjectStore::objectCount() const {
  return dense_.size();
}

void ObjectStore::bumpVersion(const UUIDKey& id) {
  auto it = sparse_.find(id);
  if (it == sparse_.end()) return;
  dense_[it->second].version++;

  auto* lc = lifecycle_.getMutable(id);
  if (lc) {
    lc->version = dense_[it->second].version;
  }
}

void ObjectStore::removeFromAllComponents(const UUIDKey& id) {
  identities_.remove(id);
  kinematics_.remove(id);
  milclass_.remove(id);
  correlation_.remove(id);
  quality_.remove(id);
  geometries_.remove(id);
  lifecycle_.remove(id);
  behaviors_.remove(id);
  zones_.remove(id);
}

uint16_t ObjectStore::getDirtyMask(const UUIDKey& id) const {
  auto it = dirty_masks_.find(id);
  return it != dirty_masks_.end() ? it->second : 0u;
}

void ObjectStore::setDirtyBits(const UUIDKey& id, uint16_t bits) {
  dirty_masks_[id] |= bits;
}

void ObjectStore::clearDirtyMask(const UUIDKey& id) {
  dirty_masks_.erase(id);
}

void ObjectStore::clearAllDirtyMasks() {
  dirty_masks_.clear();
}

} // namespace tactical_objects
