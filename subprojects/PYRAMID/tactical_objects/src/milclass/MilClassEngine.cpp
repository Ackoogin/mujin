#include <milclass/MilClassEngine.h>

#include <sstream>

namespace tactical_objects {

MilClassEngine::MilClassEngine(std::shared_ptr<ObjectStore> store)
  : store_(std::move(store)) {}

void MilClassEngine::setProfile(const UUIDKey& id, const MilClassProfile& profile) {
  MilClassComponent comp;
  comp.profile = profile;
  store_->milclass().set(id, comp);
  store_->bumpVersion(id);
}

tl::optional<MilClassProfile> MilClassEngine::getProfile(const UUIDKey& id) const {
  const auto* comp = store_->milclass().get(id);
  if (!comp) return tl::nullopt;
  return comp->profile;
}

std::string MilClassEngine::deriveSymbolKey(const UUIDKey& id) const {
  const auto* comp = store_->milclass().get(id);
  if (!comp) return "";

  const auto& p = comp->profile;
  std::ostringstream oss;
  oss << static_cast<int>(p.battle_dim) << ":"
      << static_cast<int>(p.affiliation) << ":"
      << p.role << ":"
      << static_cast<int>(p.status) << ":"
      << static_cast<int>(p.echelon) << ":"
      << static_cast<int>(p.mobility) << ":"
      << (p.hq ? "1" : "0")
      << (p.task_force ? "1" : "0")
      << (p.feint_dummy ? "1" : "0")
      << (p.installation ? "1" : "0");
  return oss.str();
}

} // namespace tactical_objects
