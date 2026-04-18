#include <correlation/CorrelationEngine.h>
#include <geo/GeoMath.h>
#include <uuid/UUIDHelper.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace tactical_objects {

CorrelationEngine::CorrelationEngine(std::shared_ptr<ObjectStore> store,
                                     std::shared_ptr<SpatialIndex> spatial,
                                     std::shared_ptr<MilClassEngine> milclass,
                                     CorrelationConfig config)
  : store_(std::move(store)),
    spatial_(std::move(spatial)),
    milclass_(std::move(milclass)),
    config_(config) {}

double CorrelationEngine::scoreCandidate(const Observation& obs, const UUIDKey& candidate) const {
  double score = 0.0;
  double total_weight = 0.0;

  const auto* kc = store_->kinematics().get(candidate);
  if (kc) {
    double dist = geo::haversineMeters(obs.position, kc->position);
    double gate_m = config_.gate_radius_rad * geo::EARTH_RADIUS_M;
    if (dist > gate_m) return 0.0;
    double spatial_score = 1.0 - (dist / gate_m);
    score += spatial_score * 0.4;
    total_weight += 0.4;
  }

  const auto* cc = store_->correlation().get(candidate);
  if (cc) {
    for (auto& sr : cc->source_refs) {
      if (!obs.source_ref.source_entity_id.empty() &&
          sr.source_entity_id == obs.source_ref.source_entity_id &&
          sr.source_system == obs.source_ref.source_system) {
        score += 0.3;
        total_weight += 0.3;
        break;
      }
    }
  }

  auto profile = milclass_->getProfile(candidate);
  if (profile) {
    if (profile->affiliation == obs.affiliation_hint) {
      score += 0.2;
    }
    total_weight += 0.2;
  }

  score += 0.1;
  total_weight += 0.1;

  return (total_weight > 0.0) ? score / total_weight * total_weight : 0.0;
}

UUIDKey CorrelationEngine::createNewEntity(const Observation& obs) {
  auto track_id = store_->createObject(ObjectType::Platform);
  auto entity_id = store_->createObject(obs.object_hint_type);

  KinematicsComponent kc;
  kc.position = obs.position;
  kc.velocity = obs.velocity;
  kc.timestamp = obs.observed_at;
  store_->kinematics().set(entity_id, kc);

  CorrelationComponent cc;
  cc.contributing_observations.push_back(UUIDKey{obs.observation_id});
  cc.contributing_tracks.push_back(track_id);
  cc.source_refs.push_back(obs.source_ref);
  cc.confidence = obs.confidence;
  cc.provenance = Provenance::Correlated;
  store_->correlation().set(entity_id, cc);

  QualityComponent qc;
  qc.confidence = obs.confidence;
  qc.freshness_timestamp = obs.observed_at;
  store_->quality().set(entity_id, qc);

  MilClassProfile profile;
  profile.affiliation = obs.affiliation_hint;
  profile.source_sidc = obs.source_sidc;
  // Derive battle dimension from SIDC position 2 (APP-6B)
  if (obs.source_sidc.size() >= 3) {
    switch (obs.source_sidc[2]) {
      case 'P': profile.battle_dim = BattleDimension::Space;      break;
      case 'A': profile.battle_dim = BattleDimension::Air;        break;
      case 'G': profile.battle_dim = BattleDimension::Ground;     break;
      case 'S': profile.battle_dim = BattleDimension::SeaSurface; break;
      case 'U': profile.battle_dim = BattleDimension::Subsurface; break;
      case 'F': profile.battle_dim = BattleDimension::SOF;        break;
      default: break;
    }
  }
  milclass_->setProfile(entity_id, profile);

  spatial_->insert(entity_id, obs.position);

  return entity_id;
}

void CorrelationEngine::mergeIntoEntity(const Observation& obs, const UUIDKey& entity_id) {
  auto* cc = store_->correlation().getMutable(entity_id);
  if (cc) {
    cc->contributing_observations.push_back(UUIDKey{obs.observation_id});

    bool found = false;
    for (auto& sr : cc->source_refs) {
      if (sr.source_system == obs.source_ref.source_system &&
          sr.source_entity_id == obs.source_ref.source_entity_id) {
        sr.last_seen = obs.observed_at;
        found = true;
        break;
      }
    }
    if (!found) {
      cc->source_refs.push_back(obs.source_ref);
    }

    double n = static_cast<double>(cc->contributing_observations.size());
    cc->confidence = cc->confidence * ((n - 1.0) / n) + obs.confidence * (1.0 / n);
  }

  auto* kc = store_->kinematics().getMutable(entity_id);
  Position old_pos;
  if (kc) {
    old_pos = kc->position;
    kc->position = obs.position;
    kc->velocity = obs.velocity;
    kc->timestamp = obs.observed_at;
    spatial_->update(entity_id, old_pos, obs.position);
  }

  auto* qc = store_->quality().getMutable(entity_id);
  if (qc && cc) {
    qc->confidence = cc->confidence;
    qc->freshness_timestamp = obs.observed_at;
  }

  store_->bumpVersion(entity_id);
}

UUIDKey CorrelationEngine::splitEntity(const Observation& obs, const UUIDKey& source_id) {
  auto new_entity = createNewEntity(obs);
  incompatibility_.erase(source_id);
  return new_entity;
}

CorrelationResult CorrelationEngine::processObservation(const Observation& obs) {
  auto candidates = spatial_->queryRegion(
    obs.position.lat - config_.gate_radius_rad,
    obs.position.lat + config_.gate_radius_rad,
    obs.position.lon - config_.gate_radius_rad,
    obs.position.lon + config_.gate_radius_rad);

  double best_score = 0.0;
  UUIDKey best_candidate;

  for (auto& cand : candidates) {
    const auto* cc = store_->correlation().get(cand);
    if (!cc || cc->provenance != Provenance::Correlated) continue;

    double s = scoreCandidate(obs, cand);
    if (s > best_score) {
      best_score = s;
      best_candidate = cand;
    }
  }

  if (best_score >= config_.merge_threshold) {
    auto profile = milclass_->getProfile(best_candidate);
    bool incompatible = profile && profile->affiliation != obs.affiliation_hint &&
                        obs.affiliation_hint != Affiliation::Unknown;

    if (incompatible) {
      auto& tracker = incompatibility_[best_candidate];
      tracker.count++;
      if (tracker.count >= config_.split_incompatibility_count) {
        auto new_id = splitEntity(obs, best_candidate);
        return CorrelationResult{CorrelationOutcome::Split, new_id, UUIDKey{}};
      }
    } else {
      incompatibility_.erase(best_candidate);
    }

    mergeIntoEntity(obs, best_candidate);
    return CorrelationResult{CorrelationOutcome::Merged, best_candidate, UUIDKey{}};
  }

  auto entity_id = createNewEntity(obs);
  return CorrelationResult{CorrelationOutcome::Created, entity_id, UUIDKey{}};
}

} // namespace tactical_objects
