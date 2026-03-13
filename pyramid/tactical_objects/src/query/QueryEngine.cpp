#include <query/QueryEngine.h>

#include <algorithm>

namespace tactical_objects {

QueryEngine::QueryEngine(std::shared_ptr<ObjectStore> store,
                         std::shared_ptr<SpatialIndex> spatial)
  : store_(std::move(store)), spatial_(std::move(spatial)) {}

bool QueryEngine::matchesPredicates(const UUIDKey& id, const QueryRequest& req) const {
  const auto* rec = store_->getRecord(id);
  if (!rec) return false;

  if (req.by_type && rec->type != *req.by_type) return false;

  if (req.by_affiliation) {
    const auto* mc = store_->milclass().get(id);
    if (!mc || mc->profile.affiliation != *req.by_affiliation) return false;
  }

  if (req.by_source_system || req.by_source_entity_id) {
    const auto* fc = store_->correlation().get(id);
    const auto* ic = store_->identities().get(id);

    bool found = false;

    auto checkRefs = [&](const std::vector<SourceRef>& refs) {
      for (auto& sr : refs) {
        bool sys_match = !req.by_source_system || sr.source_system == *req.by_source_system;
        bool eid_match = !req.by_source_entity_id || sr.source_entity_id == *req.by_source_entity_id;
        if (sys_match && eid_match) { found = true; return; }
      }
    };

    if (fc) checkRefs(fc->source_refs);
    if (!found && ic) checkRefs(ic->source_refs);
    if (!found) return false;
  }

  if (req.max_age_seconds) {
    const auto* qc = store_->quality().get(id);
    if (qc) {
      double age = req.current_time - qc->freshness_timestamp;
      if (age > *req.max_age_seconds) return false;
    }
  }

  return true;
}

QueryResponse QueryEngine::query(const QueryRequest& req) const {
  QueryResponse resp;

  if (req.by_uuid) {
    if (matchesPredicates(*req.by_uuid, req)) {
      const auto* rec = store_->getRecord(*req.by_uuid);
      if (rec) {
        resp.entries.push_back(QueryResultEntry{*req.by_uuid, *rec});
      }
    }
    resp.total = resp.entries.size();
    return resp;
  }

  std::vector<UUIDKey> candidate_ids;
  bool used_spatial = false;

  if (req.by_region) {
    auto& r = *req.by_region;
    candidate_ids = spatial_->queryRegion(r.min_lat, r.max_lat, r.min_lon, r.max_lon);
    used_spatial = true;
  }

  if (used_spatial) {
    for (auto& cid : candidate_ids) {
      if (!matchesPredicates(cid, req)) continue;
      const auto* rec = store_->getRecord(cid);
      if (rec) {
        if (req.by_region) {
          const auto* kc = store_->kinematics().get(cid);
          if (kc && !req.by_region->contains(kc->position.lat, kc->position.lon)) {
            continue;
          }
        }
        resp.entries.push_back(QueryResultEntry{cid, *rec});
      }
    }
  } else {
    store_->forEachObject([&](const EntityRecord& rec) {
      if (matchesPredicates(rec.id, req)) {
        resp.entries.push_back(QueryResultEntry{rec.id, rec});
      }
    });
  }

  resp.total = resp.entries.size();
  return resp;
}

} // namespace tactical_objects
