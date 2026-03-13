#include <zone/ZoneEngine.h>
#include <geo/GeoMath.h>
#include <uuid/UUIDHelper.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace tactical_objects {

ZoneEngine::ZoneEngine(std::shared_ptr<ObjectStore> store,
                       std::shared_ptr<SpatialIndex> spatial)
  : store_(std::move(store)), spatial_(std::move(spatial)) {}

UUIDKey ZoneEngine::upsertZone(const ZoneDefinition& def) {
  auto id = UUIDKey{pyramid::core::uuid::UUIDHelper::generateV4()};
  upsertZone(id, def);
  return id;
}

void ZoneEngine::upsertZone(const UUIDKey& id, const ZoneDefinition& def) {
  if (!store_->getRecord(id)) {
    store_->createObject(ObjectType::Zone);
    auto created = store_->getRecord(id);
    if (!created) {
      auto new_id = UUIDKey{pyramid::core::uuid::UUIDHelper::generateV4()};
      (void)new_id;
    }
  }

  ZoneGeometry geom = def.geometry;
  geom.cached_bbox = computeBoundingBox(geom);

  GeometryComponent gc;
  gc.geometry = geom;
  store_->geometries().set(id, gc);

  ZoneComponent zc;
  zc.zone_type = def.zone_type;
  zc.active_from = def.active_from;
  zc.active_until = def.active_until;
  zc.priority = def.priority;
  zc.owner = def.owner;
  zc.semantics = def.semantics;
  store_->zones().set(id, zc);
}

tl::optional<ZoneDefinition> ZoneEngine::getZone(const UUIDKey& id) const {
  const auto* zc = store_->zones().get(id);
  if (!zc) return tl::nullopt;

  ZoneDefinition def;
  def.zone_type = zc->zone_type;
  def.active_from = zc->active_from;
  def.active_until = zc->active_until;
  def.priority = zc->priority;
  def.owner = zc->owner;
  def.semantics = zc->semantics;

  const auto* gc = store_->geometries().get(id);
  if (gc) {
    def.geometry = gc->geometry;
  }
  return def;
}

bool ZoneEngine::removeZone(const UUIDKey& id) {
  store_->geometries().remove(id);
  store_->zones().remove(id);
  return true;
}

BoundingBox ZoneEngine::computeBoundingBox(const ZoneGeometry& geom) {
  // Polygon-based zones: compute tight vertex envelope when vertices are present.
  if (!geom.vertices.empty()) {
    BoundingBox bb;
    bb.min_lat = bb.max_lat = geom.vertices[0].lat;
    bb.min_lon = bb.max_lon = geom.vertices[0].lon;
    for (size_t i = 1; i < geom.vertices.size(); ++i) {
      bb.min_lat = std::min(bb.min_lat, geom.vertices[i].lat);
      bb.max_lat = std::max(bb.max_lat, geom.vertices[i].lat);
      bb.min_lon = std::min(bb.min_lon, geom.vertices[i].lon);
      bb.max_lon = std::max(bb.max_lon, geom.vertices[i].lon);
    }
    return bb;
  }

  // All other cases (circle-based or fallback): single shared path.
  return geo::circleBoundingBox(geom.center, geom.radius_m);
}

double ZoneEngine::haversineMeters(const Position& a, const Position& b) {
  return geo::haversineMeters(a, b);
}

bool ZoneEngine::pointInPolygon(const Position& point, const std::vector<Position>& vertices) {
  size_t n = vertices.size();
  if (n < 3) return false;

  bool inside = false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    double yi = vertices[i].lat, yj = vertices[j].lat;
    double xi = vertices[i].lon, xj = vertices[j].lon;
    bool intersect = ((yi > point.lat) != (yj > point.lat)) &&
                     (point.lon < (xj - xi) * (point.lat - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

bool ZoneEngine::isInside(const Position& point, const UUIDKey& zone_id) const {
  const auto* gc = store_->geometries().get(zone_id);
  if (!gc) return false;

  const auto& geom = gc->geometry;

  if (!geom.vertices.empty() && geom.vertices.size() >= 3) {
    return pointInPolygon(point, geom.vertices);
  }

  if (geom.radius_m > 0.0) {
    double dist = haversineMeters(point, geom.center);
    return dist <= geom.radius_m;
  }

  return false;
}

ZoneRelationship ZoneEngine::evaluateTransition(const UUIDKey& object_id,
                                                 const UUIDKey& zone_id,
                                                 const Position& current_pos,
                                                 double current_time) {
  const auto* zc = store_->zones().get(zone_id);
  if (zc) {
    if (zc->active_until > 0.0 && current_time > zc->active_until) {
      return ZoneRelationship::Outside;
    }
    if (zc->active_from > 0.0 && current_time < zc->active_from) {
      return ZoneRelationship::Outside;
    }
  }

  bool currently_inside = isInside(current_pos, zone_id);

  PairKey pk{object_id, zone_id};
  auto it = prior_states_.find(pk);

  ZoneRelationship prior = ZoneRelationship::Outside;
  if (it != prior_states_.end()) {
    prior = it->second.relationship;
  }

  ZoneRelationship result;
  if (currently_inside) {
    bool was_inside = (prior == ZoneRelationship::Inside || prior == ZoneRelationship::Entering);
    result = was_inside ? ZoneRelationship::Inside : ZoneRelationship::Entering;
  } else {
    bool was_inside = (prior == ZoneRelationship::Inside || prior == ZoneRelationship::Entering);
    result = was_inside ? ZoneRelationship::Leaving : ZoneRelationship::Outside;
  }

  prior_states_[pk].relationship = result;
  return result;
}

} // namespace tactical_objects
