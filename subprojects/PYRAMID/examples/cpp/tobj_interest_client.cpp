#include "tobj_interest_client.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace tobj_example {

namespace {

void log(const char* msg) {
  std::fprintf(stderr, "[interest_client] %s\n", msg);
  std::fflush(stderr);
}

}  // namespace

void TobjInterestClient::onEntityMatches(const pcl_msg_t* msg) {
  if (!msg || !msg->data || msg->size == 0) return;

  std::vector<ObjectMatch> matches;
  if (!Provided::decodeEntityMatches(msg, &matches)) return;

  log("standard.entity_matches received");
  for (const auto& m : matches) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "  entity id=%s matching_object_id=%s",
                  m.id.c_str(), m.matching_object_id.c_str());
    log(buf);
    ++matches_received_;
    if (!m.matching_object_id.empty()) {
      found_hostile_ = true;
    }
  }
}

ObjectInterestRequirement makeActiveFindRequirement(
    DataPolicy       policy,
    BattleDimension  dimension,
    double           min_lat_rad,
    double           max_lat_rad,
    double           min_lon_rad,
    double           max_lon_rad) {
  ObjectInterestRequirement req;
  req.policy = policy;
  req.dimension.push_back(dimension);
  if (min_lat_rad == max_lat_rad && min_lon_rad == max_lon_rad) {
    common::Point point{};
    point.position.latitude  = min_lat_rad;
    point.position.longitude = min_lon_rad;
    req.point = point;
  } else {
    common::PolyArea poly{};
    poly.points.push_back({min_lat_rad, min_lon_rad});
    poly.points.push_back({min_lat_rad, max_lon_rad});
    poly.points.push_back({max_lat_rad, max_lon_rad});
    poly.points.push_back({max_lat_rad, min_lon_rad});
    req.poly_area = poly;
  }
  return req;
}

}  // namespace tobj_example
