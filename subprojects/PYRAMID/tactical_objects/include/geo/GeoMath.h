#pragma once

#include <TacticalObjectsTypes.h>

#include <cmath>

namespace tactical_objects {
namespace geo {

/// WGS-84 mean Earth radius (metres).
constexpr double EARTH_RADIUS_M = 6371000.0;

/// \brief Convert degrees to radians.
inline double degToRad(double deg) {
  constexpr double k = 3.14159265358979323846 / 180.0;
  return deg * k;
}

/// \brief Haversine great-circle distance in metres between two WGS-84 positions.
inline double haversineMeters(const Position& a, const Position& b) {
  double d_lat = degToRad(b.lat - a.lat);
  double d_lon = degToRad(b.lon - a.lon);
  double lat1 = degToRad(a.lat);
  double lat2 = degToRad(b.lat);

  double sa = std::sin(d_lat / 2.0);
  double so = std::sin(d_lon / 2.0);
  double h = sa * sa + std::cos(lat1) * std::cos(lat2) * so * so;
  return 2.0 * EARTH_RADIUS_M * std::asin(std::sqrt(h));
}

/// \brief Compute an axis-aligned bounding box for a circle centred at \p center
///        with the given radius in metres.
inline BoundingBox circleBoundingBox(const Position& center, double radius_m) {
  constexpr double PI = 3.14159265358979323846;
  double offset_deg = radius_m / EARTH_RADIUS_M * (180.0 / PI);

  BoundingBox bb;
  bb.min_lat = center.lat - offset_deg;
  bb.max_lat = center.lat + offset_deg;

  double cos_lat = std::cos(degToRad(center.lat));
  double lon_offset = (cos_lat > 1e-9) ? offset_deg / cos_lat : 180.0;
  bb.min_lon = center.lon - lon_offset;
  bb.max_lon = center.lon + lon_offset;
  return bb;
}

} // namespace geo
} // namespace tactical_objects
