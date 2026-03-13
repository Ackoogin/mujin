#pragma once

#include <TacticalObjectsTypes.h>

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace tactical_objects {

/// \brief Grid-based spatial index over WGS84 lat/lon.
class SpatialIndex {
public:
  /// \brief Construct with a configurable cell size in degrees.
  explicit SpatialIndex(double cell_size_deg = 1.0);

  /// \brief Insert an entity at a position.
  void insert(const UUIDKey& id, const Position& pos);

  /// \brief Update an entity's position (remove old, insert new).
  void update(const UUIDKey& id, const Position& old_pos, const Position& new_pos);

  /// \brief Remove an entity from the index.
  void remove(const UUIDKey& id, const Position& pos);

  /// \brief Return all entities whose cell overlaps the query region.
  std::vector<UUIDKey> queryRegion(double min_lat, double max_lat,
                                   double min_lon, double max_lon) const;

private:
  double cell_size_;

  struct CellKey {
    int64_t row;
    int64_t col;
    bool operator==(const CellKey& o) const { return row == o.row && col == o.col; }
  };

  struct CellKeyHash {
    size_t operator()(const CellKey& k) const {
      size_t h1 = std::hash<int64_t>{}(k.row);
      size_t h2 = std::hash<int64_t>{}(k.col);
      return h1 ^ (h2 * 0x9e3779b97f4a7c15ULL + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };

  std::unordered_map<CellKey, std::vector<UUIDKey>, CellKeyHash> grid_;

  CellKey cellFor(const Position& pos) const;
};

} // namespace tactical_objects
