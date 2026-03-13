#include <spatial/SpatialIndex.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace tactical_objects {

SpatialIndex::SpatialIndex(double cell_size_deg)
  : cell_size_(cell_size_deg) {}

SpatialIndex::CellKey SpatialIndex::cellFor(const Position& pos) const {
  CellKey k;
  k.row = static_cast<int64_t>(std::floor(pos.lat / cell_size_));
  k.col = static_cast<int64_t>(std::floor(pos.lon / cell_size_));
  return k;
}

void SpatialIndex::insert(const UUIDKey& id, const Position& pos) {
  auto ck = cellFor(pos);
  grid_[ck].push_back(id);
}

void SpatialIndex::update(const UUIDKey& id, const Position& old_pos, const Position& new_pos) {
  remove(id, old_pos);
  insert(id, new_pos);
}

void SpatialIndex::remove(const UUIDKey& id, const Position& pos) {
  auto ck = cellFor(pos);
  auto it = grid_.find(ck);
  if (it == grid_.end()) return;

  auto& vec = it->second;
  vec.erase(std::remove(vec.begin(), vec.end(), id), vec.end());
  if (vec.empty()) {
    grid_.erase(it);
  }
}

std::vector<UUIDKey> SpatialIndex::queryRegion(double min_lat, double max_lat,
                                                double min_lon, double max_lon) const {
  int64_t row_lo = static_cast<int64_t>(std::floor(min_lat / cell_size_));
  int64_t row_hi = static_cast<int64_t>(std::floor(max_lat / cell_size_));
  int64_t col_lo = static_cast<int64_t>(std::floor(min_lon / cell_size_));
  int64_t col_hi = static_cast<int64_t>(std::floor(max_lon / cell_size_));

  std::unordered_set<UUIDKey> seen;
  std::vector<UUIDKey> result;

  for (int64_t r = row_lo; r <= row_hi; ++r) {
    for (int64_t c = col_lo; c <= col_hi; ++c) {
      CellKey ck{r, c};
      auto it = grid_.find(ck);
      if (it == grid_.end()) continue;
      for (auto& id : it->second) {
        if (seen.insert(id).second) {
          result.push_back(id);
        }
      }
    }
  }
  return result;
}

} // namespace tactical_objects
