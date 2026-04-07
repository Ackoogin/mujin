#pragma once

#include <array>
#include <cstdint>
#include <algorithm> // For std::equal
#include <sha1/sha1.h> 

namespace pyramid {
namespace core {
namespace uuid {

/// /// Represents a Universally Unique Identifier (UUID).
/// Stores the UUID as 16 bytes.
///
struct UUID {
  std::array<uint8_t, 16> bytes{}; // Initialize to zero

  /// /// Comparison operator (less than).
  /// Needed for use in ordered containers like std::set or std::map.
  /// Compares byte-by-byte.
  ///
  bool operator<(const UUID& other) const {
    return bytes < other.bytes; // std::array provides operator<
  }

  /// /// Equality operator.
  ///
  bool operator==(const UUID& other) const {
    return bytes == other.bytes; // std::array provides operator==
  }

  /// /// Inequality operator.
  ///
  bool operator!=(const UUID& other) const {
    return !(*this == other);
  }
};

} // namespace uuid
} // namespace core
} // namespace pyramid 


