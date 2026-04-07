#pragma once

#include <uuid/UUID.h> 
#include <utility>
#include <string>
#include <vector>

// Forward declaration
namespace pyramid {
namespace core {
namespace uuid {
  struct UUID;
}
}
}

namespace pyramid {
namespace core {
namespace uuid {

/// /// Provides helper functions for generating and manipulating UUIDs.
///
class UUIDHelper {
public:
  // Static class - prevent instantiation
  UUIDHelper() = delete;

  /// /// Generates a random (Version 4) UUID according to RFC 4122.
  /// \return A randomly generated UUID.
  ///
  static UUID generateV4();

  /// /// Converts a UUID to its standard string representation.
  /// \param uuid The UUID to convert.
  /// \return A string in the format "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx".
  ///
  static std::string toString(const UUID& uuid);

  /// /// Converts a standard string representation (with or without hyphens) to a UUID.
  /// \param str The string representing the UUID.
  /// \return A std::pair containing the UUID and a boolean status.
  ///         The boolean is true if parsing is successful, false otherwise.
  ///         The UUID value is only valid if the boolean is true.
  ///
  static std::pair<UUID, bool> fromString(const std::string& str);

  /// /// Generates a name-based (Version 5) UUID using SHA-1 hashing according to RFC 4122.
  /// \param ns_uuid The namespace uuid.
  /// \param name The name string to hash.
  /// \return The generated Version 5 UUID.
  ///
  static UUID generateV5(const UUID& ns_uuid, const std::string& name);

  /// /// Returns a null UUID (all zeros).
  /// \return A null UUID with all bytes set to zero.
  ///
  static UUID getNullUUID();

  /// /// Alias for generateV4() for backward compatibility.
  /// \return A randomly generated UUID.
  ///
  static UUID generateRandomUUID();

  // Add other static helper methods later (toString, fromString, etc.)
};

} // namespace uuid
} // namespace core
} // namespace pyramid 


