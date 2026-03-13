#include <uuid/UUIDHelper.h>
#include <uuid/UUID.h>
#include <sstream>
#include <iomanip>
#include <random>
#include <string>
#include <utility>
#include <cstdint>  // For uint8_t
#include <limits>   // For std::numeric_limits
#include <cstdio>   // For snprintf
#include <cctype>   // For isxdigit, tolower
#include <sha1/sha1.h> // Include the SHA-1 header
#include <vector>       // For std::vector used in SHA-1 process
#include <cstring>      // For std::memcpy

namespace pyramid {
namespace core {
namespace uuid {

  UUID UUIDHelper::generateV4() {
    // Use a thread-local generator for better performance in multithreaded scenarios
    // and to avoid contention on a single global generator.
    thread_local static std::random_device rd; // Non-deterministic seed source
    thread_local static std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    thread_local static std::uniform_int_distribution<uint64_t> dist(0, std::numeric_limits<uint64_t>::max());

    UUID uuid;

    // Generate 16 random bytes (as two 64-bit numbers for efficiency)
    uint64_t part1 = dist(gen);
    uint64_t part2 = dist(gen);

    // Copy into the bytes array (handle potential endianness implicitly)
    for (int i = 0; i < 8; ++i) {
      uuid.bytes[i] = static_cast<uint8_t>((part1 >> (i * 8)) & 0xFF);
      uuid.bytes[i + 8] = static_cast<uint8_t>((part2 >> (i * 8)) & 0xFF);
    }

    // Set Version 4 bits (byte 6, bits 7-4 are 0100)
    uuid.bytes[6] = (uuid.bytes[6] & 0x0F) | 0x40;

    // Set Variant 1 bits (byte 8, bits 7-6 are 10)
    uuid.bytes[8] = (uuid.bytes[8] & 0x3F) | 0x80;

    return uuid;
  }

  std::string UUIDHelper::toString(const UUID& uuid) {
    // UUID string format: 8-4-4-4-12 hex digits = 36 characters + null terminator
    char buffer[37];
    const uint8_t* bytes = uuid.bytes.data(); // Get pointer to the underlying bytes

    // Use snprintf for safe and formatted output
    snprintf(buffer, sizeof(buffer),
             "%02hhx%02hhx%02hhx%02hhx-"
             "%02hhx%02hhx-"
             "%02hhx%02hhx-"
             "%02hhx%02hhx-"
             "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
             bytes[0], bytes[1], bytes[2], bytes[3], 
             bytes[4], bytes[5], 
             bytes[6], bytes[7], 
             bytes[8], bytes[9], 
             bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]);

    return std::string(buffer);
  }

  // Helper function to convert a hex character to its integer value
  // Returns -1 if the character is not a valid hex digit.
  inline int hexCharToInt(char c) {
    c = static_cast<char>(std::tolower(c));
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
      return c - 'a' + 10;
    }
    return -1; // Invalid hex character
  }

  std::pair<UUID, bool> UUIDHelper::fromString(const std::string& str) {
    UUID uuid;
    int byte_index = 0;
    bool high_nibble = true;
    int char_count = 0;

    for (char c : str) {
      if (c == '-') {
        // Hyphens are only allowed at specific positions in the 36-char format
        // Positions after 8, 13, 18, 23 hex chars (which correspond to byte indices 4, 6, 8, 10)
        int hex_chars_processed = char_count;
        bool hyphen_expected = (hex_chars_processed == 8 || hex_chars_processed == 12 || 
                                hex_chars_processed == 16 || hex_chars_processed == 20);
        
        if (!hyphen_expected || !high_nibble) { 
          return std::make_pair(uuid, false); // Return default UUID and false on error
        }
        continue; 
      }

      int nibble_value = hexCharToInt(c);
      if (nibble_value == -1) {
        return std::make_pair(uuid, false); // Invalid character
      }

      if (byte_index >= 16) {
        return std::make_pair(uuid, false); // String too long
      }

      if (high_nibble) {
        uuid.bytes[byte_index] = static_cast<uint8_t>(nibble_value << 4);
      } else {
        uuid.bytes[byte_index] |= static_cast<uint8_t>(nibble_value);
        byte_index++;
      }
      high_nibble = !high_nibble;
      char_count++;
    }

    if (char_count != 32 || !high_nibble) {
      return std::make_pair(uuid, false); // Incorrect total length or ended mid-byte
    }

    return std::make_pair(uuid, true); // Success
  }

  // Implementation for generateV5
  UUID UUIDHelper::generateV5(const UUID& ns_uuid, const std::string& name) {
    // 1. Convert namespace uuid to network byte order (big-endian).
    //    UUIDs are typically stored canonically as big-endian, 
    //    so assuming ns_uuid.bytes is already in the correct order.
    //    If not, byte swapping would be needed here.

    // 2. Concatenate namespace uuid bytes and the name string.
    std::vector<uint8_t> data_to_hash;
    data_to_hash.reserve(ns_uuid.bytes.size() + name.length());
    data_to_hash.insert(data_to_hash.end(), ns_uuid.bytes.begin(), ns_uuid.bytes.end());
    data_to_hash.insert(data_to_hash.end(), name.begin(), name.end());

    // 3. Compute the SHA-1 hash.
    std::array<uint8_t, sha1::SHA1::DIGEST_BYTES> hash = sha1::digest(data_to_hash);

    // 4. Truncate hash to 16 bytes (128 bits).
    UUID generated_uuid;
    std::memcpy(generated_uuid.bytes.data(), hash.data(), 16);

    // 5. Set the version bits (Version 5).
    //    Byte 6, bits 7-4 should be 0101.
    generated_uuid.bytes[6] = (generated_uuid.bytes[6] & 0x0F) | 0x50;

    // 6. Set the variant bits (RFC 4122 variant).
    //    Byte 8, bits 7-6 should be 10.
    generated_uuid.bytes[8] = (generated_uuid.bytes[8] & 0x3F) | 0x80;

    return generated_uuid;
  }

  UUID UUIDHelper::getNullUUID() {
    UUID null_uuid;
    // Default constructor initializes bytes to zero
    return null_uuid;
  }

  UUID UUIDHelper::generateRandomUUID() {
    return generateV4();
  }

  // Implement other static helper methods later

} // namespace uuid
} // namespace core
} // namespace pyramid


