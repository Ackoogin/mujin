#pragma once

/// \file agra_oms_uuid.h
/// \brief Convert A-GRA identifiers between AME's native form and the
///        OMS-JSON wire form.
///
/// AME and the generated OMS-JSON codec disagree about what a UUID *is*, and
/// the disagreement is silent unless something converts at the seam:
///
///   - AME stores 16 raw bytes. `AgraMaBridge::requireUuid` rejects anything
///     else, and `AgraMaBridge::deterministicUuid` produces 16 bytes.
///     AutoMTK's `automtk-agra` agrees -- its UUIDv5 identifiers are 16 raw
///     bytes too.
///   - The OMS-JSON codec requires the field to already hold 32 hexadecimal
///     characters. Its generated `uuid_hex()` checks `size() == 32` and hex
///     digits, and `encode_ID_Type` throws when that does not hold. The codec
///     performs no conversion of its own; it validates and passes through.
///
/// So a message built with AME identifiers cannot be encoded, and a message
/// decoded from the wire cannot be handed to the bridge, without this
/// conversion. It was first written inline in the shared-memory boundary
/// proof; it lives here so that every consumer of that seam shares one
/// implementation rather than reinventing it -- which is the drift the
/// convergence work exists to remove.
///
/// Both directions fail closed. A wrong length or a non-hexadecimal character
/// throws `std::invalid_argument` rather than producing a plausible-looking
/// identifier, because a silently mangled UUID correlates nothing and the
/// failure would surface far from its cause.

#include <stdexcept>
#include <string>

namespace ame {

/// \brief Render 16 native identifier bytes as the OMS-JSON 32-character form.
inline std::string omsUuidFromNative(const std::string& uuid) {
  if (uuid.size() != 16u) {
    throw std::invalid_argument(
        "native A-GRA UUID must contain exactly 16 bytes; received " +
        std::to_string(uuid.size()));
  }
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string result;
  result.reserve(uuid.size() * 2u);
  for (const unsigned char byte : uuid) {
    result.push_back(kDigits[byte >> 4]);
    result.push_back(kDigits[byte & 0x0f]);
  }
  return result;
}

/// \brief Parse the OMS-JSON 32-character form into 16 native bytes.
inline std::string nativeUuidFromOms(const std::string& uuid) {
  if (uuid.size() != 32u) {
    throw std::invalid_argument(
        "OMS-JSON A-GRA UUID must contain exactly 32 hexadecimal "
        "characters; received " +
        std::to_string(uuid.size()));
  }
  const auto nibble = [](char value) -> int {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
  };
  std::string decoded(16u, '\0');
  for (std::size_t index = 0; index < decoded.size(); ++index) {
    const int high = nibble(uuid[index * 2u]);
    const int low = nibble(uuid[index * 2u + 1u]);
    if (high < 0 || low < 0) {
      throw std::invalid_argument(
          "OMS-JSON A-GRA UUID contains a non-hexadecimal character");
    }
    decoded[index] = static_cast<char>((high << 4) | low);
  }
  return decoded;
}

}  // namespace ame
