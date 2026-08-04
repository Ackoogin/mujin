#pragma once

/// \file agra_native_json_uuid.h
/// \brief Convert A-GRA identifiers between AME's native form and the
///        "native JSON" pivot dialect `ame_py.cpp`'s protobuf<->native
///        conversion produces and consumes.
///
/// This is a *different* dialect from `agra_oms_uuid.h`'s OMS-JSON wire
/// form, and confusing the two silently corrupts every UUID rather than
/// failing loudly -- both are 32+ character strings that look plausible.
/// The difference is real and load-bearing:
///
///   - OMS-JSON (`agra_oms_uuid.h`) is the genuine wire form real A-GRA
///     peers, Sleet, and the generated Port codec plugins use: 32
///     hexadecimal characters.
///   - "Native JSON" is `agra_codec::fromJson`/`toJson`'s own pivot format
///     when the caller is `_agra_protobuf_to_native_json`
///     (`ame_py.cpp`), not a real wire peer. `google.protobuf.json_format`
///     encodes proto `bytes` fields -- which is what a native A-GRA UUID
///     is -- as base64, always, with no configuration; AME's native
///     identifiers are 16 raw bytes (see `agra_oms_uuid.h`), so the pivot
///     produced by that path carries base64, not hex.
///
/// `agra_codec::fromJson`/`toJson` are schema-driven and have no opinion on
/// which dialect a `uuid` field holds -- they just move the JSON string
/// value into and out of the C++ struct's `std::string uuid` verbatim.
/// Deciding the dialect, and converting it, is always the caller's job;
/// this header is that conversion for the native-JSON dialect, exactly as
/// `ame_py.cpp`'s `decodeUuid`/`encodeUuid`/`decodeHeader`/`encodeHeader`
/// already do it -- extracted here so a consumer that has no embedded
/// Python interpreter (a standalone composite process, unlike the pybind
/// module those functions started in) can still decode this pivot in pure
/// C++, with no `py::module_::import("base64")` round trip required.
///
/// Both directions fail closed: malformed base64 throws
/// `std::invalid_argument` rather than producing a plausible-looking but
/// wrong identifier.

#include <cstdint>
#include <stdexcept>
#include <string>

namespace ame {

namespace detail {

inline const char* base64Alphabet() {
  return "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}

inline int base64DigitValue(char digit) {
  if (digit >= 'A' && digit <= 'Z') return digit - 'A';
  if (digit >= 'a' && digit <= 'z') return digit - 'a' + 26;
  if (digit >= '0' && digit <= '9') return digit - '0' + 52;
  if (digit == '+') return 62;
  if (digit == '/') return 63;
  return -1;
}

}  // namespace detail

/// \brief Standard base64 (RFC 4648 with `+`/`/` and `=` padding).
inline std::string base64Encode(const std::string& value) {
  const char* alphabet = detail::base64Alphabet();
  std::string result;
  result.reserve(((value.size() + 2u) / 3u) * 4u);
  std::size_t index = 0;
  while (index + 3u <= value.size()) {
    const std::uint32_t chunk =
        (static_cast<unsigned char>(value[index]) << 16) |
        (static_cast<unsigned char>(value[index + 1u]) << 8) |
        static_cast<unsigned char>(value[index + 2u]);
    result.push_back(alphabet[(chunk >> 18) & 0x3f]);
    result.push_back(alphabet[(chunk >> 12) & 0x3f]);
    result.push_back(alphabet[(chunk >> 6) & 0x3f]);
    result.push_back(alphabet[chunk & 0x3f]);
    index += 3u;
  }
  const std::size_t remaining = value.size() - index;
  if (remaining == 1u) {
    const std::uint32_t chunk = static_cast<unsigned char>(value[index])
                                 << 16;
    result.push_back(alphabet[(chunk >> 18) & 0x3f]);
    result.push_back(alphabet[(chunk >> 12) & 0x3f]);
    result.push_back('=');
    result.push_back('=');
  } else if (remaining == 2u) {
    const std::uint32_t chunk =
        (static_cast<unsigned char>(value[index]) << 16) |
        (static_cast<unsigned char>(value[index + 1u]) << 8);
    result.push_back(alphabet[(chunk >> 18) & 0x3f]);
    result.push_back(alphabet[(chunk >> 12) & 0x3f]);
    result.push_back(alphabet[(chunk >> 6) & 0x3f]);
    result.push_back('=');
  }
  return result;
}

/// \brief Decode standard base64. Rejects malformed input rather than
///        silently dropping or zero-filling invalid characters.
inline std::string base64Decode(const std::string& value) {
  if (value.size() % 4u != 0u) {
    throw std::invalid_argument(
        "base64 input length must be a multiple of 4; received " +
        std::to_string(value.size()));
  }
  std::string result;
  result.reserve((value.size() / 4u) * 3u);
  for (std::size_t index = 0; index < value.size(); index += 4u) {
    const bool pad2 = value[index + 2u] == '=';
    const bool pad3 = value[index + 3u] == '=';
    if ((pad2 && !pad3) ||
        (pad2 && index + 4u != value.size()) ||
        (pad3 && index + 4u != value.size())) {
      throw std::invalid_argument(
          "base64 input has padding before the final block");
    }
    const int digit0 = detail::base64DigitValue(value[index]);
    const int digit1 = detail::base64DigitValue(value[index + 1u]);
    const int digit2 = pad2 ? 0 : detail::base64DigitValue(value[index + 2u]);
    const int digit3 = pad3 ? 0 : detail::base64DigitValue(value[index + 3u]);
    if (digit0 < 0 || digit1 < 0 || (!pad2 && digit2 < 0) ||
        (!pad3 && digit3 < 0)) {
      throw std::invalid_argument(
          "base64 input contains a character outside the base64 alphabet");
    }
    const std::uint32_t chunk = (static_cast<std::uint32_t>(digit0) << 18) |
                                 (static_cast<std::uint32_t>(digit1) << 12) |
                                 (static_cast<std::uint32_t>(digit2) << 6) |
                                 static_cast<std::uint32_t>(digit3);
    result.push_back(static_cast<char>((chunk >> 16) & 0xff));
    if (!pad2) result.push_back(static_cast<char>((chunk >> 8) & 0xff));
    if (!pad3) result.push_back(static_cast<char>(chunk & 0xff));
  }
  return result;
}

/// \brief Render 16 native identifier bytes as this pivot's base64 form.
inline std::string nativeJsonUuidFromNative(const std::string& uuid) {
  if (uuid.size() != 16u) {
    throw std::invalid_argument(
        "native A-GRA UUID must contain exactly 16 bytes; received " +
        std::to_string(uuid.size()));
  }
  return base64Encode(uuid);
}

/// \brief Parse this pivot's base64 form into 16 native bytes.
inline std::string nativeUuidFromNativeJson(const std::string& uuid) {
  const std::string decoded = base64Decode(uuid);
  if (decoded.size() != 16u) {
    throw std::invalid_argument(
        "native-JSON A-GRA UUID did not decode to 16 bytes; decoded " +
        std::to_string(decoded.size()));
  }
  return decoded;
}

}  // namespace ame
