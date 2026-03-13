// TinySHA1 - A simple header only SHA1 implementation. Based on the public domain implementation by Steve Reid <steve@edmweb.com>
// Created by mohaps on 2018-12-19
// Licence: MIT License (as per the original GitHub repo, overriding the Public Domain comment for clarity)

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

namespace sha1 {

namespace { // Anonymous namespace for internal linkage

// Rotate Left operation
inline uint32_t rotl(const uint32_t value, const size_t bits) {
  return (value << bits) | (value >> (32 - bits));
}

// SHA-1 Transformation (single block)
// Endianness handling added for portability
inline void transform(uint32_t state[5], const uint8_t buffer[64]) {
  uint32_t a = state[0], b = state[1], c = state[2], d = state[3], e = state[4];
  uint32_t block[16];

  // Helper to convert big-endian bytes to uint32_t
  auto bytes_to_uint32_be = [](const uint8_t* bytes) {
  return (static_cast<uint32_t>(bytes[0]) << 24) |
     (static_cast<uint32_t>(bytes[1]) << 16) |
     (static_cast<uint32_t>(bytes[2]) << 8)  |
     (static_cast<uint32_t>(bytes[3]));
  };

  for (size_t i = 0; i < 16; ++i) {
  block[i] = bytes_to_uint32_be(buffer + i * 4);
  }

  auto R0 = [&](uint32_t v, uint32_t& w, uint32_t x, uint32_t y, uint32_t& z, size_t i) {
  z += ((w & (x ^ y)) ^ y) + block[i] + 0x5A827999 + rotl(v, 5);
  w = rotl(w, 30);
  };
  auto R1 = [&](uint32_t v, uint32_t& w, uint32_t x, uint32_t y, uint32_t& z, size_t i) {
  block[i] = rotl(block[(i + 13) & 15] ^ block[(i + 8) & 15] ^ block[(i + 2) & 15] ^ block[i], 1);
  z += ((w & (x ^ y)) ^ y) + block[i] + 0x5A827999 + rotl(v, 5);
  w = rotl(w, 30);
  };
  auto R2 = [&](uint32_t v, uint32_t& w, uint32_t x, uint32_t y, uint32_t& z, size_t i) {
  block[i] = rotl(block[(i + 13) & 15] ^ block[(i + 8) & 15] ^ block[(i + 2) & 15] ^ block[i], 1);
  z += (w ^ x ^ y) + block[i] + 0x6ED9EBA1 + rotl(v, 5);
  w = rotl(w, 30);
  };
  auto R3 = [&](uint32_t v, uint32_t& w, uint32_t x, uint32_t y, uint32_t& z, size_t i) {
  block[i] = rotl(block[(i + 13) & 15] ^ block[(i + 8) & 15] ^ block[(i + 2) & 15] ^ block[i], 1);
  z += (((w | x) & y) | (w & x)) + block[i] + 0x8F1BBCDC + rotl(v, 5);
  w = rotl(w, 30);
  };
  auto R4 = [&](uint32_t v, uint32_t& w, uint32_t x, uint32_t y, uint32_t& z, size_t i) {
  block[i] = rotl(block[(i + 13) & 15] ^ block[(i + 8) & 15] ^ block[(i + 2) & 15] ^ block[i], 1);
  z += (w ^ x ^ y) + block[i] + 0xCA62C1D6 + rotl(v, 5);
  w = rotl(w, 30);
  };

  // Rounds 0-15
  R0(a,b,c,d,e, 0); R0(e,a,b,c,d, 1); R0(d,e,a,b,c, 2); R0(c,d,e,a,b, 3);
  R0(b,c,d,e,a, 4); R0(a,b,c,d,e, 5); R0(e,a,b,c,d, 6); R0(d,e,a,b,c, 7);
  R0(c,d,e,a,b, 8); R0(b,c,d,e,a, 9); R0(a,b,c,d,e,10); R0(e,a,b,c,d,11);
  R0(d,e,a,b,c,12); R0(c,d,e,a,b,13); R0(b,c,d,e,a,14); R0(a,b,c,d,e,15);
  // Rounds 16-19
  R1(e,a,b,c,d, 0); R1(d,e,a,b,c, 1); R1(c,d,e,a,b, 2); R1(b,c,d,e,a, 3);
  // Rounds 20-39
  R2(a,b,c,d,e, 4); R2(e,a,b,c,d, 5); R2(d,e,a,b,c, 6); R2(c,d,e,a,b, 7);
  R2(b,c,d,e,a, 8); R2(a,b,c,d,e, 9); R2(e,a,b,c,d,10); R2(d,e,a,b,c,11);
  R2(c,d,e,a,b,12); R2(b,c,d,e,a,13); R2(a,b,c,d,e,14); R2(e,a,b,c,d,15);
  R2(d,e,a,b,c, 0); R2(c,d,e,a,b, 1); R2(b,c,d,e,a, 2); R2(a,b,c,d,e, 3);
  // Rounds 40-59
  R3(e,a,b,c,d, 4); R3(d,e,a,b,c, 5); R3(c,d,e,a,b, 6); R3(b,c,d,e,a, 7);
  R3(a,b,c,d,e, 8); R3(e,a,b,c,d, 9); R3(d,e,a,b,c,10); R3(c,d,e,a,b,11);
  R3(b,c,d,e,a,12); R3(a,b,c,d,e,13); R3(e,a,b,c,d,14); R3(d,e,a,b,c,15);
  R3(c,d,e,a,b, 0); R3(b,c,d,e,a, 1); R3(a,b,c,d,e, 2); R3(e,a,b,c,d, 3);
  // Rounds 60-79
  R4(d,e,a,b,c, 4); R4(c,d,e,a,b, 5); R4(b,c,d,e,a, 6); R4(a,b,c,d,e, 7);
  R4(e,a,b,c,d, 8); R4(d,e,a,b,c, 9); R4(c,d,e,a,b,10); R4(b,c,d,e,a,11);
  R4(a,b,c,d,e,12); R4(e,a,b,c,d,13); R4(d,e,a,b,c,14); R4(c,d,e,a,b,15);
  R4(b,c,d,e,a, 0); R4(a,b,c,d,e, 1); R4(e,a,b,c,d, 2); R4(d,e,a,b,c, 3);
  R4(c,d,e,a,b, 4); R4(b,c,d,e,a, 5); R4(a,b,c,d,e, 6); R4(e,a,b,c,d, 7);
  R4(d,e,a,b,c, 8); R4(c,d,e,a,b, 9); R4(b,c,d,e,a,10); R4(a,b,c,d,e,11);
  R4(e,a,b,c,d,12); R4(d,e,a,b,c,13); R4(c,d,e,a,b,14); R4(b,c,d,e,a,15);

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
}


} // namespace

class SHA1 {
public:
  static constexpr size_t BLOCK_INTS = 16; // number of 32bit integers per SHA1 block
  static constexpr size_t BLOCK_BYTES = BLOCK_INTS * 4;
  static constexpr size_t DIGEST_BYTES = 20;

  SHA1() { reset(); }

  void reset() {
  digest[0] = 0x67452301;
  digest[1] = 0xefcdab89;
  digest[2] = 0x98badcfe;
  digest[3] = 0x10325476;
  digest[4] = 0xc3d2e1f0;
  transforms = 0;
  buffer_offset = 0;
  std::memset(buffer.data(), 0, BLOCK_BYTES); // Zero out buffer on reset
  }

  void update(const std::string& s) { update(reinterpret_cast<const uint8_t*>(s.c_str()), s.size()); }

  void update(const char* data, size_t len) {
  update(reinterpret_cast<const uint8_t*>(data), len);
  }

  void update(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
  buffer[buffer_offset++] = data[i];
  if (buffer_offset == BLOCK_BYTES) {
  transform(digest, buffer.data());
  transforms++;
  buffer_offset = 0;
  // No need to zero buffer here, it will be overwritten
  }
  }
  }

  // Add padding and return the message digest.
  std::array<uint8_t, DIGEST_BYTES> final() {
  uint64_t total_bits = (transforms * BLOCK_BYTES + buffer_offset) * 8;

  // Add padding: first a 0x80 byte
  buffer[buffer_offset++] = 0x80;

  // Check if we need an extra block for padding and length
  if (buffer_offset > BLOCK_BYTES - 8) {
  // Fill remaining buffer with zeros and transform
  std::memset(&buffer[buffer_offset], 0, BLOCK_BYTES - buffer_offset);
  transform(digest, buffer.data());
  // Prepare a new block, mostly zeros, for the length
  std::memset(buffer.data(), 0, BLOCK_BYTES - 8);
  buffer_offset = 0; // Reset offset for the new block logic below
  } else {
  // Fill remaining buffer (up to length) with zeros
  std::memset(&buffer[buffer_offset], 0, BLOCK_BYTES - 8 - buffer_offset);
  }


  // Append total length in bits (big-endian) to the end of the buffer
  // Note: The original code placed this incorrectly if an extra block was needed.
  auto uint64_to_bytes_be = [](uint64_t val, uint8_t* bytes) {
  bytes[0] = static_cast<uint8_t>((val >> 56) & 0xFF);
  bytes[1] = static_cast<uint8_t>((val >> 48) & 0xFF);
  bytes[2] = static_cast<uint8_t>((val >> 40) & 0xFF);
  bytes[3] = static_cast<uint8_t>((val >> 32) & 0xFF);
  bytes[4] = static_cast<uint8_t>((val >> 24) & 0xFF);
  bytes[5] = static_cast<uint8_t>((val >> 16) & 0xFF);
  bytes[6] = static_cast<uint8_t>((val >> 8)  & 0xFF);
  bytes[7] = static_cast<uint8_t>((val >> 0)  & 0xFF);
  };
  uint64_to_bytes_be(total_bits, &buffer[BLOCK_BYTES - 8]);


  // Perform final transformation on the potentially modified/new block
  transform(digest, buffer.data());

  // Convert digest words to byte array (big-endian)
  std::array<uint8_t, DIGEST_BYTES> hash;
  auto uint32_to_bytes_be = [](uint32_t val, uint8_t* bytes) {
  bytes[0] = static_cast<uint8_t>((val >> 24) & 0xFF);
  bytes[1] = static_cast<uint8_t>((val >> 16) & 0xFF);
  bytes[2] = static_cast<uint8_t>((val >> 8)  & 0xFF);
  bytes[3] = static_cast<uint8_t>((val >> 0)  & 0xFF);
  };
  for (size_t i = 0; i < 5; ++i) {
  uint32_to_bytes_be(digest[i], &hash[i * 4]);
  }

  reset(); // Reset for potential reuse

  return hash;
  }

private:
  uint32_t digest[5];
  std::array<uint8_t, BLOCK_BYTES> buffer;
  size_t buffer_offset;
  uint64_t transforms; // Number of full blocks processed
};

// Helper function for simple usage with string
inline std::array<uint8_t, SHA1::DIGEST_BYTES> digest(const std::string& s) {
  SHA1 hasher;
  hasher.update(s);
  return hasher.final();
}

// Helper function for simple usage with raw bytes
inline std::array<uint8_t, SHA1::DIGEST_BYTES> digest(const uint8_t* data, size_t len) {
  SHA1 hasher;
  hasher.update(data, len);
  return hasher.final();
}

// Overload for convenience with vector<uint8_t>
inline std::array<uint8_t, SHA1::DIGEST_BYTES> digest(const std::vector<uint8_t>& data) {
  SHA1 hasher;
  // Use the uint8_t* overload directly
  hasher.update(data.data(), data.size());
  return hasher.final();
}

} // namespace sha1 