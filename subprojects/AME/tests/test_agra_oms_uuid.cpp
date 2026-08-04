#include "ame/agra_oms_uuid.h"

#include <gtest/gtest.h>

#include <string>

namespace {

std::string nativeBytes() {
  std::string value(16u, '\0');
  for (std::size_t index = 0; index < value.size(); ++index) {
    value[index] = static_cast<char>(index * 17u);
  }
  return value;
}

}  // namespace

TEST(AgraOmsUuid, RoundTripsEveryByteValueIncludingZeroAndHighBit) {
  // 0x00 and bytes above 0x7f are the ones a text-oriented conversion gets
  // wrong -- a NUL truncates a C string and a high byte is not valid UTF-8.
  const std::string native = nativeBytes();
  const std::string oms = ame::omsUuidFromNative(native);
  EXPECT_EQ(oms.size(), 32u);
  EXPECT_EQ(ame::nativeUuidFromOms(oms), native);
}

TEST(AgraOmsUuid, ProducesTheLowercaseHexTheCodecDemands) {
  std::string native(16u, '\0');
  native[0] = static_cast<char>(0xab);
  native[15] = static_cast<char>(0x0f);
  const std::string oms = ame::omsUuidFromNative(native);
  EXPECT_EQ(oms.substr(0, 2), "ab");
  EXPECT_EQ(oms.substr(30, 2), "0f");
}

TEST(AgraOmsUuid, AcceptsUppercaseHexOnTheWayIn) {
  // The codec's own validator accepts either case, so refusing uppercase here
  // would reject identifiers it would happily have encoded.
  EXPECT_EQ(ame::nativeUuidFromOms(std::string(32u, 'A')),
            ame::nativeUuidFromOms(std::string(32u, 'a')));
}

TEST(AgraOmsUuid, RejectsWrongLengthRatherThanPaddingOrTruncating) {
  EXPECT_THROW(ame::omsUuidFromNative(std::string(15u, '\0')),
               std::invalid_argument);
  EXPECT_THROW(ame::omsUuidFromNative(std::string(17u, '\0')),
               std::invalid_argument);
  EXPECT_THROW(ame::nativeUuidFromOms(std::string(31u, 'a')),
               std::invalid_argument);
  EXPECT_THROW(ame::nativeUuidFromOms(std::string(33u, 'a')),
               std::invalid_argument);
}

TEST(AgraOmsUuid, RejectsNonHexRatherThanDecodingItAsZero) {
  // Treating 'g' as 0 would yield a well-formed identifier that correlates
  // with nothing, and the mistake would surface far from its cause.
  std::string invalid(32u, 'a');
  invalid[7] = 'g';
  EXPECT_THROW(ame::nativeUuidFromOms(invalid), std::invalid_argument);
}
