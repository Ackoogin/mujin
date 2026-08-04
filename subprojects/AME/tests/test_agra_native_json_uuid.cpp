#include "ame/agra_native_json_uuid.h"

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

TEST(AgraNativeJsonUuid, Base64RoundTripsEveryByteValue) {
  // 0x00 and bytes above 0x7f are the ones a text-oriented conversion gets
  // wrong -- a NUL truncates a C string and a high byte is not valid UTF-8.
  const std::string native = nativeBytes();
  const std::string encoded = ame::base64Encode(native);
  EXPECT_EQ(ame::base64Decode(encoded), native);
}

TEST(AgraNativeJsonUuid, Base64MatchesKnownVectors) {
  // RFC 4648 test vectors.
  EXPECT_EQ(ame::base64Encode(""), "");
  EXPECT_EQ(ame::base64Encode("f"), "Zg==");
  EXPECT_EQ(ame::base64Encode("fo"), "Zm8=");
  EXPECT_EQ(ame::base64Encode("foo"), "Zm9v");
  EXPECT_EQ(ame::base64Encode("foob"), "Zm9vYg==");
  EXPECT_EQ(ame::base64Encode("fooba"), "Zm9vYmE=");
  EXPECT_EQ(ame::base64Encode("foobar"), "Zm9vYmFy");

  EXPECT_EQ(ame::base64Decode("Zg=="), "f");
  EXPECT_EQ(ame::base64Decode("Zm9vYmFy"), "foobar");
}

TEST(AgraNativeJsonUuid, Base64RejectsWrongLength) {
  EXPECT_THROW(ame::base64Decode("Zg="), std::invalid_argument);
  EXPECT_THROW(ame::base64Decode("Z"), std::invalid_argument);
}

TEST(AgraNativeJsonUuid, Base64RejectsNonAlphabetCharacters) {
  EXPECT_THROW(ame::base64Decode("Zg#="), std::invalid_argument);
}

TEST(AgraNativeJsonUuid, Base64RejectsPaddingBeforeTheFinalBlock) {
  EXPECT_THROW(ame::base64Decode("Zg==AAAA"), std::invalid_argument);
}

TEST(AgraNativeJsonUuid, UuidRoundTripsThrough16Bytes) {
  const std::string native = nativeBytes();
  const std::string pivot = ame::nativeJsonUuidFromNative(native);
  EXPECT_EQ(ame::nativeUuidFromNativeJson(pivot), native);
}

TEST(AgraNativeJsonUuid, UuidRejectsWrongNativeLength) {
  EXPECT_THROW(ame::nativeJsonUuidFromNative(std::string(15u, '\0')),
               std::invalid_argument);
  EXPECT_THROW(ame::nativeJsonUuidFromNative(std::string(17u, '\0')),
               std::invalid_argument);
}

TEST(AgraNativeJsonUuid, UuidRejectsPivotThatDoesNotDecodeTo16Bytes) {
  EXPECT_THROW(ame::nativeUuidFromNativeJson(ame::base64Encode("short")),
               std::invalid_argument);
}
