#include "pcl/pcl_codec_registry.h"

#include <gtest/gtest.h>

#include <cstring>

namespace {

struct StubValue {
  int value;
};

struct StubCodecCtx {
  bool freed = false;
};

pcl_status_t stubEncode(void*       codec_ctx,
                        const char* schema_id,
                        const void* value,
                        pcl_msg_t*  out_msg) {
  static_cast<void>(codec_ctx);
  if (!schema_id || !value || !out_msg) {
    return PCL_ERR_INVALID;
  }
  if (std::strcmp(schema_id, "StubValue") != 0) {
    return PCL_ERR_NOT_FOUND;
  }

  const StubValue* typed_value = static_cast<const StubValue*>(value);
  out_msg->data = &typed_value->value;
  out_msg->size = sizeof(typed_value->value);
  out_msg->type_name = schema_id;
  return PCL_OK;
}

pcl_status_t stubDecode(void*            codec_ctx,
                        const char*      schema_id,
                        const pcl_msg_t* msg,
                        void*            out_value) {
  static_cast<void>(codec_ctx);
  if (!schema_id || !msg || !out_value) {
    return PCL_ERR_INVALID;
  }
  if (std::strcmp(schema_id, "StubValue") != 0 ||
      msg->size != sizeof(int) ||
      !msg->data) {
    return PCL_ERR_INVALID;
  }

  StubValue* typed_value = static_cast<StubValue*>(out_value);
  std::memcpy(&typed_value->value, msg->data, sizeof(typed_value->value));
  return PCL_OK;
}

void stubFreeMsg(void* codec_ctx, pcl_msg_t* msg) {
  StubCodecCtx* ctx = static_cast<StubCodecCtx*>(codec_ctx);
  if (ctx) {
    ctx->freed = true;
  }
  if (msg) {
    msg->data = nullptr;
    msg->size = 0u;
    msg->type_name = nullptr;
  }
}

pcl_codec_t makeCodec(const char* content_type,
                      uint32_t    abi_version = PCL_CODEC_ABI_VERSION,
                      void*       codec_ctx = nullptr) {
  pcl_codec_t codec = {};
  codec.abi_version = abi_version;
  codec.content_type = content_type;
  codec.encode = stubEncode;
  codec.decode = stubDecode;
  codec.free_msg = stubFreeMsg;
  codec.codec_ctx = codec_ctx;
  return codec;
}

}  // namespace

TEST(PclCodecRegistry, RegisterAndGetReturnsSamePointer) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_codec_t codec = makeCodec("application/test");

  EXPECT_EQ(pcl_codec_registry_register(registry, &codec), PCL_OK);
  EXPECT_EQ(pcl_codec_registry_get(registry, "application/test"), &codec);

  pcl_codec_registry_destroy(registry);
}

TEST(PclCodecRegistry, GetUnregisteredReturnsNull) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  EXPECT_EQ(pcl_codec_registry_get(registry, "application/missing"), nullptr);

  pcl_codec_registry_destroy(registry);
}

TEST(PclCodecRegistry, BadAbiVersionRejectedWithState) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_codec_t codec = makeCodec("application/bad-abi",
                                PCL_CODEC_ABI_VERSION + 1u);

  EXPECT_EQ(pcl_codec_registry_register(registry, &codec), PCL_ERR_STATE);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);

  pcl_codec_registry_destroy(registry);
}

TEST(PclCodecRegistry, NullArgsRejectedWithInvalid) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_codec_t codec = makeCodec("application/test");
  pcl_codec_t no_content_type = makeCodec(nullptr);

  EXPECT_EQ(pcl_codec_registry_register(nullptr, &codec), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_register(registry, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_register(registry, &no_content_type),
            PCL_ERR_INVALID);
  EXPECT_EQ(pcl_codec_registry_get(nullptr, "application/test"), nullptr);
  EXPECT_EQ(pcl_codec_registry_get(registry, nullptr), nullptr);

  pcl_codec_registry_destroy(registry);
}

TEST(PclCodecRegistry, DuplicateContentTypeRejectedWithStateAndFirstWins) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_codec_t first = makeCodec("application/duplicate");
  pcl_codec_t second = makeCodec("application/duplicate");

  EXPECT_EQ(pcl_codec_registry_register(registry, &first), PCL_OK);
  EXPECT_EQ(pcl_codec_registry_register(registry, &second), PCL_ERR_STATE);
  EXPECT_EQ(pcl_codec_registry_get(registry, "application/duplicate"), &first);
  EXPECT_EQ(pcl_codec_registry_count(registry), 1u);

  pcl_codec_registry_destroy(registry);
}

TEST(PclCodecRegistry, CountAndClearWork) {
  pcl_codec_registry_t* registry = pcl_codec_registry_create();
  ASSERT_NE(registry, nullptr);

  pcl_codec_t first = makeCodec("application/one");
  pcl_codec_t second = makeCodec("application/two");

  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);
  EXPECT_EQ(pcl_codec_registry_register(registry, &first), PCL_OK);
  EXPECT_EQ(pcl_codec_registry_register(registry, &second), PCL_OK);
  EXPECT_EQ(pcl_codec_registry_count(registry), 2u);

  pcl_codec_registry_clear(registry);
  EXPECT_EQ(pcl_codec_registry_count(registry), 0u);
  EXPECT_EQ(pcl_codec_registry_get(registry, "application/one"), nullptr);

  pcl_codec_registry_destroy(registry);
}

TEST(PclCodecRegistry, DefaultReturnsStableNonNullPointer) {
  pcl_codec_registry_t* first = pcl_codec_registry_default();
  pcl_codec_registry_t* second = pcl_codec_registry_default();

  ASSERT_NE(first, nullptr);
  EXPECT_EQ(second, first);
}

TEST(PclCodecRegistry, RoundTripEncodeDecodeFreeMsgThroughStubCodec) {
  StubCodecCtx ctx;
  pcl_codec_t codec = makeCodec("application/stub", PCL_CODEC_ABI_VERSION, &ctx);
  StubValue input{42};
  StubValue output{0};
  pcl_msg_t msg = {};

  ASSERT_NE(codec.encode, nullptr);
  ASSERT_NE(codec.decode, nullptr);
  ASSERT_NE(codec.free_msg, nullptr);

  EXPECT_EQ(codec.encode(codec.codec_ctx, "StubValue", &input, &msg), PCL_OK);
  EXPECT_EQ(msg.size, sizeof(int));
  EXPECT_EQ(codec.decode(codec.codec_ctx, "StubValue", &msg, &output), PCL_OK);
  EXPECT_EQ(output.value, input.value);

  codec.free_msg(codec.codec_ctx, &msg);
  EXPECT_TRUE(ctx.freed);
  EXPECT_EQ(msg.data, nullptr);
  EXPECT_EQ(msg.size, 0u);
  EXPECT_EQ(msg.type_name, nullptr);
}
