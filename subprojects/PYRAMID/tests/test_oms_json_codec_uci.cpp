/// \file test_oms_json_codec_uci.cpp
/// \brief Golden and round-trip tests for the UCI 2.5 OMS JSON codec subset.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_codec.h>
#include <pcl/pcl_plugin_loader.h>
}

#include <pyramid/oms_json_types.h>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>

#ifndef PYRAMID_OMS_JSON_CODEC_PATH
#error "PYRAMID_OMS_JSON_CODEC_PATH must be defined by the build"
#endif

#ifndef PYRAMID_LACAL_FIXTURE_DIR
#error "PYRAMID_LACAL_FIXTURE_DIR must be defined by the build"
#endif

namespace {

using CodecEntry = const pcl_codec_t* (*)(const char* config_json);

template <size_t Size>
void set_text(char (&field)[Size], const char* value) {
  ASSERT_LT(std::strlen(value), Size);
  std::memcpy(field, value, std::strlen(value) + 1u);
}

nlohmann::json load_fixture(const char* name) {
  const std::string path =
      std::string(PYRAMID_LACAL_FIXTURE_DIR) + "/" + name;
  std::ifstream input(path);
  EXPECT_TRUE(input.good()) << path;
  return nlohmann::json::parse(input);
}

const pcl_codec_t* load_codec(pcl_plugin_handle_t** handle) {
  *handle = pcl_plugin_open(PYRAMID_OMS_JSON_CODEC_PATH);
  EXPECT_NE(*handle, nullptr);
  if (!*handle) return nullptr;
  auto entry = reinterpret_cast<CodecEntry>(
      pcl_plugin_symbol(*handle, PCL_CODEC_PLUGIN_ENTRY_SYMBOL));
  EXPECT_NE(entry, nullptr);
  return entry ? entry(nullptr) : nullptr;
}

void fill_signal_report(pyramid_uci_signal_report_c* report) {
  std::memset(report, 0, sizeof(*report));
  set_text(report->header.classification, "U");
  set_text(report->header.owner_producer, "USA");
  set_text(report->header.mission_uuid,
           "11111111-2222-3333-4444-555555555555");
  set_text(report->header.mission_label, "mission");
  set_text(report->header.system_uuid,
           "550e8400-e29b-41d4-a716-446655440000");
  set_text(report->header.service_uuid,
           "22222222-2222-4222-8222-222222222222");
  set_text(report->header.timestamp, "2026-01-01T00:00:00Z");
  set_text(report->header.schema_version, "002.5.0");
  set_text(report->header.mode, "LIVE");
  set_text(report->signal_report_uuid,
           "550e8400-e29b-41d4-a716-446655440010");
  set_text(report->signal_uuid,
           "550e8400-e29b-41d4-a716-446655440020");
  set_text(report->signal_state, "NEW");
  report->has_frequency_average = true;
  report->frequency_average_hz = 106300000.0;
}

void fill_position_report(pyramid_uci_position_report_c* report) {
  std::memset(report, 0, sizeof(*report));
  set_text(report->header.classification, "R");
  set_text(report->header.owner_producer, "FGI");
  set_text(report->header.system_uuid,
           "550e8400-e29b-41d4-a716-446655440000");
  set_text(report->header.timestamp, "2026-01-01T00:00:00Z");
  set_text(report->header.schema_version, "002.5.0");
  set_text(report->header.mode, "LIVE");
  set_text(report->report_system_uuid,
           "550e8400-e29b-41d4-a716-446655440000");
  set_text(report->source, "ACTUAL");
  set_text(report->current_operating_domain, "AIR");
  report->latitude_deg = 1.0;
  report->longitude_deg = 1.0;
  report->altitude_m = 1.0;
  set_text(report->position_timestamp, "2026-01-01T00:00:00Z");
  report->has_altitude_reference = true;
  set_text(report->altitude_reference, "WGS84");
}

TEST(OmsJsonUciCodec, DeclaresCodecAbiAndContentType) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = load_codec(&handle);
  ASSERT_NE(codec, nullptr);
  EXPECT_EQ(codec->abi_version, PCL_CODEC_ABI_VERSION);
  EXPECT_STREQ(codec->content_type, "application/oms-json");
  ASSERT_NE(codec->encode, nullptr);
  ASSERT_NE(codec->decode, nullptr);
  ASSERT_NE(codec->free_msg, nullptr);
  pcl_plugin_unload(handle);
}

TEST(OmsJsonUciCodec, SignalReportMatchesForeignSkillFixture) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = load_codec(&handle);
  ASSERT_NE(codec, nullptr);

  pyramid_uci_signal_report_c report;
  fill_signal_report(&report);
  pcl_msg_t encoded{};
  ASSERT_EQ(codec->encode(codec->codec_ctx, "SignalReport", &report, &encoded),
            PCL_OK);
  const auto actual = nlohmann::json::parse(
      static_cast<const char*>(encoded.data),
      static_cast<const char*>(encoded.data) + encoded.size);
  EXPECT_EQ(actual, load_fixture("signal_report.json"));
  EXPECT_STREQ(encoded.type_name, "application/oms-json");

  codec->free_msg(codec->codec_ctx, &encoded);
  pcl_plugin_unload(handle);
}

TEST(OmsJsonUciCodec, DecodesEnrichedForeignSignalReport) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = load_codec(&handle);
  ASSERT_NE(codec, nullptr);

  std::string payload = load_fixture("signal_report.json").dump();
  pcl_msg_t msg{payload.data(), static_cast<uint32_t>(payload.size()),
                "application/oms-json"};
  pyramid_uci_signal_report_c report{};
  ASSERT_EQ(codec->decode(codec->codec_ctx, "SignalReport", &msg, &report),
            PCL_OK);
  EXPECT_STREQ(report.header.system_uuid,
               "550e8400-e29b-41d4-a716-446655440000");
  EXPECT_STREQ(report.signal_report_uuid,
               "550e8400-e29b-41d4-a716-446655440010");
  EXPECT_STREQ(report.signal_uuid,
               "550e8400-e29b-41d4-a716-446655440020");
  EXPECT_STREQ(report.signal_state, "NEW");
  EXPECT_TRUE(report.has_frequency_average);
  EXPECT_DOUBLE_EQ(report.frequency_average_hz, 106300000.0);

  pcl_plugin_unload(handle);
}

TEST(OmsJsonUciCodec, PositionReportMatchesForeignSkillFixtureAndRoundTrips) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = load_codec(&handle);
  ASSERT_NE(codec, nullptr);

  pyramid_uci_position_report_c report;
  fill_position_report(&report);
  pcl_msg_t encoded{};
  ASSERT_EQ(codec->encode(codec->codec_ctx, "PositionReport", &report,
                          &encoded),
            PCL_OK);
  const auto actual = nlohmann::json::parse(
      static_cast<const char*>(encoded.data),
      static_cast<const char*>(encoded.data) + encoded.size);
  EXPECT_EQ(actual, load_fixture("position_report.json"));

  pyramid_uci_position_report_c decoded{};
  ASSERT_EQ(codec->decode(codec->codec_ctx, "PositionReport", &encoded,
                          &decoded),
            PCL_OK);
  EXPECT_STREQ(decoded.report_system_uuid,
               "550e8400-e29b-41d4-a716-446655440000");
  EXPECT_DOUBLE_EQ(decoded.latitude_deg, 1.0);
  EXPECT_DOUBLE_EQ(decoded.longitude_deg, 1.0);
  EXPECT_DOUBLE_EQ(decoded.altitude_m, 1.0);
  EXPECT_TRUE(decoded.has_altitude_reference);
  EXPECT_STREQ(decoded.altitude_reference, "WGS84");

  codec->free_msg(codec->codec_ctx, &encoded);
  pcl_plugin_unload(handle);
}

TEST(OmsJsonUciCodec, HandlesOmsSpecialFloatingPointTokens) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = load_codec(&handle);
  ASSERT_NE(codec, nullptr);

  pyramid_uci_position_report_c report;
  fill_position_report(&report);
  report.latitude_deg = std::numeric_limits<double>::quiet_NaN();
  report.longitude_deg = std::numeric_limits<double>::infinity();
  report.altitude_m = -std::numeric_limits<double>::infinity();
  pcl_msg_t encoded{};
  ASSERT_EQ(codec->encode(codec->codec_ctx, "PositionReport", &report,
                          &encoded),
            PCL_OK);
  const auto json = nlohmann::json::parse(
      static_cast<const char*>(encoded.data),
      static_cast<const char*>(encoded.data) + encoded.size);
  const auto& position = json.at("PositionReport")
                             .at("MessageData")
                             .at("InertialState")
                             .at("Position");
  EXPECT_EQ(position.at("Latitude"), "NaN");
  EXPECT_EQ(position.at("Longitude"), "Infinity");
  EXPECT_EQ(position.at("Altitude"), "-Infinity");

  pyramid_uci_position_report_c decoded{};
  ASSERT_EQ(codec->decode(codec->codec_ctx, "PositionReport", &encoded,
                          &decoded),
            PCL_OK);
  EXPECT_TRUE(std::isnan(decoded.latitude_deg));
  EXPECT_EQ(decoded.longitude_deg, std::numeric_limits<double>::infinity());
  EXPECT_EQ(decoded.altitude_m, -std::numeric_limits<double>::infinity());

  codec->free_msg(codec->codec_ctx, &encoded);
  pcl_plugin_unload(handle);
}

TEST(OmsJsonUciCodec, RejectsInvalidUuidAndUnknownSchema) {
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_codec_t* codec = load_codec(&handle);
  ASSERT_NE(codec, nullptr);

  pyramid_uci_signal_report_c report;
  fill_signal_report(&report);
  set_text(report.signal_uuid, "not-a-uuid");
  pcl_msg_t encoded{};
  EXPECT_EQ(codec->encode(codec->codec_ctx, "SignalReport", &report, &encoded),
            PCL_ERR_INVALID);
  EXPECT_EQ(codec->encode(codec->codec_ctx, "Unknown", &report, &encoded),
            PCL_ERR_NOT_FOUND);

  pcl_plugin_unload(handle);
}

}  // namespace
