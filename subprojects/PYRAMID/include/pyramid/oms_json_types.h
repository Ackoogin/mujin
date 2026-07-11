/// \file oms_json_types.h
/// \brief Frozen C ABI types for the initial UCI 2.5 OMS JSON codec subset.
#ifndef PYRAMID_OMS_JSON_TYPES_H
#define PYRAMID_OMS_JSON_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PYRAMID_OMS_JSON_TYPES_ABI_VERSION 1u
#define PYRAMID_OMS_UUID_SIZE 37u
#define PYRAMID_OMS_TIMESTAMP_SIZE 40u
#define PYRAMID_OMS_SCHEMA_VERSION_SIZE 32u
#define PYRAMID_OMS_LABEL_SIZE 257u
#define PYRAMID_OMS_TOKEN_SIZE 32u

/// \brief Common UCI message security and header fields used by this subset.
typedef struct {
  char classification[PYRAMID_OMS_TOKEN_SIZE];
  char owner_producer[PYRAMID_OMS_TOKEN_SIZE];
  char mission_uuid[PYRAMID_OMS_UUID_SIZE];
  char mission_label[PYRAMID_OMS_LABEL_SIZE];
  char system_uuid[PYRAMID_OMS_UUID_SIZE];
  char service_uuid[PYRAMID_OMS_UUID_SIZE];
  char timestamp[PYRAMID_OMS_TIMESTAMP_SIZE];
  char schema_version[PYRAMID_OMS_SCHEMA_VERSION_SIZE];
  char mode[PYRAMID_OMS_TOKEN_SIZE];
} pyramid_uci_oms_header_c;

/// \brief UCI 2.5 SignalReport subset produced by the starter-kit RF skill.
typedef struct {
  pyramid_uci_oms_header_c header;
  char signal_report_uuid[PYRAMID_OMS_UUID_SIZE];
  char signal_uuid[PYRAMID_OMS_UUID_SIZE];
  char signal_state[PYRAMID_OMS_TOKEN_SIZE];
  bool has_frequency_average;
  double frequency_average_hz;
} pyramid_uci_signal_report_c;

/// \brief UCI 2.5 PositionReport subset consumed by the starter-kit RF skill.
typedef struct {
  pyramid_uci_oms_header_c header;
  char report_system_uuid[PYRAMID_OMS_UUID_SIZE];
  char source[PYRAMID_OMS_TOKEN_SIZE];
  char current_operating_domain[PYRAMID_OMS_TOKEN_SIZE];
  double latitude_rad;
  double longitude_rad;
  double altitude_m;
  char position_timestamp[PYRAMID_OMS_TIMESTAMP_SIZE];
  bool has_altitude_reference;
  char altitude_reference[PYRAMID_OMS_TOKEN_SIZE];
} pyramid_uci_position_report_c;

#ifdef __cplusplus
}
#endif

#endif  // PYRAMID_OMS_JSON_TYPES_H
