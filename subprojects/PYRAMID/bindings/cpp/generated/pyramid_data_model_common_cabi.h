#ifndef PYRAMID_DATA_MODEL_COMMON_CABI_H
#define PYRAMID_DATA_MODEL_COMMON_CABI_H

#include <stdint.h>
#include "pyramid_datamodel_cabi.h"
#include "pyramid_data_model_base_cabi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pyramid_GeodeticPosition_c {
  double latitude;
  double longitude;
} pyramid_GeodeticPosition_c;

typedef struct pyramid_PolyArea_c {
  pyramid_slice_t points;
} pyramid_PolyArea_c;

typedef struct pyramid_Entity_c {
  uint8_t has_update_time;
  double update_time;
  pyramid_str_t id;
  pyramid_str_t source;
} pyramid_Entity_c;

typedef struct pyramid_Achievement_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  int32_t status;
  uint8_t has_quality;
  double quality;
  int32_t achieveability;
} pyramid_Achievement_c;

typedef struct pyramid_Requirement_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_Achievement_c status;
} pyramid_Requirement_c;

typedef struct pyramid_Contraint_c {
  pyramid_str_t name;
  int32_t value;
} pyramid_Contraint_c;

typedef struct pyramid_Capability_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  uint8_t availability;
  pyramid_str_t name;
  pyramid_slice_t contraint;
} pyramid_Capability_c;

typedef struct pyramid_CircleArea_c {
  pyramid_GeodeticPosition_c position;
  double radius;
} pyramid_CircleArea_c;

typedef struct pyramid_Point_c {
  pyramid_GeodeticPosition_c position;
} pyramid_Point_c;

typedef struct pyramid_Ack_c {
  uint8_t success;
} pyramid_Ack_c;

typedef struct pyramid_Query_c {
  pyramid_slice_t id;
  uint8_t has_one_shot;
  uint8_t one_shot;
} pyramid_Query_c;

void pyramid_GeodeticPosition_c_free(pyramid_GeodeticPosition_c* value);
void pyramid_PolyArea_c_free(pyramid_PolyArea_c* value);
void pyramid_Achievement_c_free(pyramid_Achievement_c* value);
void pyramid_Requirement_c_free(pyramid_Requirement_c* value);
void pyramid_Capability_c_free(pyramid_Capability_c* value);
void pyramid_Entity_c_free(pyramid_Entity_c* value);
void pyramid_CircleArea_c_free(pyramid_CircleArea_c* value);
void pyramid_Point_c_free(pyramid_Point_c* value);
void pyramid_Contraint_c_free(pyramid_Contraint_c* value);
void pyramid_Ack_c_free(pyramid_Ack_c* value);
void pyramid_Query_c_free(pyramid_Query_c* value);

#ifdef __cplusplus
}
#endif

#endif /* PYRAMID_DATA_MODEL_COMMON_CABI_H */
