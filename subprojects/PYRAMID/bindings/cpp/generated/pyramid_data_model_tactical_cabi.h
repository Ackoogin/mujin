#ifndef PYRAMID_DATA_MODEL_TACTICAL_CABI_H
#define PYRAMID_DATA_MODEL_TACTICAL_CABI_H

#include <stdint.h>
#include "pyramid_datamodel_cabi.h"
#include "pyramid_data_model_base_cabi.h"
#include "pyramid_data_model_common_cabi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pyramid_ObjectDetail_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t entity_source;  /* from Entity */
  pyramid_slice_t source;
  pyramid_GeodeticPosition_c position;
  double creation_time;
  uint8_t has_quality;
  double quality;
  uint8_t has_course;
  double course;
  uint8_t has_speed;
  double speed;
  uint8_t has_length;
  double length;
  int32_t identity;
  int32_t dimension;
} pyramid_ObjectDetail_c;

typedef struct pyramid_ObjectEvidenceRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  int32_t policy;
  pyramid_slice_t dimension;
  uint8_t has_poly_area;
  pyramid_PolyArea_c poly_area;
  uint8_t has_circle_area;
  pyramid_CircleArea_c circle_area;
  uint8_t has_point;
  pyramid_Point_c point;
} pyramid_ObjectEvidenceRequirement_c;

typedef struct pyramid_ObjectInterestRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  uint8_t has_source;
  int32_t source;
  int32_t policy;
  pyramid_slice_t dimension;
  uint8_t has_poly_area;
  pyramid_PolyArea_c poly_area;
  uint8_t has_circle_area;
  pyramid_CircleArea_c circle_area;
  uint8_t has_point;
  pyramid_Point_c point;
} pyramid_ObjectInterestRequirement_c;

typedef struct pyramid_ObjectMatch_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
  pyramid_str_t matching_object_id;
} pyramid_ObjectMatch_c;

void pyramid_ObjectDetail_c_free(pyramid_ObjectDetail_c* value);
void pyramid_ObjectEvidenceRequirement_c_free(pyramid_ObjectEvidenceRequirement_c* value);
void pyramid_ObjectInterestRequirement_c_free(pyramid_ObjectInterestRequirement_c* value);
void pyramid_ObjectMatch_c_free(pyramid_ObjectMatch_c* value);

#ifdef __cplusplus
}
#endif

#endif /* PYRAMID_DATA_MODEL_TACTICAL_CABI_H */
