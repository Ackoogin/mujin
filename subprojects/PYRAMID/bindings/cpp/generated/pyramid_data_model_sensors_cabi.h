#ifndef PYRAMID_DATA_MODEL_SENSORS_CABI_H
#define PYRAMID_DATA_MODEL_SENSORS_CABI_H

#include <stdint.h>
#include "pyramid_datamodel_cabi.h"
#include "pyramid_data_model_base_cabi.h"
#include "pyramid_data_model_common_cabi.h"
#include "pyramid_data_model_radar_cabi.h"
#include "pyramid_data_model_sensorproducts_cabi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pyramid_InterpretationRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  int32_t policy;
  int32_t type;
  uint8_t has_poly_area;
  pyramid_PolyArea_c poly_area;
  uint8_t has_circle_area;
  pyramid_CircleArea_c circle_area;
  uint8_t has_point;
  pyramid_Point_c point;
} pyramid_InterpretationRequirement_c;

typedef struct pyramid_ManualTrackRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  pyramid_GeodeticPosition_c position;
} pyramid_ManualTrackRequirement_c;

typedef struct pyramid_ATIRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  pyramid_PolyArea_c auto_zone;
} pyramid_ATIRequirement_c;

typedef struct pyramid_TrackProvisionRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  uint8_t has_poly_area;
  pyramid_PolyArea_c poly_area;
  uint8_t has_circle_area;
  pyramid_CircleArea_c circle_area;
  uint8_t has_point;
  pyramid_Point_c point;
} pyramid_TrackProvisionRequirement_c;

typedef struct pyramid_ObjectEvidenceProvisionRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  uint8_t has_poly_area;
  pyramid_PolyArea_c poly_area;
  uint8_t has_circle_area;
  pyramid_CircleArea_c circle_area;
  uint8_t has_point;
  pyramid_Point_c point;
} pyramid_ObjectEvidenceProvisionRequirement_c;

typedef struct pyramid_ObjectAquisitionRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  uint8_t has_poly_area;
  pyramid_PolyArea_c poly_area;
  uint8_t has_circle_area;
  pyramid_CircleArea_c circle_area;
  uint8_t has_point;
  pyramid_Point_c point;
} pyramid_ObjectAquisitionRequirement_c;

typedef struct pyramid_SensorObject_c {
  uint8_t has_update_time;
  double update_time;  /* from Entity */
  pyramid_str_t id;  /* from Entity */
  pyramid_str_t source;  /* from Entity */
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
} pyramid_SensorObject_c;

typedef struct pyramid_RadarModeChangeRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
  int32_t mode;
} pyramid_RadarModeChangeRequirement_c;

void pyramid_InterpretationRequirement_c_free(pyramid_InterpretationRequirement_c* value);
void pyramid_ManualTrackRequirement_c_free(pyramid_ManualTrackRequirement_c* value);
void pyramid_ATIRequirement_c_free(pyramid_ATIRequirement_c* value);
void pyramid_TrackProvisionRequirement_c_free(pyramid_TrackProvisionRequirement_c* value);
void pyramid_ObjectEvidenceProvisionRequirement_c_free(pyramid_ObjectEvidenceProvisionRequirement_c* value);
void pyramid_ObjectAquisitionRequirement_c_free(pyramid_ObjectAquisitionRequirement_c* value);
void pyramid_SensorObject_c_free(pyramid_SensorObject_c* value);
void pyramid_RadarModeChangeRequirement_c_free(pyramid_RadarModeChangeRequirement_c* value);

#ifdef __cplusplus
}
#endif

#endif /* PYRAMID_DATA_MODEL_SENSORS_CABI_H */
