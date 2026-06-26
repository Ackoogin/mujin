#ifndef PYRAMID_DATA_MODEL_SENSORPRODUCTS_CABI_H
#define PYRAMID_DATA_MODEL_SENSORPRODUCTS_CABI_H

#include <stdint.h>
#include "pyramid_datamodel_cabi.h"
#include "pyramid_data_model_common_cabi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pyramid_RadarDisplayProductRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
} pyramid_RadarDisplayProductRequirement_c;

typedef struct pyramid_RadarProductRequirement_c {
  pyramid_Entity_c base;  /* from Requirement */
  pyramid_Achievement_c status;  /* from Requirement */
} pyramid_RadarProductRequirement_c;

void pyramid_RadarDisplayProductRequirement_c_free(pyramid_RadarDisplayProductRequirement_c* value);
void pyramid_RadarProductRequirement_c_free(pyramid_RadarProductRequirement_c* value);

#ifdef __cplusplus
}
#endif

#endif /* PYRAMID_DATA_MODEL_SENSORPRODUCTS_CABI_H */
