/// \file pcl_internal.h
/// \brief PCL internal shared types between pcl_container.c and pcl_executor.c.
///
/// NOT part of the public API.
#ifndef PCL_INTERNAL_H
#define PCL_INTERNAL_H

#include "pcl/pcl_container.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCL_MAX_PORTS  64
#define PCL_MAX_PARAMS 128

// -- Parameter value tagged union ----------------------------------------

typedef enum {
  PCL_PARAM_STR,
  PCL_PARAM_F64,
  PCL_PARAM_I64,
  PCL_PARAM_BOOL,
} pcl_param_kind_t;

typedef struct {
  char              key[128];
  pcl_param_kind_t  kind;
  union {
    char    str_val[256];
    double  f64_val;
    int64_t i64_val;
    bool    bool_val;
  } u;
} pcl_param_t;

// -- Internal port representation ----------------------------------------

struct pcl_port_t {
  pcl_port_type_t       type;
  char                  name[128];
  char                  type_name[128];

  pcl_sub_callback_t    sub_cb;
  void*                 sub_user_data;

  pcl_service_handler_t svc_handler;
  void*                 svc_user_data;

  pcl_container_t*      owner;
};

// -- Internal container representation -----------------------------------

struct pcl_executor_t;

struct pcl_container_t {
  char              name[128];
  pcl_state_t       state;
  pcl_callbacks_t   callbacks;
  void*             user_data;

  double            tick_rate_hz;
  double            tick_accumulator;

  bool              configuring;

  struct pcl_port_t ports[PCL_MAX_PORTS];
  uint32_t          port_count;

  pcl_param_t       params[PCL_MAX_PARAMS];
  uint32_t          param_count;

  struct pcl_executor_t* executor;
};

#ifdef __cplusplus
}
#endif

#endif /* PCL_INTERNAL_H */
