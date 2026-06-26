#ifndef PYRAMID_DATAMODEL_CABI_H
#define PYRAMID_DATAMODEL_CABI_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define PYRAMID_DATAMODEL_ABI_VERSION 1u
typedef struct { const char* ptr; uint32_t len; } pyramid_str_t;
typedef struct { const void* ptr; uint32_t len; } pyramid_slice_t;
#ifdef __cplusplus
}
#endif
#endif
