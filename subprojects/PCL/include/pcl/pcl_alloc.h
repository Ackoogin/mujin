/// \file pcl_alloc.h
/// \brief PYRAMID Composition Library portable heap allocator.
#ifndef PCL_ALLOC_H
#define PCL_ALLOC_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Portable heap allocation for data that may be allocated in one
/// DLL/EXE and freed in another compiled with a different C runtime (e.g.
/// an MSVC codec plugin vs. a GNAT/MinGW Ada executable).
///
/// On Windows this goes through the process's single default heap
/// (HeapAlloc/HeapFree via kernel32), which is CRT-independent; elsewhere
/// it is plain malloc/free. Every C-ABI struct's variable-length field
/// (strings, arrays) must be allocated and freed through this pair, never
/// std::malloc/free directly.
void* pcl_alloc(size_t size);
/// \brief Zero-initialized allocation, equivalent to calloc(nmemb, size).
void* pcl_calloc(size_t nmemb, size_t size);
/// \brief Resize a pcl_alloc/pcl_calloc allocation, equivalent to realloc().
void* pcl_realloc(void* ptr, size_t size);
void  pcl_free(void* ptr);

#ifdef __cplusplus
}
#endif

#endif // PCL_ALLOC_H
