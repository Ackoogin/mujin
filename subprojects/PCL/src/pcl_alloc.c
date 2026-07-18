#include "pcl/pcl_alloc.h"

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <stdlib.h>
#  include <string.h>
#endif

/* Implements: REQ_PCL_213. */
void* pcl_alloc(size_t size) {
  if (size == 0) return NULL;
#if defined(_WIN32)
  return HeapAlloc(GetProcessHeap(), 0, size);
#else
  return malloc(size);
#endif
}

/* Implements: REQ_PCL_214. */
void* pcl_calloc(size_t nmemb, size_t size) {
  size_t total;
  void*  ptr;
  if (nmemb == 0 || size == 0) return NULL;
  total = nmemb * size;
  if (total / nmemb != size) return NULL; /* overflow */
#if defined(_WIN32)
  ptr = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, total);
#else
  ptr = malloc(total);
  if (ptr) memset(ptr, 0, total);
#endif
  return ptr;
}

/* Implements: REQ_PCL_215, REQ_PCL_105, REQ_PCL_106. */
void* pcl_realloc(void* ptr, size_t size) {
  if (!ptr) return pcl_alloc(size);
  if (size == 0) {
    pcl_free(ptr);
    return NULL;
  }
#if defined(_WIN32)
  return HeapReAlloc(GetProcessHeap(), 0, ptr, size);
#else
  return realloc(ptr, size);
#endif
}

/* Implements: REQ_PCL_104, REQ_PCL_216. */
void pcl_free(void* ptr) {
  if (!ptr) return;
#if defined(_WIN32)
  HeapFree(GetProcessHeap(), 0, ptr);
#else
  free(ptr);
#endif
}
