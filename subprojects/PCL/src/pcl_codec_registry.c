/// \file pcl_codec_registry.c
/// \brief PCL codec registry implementation.
#include "pcl/pcl_codec_registry.h"
#include "pcl/pcl_alloc.h"

#include <stdlib.h>
#include <string.h>

struct pcl_codec_registry_t {
  const pcl_codec_t** codecs;
  uint32_t            count;
  uint32_t            capacity;
};

static pcl_codec_registry_t* pcl_default_codec_registry = NULL;

static const pcl_codec_t* find_codec_at(const pcl_codec_registry_t* registry,
                                        const char*                 content_type,
                                        uint32_t                    index) {
  uint32_t i;
  uint32_t matched = 0u;

  if (!registry || !content_type) return NULL;
  for (i = 0; i < registry->count; ++i) {
    const pcl_codec_t* codec = registry->codecs[i];
    if (codec && codec->content_type &&
        strcmp(codec->content_type, content_type) == 0) {
      if (matched == index) {
        return codec;
      }
      ++matched;
    }
  }
  return NULL;
}

static const pcl_codec_t* find_codec(const pcl_codec_registry_t* registry,
                                     const char*                 content_type) {
  return find_codec_at(registry, content_type, 0u);
}

static int contains_codec(const pcl_codec_registry_t* registry,
                          const pcl_codec_t*          codec) {
  uint32_t i;

  if (!registry) return 0;
  for (i = 0; i < registry->count; ++i) {
    if (registry->codecs[i] == codec) {
      return 1;
    }
  }
  return 0;
}

static pcl_status_t reserve_codec_slots(pcl_codec_registry_t* registry,
                                        uint32_t              required) {
  const pcl_codec_t** next_codecs;
  uint32_t next_capacity;
  size_t next_size;

  if (!registry) return PCL_ERR_INVALID;
  if (required <= registry->capacity) return PCL_OK;

  next_capacity = registry->capacity ? registry->capacity * 2u : 4u;
  while (next_capacity < required) {
    next_capacity *= 2u;
  }

  next_size = (size_t)next_capacity * sizeof(*registry->codecs);
  next_codecs = (const pcl_codec_t**)pcl_realloc(registry->codecs, next_size);
  if (!next_codecs) return PCL_ERR_NOMEM;

  registry->codecs = next_codecs;
  registry->capacity = next_capacity;
  return PCL_OK;
}

pcl_codec_registry_t* pcl_codec_registry_create(void) {
  return (pcl_codec_registry_t*)pcl_calloc(1, sizeof(pcl_codec_registry_t));
}

void pcl_codec_registry_destroy(pcl_codec_registry_t* registry) {
  if (!registry) return;
  pcl_free(registry->codecs);
  pcl_free(registry);
}

pcl_codec_registry_t* pcl_codec_registry_default(void) {
  if (!pcl_default_codec_registry) {
    pcl_default_codec_registry = pcl_codec_registry_create();
  }
  return pcl_default_codec_registry;
}

void pcl_codec_registry_clear(pcl_codec_registry_t* registry) {
  if (!registry) return;
  registry->count = 0u;
}

pcl_status_t pcl_codec_registry_register(pcl_codec_registry_t* registry,
                                         const pcl_codec_t*    codec) {
  pcl_status_t rc;

  if (!registry || !codec || !codec->content_type) return PCL_ERR_INVALID;
  if (codec->abi_version != PCL_CODEC_ABI_VERSION) return PCL_ERR_STATE;
  // Multiple codecs may share a content_type so a single process (e.g. a
  // PYRAMID bridge) can load per-component plugins side by side; dispatch then
  // selects by schema_id.  Re-registering the same vtable is still rejected.
  if (contains_codec(registry, codec)) return PCL_ERR_STATE;

  rc = reserve_codec_slots(registry, registry->count + 1u);
  if (rc != PCL_OK) return rc;

  registry->codecs[registry->count++] = codec;
  return PCL_OK;
}

const pcl_codec_t* pcl_codec_registry_get(const pcl_codec_registry_t* registry,
                                          const char*                 content_type) {
  return find_codec(registry, content_type);
}

const pcl_codec_t* pcl_codec_registry_get_at(const pcl_codec_registry_t* registry,
                                             const char*                 content_type,
                                             uint32_t                    index) {
  return find_codec_at(registry, content_type, index);
}

uint32_t pcl_codec_registry_count(const pcl_codec_registry_t* registry) {
  return registry ? registry->count : 0u;
}
