/// \file pcl_codec_registry.h
/// \brief PYRAMID Composition Library runtime codec registry.
///
/// The codec registry maps content_type strings to borrowed codec vtables.
/// It is intentionally small and PCL-only: no dynamic loading, generator
/// dispatch, or transport behavior is implemented here.
#ifndef PCL_CODEC_REGISTRY_H
#define PCL_CODEC_REGISTRY_H

#include "pcl_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

// -- Opaque handles ------------------------------------------------------

typedef struct pcl_codec_registry_t pcl_codec_registry_t;

// -- Registry lifecycle --------------------------------------------------

/// \brief Create an empty codec registry.
///
/// Returns NULL on allocation failure.
pcl_codec_registry_t* pcl_codec_registry_create(void);

/// \brief Destroy a codec registry.
///
/// The registry only borrows codec vtable pointers; destroying it does not
/// destroy registered codecs.
void pcl_codec_registry_destroy(pcl_codec_registry_t* registry);

/// \brief Return the process-global default codec registry.
///
/// The registry is created lazily and the returned pointer remains stable for
/// the lifetime of the process.  Returns NULL only if allocation fails.
pcl_codec_registry_t* pcl_codec_registry_default(void);

/// \brief Remove all codec registrations from a registry.
///
/// Registered codec vtables are borrowed and are not destroyed.
void pcl_codec_registry_clear(pcl_codec_registry_t* registry);

// -- Registry operations -------------------------------------------------

/// \brief Register a codec vtable by content_type.
///
/// The registry borrows \p codec.  NULL arguments are rejected with
/// PCL_ERR_INVALID.  ABI-version mismatches are rejected with PCL_ERR_STATE.
///
/// Multiple codecs may share the same content_type: a single process (such as
/// a PYRAMID bridge spanning several components) can load per-component codec
/// plugins side by side, and dispatch selects among them by schema_id.
/// Re-registering the identical \p codec vtable pointer is rejected with
/// PCL_ERR_STATE.
pcl_status_t pcl_codec_registry_register(pcl_codec_registry_t* registry,
                                         const pcl_codec_t*    codec);

/// \brief Look up the first codec registered for \p content_type.
///
/// Returns NULL when arguments are invalid or the content type is not
/// registered.  When several codecs share a content_type, prefer
/// \ref pcl_codec_registry_get_at to iterate and select by schema_id.
const pcl_codec_t* pcl_codec_registry_get(const pcl_codec_registry_t* registry,
                                          const char*                 content_type);

/// \brief Look up the \p index-th codec registered for \p content_type.
///
/// Codecs are returned in registration order.  Returns NULL when \p index is
/// out of range, arguments are invalid, or the content type is not registered.
/// Callers iterate from index 0 upward (stopping at the first NULL) and try
/// each codec's encode/decode for a given schema_id until one succeeds.
const pcl_codec_t* pcl_codec_registry_get_at(const pcl_codec_registry_t* registry,
                                             const char*                 content_type,
                                             uint32_t                    index);

/// \brief Return the number of registered codecs.
///
/// Returns 0 for a NULL registry.
uint32_t pcl_codec_registry_count(const pcl_codec_registry_t* registry);

#ifdef __cplusplus
}
#endif

#endif // PCL_CODEC_REGISTRY_H
