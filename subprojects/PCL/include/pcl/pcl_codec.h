/// \file pcl_codec.h
/// \brief PYRAMID Container Library codec plugin interface.
///
/// Codecs translate typed values to and from generic PCL message buffers.
/// The codec interface is a stable C ABI vtable so implementations can be
/// linked statically today and moved behind shared-library plugins later.
#ifndef PCL_CODEC_H
#define PCL_CODEC_H

#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCL_CODEC_ABI_VERSION 2u

// -- Codec adapter vtable ------------------------------------------------

/// \brief Function pointers that a codec adapter implements.
///
/// All functions are called by generated facades or dispatch code.  The codec
/// is selected by content_type, and the schema_id identifies the concrete typed
/// value crossing the ABI.
typedef struct {
  /// \brief ABI version implemented by this vtable.
  ///
  /// Must equal PCL_CODEC_ABI_VERSION.  Registry and plugin-loader code must
  /// fail closed when this value does not match.
  uint32_t abi_version;

  /// \brief MIME-style content type handled by this codec.
  ///
  /// Examples include "application/json" and "application/flatbuffers".
  const char* content_type;

  /// \brief Encode a typed value into a PCL message buffer.
  ///
  /// \param codec_ctx Codec-owned context pointer.
  /// \param schema_id Stable identifier for the typed value schema.
  /// \param value     Typed value to encode.
  /// \param out_msg   Receives the encoded message buffer.
  /// \return PCL_OK on success, error code on failure.
  pcl_status_t (*encode)(void*       codec_ctx,
                         const char* schema_id,
                         const void* value,
                         pcl_msg_t*  out_msg);

  /// \brief Decode a PCL message buffer into a caller-allocated typed value.
  ///
  /// \param codec_ctx Codec-owned context pointer.
  /// \param schema_id Stable identifier for the typed value schema.
  /// \param msg       Encoded message buffer.
  /// \param out_value Caller-allocated typed value receiving decoded data.
  /// \return PCL_OK on success, error code on failure.
  pcl_status_t (*decode)(void*            codec_ctx,
                         const char*      schema_id,
                         const pcl_msg_t* msg,
                         void*            out_value);

  /// \brief Release a message buffer allocated by encode.
  ///
  /// This is an intentional addition beyond the original transition-plan
  /// sketch.  It gives codecs an explicit ownership hook for encoded buffers
  /// whose lifetime crosses the C ABI.  May be NULL if encode only returns
  /// caller-owned or otherwise externally managed buffers.
  void (*free_msg)(void* codec_ctx, pcl_msg_t* msg);

  /// \brief Opaque context owned by the codec implementation.
  void* codec_ctx;
} pcl_codec_t;

// -- Codec plugin entry contract -----------------------------------------

#define PCL_CODEC_PLUGIN_ENTRY_SYMBOL "pcl_codec_plugin_entry"

/// \brief Function signature exported by each codec plugin.
///
/// A codec plugin exports pcl_codec_plugin_entry() with this signature and
/// returns a borrowed pointer to a stable codec vtable.  The \p config_json
/// string is opaque, plugin-specific configuration threaded through the loader
/// (uniform with the transport plugin entry contract); it may be NULL when no
/// configuration is supplied.  Plugins should treat NULL as "no configuration".
typedef const pcl_codec_t* (*pcl_codec_plugin_entry_fn)(const char* config_json);

#ifdef __cplusplus
}
#endif

#endif // PCL_CODEC_H
