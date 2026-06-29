/// \file pcl_plugin_loader.h
/// \brief Runtime loader for PCL transport and codec plugins.
#ifndef PCL_PLUGIN_LOADER_H
#define PCL_PLUGIN_LOADER_H

#include "pcl_codec.h"
#include "pcl_codec_registry.h"
#include "pcl_plugin.h"
#include "pcl_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

// -- Opaque handles ------------------------------------------------------

typedef struct pcl_plugin_handle_t pcl_plugin_handle_t;

// -- Plugin loading ------------------------------------------------------

/// \brief Load a codec plugin and register its codec vtable.
///
/// The loaded plugin remains owned by \p out_handle on success.  The registry
/// borrows the codec vtable pointer, so the handle must remain loaded while
/// the registered codec may be used.  \p config_json is opaque, plugin-specific
/// configuration threaded into the plugin's entry point (uniform with
/// \ref pcl_plugin_load_transport); it may be NULL when no configuration is
/// supplied.
pcl_status_t pcl_plugin_load_codec(const char*             path,
                                   const char*             config_json,
                                   pcl_codec_registry_t*   registry,
                                   pcl_plugin_handle_t**   out_handle);

/// \brief Load codec plugins from a path array into \p registry.
///
/// Missing or invalid plugin paths are skipped. Successfully loaded plugins
/// remain resident for the lifetime of the process because the registry borrows
/// their codec vtables.
pcl_status_t pcl_codec_registry_load_plugins_from_paths(
    pcl_codec_registry_t* registry,
    const char* const*    paths,
    size_t               n);

/// \brief Load codec plugins from a path-list environment variable.
///
/// The environment value is split on ':' on POSIX and ';' on Windows. Missing
/// environment variables and individual missing plugin paths are skipped.
pcl_status_t pcl_codec_registry_load_plugins_from_env(
    pcl_codec_registry_t* registry,
    const char*           env_var);

/// \brief Load codec plugins listed in a manifest file.
///
/// The manifest is a newline-separated list of plugin paths.  Blank lines and
/// lines whose first non-space character is '#' are ignored, and surrounding
/// whitespace is trimmed.  Individual paths that do not load as codec plugins
/// (for example a transport plugin listed in a shared manifest) are skipped, so
/// a single deployment manifest may enumerate every plugin for a component.
///
/// A missing manifest file is reported as PCL_ERR_NOT_FOUND; an empty or
/// comment-only manifest is PCL_OK with nothing loaded.
pcl_status_t pcl_codec_registry_load_plugins_from_manifest(
    pcl_codec_registry_t* registry,
    const char*           manifest_path);

/// \brief Load a transport plugin and return its transport vtable.
///
/// The returned vtable pointer is borrowed from the plugin and remains valid
/// until \ref pcl_plugin_unload is called for \p out_handle.
pcl_status_t pcl_plugin_load_transport(const char*             path,
                                       const char*             config_json,
                                       pcl_plugin_handle_t**   out_handle,
                                       const pcl_transport_t** out_vtable);

/// \brief Report the interaction capabilities of a loaded transport plugin.
///
/// If the plugin exports the optional \ref PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL
/// symbol, its declared mask is returned (the plugin may key the mask off
/// \p config_json, so the same configuration string used to load it should be
/// passed here). Otherwise the mask is conservatively derived from \p vtable via
/// \ref pcl_transport_caps_from_vtable.
///
/// \param handle      Handle returned by \ref pcl_plugin_load_transport.
/// \param config_json Configuration string (may be NULL); forwarded to the caps
///                    symbol when present.
/// \param vtable      Vtable returned alongside \p handle, used for derivation.
/// \param out_caps    Receives the resolved capability mask.
/// \return PCL_OK on success, PCL_ERR_INVALID if \p handle or \p out_caps is
///         NULL.
pcl_status_t pcl_plugin_transport_caps(pcl_plugin_handle_t*   handle,
                                       const char*            config_json,
                                       const pcl_transport_t* vtable,
                                       pcl_transport_caps_t*  out_caps);

/// \brief Report the QoS a loaded transport plugin offers.
///
/// If the plugin exports the optional \ref PCL_TRANSPORT_PLUGIN_QOS_SYMBOL
/// symbol, its declared QoS is returned (keyed off \p config_json, so pass the
/// same configuration used to load it). Otherwise the offered QoS is
/// PCL_QOS_RELIABILITY_UNSPECIFIED -- there is nothing to derive from a vtable.
///
/// \param handle      Handle returned by \ref pcl_plugin_load_transport.
/// \param config_json Configuration string (may be NULL); forwarded to the QoS
///                    symbol when present.
/// \param out_qos     Receives the resolved offered QoS.
/// \return PCL_OK on success, PCL_ERR_INVALID if \p handle or \p out_qos is NULL.
pcl_status_t pcl_plugin_transport_qos(pcl_plugin_handle_t* handle,
                                      const char*          config_json,
                                      pcl_qos_t*           out_qos);

/// \brief Tear down a loaded transport instance, then unload its library.
///
/// Enforces the only safe order for coupled plugins that own live resources: if
/// the plugin exports \ref PCL_TRANSPORT_PLUGIN_TEARDOWN_SYMBOL it is called with
/// \p vtable (stopping spin threads, freeing the context) *before* the library is
/// unloaded — so a background thread is never left executing in code that
/// `dlclose` has unmapped. Plugins with no live state omit the symbol and this
/// degrades to a plain unload. Prefer this over a bare \ref pcl_plugin_unload for
/// any transport obtained via \ref pcl_plugin_load_transport.
///
/// \param handle Handle from \ref pcl_plugin_load_transport.
/// \param vtable Vtable returned alongside \p handle (may be NULL if unknown).
/// \return PCL_OK on success, PCL_ERR_INVALID if \p handle is NULL.
pcl_status_t pcl_plugin_unload_transport(pcl_plugin_handle_t*   handle,
                                         const pcl_transport_t* vtable);

/// \brief Open an arbitrary shared library and return a plugin handle.
///
/// Unlike \ref pcl_plugin_load_transport / \ref pcl_plugin_load_codec, this does
/// not require any PCL entry point: it is a portable wrapper over the platform
/// dynamic loader (dlopen / LoadLibrary) for libraries that only export plain
/// C symbols (e.g. the generated gRPC C shim). Resolve symbols with
/// \ref pcl_plugin_symbol and release the handle with \ref pcl_plugin_unload.
///
/// Returns NULL if \p path is NULL or the library cannot be opened.
pcl_plugin_handle_t* pcl_plugin_open(const char* path);

/// \brief Unload a plugin handle returned by a successful load call.
///
/// NULL-safe.
void pcl_plugin_unload(pcl_plugin_handle_t* handle);

/// \brief Resolve an arbitrary exported symbol from a loaded plugin.
///
/// Returns NULL when arguments are invalid or the symbol is not found.
void* pcl_plugin_symbol(pcl_plugin_handle_t* handle, const char* name);

#ifdef __cplusplus
}
#endif

#endif // PCL_PLUGIN_LOADER_H
