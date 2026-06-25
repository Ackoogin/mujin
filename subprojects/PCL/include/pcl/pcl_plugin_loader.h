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
/// the registered codec may be used.
pcl_status_t pcl_plugin_load_codec(const char*             path,
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

/// \brief Load a transport plugin and return its transport vtable.
///
/// The returned vtable pointer is borrowed from the plugin and remains valid
/// until \ref pcl_plugin_unload is called for \p out_handle.
pcl_status_t pcl_plugin_load_transport(const char*             path,
                                       const char*             config_json,
                                       pcl_plugin_handle_t**   out_handle,
                                       const pcl_transport_t** out_vtable);

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
