/// \file pcl_plugin.h
/// \brief PYRAMID Composition Library plugin entry-point contracts.
///
/// This header defines the stable symbols used by transport plugins.  Loading
/// code resolves these symbols from a shared library, checks the ABI version,
/// then installs the returned transport vtable on an executor.
#ifndef PCL_PLUGIN_H
#define PCL_PLUGIN_H

#include "pcl_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCL_TRANSPORT_ABI_VERSION 1u

// -- Transport plugin entry contract -------------------------------------

#define PCL_TRANSPORT_ABI_VERSION_SYMBOL "pcl_transport_abi_version"
#define PCL_TRANSPORT_PLUGIN_ENTRY_SYMBOL "pcl_transport_plugin_entry"

/// \brief Function signature exported to report the transport ABI version.
///
/// A transport plugin exports pcl_transport_abi_version() with this signature.
/// The return value must equal PCL_TRANSPORT_ABI_VERSION.
typedef uint32_t (*pcl_transport_abi_version_fn)(void);

/// \brief Function signature exported to create or return a transport vtable.
///
/// A transport plugin exports pcl_transport_plugin_entry() with this signature.
/// The \p config_json string is plugin-specific configuration, and the returned
/// vtable pointer is borrowed by PCL.
typedef const pcl_transport_t* (*pcl_transport_plugin_entry_fn)(
    const char* config_json);

#ifdef __cplusplus
}
#endif

#endif // PCL_PLUGIN_H
