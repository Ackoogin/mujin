/// \file pcl_plugin.h
/// \brief PYRAMID Composition Library plugin entry-point contracts.
///
/// This header defines the stable symbols used by transport plugins.  Loading
/// code resolves these symbols from a shared library, checks the ABI version,
/// then installs the returned transport vtable on an executor.
#ifndef PCL_PLUGIN_H
#define PCL_PLUGIN_H

#include "pcl_capabilities.h"
#include "pcl_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCL_TRANSPORT_ABI_VERSION 1u

// -- Transport plugin entry contract -------------------------------------

#define PCL_TRANSPORT_ABI_VERSION_SYMBOL "pcl_transport_abi_version"
#define PCL_TRANSPORT_PLUGIN_ENTRY_SYMBOL "pcl_transport_plugin_entry"

/// \brief Optional symbol a transport plugin may export to declare its
/// interaction capabilities (see pcl_capabilities.h).
///
/// When absent, the loader conservatively derives the capability mask from the
/// returned vtable via \ref pcl_transport_caps_from_vtable.
#define PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL "pcl_transport_plugin_caps"

/// \brief Optional symbol a transport plugin may export to declare the QoS it
/// offers (see pcl_capabilities.h / pcl_qos_t).
///
/// When absent, the offered QoS is PCL_QOS_RELIABILITY_UNSPECIFIED, which
/// satisfies only an UNSPECIFIED endpoint floor (fail closed for reliability).
#define PCL_TRANSPORT_PLUGIN_QOS_SYMBOL "pcl_transport_plugin_qos"

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

/// \brief Function signature exported to declare transport capabilities.
///
/// A transport plugin may optionally export pcl_transport_plugin_caps() with
/// this signature. The \p config_json string is the same plugin-specific
/// configuration passed to the entry point, so a plugin whose capabilities
/// depend on configuration (e.g. server vs client role) can report accurately.
typedef pcl_transport_caps_t (*pcl_transport_plugin_caps_fn)(
    const char* config_json);

/// \brief Function signature exported to declare the transport's offered QoS.
///
/// A transport plugin may optionally export pcl_transport_plugin_qos() with this
/// signature. As with capabilities, \p config_json is the same configuration
/// passed to the entry point, so a plugin whose QoS depends on configuration can
/// report accurately.
typedef pcl_qos_t (*pcl_transport_plugin_qos_fn)(const char* config_json);

#ifdef __cplusplus
}
#endif

#endif // PCL_PLUGIN_H
