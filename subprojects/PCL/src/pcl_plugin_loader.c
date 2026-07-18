/// \file pcl_plugin_loader.c
/// \brief Runtime loader for PCL transport and codec plugins.
#include "pcl/pcl_plugin_loader.h"
#include "pcl/pcl_alloc.h"
#include "pcl/pcl_log.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

struct pcl_plugin_handle_t {
#ifdef _WIN32
  HMODULE library;
#else
  void* library;
#endif
};

static pcl_plugin_handle_t** pcl_resident_codec_plugins = NULL;
static size_t pcl_resident_codec_plugin_count = 0;
static size_t pcl_resident_codec_plugin_capacity = 0;

/* Implements: REQ_PCL_405. */
static int retain_codec_plugin(pcl_plugin_handle_t* handle) {
  pcl_plugin_handle_t** next;
  size_t next_capacity;

  if (!handle) return 0;
  if (pcl_resident_codec_plugin_count < pcl_resident_codec_plugin_capacity) {
    pcl_resident_codec_plugins[pcl_resident_codec_plugin_count++] = handle;
    return 1;
  }

  next_capacity = pcl_resident_codec_plugin_capacity == 0
      ? 8u
      : pcl_resident_codec_plugin_capacity * 2u;
  next = (pcl_plugin_handle_t**)pcl_realloc(
      pcl_resident_codec_plugins, next_capacity * sizeof(*next));
  if (!next) return 0;

  pcl_resident_codec_plugins = next;
  pcl_resident_codec_plugin_capacity = next_capacity;
  pcl_resident_codec_plugins[pcl_resident_codec_plugin_count++] = handle;
  return 1;
}

static void close_library(pcl_plugin_handle_t* handle) {
  if (!handle) return;
#ifdef _WIN32
  if (handle->library) {
    FreeLibrary(handle->library);
  }
#else
  if (handle->library) {
    dlclose(handle->library);
  }
#endif
  pcl_free(handle);
}

static pcl_plugin_handle_t* open_library(const char* path) {
  pcl_plugin_handle_t* handle;

  if (!path) return NULL;

  handle = (pcl_plugin_handle_t*)pcl_calloc(1, sizeof(pcl_plugin_handle_t));
  if (!handle) return NULL;

#ifdef _WIN32
  handle->library = LoadLibraryA(path);
#else
  handle->library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
#endif
  if (!handle->library) {
    pcl_free(handle);
    return NULL;
  }

  return handle;
}

static void* resolve_symbol(pcl_plugin_handle_t* handle, const char* name) {
  if (!handle || !handle->library || !name) return NULL;
#ifdef _WIN32
  return (void*)GetProcAddress(handle->library, name);
#else
  return dlsym(handle->library, name);
#endif
}

/// \brief Load a codec plugin and register its codec vtable.
/* Implements: REQ_PCL_385, REQ_PCL_386, REQ_PCL_393, REQ_PCL_394,
   REQ_PCL_395. */
pcl_status_t pcl_plugin_load_codec(const char*             path,
                                   const char*             config_json,
                                   pcl_codec_registry_t*   registry,
                                   pcl_plugin_handle_t**   out_handle) {
  pcl_plugin_handle_t* handle;
  pcl_codec_plugin_entry_fn entry;
  const pcl_codec_t* codec;
  pcl_status_t rc;

  if (!path || path[0] == '\0' || !registry || !out_handle) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "codec plugin load failed: path, registry, and output handle are required");
    return PCL_ERR_INVALID;
  }
  *out_handle = NULL;

  handle = open_library(path);
  if (!handle) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "codec plugin load failed: cannot open '%s'", path);
    return PCL_ERR_NOT_FOUND;
  }

  entry = (pcl_codec_plugin_entry_fn)resolve_symbol(
      handle, PCL_CODEC_PLUGIN_ENTRY_SYMBOL);
  if (!entry) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "codec plugin load failed: '%s' does not export %s",
            path, PCL_CODEC_PLUGIN_ENTRY_SYMBOL);
    close_library(handle);
    return PCL_ERR_NOT_FOUND;
  }

  codec = entry(config_json);
  if (!codec) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "codec plugin load failed: entry point in '%s' rejected its configuration",
            path);
    close_library(handle);
    return PCL_ERR_STATE;
  }
  if (codec->abi_version != PCL_CODEC_ABI_VERSION) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "codec plugin load failed: '%s' uses ABI version %u, required version is %u",
            path, (unsigned)codec->abi_version,
            (unsigned)PCL_CODEC_ABI_VERSION);
    close_library(handle);
    return PCL_ERR_STATE;
  }

  rc = pcl_codec_registry_register(registry, codec);
  if (rc != PCL_OK) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "codec plugin load failed: could not register codec from '%s' (rc=%d)",
            path, (int)rc);
    close_library(handle);
    return rc;
  }

  *out_handle = handle;
  return PCL_OK;
}

/* Implements: REQ_PCL_399, REQ_PCL_221, REQ_PCL_405. */
pcl_status_t pcl_codec_registry_load_plugins_from_paths(
    pcl_codec_registry_t* registry,
    const char* const*    paths,
    size_t               n) {
  size_t i;

  if (!registry || (!paths && n != 0)) return PCL_ERR_INVALID;

  for (i = 0; i < n; ++i) {
    pcl_plugin_handle_t* handle = NULL;
    if (!paths[i] || paths[i][0] == '\0') continue;
    if (pcl_plugin_load_codec(paths[i], NULL, registry, &handle) != PCL_OK) {
      continue;
    }
    if (!retain_codec_plugin(handle)) {
      // GCOVR_EXCL_START: only reachable when the resident-list allocation
      // fails; heap exhaustion is not injectable through this call path.
      pcl_plugin_unload(handle);
      return PCL_ERR_NOMEM;
      // GCOVR_EXCL_STOP
    }
  }

  return PCL_OK;
}

/* Implements: REQ_PCL_400. */
pcl_status_t pcl_codec_registry_load_plugins_from_env(
    pcl_codec_registry_t* registry,
    const char*           env_var) {
  const char* value;
  char* copy;
  char* cursor;
  char* token_start;
  pcl_status_t status;
#ifdef _WIN32
  const char separator = ';';
#else
  const char separator = ':';
#endif

  if (!registry || !env_var || env_var[0] == '\0') return PCL_ERR_INVALID;

  value = getenv(env_var);
  if (!value || value[0] == '\0') return PCL_OK;

  copy = (char*)pcl_alloc(strlen(value) + 1u);
  if (!copy) return PCL_ERR_NOMEM;
  strcpy(copy, value);

  token_start = copy;
  cursor = copy;
  while (1) {
    if (*cursor == separator || *cursor == '\0') {
      const char saved = *cursor;
      *cursor = '\0';
      status = pcl_codec_registry_load_plugins_from_paths(
          registry, (const char* const*)&token_start, 1u);
      if (status != PCL_OK) {
        // GCOVR_EXCL_START: from_paths skips bad plugins and only errors on
        // resident-list heap exhaustion, which is not injectable here.
        pcl_free(copy);
        return status;
        // GCOVR_EXCL_STOP
      }
      if (saved == '\0') break;
      token_start = cursor + 1;
    }
    ++cursor;
  }

  pcl_free(copy);
  return PCL_OK;
}

/* Implements: REQ_PCL_391, REQ_PCL_392, REQ_PCL_401. */
pcl_status_t pcl_codec_registry_load_plugins_from_manifest(
    pcl_codec_registry_t* registry,
    const char*           manifest_path) {
  FILE*  file;
  char   line[1024];
  pcl_status_t status;

  if (!registry || !manifest_path || manifest_path[0] == '\0') {
    return PCL_ERR_INVALID;
  }

  file = fopen(manifest_path, "r");
  if (!file) return PCL_ERR_NOT_FOUND;

  while (fgets(line, (int)sizeof(line), file)) {
    char*  start = line;
    size_t len;

    /* Trim leading whitespace. */
    while (*start == ' ' || *start == '\t') ++start;

    /* Trim trailing whitespace / newline. */
    len = strlen(start);
    while (len > 0u &&
           (start[len - 1u] == '\n' || start[len - 1u] == '\r' ||
            start[len - 1u] == ' ' || start[len - 1u] == '\t')) {
      start[--len] = '\0';
    }

    if (len == 0u || start[0] == '#') continue;

    status = pcl_codec_registry_load_plugins_from_paths(
        registry, (const char* const*)&start, 1u);
    if (status != PCL_OK) {
      // GCOVR_EXCL_START: from_paths skips bad plugins and only errors on
      // resident-list heap exhaustion, which is not injectable here.
      fclose(file);
      return status;
      // GCOVR_EXCL_STOP
    }
  }

  fclose(file);
  return PCL_OK;
}

/// \brief Load a transport plugin and return its transport vtable.
/* Implements: REQ_PCL_383, REQ_PCL_396, REQ_PCL_397, REQ_PCL_398. */
pcl_status_t pcl_plugin_load_transport(const char*             path,
                                       const char*             config_json,
                                       pcl_plugin_handle_t**   out_handle,
                                       const pcl_transport_t** out_vtable) {
  pcl_plugin_handle_t* handle;
  pcl_transport_abi_version_fn abi_version;
  pcl_transport_plugin_entry_fn entry;
  const pcl_transport_t* transport;
  uint32_t actual_abi_version;

  if (!path || path[0] == '\0' || !out_handle || !out_vtable) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport plugin load failed: path and output pointers are required");
    return PCL_ERR_INVALID;
  }
  *out_handle = NULL;
  *out_vtable = NULL;

  handle = open_library(path);
  if (!handle) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport plugin load failed: cannot open '%s'", path);
    return PCL_ERR_NOT_FOUND;
  }

  abi_version = (pcl_transport_abi_version_fn)resolve_symbol(
      handle, PCL_TRANSPORT_ABI_VERSION_SYMBOL);
  if (!abi_version) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport plugin load failed: '%s' does not export %s",
            path, PCL_TRANSPORT_ABI_VERSION_SYMBOL);
    close_library(handle);
    return PCL_ERR_NOT_FOUND;
  }
  actual_abi_version = abi_version();
  if (actual_abi_version != PCL_TRANSPORT_ABI_VERSION) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport plugin load failed: '%s' uses ABI version %u, required version is %u",
            path, (unsigned)actual_abi_version,
            (unsigned)PCL_TRANSPORT_ABI_VERSION);
    close_library(handle);
    return PCL_ERR_STATE;
  }

  entry = (pcl_transport_plugin_entry_fn)resolve_symbol(
      handle, PCL_TRANSPORT_PLUGIN_ENTRY_SYMBOL);
  if (!entry) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport plugin load failed: '%s' does not export %s",
            path, PCL_TRANSPORT_PLUGIN_ENTRY_SYMBOL);
    close_library(handle);
    return PCL_ERR_NOT_FOUND;
  }

  transport = entry(config_json);
  if (!transport) {
    pcl_log(NULL, PCL_LOG_ERROR,
            "transport plugin load failed: entry point in '%s' rejected its configuration",
            path);
    close_library(handle);
    return PCL_ERR_STATE;
  }

  *out_handle = handle;
  *out_vtable = transport;
  return PCL_OK;
}

/// \brief Report the interaction capabilities of a loaded transport plugin.
/* Implements: REQ_PCL_353, REQ_PCL_354, REQ_PCL_355. */
pcl_status_t pcl_plugin_transport_caps(pcl_plugin_handle_t*   handle,
                                       const char*            config_json,
                                       const pcl_transport_t* vtable,
                                       pcl_transport_caps_t*  out_caps) {
  pcl_transport_plugin_caps_fn caps_fn;

  if (!handle || !out_caps) return PCL_ERR_INVALID;

  caps_fn = (pcl_transport_plugin_caps_fn)resolve_symbol(
      handle, PCL_TRANSPORT_PLUGIN_CAPS_SYMBOL);
  if (caps_fn) {
    *out_caps = caps_fn(config_json);
  } else {
    *out_caps = pcl_transport_caps_from_vtable(vtable);
  }
  return PCL_OK;
}

/// \brief Report the QoS a loaded transport plugin offers.
///
/// Reads the optional pcl_transport_plugin_qos symbol; when absent the offered
/// QoS is PCL_QOS_RELIABILITY_UNSPECIFIED (there is nothing to derive from a
/// vtable).
/* Implements: REQ_PCL_362. */
pcl_status_t pcl_plugin_transport_qos(pcl_plugin_handle_t* handle,
                                      const char*          config_json,
                                      pcl_qos_t*           out_qos) {
  pcl_transport_plugin_qos_fn qos_fn;

  if (!handle || !out_qos) return PCL_ERR_INVALID;

  qos_fn = (pcl_transport_plugin_qos_fn)resolve_symbol(
      handle, PCL_TRANSPORT_PLUGIN_QOS_SYMBOL);
  if (qos_fn) {
    *out_qos = qos_fn(config_json);
  } else {
    out_qos->reliability = PCL_QOS_RELIABILITY_UNSPECIFIED;
  }
  return PCL_OK;
}

/// \brief Tear down a loaded transport, then unload its library, in that order.
/* Implements: REQ_PCL_381, REQ_PCL_382, REQ_PCL_407. */
pcl_status_t pcl_plugin_unload_transport(pcl_plugin_handle_t*   handle,
                                         const pcl_transport_t* vtable) {
  pcl_transport_plugin_teardown_fn teardown_fn;

  if (!handle) return PCL_ERR_INVALID;

  /* Release the instance (stop spin threads, free context) BEFORE dlclose, so a
     coupled plugin's background thread is never left running in unmapped code.
     Stateless plugins simply omit the teardown symbol. */
  teardown_fn = (pcl_transport_plugin_teardown_fn)resolve_symbol(
      handle, PCL_TRANSPORT_PLUGIN_TEARDOWN_SYMBOL);
  if (teardown_fn) {
    teardown_fn(vtable);
  }
  close_library(handle);
  return PCL_OK;
}

/// \brief Open an arbitrary shared library (no PCL entry point required).
/* Implements: REQ_PCL_402. */
pcl_plugin_handle_t* pcl_plugin_open(const char* path) {
  return open_library(path);
}

/// \brief Unload a plugin handle returned by a successful load call.
/* Implements: REQ_PCL_402. */
void pcl_plugin_unload(pcl_plugin_handle_t* handle) {
  close_library(handle);
}

/// \brief Resolve an arbitrary exported symbol from a loaded plugin.
/* Implements: REQ_PCL_383, REQ_PCL_402. */
void* pcl_plugin_symbol(pcl_plugin_handle_t* handle, const char* name) {
  return resolve_symbol(handle, name);
}
