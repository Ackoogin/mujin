#if !defined(_WIN32)
#  define _POSIX_C_SOURCE 200809L
#endif

#include "pcl/pcl_process_runtime.h"

#include "pcl/pcl_codec_registry.h"
#include "pcl/pcl_plugin_loader.h"
#include "pcl/pcl_transport_routing.h"

#include <errno.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <unistd.h>
#endif

#define PCL_RUNTIME_MAX_CODECS 32u
#define PCL_RUNTIME_MAX_PORTS 128u
#define PCL_RUNTIME_MAX_PEERS 64u
#define PCL_RUNTIME_NAME_SIZE 128u
#define PCL_RUNTIME_PATH_SIZE 1024u
#define PCL_RUNTIME_CONFIG_SIZE 2048u

typedef struct {
  char mode[8];
  char peer[PCL_RUNTIME_NAME_SIZE];
  char plugin[PCL_RUNTIME_PATH_SIZE];
  char plugin_config[PCL_RUNTIME_CONFIG_SIZE];
  int configured;
} pcl_runtime_port_config_t;

typedef struct {
  char peer[PCL_RUNTIME_NAME_SIZE];
  char plugin[PCL_RUNTIME_PATH_SIZE];
  char plugin_config[PCL_RUNTIME_CONFIG_SIZE];
} pcl_runtime_peer_config_t;

struct pcl_process_runtime_t {
  pcl_executor_t* executor;
  pcl_plugin_handle_t* codec_plugins[PCL_RUNTIME_MAX_CODECS];
  size_t codec_count;
  pcl_transport_routing_t* routing;
  pcl_container_t* gateways[PCL_RUNTIME_MAX_PEERS];
  size_t gateway_count;
  uint32_t duration_seconds;
  volatile sig_atomic_t stop_requested;
  char error[PCL_RUNTIME_CONFIG_SIZE];
};

static pcl_process_runtime_t* signal_runtime = NULL;

static void runtime_signal_handler(int signal_number) {
  (void)signal_number;
  if (signal_runtime) {
    signal_runtime->stop_requested = 1;
  }
}

static pcl_status_t set_error(
    pcl_process_runtime_t* runtime,
    pcl_status_t status,
    const char* format,
    ...) {
  va_list args;
  if (runtime) {
    va_start(args, format);
    vsnprintf(runtime->error, sizeof(runtime->error), format, args);
    va_end(args);
  }
  return status;
}

static void clear_error(pcl_process_runtime_t* runtime) {
  if (runtime) runtime->error[0] = '\0';
}

static char* trim(char* text) {
  char* end;
  while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') {
    ++text;
  }
  end = text + strlen(text);
  while (end > text &&
         (end[-1] == ' ' || end[-1] == '\t' ||
          end[-1] == '\r' || end[-1] == '\n')) {
    *--end = '\0';
  }
  return text;
}

static const char* kind_name(pcl_endpoint_kind_t kind) {
  switch (kind) {
    case PCL_ENDPOINT_PUBLISHER: return "publisher";
    case PCL_ENDPOINT_SUBSCRIBER: return "subscriber";
    case PCL_ENDPOINT_PROVIDED: return "provided";
    case PCL_ENDPOINT_CONSUMED: return "consumed";
    case PCL_ENDPOINT_STREAM_PROVIDED: return "stream_provided";
    case PCL_ENDPOINT_STREAM_CONSUMED: return "stream_consumed";
    default: return NULL;
  }
}

static int find_port(
    const pcl_process_port_descriptor_t* ports,
    size_t port_count,
    const char* name) {
  size_t index;
  for (index = 0; index < port_count; ++index) {
    if (ports[index].name && strcmp(ports[index].name, name) == 0) {
      return (int)index;
    }
  }
  return -1;
}

static int find_peer(
    const pcl_runtime_peer_config_t* peers,
    size_t peer_count,
    const char* name) {
  size_t index;
  for (index = 0; index < peer_count; ++index) {
    if (strcmp(peers[index].peer, name) == 0) return (int)index;
  }
  return -1;
}

static int make_temporary_path(char* path, size_t path_size) {
#if defined(_WIN32)
  char directory[MAX_PATH];
  char filename[MAX_PATH];
  DWORD length = GetTempPathA((DWORD)sizeof(directory), directory);
  if (length == 0 || length >= sizeof(directory)) return 0;
  if (GetTempFileNameA(directory, "pcl", 0, filename) == 0) return 0;
  if (strlen(filename) + 1u > path_size) {
    DeleteFileA(filename);
    return 0;
  }
  snprintf(path, path_size, "%s", filename);
  return 1;
#else
  const char* directory = getenv("TMPDIR");
  int fd;
  if (!directory || directory[0] == '\0') directory = "/tmp";
  if (snprintf(path, path_size, "%s/pcl-routes-XXXXXX", directory) >=
      (int)path_size) {
    return 0;
  }
  fd = mkstemp(path);
  if (fd < 0) return 0;
  close(fd);
  return 1;
#endif
}

static void remove_temporary_path(const char* path) {
#if defined(_WIN32)
  DeleteFileA(path);
#else
  unlink(path);
#endif
}

static void sleep_milliseconds(uint32_t milliseconds) {
#if defined(_WIN32)
  Sleep((DWORD)milliseconds);
#else
  struct timespec request;
  request.tv_sec = (time_t)(milliseconds / 1000u);
  request.tv_nsec = (long)((milliseconds % 1000u) * 1000000u);
  while (nanosleep(&request, &request) != 0 && errno == EINTR) {
  }
#endif
}

static uint64_t monotonic_milliseconds(void) {
#if defined(_WIN32)
  return (uint64_t)GetTickCount64();
#else
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0u;
  return ((uint64_t)now.tv_sec * 1000u) +
         ((uint64_t)now.tv_nsec / 1000000u);
#endif
}

static pcl_status_t activate_gateways(
    pcl_process_runtime_t* runtime,
    const pcl_runtime_peer_config_t* peers,
    size_t peer_count) {
  size_t index;
  for (index = 0; index < peer_count; ++index) {
    pcl_container_t* gateway = NULL;
    pcl_status_t status = pcl_transport_routing_get_gateway(
        runtime->routing, peers[index].peer, &gateway);
    if (status != PCL_OK) {
      // GCOVR_EXCL_START: peers comes from the same successfully loaded
      // routing handle, so every peer lookup is guaranteed to exist.
      return set_error(runtime, status, "get gateway for peer '%s' failed",
                       peers[index].peer);
      // GCOVR_EXCL_STOP
    }
    if (!gateway) continue;
    if (runtime->gateway_count >= PCL_RUNTIME_MAX_PEERS) {
      // GCOVR_EXCL_START: peer_count is bounded to this same maximum, so the
      // count cannot be full before processing another peer.
      return set_error(runtime, PCL_ERR_NOMEM, "too many transport gateways");
      // GCOVR_EXCL_STOP
    }
    status = pcl_container_configure(gateway);
    if (status == PCL_OK) status = pcl_container_activate(gateway);
    if (status == PCL_OK) status = pcl_executor_add(runtime->executor, gateway);
    if (status != PCL_OK) {
      // GCOVR_EXCL_START: production plugin gateways are created with a valid
      // fixed port set, and the new runtime executor is empty at this point.
      return set_error(runtime, status, "activate gateway for peer '%s' failed",
                       peers[index].peer);
      // GCOVR_EXCL_STOP
    }
    runtime->gateways[runtime->gateway_count++] = gateway;
  }
  return PCL_OK;
}

pcl_status_t pcl_process_runtime_create(
    uint32_t duration_seconds,
    pcl_process_runtime_t** out_runtime) {
  pcl_process_runtime_t* runtime;
  if (!out_runtime) return PCL_ERR_INVALID;
  *out_runtime = NULL;
  runtime = (pcl_process_runtime_t*)calloc(1u, sizeof(*runtime));
  if (!runtime) return PCL_ERR_NOMEM;
  runtime->executor = pcl_executor_create();
  if (!runtime->executor) {
    free(runtime);
    return PCL_ERR_NOMEM;
  }
  runtime->duration_seconds = duration_seconds;
  *out_runtime = runtime;
  return PCL_OK;
}

pcl_executor_t* pcl_process_runtime_executor(pcl_process_runtime_t* runtime) {
  return runtime ? runtime->executor : NULL;
}

pcl_status_t pcl_process_runtime_load_codec(
    pcl_process_runtime_t* runtime,
    const char* plugin_path) {
  pcl_plugin_handle_t* handle = NULL;
  pcl_status_t status;
  if (!runtime || !plugin_path || plugin_path[0] == '\0') {
    return set_error(runtime, PCL_ERR_INVALID, "codec plugin path is empty");
  }
  if (runtime->codec_count >= PCL_RUNTIME_MAX_CODECS) {
    return set_error(runtime, PCL_ERR_NOMEM, "too many codec plugins");
  }
  clear_error(runtime);
  status = pcl_plugin_load_codec(
      plugin_path, NULL, pcl_codec_registry_default(), &handle);
  if (status != PCL_OK) {
    return set_error(runtime, status, "load codec plugin '%s' failed",
                     plugin_path);
  }
  runtime->codec_plugins[runtime->codec_count++] = handle;
  return PCL_OK;
}

pcl_status_t pcl_process_runtime_load_ports_file(
    pcl_process_runtime_t* runtime,
    const char* config_path,
    const pcl_process_port_descriptor_t* ports,
    size_t port_count) {
  FILE* config = NULL;
  FILE* manifest = NULL;
  pcl_runtime_port_config_t configured[PCL_RUNTIME_MAX_PORTS];
  pcl_runtime_peer_config_t peers[PCL_RUNTIME_MAX_PEERS];
  size_t peer_count = 0u;
  size_t configured_count = 0u;
  size_t line_number = 0u;
  char line[4096];
  char manifest_path[PCL_RUNTIME_PATH_SIZE];
  char routing_diagnostic[PCL_RUNTIME_CONFIG_SIZE] = {0};
  pcl_status_t status = PCL_OK;
  size_t index;

  if (!runtime || !config_path || !ports ||
      port_count == 0u || port_count > PCL_RUNTIME_MAX_PORTS) {
    return set_error(runtime, PCL_ERR_INVALID, "invalid ports-file arguments");
  }
  if (runtime->routing) {
    return set_error(runtime, PCL_ERR_STATE, "ports file is already loaded");
  }
  memset(configured, 0, sizeof(configured));
  memset(peers, 0, sizeof(peers));
  for (index = 0; index < port_count; ++index) {
    size_t other;
    size_t endpoint_index;
    if (!ports[index].name || ports[index].name[0] == '\0') {
      return set_error(runtime, PCL_ERR_INVALID,
                       "deployment port name is empty");
    }
    for (other = 0; other < index; ++other) {
      if (strcmp(ports[other].name, ports[index].name) == 0) {
        return set_error(runtime, PCL_ERR_INVALID,
                          "duplicate deployment port '%s'", ports[index].name);
      }
    }
    if ((ports[index].rpc_endpoint_count > 0u &&
         !ports[index].rpc_endpoints) ||
        (ports[index].pubsub_endpoint_count > 0u &&
         !ports[index].pubsub_endpoints)) {
      return set_error(runtime, PCL_ERR_INVALID,
                       "deployment port '%s' has a null endpoint array",
                       ports[index].name);
    }
    for (endpoint_index = 0u;
         endpoint_index < ports[index].rpc_endpoint_count;
         ++endpoint_index) {
      const pcl_process_endpoint_descriptor_t* endpoint =
          &ports[index].rpc_endpoints[endpoint_index];
      if (!endpoint->name || !kind_name(endpoint->kind)) {
        return set_error(runtime, PCL_ERR_INVALID,
                         "unsupported endpoint for port '%s'",
                         ports[index].name);
      }
    }
    for (endpoint_index = 0u;
         endpoint_index < ports[index].pubsub_endpoint_count;
         ++endpoint_index) {
      const pcl_process_endpoint_descriptor_t* endpoint =
          &ports[index].pubsub_endpoints[endpoint_index];
      if (!endpoint->name || !kind_name(endpoint->kind)) {
        return set_error(runtime, PCL_ERR_INVALID,
                         "unsupported endpoint for port '%s'",
                         ports[index].name);
      }
    }
  }

  clear_error(runtime);
  config = fopen(config_path, "r");
  if (!config) {
    return set_error(runtime, PCL_ERR_NOT_FOUND,
                     "cannot open port config '%s'", config_path);
  }
  while (fgets(line, sizeof(line), config)) {
    char* cursor;
    char directive[16];
    char name[PCL_RUNTIME_NAME_SIZE];
    char mode[8];
    char peer[PCL_RUNTIME_NAME_SIZE];
    char plugin[PCL_RUNTIME_PATH_SIZE];
    int offset = 0;
    int port_index;
    char* plugin_config;
    int peer_index;
    ++line_number;
    cursor = trim(line);
    if (cursor[0] == '\0' || cursor[0] == '#') continue;
    if (sscanf(cursor, "%15s %127s %7s %127s %1023s %n",
               directive, name, mode, peer, plugin, &offset) != 5) {
      status = set_error(runtime, PCL_ERR_INVALID,
                         "invalid port config line %lu",
                         (unsigned long)line_number);
      goto cleanup;
    }
    plugin_config = trim(cursor + offset);
    if (strcmp(directive, "port") != 0 ||
        (strcmp(mode, "rpc") != 0 && strcmp(mode, "pubsub") != 0) ||
        plugin_config[0] == '\0') {
      status = set_error(runtime, PCL_ERR_INVALID,
                         "invalid port config line %lu",
                         (unsigned long)line_number);
      goto cleanup;
    }
    port_index = find_port(ports, port_count, name);
    if (port_index < 0) {
      status = set_error(runtime, PCL_ERR_INVALID,
                         "unknown port '%s' on line %lu", name,
                         (unsigned long)line_number);
      goto cleanup;
    }
    if (configured[port_index].configured) {
      status = set_error(runtime, PCL_ERR_INVALID,
                         "duplicate port '%s'", name);
      goto cleanup;
    }
    snprintf(configured[port_index].mode,
             sizeof(configured[port_index].mode), "%s", mode);
    snprintf(configured[port_index].peer,
             sizeof(configured[port_index].peer), "%s", peer);
    snprintf(configured[port_index].plugin,
             sizeof(configured[port_index].plugin), "%s", plugin);
    snprintf(configured[port_index].plugin_config,
             sizeof(configured[port_index].plugin_config), "%s", plugin_config);
    configured[port_index].configured = 1;
    ++configured_count;

    peer_index = find_peer(peers, peer_count, peer);
    if (peer_index >= 0) {
      if (strcmp(peers[peer_index].plugin, plugin) != 0 ||
          strcmp(peers[peer_index].plugin_config, plugin_config) != 0) {
        status = set_error(runtime, PCL_ERR_INVALID,
                           "peer '%s' has conflicting plugin configurations",
                           peer);
        goto cleanup;
      }
    } else {
      if (peer_count >= PCL_RUNTIME_MAX_PEERS) {
        status = set_error(runtime, PCL_ERR_NOMEM,
                           "too many configured peers");
        goto cleanup;
      }
      snprintf(peers[peer_count].peer, sizeof(peers[peer_count].peer),
               "%s", peer);
      snprintf(peers[peer_count].plugin, sizeof(peers[peer_count].plugin),
               "%s", plugin);
      snprintf(peers[peer_count].plugin_config,
               sizeof(peers[peer_count].plugin_config), "%s", plugin_config);
      ++peer_count;
    }
  }
  if (ferror(config)) {
    status = set_error(runtime, PCL_ERR_INVALID,
                       "failed reading port config '%s'", config_path);
    goto cleanup;
  }
  if (configured_count != port_count) {
    status = set_error(runtime, PCL_ERR_INVALID,
                       "port config must contain each component port");
    goto cleanup;
  }
  if (!make_temporary_path(manifest_path, sizeof(manifest_path))) {
    status = set_error(runtime, PCL_ERR_INVALID,
                       "cannot create temporary routing manifest");
    goto cleanup;
  }
  manifest = fopen(manifest_path, "w");
  if (!manifest) {
    // GCOVR_EXCL_START: make_temporary_path just created this same file for
    // the process; failure to reopen it requires an external OS/filesystem
    // fault between the two calls.
    remove_temporary_path(manifest_path);
    status = set_error(runtime, PCL_ERR_INVALID,
                       "cannot write temporary routing manifest");
    goto cleanup;
    // GCOVR_EXCL_STOP
  }
  fprintf(manifest, "# Generated from per-port configuration.\n");
  for (index = 0; index < peer_count; ++index) {
    fprintf(manifest, "transport %s %s %s\n",
            peers[index].peer, peers[index].plugin,
            peers[index].plugin_config);
  }
  for (index = 0; index < port_count; ++index) {
    const pcl_process_port_descriptor_t* definition = &ports[index];
    const pcl_process_endpoint_descriptor_t* selected;
    size_t selected_count;
    size_t endpoint_index;
    fprintf(manifest, "exclusive %s ", definition->name);
    for (endpoint_index = 0;
         endpoint_index < definition->rpc_endpoint_count; ++endpoint_index) {
      if (endpoint_index > 0u) fputc(',', manifest);
      fprintf(manifest, "%s", definition->rpc_endpoints[endpoint_index].name);
    }
    fputc(' ', manifest);
    for (endpoint_index = 0;
         endpoint_index < definition->pubsub_endpoint_count; ++endpoint_index) {
      if (endpoint_index > 0u) fputc(',', manifest);
      fprintf(manifest, "%s",
              definition->pubsub_endpoints[endpoint_index].name);
    }
    fputc('\n', manifest);
    if (strcmp(configured[index].mode, "rpc") == 0) {
      selected = definition->rpc_endpoints;
      selected_count = definition->rpc_endpoint_count;
    } else {
      selected = definition->pubsub_endpoints;
      selected_count = definition->pubsub_endpoint_count;
    }
    for (endpoint_index = 0; endpoint_index < selected_count;
         ++endpoint_index) {
      const char* endpoint_kind = kind_name(selected[endpoint_index].kind);
      fprintf(manifest, "route %s %s %s reliable\n",
              selected[endpoint_index].name, endpoint_kind,
              configured[index].peer);
    }
  }
  if (fclose(manifest) != 0) {
    // GCOVR_EXCL_START: a close failure for this small local temporary file
    // requires an external OS/filesystem fault and cannot be injected here.
    manifest = NULL;
    remove_temporary_path(manifest_path);
    status = set_error(runtime, PCL_ERR_INVALID,
                       "cannot finish temporary routing manifest");
    goto cleanup;
    // GCOVR_EXCL_STOP
  }
  manifest = NULL;
  status = pcl_transport_routing_load(
      runtime->executor, manifest_path, &runtime->routing,
      routing_diagnostic, sizeof(routing_diagnostic));
  remove_temporary_path(manifest_path);
  if (status != PCL_OK) {
    status = set_error(runtime, status, "load port config '%s' failed: %s",
                       config_path, routing_diagnostic);
    goto cleanup;
  }
  status = activate_gateways(runtime, peers, peer_count);

cleanup:
  if (manifest) fclose(manifest);
  if (config) fclose(config);
  return status;
}

pcl_status_t pcl_process_runtime_run(
    pcl_process_runtime_t* runtime,
    pcl_container_t* component) {
  pcl_status_t status;
  uint64_t deadline = 0u;
  void (*old_sigint)(int);
  void (*old_sigterm)(int);
  int added = 0;
  int active = 0;
  int configured = 0;
  if (!runtime || !component) {
    return set_error(runtime, PCL_ERR_INVALID, "invalid run arguments");
  }
  clear_error(runtime);
  runtime->stop_requested = 0;
  status = pcl_container_configure(component);
  if (status != PCL_OK) {
    return set_error(runtime, status, "configure component '%s' failed",
                     pcl_container_name(component));
  }
  configured = 1;
  status = pcl_container_activate(component);
  if (status != PCL_OK) goto finish;
  active = 1;
  status = pcl_executor_add(runtime->executor, component);
  if (status != PCL_OK) goto finish;
  added = 1;

  signal_runtime = runtime;
  old_sigint = signal(SIGINT, runtime_signal_handler);
  old_sigterm = signal(SIGTERM, runtime_signal_handler);
  if (runtime->duration_seconds > 0u) {
    deadline = monotonic_milliseconds() +
               ((uint64_t)runtime->duration_seconds * 1000u);
  }
  while (!runtime->stop_requested &&
         (deadline == 0u || monotonic_milliseconds() < deadline)) {
    status = pcl_executor_spin_once(runtime->executor, 20u);
    if (status != PCL_OK) break;
    sleep_milliseconds(5u);
  }
  signal(SIGINT, old_sigint);
  signal(SIGTERM, old_sigterm);
  signal_runtime = NULL;

finish:
  if (added) {
    pcl_status_t cleanup_status =
        pcl_executor_remove(runtime->executor, component);
    if (status == PCL_OK) status = cleanup_status;
  }
  if (active) {
    pcl_status_t cleanup_status = pcl_container_deactivate(component);
    if (status == PCL_OK) status = cleanup_status;
  }
  if (configured) {
    pcl_status_t cleanup_status = pcl_container_cleanup(component);
    if (status == PCL_OK) status = cleanup_status;
  }
  if (status != PCL_OK) {
    return set_error(runtime, status, "component '%s' run failed",
                     pcl_container_name(component));
  }
  return PCL_OK;
}

void pcl_process_runtime_request_shutdown(pcl_process_runtime_t* runtime) {
  if (!runtime) return;
  runtime->stop_requested = 1;
  pcl_executor_request_shutdown(runtime->executor);
}

const char* pcl_process_runtime_error(const pcl_process_runtime_t* runtime) {
  return runtime ? runtime->error : "process runtime is null";
}

void pcl_process_runtime_destroy(pcl_process_runtime_t* runtime) {
  size_t index;
  if (!runtime) return;
  if (signal_runtime == runtime) signal_runtime = NULL;
  for (index = runtime->gateway_count; index > 0u; --index) {
    pcl_container_t* gateway = runtime->gateways[index - 1u];
    pcl_executor_remove(runtime->executor, gateway);
    pcl_container_deactivate(gateway);
    pcl_container_cleanup(gateway);
  }
  pcl_transport_routing_destroy(runtime->routing);
  pcl_executor_destroy(runtime->executor);
  free(runtime);
}
