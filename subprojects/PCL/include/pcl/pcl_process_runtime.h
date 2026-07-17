/// \file pcl_process_runtime.h
/// \brief Process-level PCL lifecycle, codec, and deployment routing runtime.
#ifndef PCL_PROCESS_RUNTIME_H
#define PCL_PROCESS_RUNTIME_H

#include "pcl_container.h"
#include "pcl_executor.h"
#include "pcl_types.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Opaque owner of one process executor and its loaded routes.
typedef struct pcl_process_runtime_t pcl_process_runtime_t;

/// \brief One generated wire endpoint in a logical port realization.
typedef struct {
  const char* name;
  pcl_endpoint_kind_t kind;
} pcl_process_endpoint_descriptor_t;

/// \brief RPC and pub/sub endpoint alternatives for one logical port.
typedef struct {
  const char* name;
  const pcl_process_endpoint_descriptor_t* rpc_endpoints;
  size_t rpc_endpoint_count;
  const pcl_process_endpoint_descriptor_t* pubsub_endpoints;
  size_t pubsub_endpoint_count;
} pcl_process_port_descriptor_t;

/// \brief Create a process runtime.
///
/// A duration of zero runs until SIGINT, SIGTERM, or an explicit shutdown
/// request. A positive duration is intended for tests and demonstrations.
pcl_status_t pcl_process_runtime_create(
    uint32_t duration_seconds,
    pcl_process_runtime_t** out_runtime);

/// \brief Return the runtime-owned executor.
pcl_executor_t* pcl_process_runtime_executor(pcl_process_runtime_t* runtime);

/// \brief Load one codec plugin into the process-wide codec registry.
pcl_status_t pcl_process_runtime_load_codec(
    pcl_process_runtime_t* runtime,
    const char* plugin_path);

/// \brief Load a `.ports` file and install its generated endpoint routes.
pcl_status_t pcl_process_runtime_load_ports_file(
    pcl_process_runtime_t* runtime,
    const char* config_path,
    const pcl_process_port_descriptor_t* ports,
    size_t port_count);

/// \brief Run one component through configure, activate, spin, and cleanup.
pcl_status_t pcl_process_runtime_run(
    pcl_process_runtime_t* runtime,
    pcl_container_t* component);

/// \brief Ask a running zero-duration runtime to stop.
void pcl_process_runtime_request_shutdown(pcl_process_runtime_t* runtime);

/// \brief Last diagnostic recorded by an operation on this runtime.
const char* pcl_process_runtime_error(const pcl_process_runtime_t* runtime);

/// \brief Destroy the runtime, gateways, routes, and executor.
void pcl_process_runtime_destroy(pcl_process_runtime_t* runtime);

#ifdef __cplusplus
}
#endif

#endif
