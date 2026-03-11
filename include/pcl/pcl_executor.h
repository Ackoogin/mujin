/// \file pcl_executor.h
/// \brief PYRAMID Container Library executor API.
///
/// The executor runs one or more containers on a single thread.
/// It drives the tick loop, dispatches incoming messages to subscriber
/// callbacks, and routes service requests to handlers — all sequentially.
#ifndef PCL_EXECUTOR_H
#define PCL_EXECUTOR_H

#include "pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Create / destroy ────────────────────────────────────────────────────

/// \brief Create a new executor.
pcl_executor_t* pcl_executor_create(void);

/// \brief Destroy the executor and free resources.
///
/// Does NOT destroy managed containers — caller retains ownership.
void pcl_executor_destroy(pcl_executor_t* e);

// ── Container management ────────────────────────────────────────────────

/// \brief Add a container to the executor.
///
/// Must be called before spin().  The executor does NOT take ownership —
/// caller is responsible for destroying containers after the executor
/// is destroyed.
pcl_status_t pcl_executor_add(pcl_executor_t* e, pcl_container_t* c);

// ── Spin ────────────────────────────────────────────────────────────────

/// \brief Block and run the tick loop for all managed containers.
///
/// Returns PCL_OK when shutdown is requested.
pcl_status_t pcl_executor_spin(pcl_executor_t* e);

/// \brief Process one round of pending work then return.
///
/// Dispatches incoming messages and runs one tick per eligible container.
/// \param timeout_ms  Maximum time to wait for pending work (0 = no wait).
pcl_status_t pcl_executor_spin_once(pcl_executor_t* e, uint32_t timeout_ms);

// ── Shutdown ────────────────────────────────────────────────────────────

/// \brief Request the executor to stop spinning.
///
/// Thread-safe and async-signal-safe (uses atomic store).
/// Does NOT call lifecycle transitions — containers remain in current state.
void pcl_executor_request_shutdown(pcl_executor_t* e);

/// \brief Graceful shutdown with timeout.
///
/// For each ACTIVE container, calls deactivate then shutdown.
/// If any transition exceeds timeout_ms, forces termination.
/// \param timeout_ms  Maximum wall-clock time for the entire shutdown
///                    sequence (0 = no timeout).
/// \return PCL_OK on clean shutdown, PCL_ERR_TIMEOUT if deadline exceeded.
pcl_status_t pcl_executor_shutdown_graceful(pcl_executor_t* e,
                                            uint32_t        timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // PCL_EXECUTOR_H
