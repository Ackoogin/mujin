/// \file pcl_transport_apos.h
/// \brief APOS Local Virtual Channel transport adapter for PCL.
#ifndef PCL_TRANSPORT_APOS_H
#define PCL_TRANSPORT_APOS_H

#include "pcl/pcl_transport.h"
#include "pcl/pcl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Opaque APOS transport handle.
typedef struct pcl_apos_transport_t pcl_apos_transport_t;

/// \brief Create a PCL transport over APOS Local Virtual Channels.
///
/// The transport sends framed PCL messages as raw APOS payloads to
/// \p send_lvc and receives raw payloads from \p recv_lvc.  Pair two
/// transports by crossing their channels: A(send=2, recv=1) and
/// B(send=1, recv=2).
///
/// \param process_id  Unique APOS process identifier for setupAPOS().
/// \param send_lvc    Local Virtual Channel used for outbound frames.
/// \param recv_lvc    Local Virtual Channel used for inbound frames.
/// \param executor    Executor receiving inbound PCL frames.
/// \return Handle on success, NULL on failure.
pcl_apos_transport_t* pcl_apos_transport_create(unsigned int    process_id,
                                                int             send_lvc,
                                                int             recv_lvc,
                                                pcl_executor_t* executor);

/// \brief Set the logical peer identifier used for endpoint routing.
pcl_status_t pcl_apos_transport_set_peer_id(pcl_apos_transport_t* ctx,
                                            const char*           peer_id);

/// \brief Get the transport vtable for pcl_executor_set_transport().
const pcl_transport_t* pcl_apos_transport_get_transport(
    pcl_apos_transport_t* ctx);

/// \brief Destroy the APOS transport and release resources.
void pcl_apos_transport_destroy(pcl_apos_transport_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // PCL_TRANSPORT_APOS_H
