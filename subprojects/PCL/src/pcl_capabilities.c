/// \file pcl_capabilities.c
/// \brief Derivation of transport capabilities from a vtable.
#include "pcl/pcl_capabilities.h"

pcl_transport_caps_t pcl_transport_caps_from_vtable(
    const pcl_transport_t* transport) {
  pcl_transport_caps_t caps = PCL_CAP_NONE;

  if (!transport) return PCL_CAP_NONE;

  if (transport->publish || transport->subscribe) {
    caps |= PCL_CAP_PUBSUB;
  }
  if (transport->serve || transport->invoke_async) {
    caps |= PCL_CAP_RPC_UNARY;
  }
  if (transport->invoke_stream || transport->stream_send) {
    caps |= PCL_CAP_RPC_STREAM;
  }
  /* PCL_CAP_RPC_ACTION has no vtable slot; it is never derived. */

  return caps;
}
