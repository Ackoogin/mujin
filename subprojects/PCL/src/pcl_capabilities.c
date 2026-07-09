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

pcl_transport_caps_t pcl_endpoint_required_caps(pcl_endpoint_kind_t kind) {
  switch (kind) {
    case PCL_ENDPOINT_PUBLISHER:
    case PCL_ENDPOINT_SUBSCRIBER:
      return PCL_CAP_PUBSUB;
    case PCL_ENDPOINT_PROVIDED:
    case PCL_ENDPOINT_CONSUMED:
      return PCL_CAP_RPC_UNARY;
    case PCL_ENDPOINT_STREAM_PROVIDED:
    case PCL_ENDPOINT_STREAM_CONSUMED:
      return PCL_CAP_RPC_STREAM;
    default:
      return PCL_CAP_NONE;
  }
}

int pcl_transport_caps_supports(pcl_transport_caps_t have,
                                pcl_transport_caps_t required) {
  return (have & required) == required;
}

int pcl_qos_satisfies(pcl_qos_t offered, pcl_qos_t floor) {
  /* Reliability is ordered, so "meets the floor" is offered >= floor. Extend
     this conjunction as further QoS dimensions are added. */
  return (int)offered.reliability >= (int)floor.reliability;
}

const char* pcl_qos_reliability_name(pcl_qos_reliability_t r) {
  switch (r) {
    case PCL_QOS_RELIABILITY_BEST_EFFORT: return "best_effort";
    case PCL_QOS_RELIABILITY_RELIABLE:    return "reliable";
    case PCL_QOS_RELIABILITY_UNSPECIFIED:
    default:                              return "unspecified";
  }
}
