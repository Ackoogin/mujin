# ROS2 Transport Semantics

This page defines the canonical ROS2 mapping for PYRAMID/PCL transport
semantics.

It is the design reference for the generated `ros2` backend and the shared
support layer under
[examples/ros2/cpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ros2/cpp).

## Purpose

ROS2 does not match the full PCL surface one-to-one:

- PCL has opaque pub/sub messages with `content_type`
- PCL has unary service calls
- PCL has server-streaming service calls
- PCL requires business logic to run only on the executor thread

The ROS2 backend therefore needs both:

- straight generated bindings from canonical `.proto`
- an explicit semantic mapping for the parts that are transport-specific

## Canonical Naming

PCL names are normalized into lowercase ROS2 path segments.

Examples:

- PCL topic `standard.object_evidence`
  -> ROS2 topic `/pyramid/topic/standard/object_evidence`
- PCL unary service `object_of_interest.create_requirement`
  -> ROS2 service `/pyramid/service/object_of_interest/create_requirement`
- PCL streaming service `matching_objects.read_match`
  -> ROS2 open service `/pyramid/stream/matching_objects/read_match/open`
  -> ROS2 frame topic `/pyramid/stream/matching_objects/read_match/frames`
  -> ROS2 cancel topic `/pyramid/stream/matching_objects/read_match/cancel`

## Envelope Model

The shared support layer carries PCL payloads inside a transport envelope with:

- `content_type`
- `correlation_id`
- `payload`
- `end_of_stream`
- `status`

This keeps codec selection independent from ROS2 transport selection. The ROS2
transport carries JSON, FlatBuffers, or Protobuf bytes without changing the
handler surface.

## Streaming Rule

ROS2 has no direct equivalent of PCL/gRPC server-streaming unary RPCs.

The canonical mapping is:

1. client calls the stream `open` service with the request envelope
2. server emits zero or more frame envelopes on the mapped `frames` topic
3. all frame envelopes carry the same `correlation_id`
4. the final envelope sets `end_of_stream = true`
5. clients may publish cancellation on the mapped `cancel` topic

The current standalone proof covers the `open` plus `frames` path. Explicit
cancel wiring is reserved for the first real ROS2 runtime binding.

## Threading Rule

ROS2 callback threads must never call PCL business logic directly.

Required handoff:

- inbound topic traffic posts through `pcl_executor_post_incoming(...)`
- inbound unary/stream-open service traffic posts through
  `pcl_executor_post_service_request(...)`
- outbound async responses are delivered back onto the executor thread before
  any business-logic callback runs

The generated ROS2 support layer and standalone tests enforce this rule.

## Current Scope

Implemented today:

- generated Tactical Objects ROS2 transport projection
- canonical ROS2 name mapping for topics, unary services, and streaming
  services
- generic ROS2 envelope package `pyramid_ros2_transport` with:
  - `msg/PclEnvelope`
  - `srv/PclService`
  - `srv/PclOpenStream`
- direct `rclcpp` runtime adapter in
  `subprojects/PYRAMID/ros2/include/pyramid_ros2_transport/rclcpp_runtime_adapter.hpp`
- standalone fake-adapter proof for pub/sub, unary service, and streaming
  service ingress
- standalone `rclcpp` runtime proof for pub/sub, unary service, streaming
  service, and outbound publish
- executor-thread assertion for the business-logic path

Not yet implemented:

- ROS2 action mapping
- Ada ROS2 runtime beyond generated constants/specs
- top-level plain-CMake integration for the ament package build

## AME Note

AME is the clearest in-repo ROS2 deployment example, but it does not yet expose
its external interface through canonical PYRAMID `.proto` contracts.

That means the new ROS2 semantic mapping can inform the AME Phase 6 transport
adapter immediately, but AME cannot yet consume the generated PYRAMID ROS2
bindings directly until its contract is canonicalized.
