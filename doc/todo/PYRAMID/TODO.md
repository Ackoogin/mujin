# PYRAMID — Tracked Follow-ups

The plugin binding (v1), gRPC + ROS2 coupled plugins, codec plugins (json/
flatbuffers/protobuf), the transport capability model, and native ROS2 IDL +
marshalling are **delivered**. Current capability and remaining work are tracked
in a single doc:

➡ **[`doc/plans/PYRAMID/transport_plugins.md`](../../plans/PYRAMID/transport_plugins.md)**

Headline remaining item: **put the typed ROS2 codec on the live wire**
(codec-plugin registration + typed adapter), then ROS2 actions and the deferred
capability adapters. How the plugin system works and how to use it lives in the
architecture reference:
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
and
[`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md).
