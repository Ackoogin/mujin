#pragma once

#include "mujin/bt_logger.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mujin {

/// Foxglove WebSocket bridge for live BT and WM monitoring.
///
/// Implements the Foxglove WebSocket protocol, allowing Foxglove Studio
/// to connect and visualize BT state transitions and WM fact changes
/// in real-time.
///
/// Protocol reference: https://github.com/foxglove/ws-protocol
///
/// Usage:
///   FoxgloveBridge bridge({.port = 8765});
///   bridge.start();
///   // ... attach as sinks to MujinBTLogger / WmAuditLog ...
///   bt_logger.addCallbackSink(bridge.btEventSink());
///   bridge.stop();
class FoxgloveBridge {
public:
    struct Options {
        uint16_t port = 8765;
        std::string server_name = "mujin";
    };

    /// Construct with default options.
    FoxgloveBridge();

    /// Construct with custom options.
    explicit FoxgloveBridge(const Options& opts);
    ~FoxgloveBridge();

    /// Start the WebSocket server in a background thread.
    void start();

    /// Stop the server and join the background thread.
    void stop();

    /// Whether the server is currently running.
    bool running() const { return running_.load(); }

    /// Returns a callback suitable for MujinBTLogger::addCallbackSink().
    MujinBTLogger::SinkCallback btEventSink();

    /// Returns a callback suitable for WorldModel audit log forwarding.
    /// The caller should format the WM event as a JSON string before calling.
    std::function<void(const std::string&)> wmEventSink();

    /// Publish a raw JSON message on a channel.
    /// @param channel_id  1 = bt_events, 2 = wm_audit
    void publish(uint32_t channel_id, const std::string& json_data);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::atomic<bool> running_{false};
};

} // namespace mujin
