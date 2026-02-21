#include "mujin/foxglove_bridge.h"

#define ASIO_STANDALONE
#define _WEBSOCKETPP_CPP11_STL_

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>

namespace mujin {

// =========================================================================
// Foxglove WebSocket protocol constants
// =========================================================================

// Server-to-client binary opcodes
static constexpr uint8_t OP_SERVER_INFO    = 0x01;  // not binary, but JSON text
static constexpr uint8_t OP_MESSAGE_DATA   = 0x01;  // binary opcode for data

// JSON Schema for BT events channel
static const char* BT_EVENT_SCHEMA = R"({
  "type": "object",
  "properties": {
    "ts_us":       {"type": "integer"},
    "node":        {"type": "string"},
    "type":        {"type": "string"},
    "prev":        {"type": "string"},
    "status":      {"type": "string"},
    "tree_id":     {"type": "string"},
    "wm_version":  {"type": "integer"}
  }
})";

// JSON Schema for WM audit channel
static const char* WM_AUDIT_SCHEMA = R"({
  "type": "object",
  "properties": {
    "wm_version":  {"type": "integer"},
    "ts_us":       {"type": "integer"},
    "fact":        {"type": "string"},
    "value":       {"type": "boolean"},
    "source":      {"type": "string"}
  }
})";

// =========================================================================
// Helpers
// =========================================================================

static std::string jsonEscapeStr(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 4);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;      break;
        }
    }
    return out;
}

// =========================================================================
// Implementation via websocketpp
// =========================================================================

using WsServer = websocketpp::server<websocketpp::config::asio>;
using ConnectionHdl = websocketpp::connection_hdl;

struct Subscription {
    uint32_t sub_id;
    uint32_t channel_id;
};

struct FoxgloveBridge::Impl {
    Options opts;
    WsServer server;
    std::thread server_thread;

    std::mutex mutex;
    // Map from connection handle to its subscriptions
    std::set<ConnectionHdl, std::owner_less<ConnectionHdl>> connections;
    // Global subscription list: sub_id -> channel_id
    std::map<ConnectionHdl, std::vector<Subscription>, std::owner_less<ConnectionHdl>> subscriptions;
    uint32_t next_sub_id = 1;

    void onOpen(ConnectionHdl hdl) {
        std::lock_guard<std::mutex> lk(mutex);
        connections.insert(hdl);

        // Send serverInfo (text frame)
        std::ostringstream info;
        info << "{"
             << "\"op\":\"serverInfo\","
             << "\"name\":\"" << jsonEscapeStr(opts.server_name) << "\","
             << "\"capabilities\":[],"
             << "\"supportedEncodings\":[\"json\"],"
             << "\"metadata\":{}"
             << "}";
        try {
            server.send(hdl, info.str(), websocketpp::frame::opcode::text);
        } catch (...) {}

        // Advertise channels (text frame)
        std::ostringstream adv;
        adv << "{"
            << "\"op\":\"advertise\","
            << "\"channels\":["
            << "{"
            << "\"id\":1,"
            << "\"topic\":\"/bt_events\","
            << "\"encoding\":\"json\","
            << "\"schemaName\":\"mujin.BTEvent\","
            << "\"schema\":\"" << jsonEscapeStr(BT_EVENT_SCHEMA) << "\""
            << "},"
            << "{"
            << "\"id\":2,"
            << "\"topic\":\"/wm_audit\","
            << "\"encoding\":\"json\","
            << "\"schemaName\":\"mujin.WMFactChange\","
            << "\"schema\":\"" << jsonEscapeStr(WM_AUDIT_SCHEMA) << "\""
            << "}"
            << "]"
            << "}";
        try {
            server.send(hdl, adv.str(), websocketpp::frame::opcode::text);
        } catch (...) {}
    }

    void onClose(ConnectionHdl hdl) {
        std::lock_guard<std::mutex> lk(mutex);
        connections.erase(hdl);
        subscriptions.erase(hdl);
    }

    void onMessage(ConnectionHdl hdl, WsServer::message_ptr msg) {
        // Parse client text messages (subscribe / unsubscribe)
        if (msg->get_opcode() != websocketpp::frame::opcode::text) return;

        auto payload = msg->get_payload();

        // Simple JSON parsing for subscribe/unsubscribe
        if (payload.find("\"subscribe\"") != std::string::npos) {
            // Parse subscriptions: look for channelId values
            // Format: {"op":"subscribe","subscriptions":[{"id":N,"channelId":M},...]}
            std::lock_guard<std::mutex> lk(mutex);
            auto& subs = subscriptions[hdl];

            // Find all channelId values
            size_t pos = 0;
            while ((pos = payload.find("\"channelId\":", pos)) != std::string::npos) {
                pos += 12; // skip past "channelId":
                uint32_t ch_id = static_cast<uint32_t>(std::stoul(payload.substr(pos)));

                // Find corresponding "id" (subscription id from client)
                size_t id_pos = payload.rfind("\"id\":", pos);
                uint32_t client_sub_id = 0;
                if (id_pos != std::string::npos) {
                    client_sub_id = static_cast<uint32_t>(std::stoul(payload.substr(id_pos + 5)));
                }

                subs.push_back({client_sub_id, ch_id});
            }
        } else if (payload.find("\"unsubscribe\"") != std::string::npos) {
            // Parse unsubscriptions
            std::lock_guard<std::mutex> lk(mutex);
            auto& subs = subscriptions[hdl];

            size_t pos = 0;
            while ((pos = payload.find("\"id\":", pos)) != std::string::npos) {
                pos += 5;
                uint32_t sub_id = static_cast<uint32_t>(std::stoul(payload.substr(pos)));
                subs.erase(
                    std::remove_if(subs.begin(), subs.end(),
                        [sub_id](const Subscription& s) { return s.sub_id == sub_id; }),
                    subs.end());
            }
        }
    }

    void broadcast(uint32_t channel_id, const std::string& json_data) {
        // Get current timestamp in nanoseconds
        auto now_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());

        std::lock_guard<std::mutex> lk(mutex);
        for (auto& [hdl, subs] : subscriptions) {
            for (auto& sub : subs) {
                if (sub.channel_id == channel_id) {
                    // Build Foxglove MessageData binary frame:
                    // opcode(1) + subscriptionId(4) + timestamp(8) + payload
                    size_t payload_size = json_data.size();
                    std::string frame(1 + 4 + 8 + payload_size, '\0');
                    frame[0] = OP_MESSAGE_DATA;
                    std::memcpy(&frame[1], &sub.sub_id, 4);
                    std::memcpy(&frame[5], &now_ns, 8);
                    std::memcpy(&frame[13], json_data.data(), payload_size);

                    try {
                        server.send(hdl, frame, websocketpp::frame::opcode::binary);
                    } catch (...) {}
                    break; // only one subscription per channel per connection
                }
            }
        }
    }
};

// =========================================================================
// Public API
// =========================================================================

FoxgloveBridge::FoxgloveBridge()
    : FoxgloveBridge(Options{})
{}

FoxgloveBridge::FoxgloveBridge(const Options& opts)
    : impl_(std::make_unique<Impl>())
{
    impl_->opts = opts;

    impl_->server.set_access_channels(websocketpp::log::alevel::none);
    impl_->server.set_error_channels(websocketpp::log::elevel::none);
    impl_->server.init_asio();
    impl_->server.set_reuse_addr(true);

    impl_->server.set_open_handler(
        [this](ConnectionHdl hdl) { impl_->onOpen(hdl); });
    impl_->server.set_close_handler(
        [this](ConnectionHdl hdl) { impl_->onClose(hdl); });
    impl_->server.set_message_handler(
        [this](ConnectionHdl hdl, WsServer::message_ptr msg) {
            impl_->onMessage(hdl, msg);
        });
}

FoxgloveBridge::~FoxgloveBridge() {
    stop();
}

void FoxgloveBridge::start() {
    if (running_.load()) return;

    impl_->server.listen(impl_->opts.port);
    impl_->server.start_accept();
    running_.store(true);

    impl_->server_thread = std::thread([this]() {
        impl_->server.run();
    });

    std::cout << "  [Foxglove] WebSocket server listening on ws://localhost:"
              << impl_->opts.port << "\n";
}

void FoxgloveBridge::stop() {
    if (!running_.load()) return;
    running_.store(false);

    impl_->server.stop_listening();

    // Close all connections
    {
        std::lock_guard<std::mutex> lk(impl_->mutex);
        for (auto& hdl : impl_->connections) {
            try {
                impl_->server.close(hdl, websocketpp::close::status::going_away, "shutdown");
            } catch (...) {}
        }
    }

    impl_->server.stop();
    if (impl_->server_thread.joinable()) {
        impl_->server_thread.join();
    }
}

void FoxgloveBridge::publish(uint32_t channel_id, const std::string& json_data) {
    impl_->broadcast(channel_id, json_data);
}

MujinBTLogger::SinkCallback FoxgloveBridge::btEventSink() {
    return [this](const std::string& json_line) {
        publish(1, json_line);  // channel 1 = /bt_events
    };
}

std::function<void(const std::string&)> FoxgloveBridge::wmEventSink() {
    return [this](const std::string& json_line) {
        publish(2, json_line);  // channel 2 = /wm_audit
    };
}

} // namespace mujin
