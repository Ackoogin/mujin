#include "owp_client.hpp"
#include "owp_frame.hpp"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace {
using namespace std::chrono_literals;
using Server = websocketpp::server<websocketpp::config::asio>;

class MockServer {
 public:
  enum class Mode { Happy, Silent, Error, Reject };
  explicit MockServer(Mode mode) : mode_(mode) {
    server_.clear_access_channels(websocketpp::log::alevel::all);
    server_.clear_error_channels(websocketpp::log::elevel::all);
    server_.init_asio();
    server_.set_validate_handler([this](websocketpp::connection_hdl hdl) {
      auto con = server_.get_con_from_hdl(hdl);
      if (mode_ == Mode::Reject || con->get_requested_subprotocols().empty()) return false;
      con->select_subprotocol("owp"); return true;
    });
    server_.set_open_handler([this](websocketpp::connection_hdl hdl) { std::lock_guard<std::mutex> lock(mutex_); hdl_ = hdl; open_ = true; cv_.notify_all(); });
    server_.set_message_handler([this](websocketpp::connection_hdl hdl, Server::message_ptr msg) { onMessage(hdl, msg->get_payload()); });
    server_.listen(websocketpp::lib::asio::ip::tcp::v4(), 0); server_.start_accept(); websocketpp::lib::error_code ec;
    port_ = server_.get_local_endpoint(ec).port();
    if (ec) throw std::runtime_error(ec.message());
    worker_ = std::thread([this] { server_.run(); });
  }
  ~MockServer() { server_.stop_listening(); server_.stop(); if (worker_.joinable()) worker_.join(); }
  std::string url() const { return "ws://127.0.0.1:" + std::to_string(port_); }
  void push(const std::string& frame) { std::lock_guard<std::mutex> lock(mutex_); websocketpp::lib::error_code ec; server_.send(hdl_, frame, websocketpp::frame::opcode::text, ec); ASSERT_FALSE(ec); }
  bool waitForPub(std::string* value) { std::unique_lock<std::mutex> lock(mutex_); return cv_.wait_for(lock, 2s, [this] { return !pub_.empty(); }) ? (*value = pub_, true) : false; }
 private:
  void onMessage(websocketpp::connection_hdl hdl, const std::string& text) {
    if (text.rfind("INIT ", 0) == 0) { if (mode_ == Mode::Error) push("-ERR Unsupported-Service service denied"); else if (mode_ == Mode::Happy) { push("+OK"); push("INFO {\"version\":\"1.0\",\"server_id\":\"mock-1\",\"uuids\":{\"system\":\"11111111-1111-1111-1111-111111111111\",\"service\":\"22222222-2222-2222-2222-222222222222\"},\"system_label\":\"mock system\"}"); } return; }
    if (text.rfind("PUB ", 0) == 0) { std::lock_guard<std::mutex> lock(mutex_); pub_ = text; cv_.notify_all(); }
  }
  Mode mode_; Server server_; uint16_t port_ = 0; std::thread worker_; mutable std::mutex mutex_; std::condition_variable cv_; websocketpp::connection_hdl hdl_; bool open_ = false; std::string pub_;
};

TEST(OwpFrame, GrammarAndGoldenFrames) {
  using namespace pyramid::owp;
  EXPECT_EQ(serializeInit({{"1.0"}, "002.5.0", "my-svc", true}), "INIT {\"versions\":[\"1.0\"],\"schema\":\"002.5.0\",\"service_id\":\"my-svc\",\"verbose\":true}");
  EXPECT_EQ(serializeSubscribe("sid", "{https://example.test}Name", "topic"), "SUB sid {https://example.test}Name topic");
  EXPECT_EQ(serializeSubscribe("sid", "Name", "topic", "group"), "SUB sid Name topic group");
  EXPECT_EQ(serializeUnsubscribe("sid"), "UNSUB sid"); EXPECT_EQ(serializePublish("topic", "a b\nc"), "PUB topic a b\nc");
  auto msg = parseServerFrame("MSG\t sid  a b\n{namespace}Name"); EXPECT_EQ(msg.subscription_id, "sid"); EXPECT_EQ(msg.body, "a b\n{namespace}Name");
  EXPECT_THROW(serializeSubscribe("sid", "Name", "bad topic"), std::invalid_argument);
}

TEST(OwpClient, VerboseInitReceivesInfo) {
  MockServer server(MockServer::Mode::Happy); pyramid::owp::Client client; client.connect(server.url(), 1s);
  const auto info = client.init({"1.0"}, "002.5.0", "my-svc", true, 1s); EXPECT_EQ(info.server_id, "mock-1"); EXPECT_EQ(info.uuids.system, "11111111-1111-1111-1111-111111111111"); EXPECT_EQ(info.uuids.service, "22222222-2222-2222-2222-222222222222"); client.close();
}
TEST(OwpClient, InitTimeoutFailsClosed) { MockServer server(MockServer::Mode::Silent); pyramid::owp::Client client; client.connect(server.url(), 1s); try { client.init({"1.0"}, "002.5.0", "my-svc", false, 100ms); FAIL(); } catch (const std::runtime_error& e) { EXPECT_NE(std::string(e.what()).find(server.url()), std::string::npos); EXPECT_NE(std::string(e.what()).find("my-svc"), std::string::npos); } client.close(); }
TEST(OwpClient, ErrorPropagates) { MockServer server(MockServer::Mode::Error); pyramid::owp::Client client; client.connect(server.url(), 1s); try { client.init({"1.0"}, "002.5.0", "my-svc", true, 1s); FAIL(); } catch (const std::runtime_error& e) { EXPECT_NE(std::string(e.what()).find("Unsupported-Service"), std::string::npos); } client.close(); }
TEST(OwpClient, RejectedSubprotocolFails) { MockServer server(MockServer::Mode::Reject); pyramid::owp::Client client; EXPECT_THROW(client.connect(server.url(), 1s), std::runtime_error); }
TEST(OwpClient, PubAndMessagePreserveBodies) { MockServer server(MockServer::Mode::Happy); pyramid::owp::Client client; client.connect(server.url(), 1s); client.init({"1.0"}, "002.5.0", "my-svc", true, 1s); std::mutex m; std::condition_variable cv; std::string received; client.setMessageHandler([&](const std::string&, const std::string& topic, const std::string& body) { EXPECT_EQ(topic, "topic"); std::lock_guard<std::mutex> lock(m); received = body; cv.notify_all(); }); client.subscribe("sid", "{https://example.test}Name", "topic"); const std::string body = "with spaces\nand {namespace}Name"; server.push("MSG sid " + body); { std::unique_lock<std::mutex> lock(m); ASSERT_TRUE(cv.wait_for(lock, 1s, [&] { return received == body; })); } client.publish("topic", body); std::string pub; ASSERT_TRUE(server.waitForPub(&pub)); EXPECT_EQ(pub, "PUB topic " + body); client.close(); }
}  // namespace
