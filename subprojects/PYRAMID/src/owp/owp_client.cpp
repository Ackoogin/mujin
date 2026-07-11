#include "owp_client.hpp"

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace pyramid::owp {
class Client::Impl {
 public:
  using WsClient = websocketpp::client<websocketpp::config::asio_client>;
  Impl() { endpoint_.clear_access_channels(websocketpp::log::alevel::all); endpoint_.clear_error_channels(websocketpp::log::elevel::all); endpoint_.init_asio(); }
  ~Impl() { close(); }
  void connect(const std::string& url, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (worker_.joinable()) throw std::runtime_error("OWP client is already running");
    url_ = url; error_.clear(); connected_ = false; closed_ = false;
    // Handlers must be registered before get_connection(): the connection copies
    // the endpoint's handlers at creation time, so setting them afterwards would
    // leave this connection with none (open/message/fail never fire).
    // A server that declines the owp subprotocol fails the handshake (fail_handler);
    // a successfully-opened connection with an empty echo is accepted -- only a
    // non-empty mismatch is treated as a rejection.
    endpoint_.set_open_handler([this](websocketpp::connection_hdl hdl) { std::lock_guard<std::mutex> guard(mutex_); auto con = endpoint_.get_con_from_hdl(hdl); const std::string& negotiated = con->get_subprotocol(); if (!negotiated.empty() && negotiated != "owp") { error_ = "OWP connect " + url_ + ": server rejected owp subprotocol"; endpoint_.close(hdl, websocketpp::close::status::protocol_error, error_); } else { hdl_ = hdl; connected_ = true; } cv_.notify_all(); });
    endpoint_.set_message_handler([this](websocketpp::connection_hdl, WsClient::message_ptr message) { receive(message->get_payload()); });
    endpoint_.set_fail_handler([this](websocketpp::connection_hdl hdl) { std::lock_guard<std::mutex> guard(mutex_); error_ = "OWP connect " + url_ + ": " + endpoint_.get_con_from_hdl(hdl)->get_ec().message(); cv_.notify_all(); });
    endpoint_.set_close_handler([this](websocketpp::connection_hdl) { std::lock_guard<std::mutex> guard(mutex_); if (!closed_ && error_.empty()) error_ = "OWP connection dropped: " + url_; connected_ = false; cv_.notify_all(); });
    websocketpp::lib::error_code ec; auto connection = endpoint_.get_connection(url, ec);
    if (ec) throw std::runtime_error("OWP connect " + url + ": " + ec.message());
    connection->add_subprotocol("owp");
    endpoint_.connect(connection); worker_ = std::thread([this] { endpoint_.run(); });
    if (!cv_.wait_for(lock, timeout, [this] { return connected_ || !error_.empty(); })) { lock.unlock(); close(); throw std::runtime_error("OWP connect timeout: " + url); }
    if (!error_.empty()) { const auto error = error_; lock.unlock(); close(); throw std::runtime_error(error); }
  }
  Info init(const Init& request, std::chrono::milliseconds timeout) {
    { std::lock_guard<std::mutex> lock(mutex_); if (!connected_) throw std::runtime_error("OWP is not connected"); waiting_info_ = true; info_ready_ = false; init_error_.clear(); }
    send(serializeInit(request)); std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this] { return info_ready_ || !init_error_.empty() || !error_.empty(); })) throw std::runtime_error("OWP INIT timed out for " + url_ + " service_id=" + request.service_id);
    waiting_info_ = false; if (!init_error_.empty()) throw std::runtime_error("OWP INIT failed for " + url_ + " service_id=" + request.service_id + ": " + init_error_); if (!error_.empty()) throw std::runtime_error(error_); return info_;
  }
  void subscribe(const std::string& sid, const std::string& name, const std::string& topic, const std::string& group) { const auto frame = serializeSubscribe(sid, name, topic, group); { std::lock_guard<std::mutex> lock(mutex_); subscriptions_[sid] = topic; } send(frame); }
  void unsubscribe(const std::string& sid) { const auto frame = serializeUnsubscribe(sid); { std::lock_guard<std::mutex> lock(mutex_); subscriptions_.erase(sid); } send(frame); }
  void publish(const std::string& topic, const std::string& body) { send(serializePublish(topic, body)); }
  void setHandler(MessageHandler handler) { std::lock_guard<std::mutex> lock(mutex_); handler_ = std::move(handler); }
  std::string lastError() const { std::lock_guard<std::mutex> lock(mutex_); return error_.empty() ? init_error_ : error_; }
  void close() { std::thread worker; { std::lock_guard<std::mutex> lock(mutex_); closed_ = true; if (connected_) { websocketpp::lib::error_code ec; endpoint_.close(hdl_, websocketpp::close::status::going_away, "close", ec); } endpoint_.stop(); worker = std::move(worker_); connected_ = false; } if (worker.joinable()) worker.join(); }
 private:
  void send(const std::string& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_) throw std::runtime_error("OWP is not connected");
    const auto hdl = hdl_;
    endpoint_.get_io_service().post([this, hdl, frame] {
      websocketpp::lib::error_code ec;
      endpoint_.send(hdl, frame, websocketpp::frame::opcode::text, ec);
      if (ec) { std::lock_guard<std::mutex> lock(mutex_); error_ = "OWP send " + url_ + ": " + ec.message(); cv_.notify_all(); }
    });
  }
  void receive(const std::string& text) { try { const auto frame = parseServerFrame(text); MessageHandler handler; std::string topic; if (frame.type == ServerFrameType::Info) { std::lock_guard<std::mutex> lock(mutex_); info_ = frame.info; info_ready_ = true; cv_.notify_all(); } else if (frame.type == ServerFrameType::Error) { std::lock_guard<std::mutex> lock(mutex_); init_error_ = frame.error_name + (frame.error_details.empty() ? "" : " " + frame.error_details); cv_.notify_all(); } else if (frame.type == ServerFrameType::Message) { { std::lock_guard<std::mutex> lock(mutex_); handler = handler_; auto it = subscriptions_.find(frame.subscription_id); if (it != subscriptions_.end()) topic = it->second; } if (handler) handler(frame.subscription_id, topic, frame.body); } } catch (const std::exception& e) { std::lock_guard<std::mutex> lock(mutex_); error_ = std::string("OWP protocol error: ") + e.what(); cv_.notify_all(); } }
  WsClient endpoint_; websocketpp::connection_hdl hdl_; std::thread worker_; mutable std::mutex mutex_; std::condition_variable cv_; std::string url_, error_, init_error_; bool connected_ = false, closed_ = false, waiting_info_ = false, info_ready_ = false; Info info_; std::unordered_map<std::string, std::string> subscriptions_; MessageHandler handler_;
  // TODO(follow-on): reconnect+resubscribe policy.
};
Client::Client() : impl_(new Impl) {}
Client::~Client() = default;
void Client::connect(const std::string& url, std::chrono::milliseconds timeout) { impl_->connect(url, timeout); }
Info Client::init(const std::vector<std::string>& versions, const std::string& schema, const std::string& service_id, bool verbose, std::chrono::milliseconds timeout) { return impl_->init({versions, schema, service_id, verbose}, timeout); }
void Client::subscribe(const std::string& a, const std::string& b, const std::string& c, const std::string& d) { impl_->subscribe(a,b,c,d); }
void Client::unsubscribe(const std::string& sid) { impl_->unsubscribe(sid); }
void Client::publish(const std::string& topic, const std::string& body) { impl_->publish(topic, body); }
void Client::setMessageHandler(MessageHandler handler) { impl_->setHandler(std::move(handler)); }
void Client::close() { impl_->close(); }
std::string Client::lastError() const { return impl_->lastError(); }
}  // namespace pyramid::owp
