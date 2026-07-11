#pragma once

#include "owp_frame.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace pyramid::owp {

class Client {
 public:
  using MessageHandler = std::function<void(const std::string& subscription_id,
                                            const std::string& topic,
                                            const std::string& body)>;
  Client();
  ~Client();
  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;

  /// \brief Opens an OWP WebSocket connection and verifies subprotocol negotiation.
  void connect(const std::string& url, std::chrono::milliseconds timeout);
  /// \brief Sends INIT and waits for authoritative identity INFO; throws on timeout/error.
  Info init(const std::vector<std::string>& versions, const std::string& schema,
            const std::string& service_id, bool verbose, std::chrono::milliseconds timeout);
  void subscribe(const std::string& subscription_id, const std::string& message_name,
                 const std::string& topic, const std::string& group = "");
  void unsubscribe(const std::string& subscription_id);
  /// \brief Queues PUB without blocking on socket I/O.
  void publish(const std::string& topic, const std::string& body);
  /// \brief The handler runs on the client's worker thread.
  void setMessageHandler(MessageHandler handler);
  void close();
  std::string lastError() const;
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pyramid::owp
