#pragma once

#include <memory>
#include <string>

namespace grpc {
class Server;
}

namespace pyramid::services::tactical_objects::consumed {
class ServiceHandler;
}

namespace pyramid::services::tactical_objects::consumed::grpc_transport {

class ServerHost {
 public:
  explicit ServerHost(ServiceHandler& handler);
  ~ServerHost();

  grpc::Server* start(const std::string& listen_address);
  void shutdown();
  void wait();
  grpc::Server* get() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

std::unique_ptr<ServerHost> buildServer(const std::string& listen_address,
                                        ServiceHandler& handler);

}  // namespace pyramid::services::tactical_objects::consumed::grpc_transport
