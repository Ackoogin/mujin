#pragma once

#include <pcl/pcl_types.h>

#include <memory>
#include <string>

namespace grpc {
class Server;
}

namespace pyramid::services::tactical_objects::provided::grpc_transport {

class ServerHost {
 public:
  explicit ServerHost(pcl_executor_t* executor);
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
                                        pcl_executor_t* executor);

}  // namespace pyramid::services::tactical_objects::provided::grpc_transport
