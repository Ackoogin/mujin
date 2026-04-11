#pragma once

#include "pyramid_services_tactical_objects_provided.hpp"

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <memory>
#include <string>

namespace pyramid::services::tactical_objects::provided::grpc_transport {

class MatchingObjectsServiceImpl final
    : public ::pyramid::components::tactical_objects::services::provided::
          Matching_Objects_Service::Service {
 public:
  explicit MatchingObjectsServiceImpl(ServiceHandler& handler) : handler_(handler) {}

  grpc::Status ReadMatch(
      grpc::ServerContext* context,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectMatch>* writer)
      override;

 private:
  ServiceHandler& handler_;
};

class ObjectOfInterestServiceImpl final
    : public ::pyramid::components::tactical_objects::services::provided::
          Object_Of_Interest_Service::Service {
 public:
  explicit ObjectOfInterestServiceImpl(ServiceHandler& handler) : handler_(handler) {}

  grpc::Status CreateRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
      ::pyramid::data_model::base::Identifier* response) override;

  grpc::Status ReadRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectInterestRequirement>*
          writer) override;

  grpc::Status UpdateRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
      ::pyramid::data_model::common::Ack* response) override;

  grpc::Status DeleteRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::base::Identifier* request,
      ::pyramid::data_model::common::Ack* response) override;

 private:
  ServiceHandler& handler_;
};

class SpecificObjectDetailServiceImpl final
    : public ::pyramid::components::tactical_objects::services::provided::
          Specific_Object_Detail_Service::Service {
 public:
  explicit SpecificObjectDetailServiceImpl(ServiceHandler& handler)
      : handler_(handler) {}

  grpc::Status ReadDetail(
      grpc::ServerContext* context,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer)
      override;

 private:
  ServiceHandler& handler_;
};

class ServerHost {
 public:
  explicit ServerHost(ServiceHandler& handler);

  grpc::Server* start(const std::string& listen_address);
  void shutdown();
  void wait();

  grpc::Server* get() const { return server_.get(); }

 private:
  MatchingObjectsServiceImpl matching_objects_service_;
  ObjectOfInterestServiceImpl object_of_interest_service_;
  SpecificObjectDetailServiceImpl specific_object_detail_service_;
  std::unique_ptr<grpc::Server> server_;
};

std::unique_ptr<ServerHost> buildServer(const std::string& listen_address,
                                        ServiceHandler& handler);

}  // namespace pyramid::services::tactical_objects::provided::grpc_transport
