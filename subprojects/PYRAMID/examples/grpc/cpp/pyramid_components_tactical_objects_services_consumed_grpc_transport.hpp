#pragma once

#include "pyramid_services_tactical_objects_consumed.hpp"

#include "pyramid/components/tactical_objects/services/consumed.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <memory>
#include <string>

namespace pyramid::services::tactical_objects::consumed::grpc_transport {

class ObjectEvidenceServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Evidence_Service::Service {
 public:
  explicit ObjectEvidenceServiceImpl(ServiceHandler& handler) : handler_(handler) {}

  grpc::Status ReadDetail(
      grpc::ServerContext* context,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer)
      override;

 private:
  ServiceHandler& handler_;
};

class ObjectSolutionEvidenceServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Solution_Evidence_Service::Service {
 public:
  explicit ObjectSolutionEvidenceServiceImpl(ServiceHandler& handler)
      : handler_(handler) {}

  grpc::Status CreateRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
      ::pyramid::data_model::base::Identifier* response) override;

  grpc::Status ReadRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectEvidenceRequirement>*
          writer) override;

  grpc::Status UpdateRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
      ::pyramid::data_model::common::Ack* response) override;

  grpc::Status DeleteRequirement(
      grpc::ServerContext* context,
      const ::pyramid::data_model::base::Identifier* request,
      ::pyramid::data_model::common::Ack* response) override;

 private:
  ServiceHandler& handler_;
};

class ObjectSourceCapabilityServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Source_Capability_Service::Service {
 public:
  explicit ObjectSourceCapabilityServiceImpl(ServiceHandler& handler)
      : handler_(handler) {}

  grpc::Status ReadCapability(
      grpc::ServerContext* context,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::common::Capability>* writer)
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
  ObjectEvidenceServiceImpl object_evidence_service_;
  ObjectSolutionEvidenceServiceImpl object_solution_evidence_service_;
  ObjectSourceCapabilityServiceImpl object_source_capability_service_;
  std::unique_ptr<grpc::Server> server_;
};

std::unique_ptr<ServerHost> buildServer(const std::string& listen_address,
                                        ServiceHandler& handler);

}  // namespace pyramid::services::tactical_objects::consumed::grpc_transport
