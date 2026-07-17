#include "pyramid_component_pim_osprey_sensors_skeleton.hpp"
#include "pyramid_services_pim_osprey_sensors_provided_components.hpp"

#include <pcl/process_runtime.hpp>

#include <chrono>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef PYRAMID_COMPONENT_CODEC_PLUGIN_PATH
#  define PYRAMID_COMPONENT_CODEC_PLUGIN_PATH ""
#endif

namespace sensors =
    pyramid::components::pim_osprey::sensors;
namespace provided = sensors::services::provided;

class FilledSensorsStub;

// Request-port deploymentDescriptor() describes the command leg. The
// role-symmetric client harness also needs to route the independent
// requirement leg so it can observe transitions in either realization.
provided::InteractionPortDescriptor providerRequirementDescriptor() {
  return {
      {{provided::kSvcSenrequirementRead, PCL_ENDPOINT_STREAM_PROVIDED}},
      {{provided::kTopicPimOspreySenRequirementRequirement,
        PCL_ENDPOINT_PUBLISHER}},
  };
}

provided::InteractionPortDescriptor clientRequirementDescriptor() {
  return {
      {{provided::kSvcSenrequirementRead, PCL_ENDPOINT_STREAM_CONSUMED}},
      {{provided::kTopicPimOspreySenRequirementRequirement,
        PCL_ENDPOINT_SUBSCRIBER}},
  };
}

class AuthorisationHandler final
    : public provided::AuthorisationDependencyRequestPortHandler {
public:
  provided::Ack onCancel(const provided::Identifier&) override {
    return pyramid::domain_model::kAckOk;
  }
  provided::Ack onCreate(
      const provided::Authorisation_Dependency_Service_Request&) override {
    return pyramid::domain_model::kAckOk;
  }
  provided::Ack onUpdate(
      const provided::Authorisation_Dependency_Service_Requirement&) override {
    return pyramid::domain_model::kAckOk;
  }
};

class SenRequirementHandler final
    : public provided::SenrequirementRequestPortHandler {
public:
  void attach(FilledSensorsStub& owner) { owner_ = &owner; }

  provided::Ack onCancel(const provided::Identifier&) override;

  provided::Ack onCreate(
      const provided::SENRequirement_Service_Request&) override {
    return pyramid::domain_model::kAckOk;
  }
  provided::Ack onUpdate(
      const provided::SENRequirement_Service_Requirement&) override {
    return pyramid::domain_model::kAckOk;
  }

private:
  FilledSensorsStub* owner_ = nullptr;
};

class FilledSensorsStub final : public sensors::SensorsSkeleton {
public:
  FilledSensorsStub(pcl::Executor& executor, Handlers handlers)
      : SensorsSkeleton(
            executor, std::move(handlers), "sensors_provider") {}

  pcl_status_t sendTransition(
      const provided::SENRequirement_Service_Requirement& transition) {
    return senRequirementRequestPort().transitionWriter().send(transition);
  }
};

provided::Ack SenRequirementHandler::onCancel(
    const provided::Identifier&) {
  if (owner_ == nullptr) {
    throw std::runtime_error("SEN requirement handler is not attached");
  }
  FilledSensorsStub* owner = owner_;
  std::thread([owner] {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    provided::SENRequirement_Service_Requirement transition{};
    transition.senrequirement =
        pyramid::domain_model::common_pim_components::sensors::SENRequirement{};
    const auto status = owner->sendTransition(transition);
    std::cout << "TRANSITION_SEND_STATUS=" << status << std::endl;
  }).detach();
  return pyramid::domain_model::kAckOk;
}

class SensorsClient final : public pcl::Component {
public:
  SensorsClient(pcl::Executor& executor, pcl::ProcessRuntime& runtime)
      : pcl::Component("sensors_client"),
        client_(*this, executor),
        runtime_(runtime) {}

  ~SensorsClient() override {
    if (worker_.joinable()) worker_.join();
  }

private:
  pcl_status_t on_configure() override {
    return client_.bind();
  }

  pcl_status_t on_activate() override {
    worker_ = std::thread([this] { exercise(); });
    return PCL_OK;
  }

  void exercise() {
    try {
      provided::Query query{};
      query.one_shot = true;
      auto subscription = client_.transitions(
          query,
          [this](const provided::SENRequirement_Service_Requirement&) {
            std::lock_guard<std::mutex> lock(mutex_);
            transition_observed_ = true;
            condition_.notify_all();
          });
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      auto result = client_.submit(provided::Identifier("e2e")).get();
      if (!result.accepted) {
        throw std::runtime_error("request transfer was not acknowledged");
      }
      std::cout << "ACK_OBSERVED" << std::endl;
      std::unique_lock<std::mutex> lock(mutex_);
      if (!condition_.wait_for(
              lock, std::chrono::seconds(5),
              [this] { return transition_observed_; })) {
        throw std::runtime_error("requirement transition timed out");
      }
      std::cout << "TRANSITION_OBSERVED" << std::endl;
    } catch (const std::exception& error) {
      std::cerr << "E2E_FAIL: " << error.what() << std::endl;
    }
    runtime_.requestShutdown();
  }

  provided::SenrequirementRequestPortClient client_;
  pcl::ProcessRuntime& runtime_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool transition_observed_ = false;
};

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: component_skeleton_e2e provider|client "
                 "<port-config> [--duration-seconds=N]\n";
    return 2;
  }
  const std::string role(argv[1]);
  std::vector<char*> runtime_arguments;
  runtime_arguments.reserve(static_cast<std::size_t>(argc - 1));
  runtime_arguments.push_back(argv[0]);
  for (int index = 2; index < argc; ++index) {
    runtime_arguments.push_back(argv[index]);
  }

  try {
    if (role == "provider") {
      auto ports = sensors::SensorsSkeleton::deploymentPorts();
      ports.emplace_back(
          "sen_requirement_transition", providerRequirementDescriptor());
      pcl::ProcessRuntime runtime(
          static_cast<int>(runtime_arguments.size()),
          runtime_arguments.data(),
          {PYRAMID_COMPONENT_CODEC_PLUGIN_PATH},
          ports);
      AuthorisationHandler authorisation_handler;
      SenRequirementHandler sen_requirement_handler;
      sensors::SensorsSkeleton::Handlers handlers{
          .authorisation_dependency_request = authorisation_handler,
          .capability_evidence_information =
              [](const provided::Capabilities&) {},
          .sen_requirement_request = sen_requirement_handler,
      };
      FilledSensorsStub provider(runtime.executor(), std::move(handlers));
      sen_requirement_handler.attach(provider);
      return runtime.run(provider);
    }
    if (role == "client") {
      const std::vector<pcl::DeploymentPort> ports{
          {"sen_requirement_request",
           provided::SenrequirementRequestPortClient::deploymentDescriptor()},
          {"sen_requirement_transition", clientRequirementDescriptor()},
      };
      pcl::ProcessRuntime runtime(
          static_cast<int>(runtime_arguments.size()),
          runtime_arguments.data(),
          {PYRAMID_COMPONENT_CODEC_PLUGIN_PATH},
          ports);
      SensorsClient client(runtime.executor(), runtime);
      return runtime.run(client);
    }
    std::cerr << "role must be provider or client\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "E2E_FAIL: " << error.what() << std::endl;
    return 1;
  }
}
