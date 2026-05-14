// User business logic for the service-binding example.
//
// This is the part an application author normally owns: subclass the generated
// ServiceHandler and implement typed methods. It contains no PCL buffers,
// transport objects, or codec branching.
#pragma once

#include "pyramid_services_tactical_objects_provided.hpp"

#include <map>
#include <vector>

namespace tobj_example {

namespace svc = pyramid::components::tactical_objects::services::provided;
namespace model = pyramid::domain_model;

class DemoObjectInterestHandler final : public svc::ServiceHandler {
public:
  model::Identifier handleObjectOfInterestCreateRequirement(
      const model::ObjectInterestRequirement& request) override;

  std::vector<model::ObjectInterestRequirement>
  handleObjectOfInterestReadRequirement(const model::Query& request) override;

  model::Ack handleObjectOfInterestDeleteRequirement(
      const model::Identifier& request) override;

  int createCount() const { return create_count_; }
  int readCount() const { return read_count_; }
  int deleteCount() const { return delete_count_; }
  bool empty() const { return requirements_.empty(); }

private:
  int next_id_ = 1;
  int create_count_ = 0;
  int read_count_ = 0;
  int delete_count_ = 0;
  std::map<model::Identifier, model::ObjectInterestRequirement> requirements_;
};

}  // namespace tobj_example
