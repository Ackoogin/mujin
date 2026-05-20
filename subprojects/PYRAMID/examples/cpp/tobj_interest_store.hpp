// InterestStore -- user-supplied business logic for the showcase.
//
// This is the entirety of what a component author writes. It subclasses the
// generated ProvidedHandler and implements one typed callback per RPC. There
// are no PCL types, no codec branches, no stream contexts here.
#pragma once

#include "pyramid_services_tactical_objects_provided_components.hpp"

#include <map>
#include <vector>

namespace tobj_example {

namespace svc   = pyramid::components::tactical_objects::services::provided;
namespace model = pyramid::domain_model;

class InterestStore final : public svc::ProvidedHandler {
public:
  model::Identifier onObjectOfInterestCreateRequirement(
      const model::ObjectInterestRequirement& request) override;

  void onObjectOfInterestReadRequirement(
      const model::Query& query,
      svc::StreamWriter<model::ObjectInterestRequirement> writer) override;

  model::Ack onObjectOfInterestDeleteRequirement(
      const model::Identifier& id) override;

  int  creates() const { return creates_; }
  int  reads() const   { return reads_; }
  int  deletes() const { return deletes_; }
  bool empty() const   { return store_.empty(); }
  size_t size() const  { return store_.size(); }

private:
  int next_id_ = 1;
  int creates_ = 0;
  int reads_   = 0;
  int deletes_ = 0;
  std::map<model::Identifier, model::ObjectInterestRequirement> store_;
};

}  // namespace tobj_example
