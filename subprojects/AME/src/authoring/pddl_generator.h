#pragma once

#include "project_model.h"

#include <string>

class PddlGenerator {
public:
  // Generate the PDDL :domain definition. ProjectModel.projectName is sluggified
  // (lowercased, non-alnum -> '-') to produce the (domain X) name. If projectName
  // is empty, falls back to "untitled".
  static std::string generateDomain(const ProjectModel& model);

  // Generate the PDDL :problem definition for the given scenario. If scenario is
  // not found in model.scenarios, returns an empty problem with no objects/init/goal.
  // The (:domain X) reference matches the slug of model.projectName.
  static std::string generateProblem(const ProjectModel& model,
                                     const std::string& scenarioName);
};
