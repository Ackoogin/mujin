#pragma once

#include "project_model.h"

#include <string>
#include <vector>

/// \brief Result returned by PddlImporter import operations.
struct PddlImportResult {
  bool ok = false;
  std::string error;
  ProjectModel model;
};

/// \brief Minimal STRIPS PDDL importer for the graphical authoring model.
class PddlImporter {
public:
  /// \brief Parse domain PDDL into a ProjectModel containing types, predicates,
  /// actions, and auto-laid-out node positions.
  static PddlImportResult importDomain(const std::string& domainPddl);

  /// \brief Parse problem PDDL into a scenario appended to an existing model.
  static PddlImportResult importProblem(const ProjectModel& model,
                                        const std::string& problemPddl,
                                        const std::string& scenarioName = "");
};
