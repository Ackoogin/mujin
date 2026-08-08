#pragma once

#include "project_model.h"
#include "simulation_engine.h"

#include <string>

/// \brief Result of writing one dated review folder.
struct ReviewPackResult {
  bool success = false;
  std::string folder;
  std::string error;
};

/// \brief Writes the model, scenario evidence and one replayable run together.
class ReviewPackExporter {
public:
  /// \brief Write a dated folder below destination.
  ///
  /// When currentRun is loaded, it is included without running or saving it
  /// first. Otherwise the first scenario is simulated for the pack.
  static ReviewPackResult write(const ProjectModel& model,
                                const std::string& destination,
                                const SimulationEngine* currentRun = nullptr);
};

