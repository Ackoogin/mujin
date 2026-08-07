#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct ProjectModel;

struct ContingencyContext {
  std::vector<std::string> trueFluents;
  bool planFound = false;
  size_t planSteps = 0;
  std::string errorMessage;
};

struct ContingencyReport {
  bool ok = false;
  std::string error;
  std::vector<std::string> contextPredicates;
  std::vector<std::string> contextFluents;
  std::vector<ContingencyContext> results;
  size_t feasibleCount = 0;
  size_t infeasibleCount = 0;
  size_t errorCount = 0;

  // How much of the space the analysis actually covered, and how. Every
  // combination is accounted for either way: what changes is whether the
  // planner was asked about it, or the answer followed from another one.
  size_t combinationsChecked = 0;
  size_t plannerCalls = 0;
  size_t answeredByReasoning = 0;
  bool declaredByUser = false;
  bool pruningUsed = false;
  bool pruningRefused = false;

  /// \brief One sentence saying how much of the space was covered.
  std::string coverageSentence() const;
};

class ContingencyAnalyser {
public:
  /// \brief Enumerate context fluent assignments and solve each resulting scenario.
  /// \brief Check whether a safe state stays reachable however the context is.
  ///
  /// The scenario decides what is being asked: when it declares which facts
  /// represent a contingency and what counts as a safe state, those are used.
  /// When it declares neither, both are inferred as they always were, so a
  /// project that has never been near this screen still gets an answer.
  static ContingencyReport analyse(const ProjectModel& model,
                                   const std::string& scenarioName,
                                   size_t maxFluents = 16);
};
