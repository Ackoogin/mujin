#pragma once

#include <string>
#include <vector>

struct ProjectModel;

enum class Severity { Error, Warning };

struct StructuralIssue {
  Severity severity;
  std::string message;
  std::string predicateName;
  std::string actionName;
  std::string typeName;
};

struct StructuralReport {
  std::vector<StructuralIssue> issues;
  size_t errorCount = 0;
  size_t warningCount = 0;

  bool hasErrors() const { return errorCount > 0; }
};

class StructuralValidator {
public:
  /// \brief Check ProjectModel structural invariants without generating PDDL.
  static StructuralReport check(const ProjectModel& model);
};
