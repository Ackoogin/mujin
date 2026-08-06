#pragma once

#include "project_model.h"
#include "relation_index.h"

#include <string>
#include <vector>

/// \brief The three possible relationships represented by one matrix cell.
struct FactActionCell {
  bool requires = false;
  bool makesTrue = false;
  bool makesFalse = false;
};

/// \brief Read-only fact-by-action matrix and assurance export formats.
class FactActionMatrix {
public:
  FactActionMatrix(const ProjectModel& model, const RelationIndex& index);

  const FactActionCell& cell(size_t predicateIndex, size_t actionIndex) const;
  std::string toCsv(const ProjectModel& model) const;
  std::string toMarkdown(const ProjectModel& model) const;
  bool exportCsv(const ProjectModel& model, const std::string& path) const;
  bool exportMarkdown(const ProjectModel& model, const std::string& path) const;

private:
  std::vector<std::vector<FactActionCell>> cells_;
};
