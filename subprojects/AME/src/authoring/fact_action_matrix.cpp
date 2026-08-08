#include "fact_action_matrix.h"

#include <fstream>
#include <sstream>

namespace {

const FactActionCell kEmptyCell;

std::string marks(const FactActionCell& cell) {
  std::string value;
  if (cell.requires) {
    value += "R";
  }
  if (cell.requiresFalse) {
    value += value.empty() ? "F" : " F";
  }
  if (cell.alternative) {
    value += value.empty() ? "A" : " A";
  }
  if (cell.makesTrue) {
    value += value.empty() ? "+" : " +";
  }
  if (cell.makesFalse) {
    value += value.empty() ? "-" : " -";
  }
  return value;
}

std::string csvQuote(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }
  std::string quoted = "\"";
  for (const char ch : value) {
    quoted += ch == '\"' ? "\"\"" : std::string(1, ch);
  }
  return quoted + "\"";
}

bool writeText(const std::string& path, const std::string& text) {
  std::ofstream output(path);
  output << text;
  return static_cast<bool>(output);
}

} // namespace

FactActionMatrix::FactActionMatrix(const ProjectModel& model,
                                   const RelationIndex& index) {
  cells_.assign(model.predicates.size(),
                std::vector<FactActionCell>(model.actions.size()));
  for (size_t predicate_index = 0; predicate_index < model.predicates.size();
       ++predicate_index) {
    const PredicateRelations& relations = index.predicate(predicate_index);
    for (const auto& relation : relations.requiredBy) {
      cells_[predicate_index][relation.actionIndex].requires = true;
    }
    for (const auto& relation : relations.requiredFalseBy) {
      cells_[predicate_index][relation.actionIndex].requiresFalse = true;
    }
    for (const auto& relation : relations.acceptedAsAlternativeBy) {
      cells_[predicate_index][relation.actionIndex].alternative = true;
    }
    for (const auto& relation : relations.madeTrueBy) {
      cells_[predicate_index][relation.actionIndex].makesTrue = true;
    }
    for (const auto& relation : relations.madeFalseBy) {
      cells_[predicate_index][relation.actionIndex].makesFalse = true;
    }
  }
}

const FactActionCell& FactActionMatrix::cell(size_t predicateIndex,
                                             size_t actionIndex) const {
  return predicateIndex < cells_.size() && actionIndex < cells_[predicateIndex].size()
             ? cells_[predicateIndex][actionIndex]
             : kEmptyCell;
}

std::string FactActionMatrix::toCsv(const ProjectModel& model) const {
  std::ostringstream output;
  output << "fact \\ action";
  for (const auto& action : model.actions) {
    output << ',' << csvQuote(action.name);
  }
  output << '\n';
  for (size_t predicate_index = 0; predicate_index < model.predicates.size();
       ++predicate_index) {
    output << csvQuote(model.predicates[predicate_index].name);
    for (size_t action_index = 0; action_index < model.actions.size(); ++action_index) {
      output << ',' << csvQuote(marks(cell(predicate_index, action_index)));
    }
    output << '\n';
  }
  return output.str();
}

std::string FactActionMatrix::toMarkdown(const ProjectModel& model) const {
  std::ostringstream output;
  output << "| fact \\ action |";
  for (const auto& action : model.actions) {
    output << ' ' << action.name << " |";
  }
  output << "\n|---|";
  for (size_t i = 0; i < model.actions.size(); ++i) {
    output << "---|";
  }
  output << '\n';
  for (size_t predicate_index = 0; predicate_index < model.predicates.size();
       ++predicate_index) {
    output << "| " << model.predicates[predicate_index].name << " |";
    for (size_t action_index = 0; action_index < model.actions.size(); ++action_index) {
      output << ' ' << marks(cell(predicate_index, action_index)) << " |";
    }
    output << '\n';
  }
  return output.str();
}

bool FactActionMatrix::exportCsv(const ProjectModel& model,
                                 const std::string& path) const {
  return writeText(path, toCsv(model));
}

bool FactActionMatrix::exportMarkdown(const ProjectModel& model,
                                      const std::string& path) const {
  return writeText(path, toMarkdown(model));
}
