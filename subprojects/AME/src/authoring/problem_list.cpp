#include "problem_list.h"

#include "pddl_validator.h"
#include "project_model.h"
#include "structural_validator.h"

#include <algorithm>
#include <string>
#include <vector>

namespace {

/// Replace every occurrence of one word, keeping the leading capital.
void replaceWord(std::string& text,
                 const std::string& from,
                 const std::string& to) {
  size_t pos = 0;
  while ((pos = text.find(from, pos)) != std::string::npos) {
    text.replace(pos, from.size(), to);
    pos += to.size();
  }
}

int indexOfFact(const ProjectModel& model, const std::string& name) {
  for (size_t i = 0; i < model.predicates.size(); ++i) {
    if (model.predicates[i].name == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int indexOfAction(const ProjectModel& model, const std::string& name) {
  for (size_t i = 0; i < model.actions.size(); ++i) {
    if (model.actions[i].name == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int indexOfType(const ProjectModel& model, const std::string& name) {
  for (size_t i = 0; i < model.types.size(); ++i) {
    if (model.types[i].name == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

/// Point the entry at whichever element it names, preferring the action when a
/// problem names both, because an action is where the user can act on it.
void aimAtElement(const ProjectModel& model,
                  ProblemEntry& entry,
                  const std::string& factName,
                  const std::string& actionName,
                  const std::string& typeName) {
  if (!actionName.empty()) {
    const int index = indexOfAction(model, actionName);
    if (index >= 0) {
      entry.target = ProblemTarget::Action;
      entry.targetName = actionName;
      entry.targetIndex = index;
      return;
    }
  }
  if (!factName.empty()) {
    const int index = indexOfFact(model, factName);
    if (index >= 0) {
      entry.target = ProblemTarget::Fact;
      entry.targetName = factName;
      entry.targetIndex = index;
      return;
    }
  }
  if (!typeName.empty()) {
    const int index = indexOfType(model, typeName);
    if (index >= 0) {
      entry.target = ProblemTarget::Type;
      entry.targetName = typeName;
      entry.targetIndex = index;
    }
  }
}

/// A sentence about a parser failure, naming what it was reading when it gave
/// up. The parser's own words describe positions in generated text, which is
/// no use to somebody who never saw that text, so they go in the detail.
std::string parserSentence(const ValidationError& error) {
  if (!error.actionNames.empty()) {
    return "The action '" + error.actionNames.front() +
           "' could not be read back from the generated PDDL.";
  }
  if (!error.predicateNames.empty()) {
    return "The fact '" + error.predicateNames.front() +
           "' could not be read back from the generated PDDL.";
  }
  return "The generated PDDL could not be read back. Nothing in the project "
         "was named, so the cause is in how the pieces fit together rather "
         "than in one of them.";
}

}  // namespace

std::string ProblemList::inPlainWords(const std::string& message) {
  std::string text = message;
  replaceWord(text, "Predicate", "Fact");
  replaceWord(text, "predicates", "facts");
  replaceWord(text, "predicate", "fact");
  replaceWord(text, "parameter", "name it involves");
  replaceWord(text, "zero effects", "no outcomes");
  replaceWord(text, "is a goal target but is never produced by an effect",
              "is wanted as a goal but no action ever makes it true");
  return text;
}

std::vector<ProblemEntry> ProblemList::build(const ProjectModel& model,
                                             const StructuralReport& structural,
                                             const ValidationReport& validation) {
  std::vector<ProblemEntry> entries;
  entries.reserve(structural.issues.size() + validation.errors.size());

  for (const StructuralIssue& issue : structural.issues) {
    ProblemEntry entry;
    entry.isError = issue.severity == Severity::Error;
    entry.sentence = inPlainWords(issue.message);
    aimAtElement(model, entry, issue.predicateName, issue.actionName,
                 issue.typeName);
    entries.push_back(std::move(entry));
  }

  for (const ValidationError& error : validation.errors) {
    ProblemEntry entry;
    entry.isError = true;
    entry.sentence = parserSentence(error);
    entry.detail = error.message;
    aimAtElement(model, entry,
                 error.predicateNames.empty() ? std::string()
                                              : error.predicateNames.front(),
                 error.actionNames.empty() ? std::string()
                                           : error.actionNames.front(),
                 std::string());
    entries.push_back(std::move(entry));
  }

  // Errors first: a warning is worth reading, but not before the thing that
  // stops the model working at all.
  std::stable_sort(entries.begin(), entries.end(),
                   [](const ProblemEntry& a, const ProblemEntry& b) {
                     return a.isError && !b.isError;
                   });
  return entries;
}
