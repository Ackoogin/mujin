#pragma once

#include "project_model.h"

#include <cstddef>
#include <string>
#include <vector>

/// \brief One dropdown choice in the guided sentence editor.
struct GuidedEditorChoice {
  size_t index = 0;
  std::string name;
  bool legal = false;
  std::string reason;
};

/// \brief Return whether an action parameter can fill a predicate slot.
bool guidedTypeCompatible(const ProjectModel& model,
                          const std::string& actualType,
                          const std::string& expectedType);

/// \brief Return whether an action input, including a union-typed input, can fill a fact slot.
bool guidedParameterCompatible(const ProjectModel& model,
                               const Parameter& actual,
                               const std::string& expectedType);

/// \brief Return every predicate, including illegal choices with an explanation.
std::vector<GuidedEditorChoice> guidedPredicateChoices(
    const ProjectModel& model, const ActionDef& action);

/// \brief Return every action parameter for one predicate argument slot.
std::vector<GuidedEditorChoice> guidedArgumentChoices(
    const ProjectModel& model, const ActionDef& action,
    const PredicateDef& predicate, size_t argumentIndex);

/// \brief Build a legal initial reference for a predicate when possible.
EffectRef makeGuidedReference(const ProjectModel& model,
                              const ActionDef& action,
                              const PredicateDef& predicate);

/// \brief Describe a saved condition with the words used by the guided editor.
std::string guidedConditionText(const ActionDef& action);

/// \brief Add an expressive condition while preserving existing simple conditions.
void appendGuidedCondition(ActionDef& action, ConditionExpression condition);
