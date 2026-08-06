#include "guided_editor_model.h"

#include <algorithm>

bool guidedTypeCompatible(const ProjectModel& model,
                          const std::string& actualType,
                          const std::string& expectedType) {
  if (actualType.empty() || expectedType.empty() || actualType == expectedType) {
    return true;
  }
  std::string current = actualType;
  for (size_t depth = 0; depth <= model.types.size(); ++depth) {
    const auto found = std::find_if(
        model.types.begin(), model.types.end(),
        [&current](const TypeDef& type) { return type.name == current; });
    if (found == model.types.end()) {
      return false;
    }
    current = found->parent;
    if (current == expectedType) {
      return true;
    }
  }
  return false;
}

std::vector<GuidedEditorChoice> guidedArgumentChoices(
    const ProjectModel& model, const ActionDef& action,
    const PredicateDef& predicate, size_t argumentIndex) {
  std::vector<GuidedEditorChoice> choices;
  const std::string expected = argumentIndex < predicate.params.size()
      ? predicate.params[argumentIndex].type : std::string{};
  choices.reserve(action.params.size());
  for (size_t index = 0; index < action.params.size(); ++index) {
    const Parameter& parameter = action.params[index];
    const bool legal = guidedTypeCompatible(model, parameter.type, expected);
    choices.push_back({index, parameter.name, legal,
                       legal ? std::string{} : "needs a " + expected});
  }
  return choices;
}

std::vector<GuidedEditorChoice> guidedPredicateChoices(
    const ProjectModel& model, const ActionDef& action) {
  std::vector<GuidedEditorChoice> choices;
  choices.reserve(model.predicates.size());
  for (size_t index = 0; index < model.predicates.size(); ++index) {
    const PredicateDef& predicate = model.predicates[index];
    bool legal = true;
    for (size_t argument = 0; argument < predicate.params.size(); ++argument) {
      const auto parameters = guidedArgumentChoices(model, action, predicate, argument);
      if (std::none_of(parameters.begin(), parameters.end(),
                       [](const GuidedEditorChoice& choice) { return choice.legal; })) {
        legal = false;
        break;
      }
    }
    const std::string appliesTo = predicate.params.empty()
        ? "fact with no object" : predicate.params.front().type;
    choices.push_back({index, predicate.name, legal,
                       legal ? std::string{} : "applies to a " + appliesTo});
  }
  return choices;
}

EffectRef makeGuidedReference(const ProjectModel& model,
                              const ActionDef& action,
                              const PredicateDef& predicate) {
  EffectRef reference;
  reference.predicateName = predicate.name;
  for (size_t argument = 0; argument < predicate.params.size(); ++argument) {
    const auto choices = guidedArgumentChoices(model, action, predicate, argument);
    const auto legal = std::find_if(
        choices.begin(), choices.end(),
        [](const GuidedEditorChoice& choice) { return choice.legal; });
    reference.argNames.push_back(legal == choices.end() ? "?" : legal->name);
  }
  return reference;
}
