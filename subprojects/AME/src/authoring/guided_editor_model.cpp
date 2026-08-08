#include "guided_editor_model.h"

#include <algorithm>
#include <sstream>

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

bool guidedParameterCompatible(const ProjectModel& model,
                               const Parameter& actual,
                               const std::string& expectedType) {
  if (actual.eitherTypes.empty()) {
    return guidedTypeCompatible(model, actual.type, expectedType);
  }
  return std::any_of(actual.eitherTypes.begin(), actual.eitherTypes.end(),
                     [&](const std::string& type) {
                       return guidedTypeCompatible(model, type, expectedType);
                     });
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
    const bool legal = guidedParameterCompatible(model, parameter, expected);
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

namespace {

std::string factText(const EffectRef& fact) {
  std::ostringstream out;
  out << fact.predicateName;
  if (!fact.argNames.empty()) {
    out << " for ";
    for (size_t i = 0; i < fact.argNames.size(); ++i) {
      if (i > 0U) out << ", ";
      out << fact.argNames[i];
    }
  }
  return out.str();
}

std::string conditionText(const ConditionExpression& condition) {
  if (condition.kind == ConditionKind::Fact) {
    return factText(condition.fact) +
        (condition.negated || condition.fact.negated
             ? " must be false" : " must be true");
  }
  if (condition.kind == ConditionKind::Equality) {
    const std::string first = condition.terms.empty() ? "one name"
                                                       : condition.terms[0];
    const std::string second = condition.terms.size() < 2U ? "another name"
                                                            : condition.terms[1];
    return first + (condition.negated ? " must be different from "
                                      : " must be the same as ") + second;
  }

  std::string prefix;
  if (condition.kind == ConditionKind::AllOf) prefix = "all of: ";
  if (condition.kind == ConditionKind::AnyOf) prefix = "any one of: ";
  if (condition.kind == ConditionKind::ForEvery) prefix = "for every ";
  if (condition.kind == ConditionKind::AtLeastOne) prefix = "for at least one ";
  if (condition.kind == ConditionKind::ForEvery ||
      condition.kind == ConditionKind::AtLeastOne) {
    if (condition.variables.empty()) {
      prefix += "thing: ";
    } else {
      const Parameter& variable = condition.variables.front();
      prefix += (variable.type.empty() ? "thing" : variable.type) + " " +
                variable.name + ": ";
    }
  }
  for (size_t i = 0; i < condition.children.size(); ++i) {
    if (i > 0U) prefix += "; ";
    prefix += conditionText(condition.children[i]);
  }
  return prefix;
}

ConditionExpression factCondition(const EffectRef& fact) {
  ConditionExpression condition;
  condition.kind = ConditionKind::Fact;
  condition.fact = fact;
  condition.negated = fact.negated;
  condition.fact.negated = false;
  return condition;
}

}  // namespace

std::string guidedConditionText(const ActionDef& action) {
  if (action.hasConditionExpression) {
    return conditionText(action.conditionExpression);
  }
  if (action.preconditions.empty()) {
    return "nothing must already be true";
  }
  ConditionExpression all;
  all.kind = ConditionKind::AllOf;
  for (const EffectRef& fact : action.preconditions) {
    all.children.push_back(factCondition(fact));
  }
  return conditionText(all);
}

void appendGuidedCondition(ActionDef& action, ConditionExpression condition) {
  if (!action.hasConditionExpression) {
    ConditionExpression all;
    all.kind = ConditionKind::AllOf;
    for (const EffectRef& fact : action.preconditions) {
      all.children.push_back(factCondition(fact));
    }
    action.conditionExpression = std::move(all);
    action.hasConditionExpression = true;
  }
  if (action.conditionExpression.kind != ConditionKind::AllOf) {
    ConditionExpression all;
    all.kind = ConditionKind::AllOf;
    all.children.push_back(std::move(action.conditionExpression));
    action.conditionExpression = std::move(all);
  }
  action.conditionExpression.children.push_back(std::move(condition));
  action.preconditions = actionConditionFacts(action);
}
