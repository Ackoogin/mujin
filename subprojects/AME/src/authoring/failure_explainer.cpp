#include "failure_explainer.h"

#include <algorithm>
#include <map>
#include <set>

namespace {

using Bindings = std::map<std::string, std::string>;

bool isVariable(const std::string& value) {
  return !value.empty() && value.front() == '?';
}

FactRef applyBindings(const EffectRef& reference, const Bindings& bindings) {
  FactRef fact{reference.predicateName, reference.argNames};
  for (auto& argument : fact.objectNames) {
    const auto found = bindings.find(argument);
    if (found != bindings.end()) {
      argument = found->second;
    }
  }
  return fact;
}

Bindings bindingsFor(const EffectRef& pattern, const FactRef& fact) {
  Bindings bindings;
  if (pattern.predicateName != fact.predicateName ||
      pattern.argNames.size() != fact.objectNames.size()) {
    return bindings;
  }
  for (size_t i = 0; i < pattern.argNames.size(); ++i) {
    if (isVariable(pattern.argNames[i]) && !isVariable(fact.objectNames[i])) {
      bindings[pattern.argNames[i]] = fact.objectNames[i];
    }
  }
  return bindings;
}

bool factsMatch(const FactRef& pattern, const FactRef& candidate) {
  if (pattern.predicateName != candidate.predicateName ||
      pattern.objectNames.size() != candidate.objectNames.size()) {
    return false;
  }
  for (size_t i = 0; i < pattern.objectNames.size(); ++i) {
    if (!isVariable(pattern.objectNames[i]) &&
        pattern.objectNames[i] != candidate.objectNames[i]) {
      return false;
    }
  }
  return true;
}

bool startsWith(const ScenarioDef& scenario, const FactRef& fact) {
  return std::any_of(scenario.initialState.begin(), scenario.initialState.end(),
                     [&fact](const FactRef& initial) {
                       return factsMatch(fact, initial);
                     });
}

void appendMandatoryPositiveFacts(const ConditionExpression& condition,
                                  std::vector<EffectRef>& facts) {
  if (condition.kind == ConditionKind::Fact) {
    if (!condition.negated && !condition.fact.negated) {
      facts.push_back(condition.fact);
    }
    return;
  }
  if (condition.kind != ConditionKind::AllOf) {
    // Alternatives and quantified names need their own grounded explanation.
    // Following either as if it were one ordinary mandatory fact can name the
    // wrong cause, so this concise backward chain stops at that boundary.
    return;
  }
  for (const ConditionExpression& child : condition.children) {
    appendMandatoryPositiveFacts(child, facts);
  }
}

std::vector<EffectRef> mandatoryPositiveFacts(const ActionDef& action) {
  if (action.hasConditionExpression) {
    std::vector<EffectRef> facts;
    appendMandatoryPositiveFacts(action.conditionExpression, facts);
    return facts;
  }
  std::vector<EffectRef> facts;
  for (const EffectRef& fact : action.preconditions) {
    if (!fact.negated) {
      facts.push_back(fact);
    }
  }
  return facts;
}

std::string producerText(const ProjectModel& model,
                         const PredicateRelations& relations,
                         const FactRef& fact) {
  if (relations.madeTrueBy.empty()) {
    return {};
  }
  const std::string action_name =
      model.actions[relations.madeTrueBy.front().actionIndex].name;
  return (relations.madeTrueBy.size() == 1U ? "Only " : "One action, ") +
         action_name + " makes " + FailureExplainer::formatFact(fact) + " true.";
}

bool actionCouldRemoveInitialFact(const ProjectModel& model,
                                  const ScenarioDef& scenario,
                                  size_t action_index,
                                  const FactRef& fact) {
  const ActionDef& action = model.actions[action_index];
  if (action.hasConditionExpression) {
    // An alternative or quantified condition cannot safely be reduced to the
    // flat "all of these" check used by this narrow lifecycle diagnostic.
    return false;
  }
  const auto removed = std::find_if(action.delEffects.begin(), action.delEffects.end(),
                                    [&fact](const EffectRef& effect) {
                                      return effect.predicateName == fact.predicateName;
                                    });
  if (removed == action.delEffects.end()) {
    return false;
  }
  const Bindings bindings = bindingsFor(*removed, fact);
  const std::vector<EffectRef> conditions = mandatoryPositiveFacts(action);
  return std::all_of(conditions.begin(), conditions.end(),
                     [&](const EffectRef& precondition) {
                       return startsWith(scenario, applyBindings(precondition, bindings));
                     });
}

} // namespace

std::string FailureExplainer::formatFact(const FactRef& fact) {
  std::string text = "(" + fact.predicateName;
  for (const auto& argument : fact.objectNames) {
    text += " " + argument;
  }
  return text + ")";
}

FailureExplanation FailureExplainer::explain(const ProjectModel& model,
                                             const RelationIndex& index,
                                             const ScenarioDef& scenario,
                                             size_t goalIndex) {
  FailureExplanation explanation;
  if (goalIndex >= scenario.goals.size()) {
    return explanation;
  }
  explanation.available = true;
  explanation.failedGoal = scenario.goals[goalIndex];

  FactRef current = explanation.failedGoal;
  std::set<std::string> visited;
  bool producer_already_named = false;
  for (size_t depth = 0; depth < model.predicates.size() + model.actions.size(); ++depth) {
    if (!visited.insert(formatFact(current)).second) {
      break;
    }
    const int predicate_index = index.predicateIndex(current.predicateName);
    if (predicate_index < 0) {
      break;
    }
    const PredicateRelations& relations =
        index.predicate(static_cast<size_t>(predicate_index));
    if (relations.madeTrueBy.empty()) {
      explanation.blockingFact = current;
      const std::string fact = formatFact(current);
      explanation.rows.push_back({FailureExplanationKind::Conclusion,
                                  "No action in this domain ever makes " + fact +
                                      " true, and the scenario does not start with it.",
                                  {}, current});
      explanation.fixes = {
          "Add " + fact + " to this scenario's starting facts",
          "Add an action that restores " + current.predicateName,
          "Mark this scenario expected-to-fail",
      };
      return explanation;
    }

    const PredicateActionRelation producer_relation = relations.madeTrueBy.front();
    const ActionDef& producer = model.actions[producer_relation.actionIndex];
    if (!producer_already_named) {
      explanation.rows.push_back({FailureExplanationKind::Producer,
                                  producerText(model, relations, current),
                                  producer.name, current});
    }
    producer_already_named = false;
    const Bindings bindings =
        bindingsFor(producer.addEffects[producer_relation.referenceIndex], current);
    const std::vector<EffectRef> producer_conditions =
        mandatoryPositiveFacts(producer);

    FactRef next;
    bool found_next = false;
    for (const auto& precondition : producer_conditions) {
      const FactRef candidate = applyBindings(precondition, bindings);
      const int candidate_index = index.predicateIndex(candidate.predicateName);
      if (candidate_index >= 0 && !startsWith(scenario, candidate) &&
          index.predicate(static_cast<size_t>(candidate_index)).madeTrueBy.empty()) {
        next = candidate;
        found_next = true;
        break;
      }
    }
    if (!found_next) {
      for (const auto& precondition : producer_conditions) {
        const FactRef candidate = applyBindings(precondition, bindings);
        const int candidate_index = index.predicateIndex(candidate.predicateName);
        if (candidate_index < 0) {
          continue;
        }
        const PredicateRelations& candidate_relations =
            index.predicate(static_cast<size_t>(candidate_index));
        const bool has_applicable_remover = std::any_of(
            candidate_relations.madeFalseBy.begin(),
            candidate_relations.madeFalseBy.end(),
            [&](const PredicateActionRelation& relation) {
              const bool same_action_restores = std::any_of(
                  candidate_relations.madeTrueBy.begin(),
                  candidate_relations.madeTrueBy.end(),
                  [&](const PredicateActionRelation& producer_relation) {
                    return producer_relation.actionIndex == relation.actionIndex;
                  });
              return !same_action_restores &&
                     actionCouldRemoveInitialFact(model, scenario,
                                                  relation.actionIndex,
                                                  candidate);
            });
        if (!candidate_relations.madeTrueBy.empty() && has_applicable_remover) {
          next = candidate;
          found_next = true;
          break;
        }
      }
    }
    if (!found_next) {
      for (const auto& precondition : producer_conditions) {
        const FactRef candidate = applyBindings(precondition, bindings);
        if (!startsWith(scenario, candidate)) {
          next = candidate;
          found_next = true;
          break;
        }
      }
    }
    if (!found_next) {
      break;
    }

    explanation.rows.push_back({FailureExplanationKind::Requirement,
                                producer.name + " needs " + formatFact(next) + ".",
                                producer.name, next});
    const int next_index = index.predicateIndex(next.predicateName);
    const PredicateRelations& next_relations =
        index.predicate(static_cast<size_t>(next_index));
    if (startsWith(scenario, next) && !next_relations.madeFalseBy.empty() &&
        !next_relations.madeTrueBy.empty()) {
      const auto removed = std::find_if(
          next_relations.madeFalseBy.begin(), next_relations.madeFalseBy.end(),
          [&](const PredicateActionRelation& relation) {
            return actionCouldRemoveInitialFact(model, scenario,
                                                relation.actionIndex, next);
          });
      if (removed != next_relations.madeFalseBy.end()) {
        const std::string removing_action = model.actions[removed->actionIndex].name;
        const std::string restoring_action =
            model.actions[next_relations.madeTrueBy.front().actionIndex].name;
        explanation.rows.push_back({FailureExplanationKind::RemovedAndRestored,
                                    removing_action + " removed it, and only " +
                                        restoring_action + " puts it back.",
                                    restoring_action, next});
        current = next;
        producer_already_named = true;
        continue;
      }
    }
    current = next;
  }
  return explanation;
}
