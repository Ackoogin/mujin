#include "fact_chooser.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace {

const PredicateDef* findFact(const ProjectModel& model,
                             const std::string& name) {
  const auto it = std::find_if(model.predicates.begin(), model.predicates.end(),
                               [&name](const PredicateDef& predicate) {
                                 return predicate.name == name;
                               });
  return it == model.predicates.end() ? nullptr : &(*it);
}

const ObjectDef* findObject(const ProjectModel& model,
                            const std::string& name) {
  const auto it = std::find_if(model.objects.begin(), model.objects.end(),
                               [&name](const ObjectDef& object) {
                                 return object.name == name;
                               });
  return it == model.objects.end() ? nullptr : &(*it);
}

}  // namespace

bool FactChooser::typeMatches(const ProjectModel& model,
                              const std::string& objectType,
                              const std::string& requiredType) {
  if (requiredType.empty() || requiredType == "object") {
    return true;
  }

  // Walk up the type hierarchy: a sector is a location, so an action wanting a
  // location accepts a sector. A limit stops a cycle in a part-built project
  // from hanging the editor.
  std::string current = objectType;
  for (size_t step = 0; step < model.types.size() + 1U; ++step) {
    if (current.empty()) {
      return false;
    }
    if (current == requiredType) {
      return true;
    }
    const auto it = std::find_if(model.types.begin(), model.types.end(),
                                 [&current](const TypeDef& type) {
                                   return type.name == current;
                                 });
    if (it == model.types.end()) {
      return false;
    }
    current = it->parent;
  }
  return false;
}

std::vector<FactChoice> FactChooser::facts(const ProjectModel& model) {
  std::vector<FactChoice> choices;
  choices.reserve(model.predicates.size());
  for (const PredicateDef& predicate : model.predicates) {
    FactChoice choice;
    choice.name = predicate.name;
    if (predicate.name.empty()) {
      choice.allowed = false;
      choice.reason = "this fact has no name yet";
    }
    choices.push_back(std::move(choice));
  }
  return choices;
}

size_t FactChooser::arity(const ProjectModel& model,
                          const std::string& factName) {
  const PredicateDef* predicate = findFact(model, factName);
  return predicate == nullptr ? 0U : predicate->params.size();
}

std::vector<FactChoice> FactChooser::objectsFor(const ProjectModel& model,
                                                const std::string& factName,
                                                size_t position) {
  std::vector<FactChoice> choices;
  const PredicateDef* predicate = findFact(model, factName);
  if (predicate == nullptr || position >= predicate->params.size()) {
    return choices;
  }

  const std::string& requiredType = predicate->params[position].type;
  choices.reserve(model.objects.size());
  for (const ObjectDef& object : model.objects) {
    FactChoice choice;
    choice.name = object.name;
    if (!typeMatches(model, object.type, requiredType)) {
      choice.allowed = false;
      choice.reason = object.name + " is a " +
                      (object.type.empty() ? "thing with no type" : object.type) +
                      ", and this has to be a " + requiredType;
    }
    choices.push_back(std::move(choice));
  }
  return choices;
}

std::string FactChooser::whyNotValid(const ProjectModel& model,
                                     const FactRef& fact) {
  const PredicateDef* predicate = findFact(model, fact.predicateName);
  if (predicate == nullptr) {
    return "there is no fact called '" + fact.predicateName + "'";
  }
  if (fact.objectNames.size() != predicate->params.size()) {
    return "'" + fact.predicateName + "' involves " +
           std::to_string(predicate->params.size()) + " thing" +
           (predicate->params.size() == 1U ? "" : "s") + ", but " +
           std::to_string(fact.objectNames.size()) + " " +
           (fact.objectNames.size() == 1U ? "was" : "were") + " given";
  }

  for (size_t i = 0; i < fact.objectNames.size(); ++i) {
    const ObjectDef* object = findObject(model, fact.objectNames[i]);
    if (object == nullptr) {
      return "there is nothing called '" + fact.objectNames[i] + "'";
    }
    const std::string& requiredType = predicate->params[i].type;
    if (!typeMatches(model, object->type, requiredType)) {
      return object->name + " is a " +
             (object->type.empty() ? "thing with no type" : object->type) +
             ", and '" + fact.predicateName + "' needs a " + requiredType +
             " there";
    }
  }
  return "";
}

FactRef FactChooser::parse(const std::string& text) {
  // Three forms are accepted, because all three are things people write:
  // "(at uav1 base)" as the generated PDDL has it, "at uav1 base" as it is
  // said aloud, and "at(uav1, base)" as most programming languages write it.
  std::string cleaned = text;
  const size_t open = cleaned.find('(');
  const size_t close = cleaned.rfind(')');
  if (open != std::string::npos && close != std::string::npos && close > open) {
    const std::string before = cleaned.substr(0, open);
    const bool named_outside =
        before.find_first_not_of(" \t\r\n") != std::string::npos;
    cleaned = (named_outside ? before + " " : std::string()) +
              cleaned.substr(open + 1, close - open - 1);
  }
  std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

  FactRef fact;
  std::istringstream stream(cleaned);
  std::string token;
  while (stream >> token) {
    if (fact.predicateName.empty()) {
      fact.predicateName = token;
    } else {
      fact.objectNames.push_back(token);
    }
  }
  return fact;
}
