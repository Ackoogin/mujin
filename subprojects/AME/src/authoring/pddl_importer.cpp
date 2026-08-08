#include "pddl_importer.h"

#include "authoring_utils.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Sexpr {
  bool isList = false;
  std::string atom;
  std::vector<Sexpr> children;
};

std::string toLower(const std::string& text) {
  std::string out = text;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return out;
}

bool isAtom(const Sexpr& expr, const std::string& text) {
  return !expr.isList && toLower(expr.atom) == text;
}

bool isKeyword(const Sexpr& expr, const std::string& text) {
  return isAtom(expr, text);
}

std::vector<std::string> tokenize(const std::string& pddl) {
  std::vector<std::string> tokens;
  std::string current;

  for (size_t i = 0; i < pddl.size(); ++i) {
    const char ch = pddl[i];
    if (ch == ';') {
      if (!current.empty()) {
        tokens.push_back(current);
        current.clear();
      }
      while (i < pddl.size() && pddl[i] != '\n') {
        ++i;
      }
      continue;
    }

    if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
      if (!current.empty()) {
        tokens.push_back(current);
        current.clear();
      }
      continue;
    }

    if (ch == '(' || ch == ')') {
      if (!current.empty()) {
        tokens.push_back(current);
        current.clear();
      }
      tokens.emplace_back(1, ch);
      continue;
    }

    current.push_back(ch);
  }

  if (!current.empty()) {
    tokens.push_back(current);
  }
  return tokens;
}

Sexpr parseSexpr(const std::vector<std::string>& tokens, size_t& pos) {
  if (pos >= tokens.size()) {
    throw std::runtime_error("unexpected end of PDDL");
  }

  const std::string token = tokens[pos++];
  if (token == "(") {
    Sexpr expr;
    expr.isList = true;
    while (pos < tokens.size() && tokens[pos] != ")") {
      expr.children.push_back(parseSexpr(tokens, pos));
    }
    if (pos >= tokens.size()) {
      throw std::runtime_error("unclosed list");
    }
    ++pos;
    return expr;
  }

  if (token == ")") {
    throw std::runtime_error("unexpected ')'");
  }

  Sexpr expr;
  expr.atom = token;
  return expr;
}

Sexpr parseDocument(const std::string& pddl) {
  const std::vector<std::string> tokens = tokenize(pddl);
  if (tokens.empty()) {
    throw std::runtime_error("empty PDDL");
  }

  size_t pos = 0;
  Sexpr root = parseSexpr(tokens, pos);
  if (pos != tokens.size()) {
    throw std::runtime_error("trailing tokens after top-level expression");
  }
  return root;
}

const Sexpr* findListSection(const Sexpr& root, const std::string& name) {
  if (!root.isList) {
    return nullptr;
  }

  for (const auto& child : root.children) {
    if (child.isList && !child.children.empty() &&
        isKeyword(child.children.front(), name)) {
      return &child;
    }
  }
  return nullptr;
}

std::string readDefineName(const Sexpr& root,
                           const std::string& kind,
                           const std::string& label) {
  const Sexpr* section = findListSection(root, kind);
  if (section == nullptr || section->children.size() < 2U ||
      section->children[1].isList) {
    throw std::runtime_error("missing " + label + " name");
  }
  return section->children[1].atom;
}

std::vector<std::string> collectAtoms(const Sexpr& list, size_t start) {
  if (!list.isList) {
    throw std::runtime_error("expected list");
  }

  std::vector<std::string> atoms;
  for (size_t i = start; i < list.children.size(); ++i) {
    if (list.children[i].isList) {
      throw std::runtime_error("expected atom in typed list");
    }
    atoms.push_back(list.children[i].atom);
  }
  return atoms;
}

std::vector<Parameter> parseParameters(const Sexpr& list,
                                       size_t start = 0U,
                                       bool allow_either = false) {
  if (!list.isList) {
    throw std::runtime_error("expected parameter list");
  }
  std::vector<Parameter> params;
  std::vector<std::string> pending;

  for (size_t i = start; i < list.children.size(); ++i) {
    const Sexpr& item = list.children[i];
    if (!item.isList && item.atom == "-") {
      if (pending.empty() || i + 1U >= list.children.size()) {
        throw std::runtime_error("missing type after '-'");
      }
      const Sexpr& type_expr = list.children[++i];
      Parameter type;
      if (!type_expr.isList) {
        type.type = type_expr.atom;
      } else if (allow_either && !type_expr.children.empty() &&
                 isKeyword(type_expr.children.front(), "either")) {
        for (size_t type_index = 1; type_index < type_expr.children.size();
             ++type_index) {
          if (type_expr.children[type_index].isList) {
            throw std::runtime_error("type names in '(either ...)' must be names");
          }
          type.eitherTypes.push_back(type_expr.children[type_index].atom);
        }
        if (type.eitherTypes.empty()) {
          throw std::runtime_error("'(either ...)' needs at least one type");
        }
      } else {
        throw std::runtime_error("expected a type name or '(either ...)' after '-'");
      }
      for (const auto& name : pending) {
        Parameter parameter = type;
        parameter.name = name;
        params.push_back(std::move(parameter));
      }
      pending.clear();
    } else if (!item.isList) {
      pending.push_back(item.atom);
    } else {
      throw std::runtime_error("expected a parameter name");
    }
  }

  for (const auto& name : pending) {
    params.push_back({name, ""});
  }
  return params;
}

std::vector<TypeDef> parseTypes(const Sexpr& list) {
  const std::vector<std::string> atoms = collectAtoms(list, 1U);
  std::vector<TypeDef> types;
  std::vector<std::string> pending;

  for (size_t i = 0; i < atoms.size(); ++i) {
    if (atoms[i] == "-") {
      if (i + 1U >= atoms.size()) {
        throw std::runtime_error("missing parent type after '-'");
      }
      const std::string parent = atoms[++i];
      for (const auto& name : pending) {
        types.push_back({name, parent});
      }
      pending.clear();
    } else {
      pending.push_back(atoms[i]);
    }
  }

  for (const auto& name : pending) {
    types.push_back({name, ""});
  }
  return types;
}

std::vector<ObjectDef> parseObjects(const Sexpr& list) {
  const std::vector<std::string> atoms = collectAtoms(list, 1U);
  std::vector<ObjectDef> objects;
  std::vector<std::string> pending;

  for (size_t i = 0; i < atoms.size(); ++i) {
    if (atoms[i] == "-") {
      if (i + 1U >= atoms.size()) {
        throw std::runtime_error("missing object type after '-'");
      }
      const std::string type = atoms[++i];
      for (const auto& name : pending) {
        objects.push_back({name, type});
      }
      pending.clear();
    } else {
      pending.push_back(atoms[i]);
    }
  }

  for (const auto& name : pending) {
    objects.push_back({name, ""});
  }
  return objects;
}

void appendUniqueObjects(std::vector<ObjectDef>& target,
                         const std::vector<ObjectDef>& source) {
  for (const auto& object : source) {
    const auto it = std::find_if(target.begin(), target.end(),
                                 [&object](const ObjectDef& existing) {
                                   return existing.name == object.name;
                                 });
    if (it == target.end()) {
      target.push_back(object);
    }
  }
}

EffectRef parseEffectRef(const Sexpr& expr) {
  if (!expr.isList) {
    return {expr.atom, {}};
  }
  if (!expr.isList || expr.children.empty() || expr.children.front().isList) {
    throw std::runtime_error("expected fact expression");
  }

  EffectRef ref;
  ref.predicateName = expr.children.front().atom;
  if (toLower(ref.predicateName) == "and" || toLower(ref.predicateName) == "not") {
    throw std::runtime_error("expected predicate fact");
  }

  for (size_t i = 1; i < expr.children.size(); ++i) {
    if (expr.children[i].isList) {
      throw std::runtime_error("expected atom argument in fact");
    }
    ref.argNames.push_back(expr.children[i].atom);
  }
  return ref;
}

ConditionExpression parseCondition(const Sexpr& expr,
                                   const std::string& action_name) {
  const auto malformed = [&action_name](const std::string& detail) {
    throw std::runtime_error("action '" + action_name + "': " + detail);
  };

  if (!expr.isList) {
    ConditionExpression condition;
    condition.kind = ConditionKind::Fact;
    condition.fact = parseEffectRef(expr);
    return condition;
  }
  if (expr.children.empty() || expr.children.front().isList) {
    malformed("expected a condition");
  }

  const std::string head = toLower(expr.children.front().atom);
  ConditionExpression condition;
  if (head == "and" || head == "or") {
    condition.kind = head == "and" ? ConditionKind::AllOf
                                     : ConditionKind::AnyOf;
    for (size_t i = 1; i < expr.children.size(); ++i) {
      condition.children.push_back(parseCondition(expr.children[i], action_name));
    }
    return condition;
  }
  if (head == "not") {
    if (expr.children.size() != 2U) {
      malformed("'must not be true' needs exactly one fact");
    }
    condition = parseCondition(expr.children[1], action_name);
    if (condition.kind != ConditionKind::Fact &&
        condition.kind != ConditionKind::Equality) {
      malformed("'must not be true' can only apply to one fact or comparison");
    }
    condition.negated = true;
    return condition;
  }
  if (head == "forall" || head == "exists") {
    if (expr.children.size() != 3U || !expr.children[1].isList) {
      malformed("'" + head + "' needs a list of names and one condition");
    }
    condition.kind = head == "forall" ? ConditionKind::ForEvery
                                        : ConditionKind::AtLeastOne;
    condition.variables = parseParameters(expr.children[1]);
    condition.children.push_back(parseCondition(expr.children[2], action_name));
    return condition;
  }
  if (head == "=") {
    if (expr.children.size() != 3U || expr.children[1].isList ||
        expr.children[2].isList) {
      malformed("a comparison needs exactly two names");
    }
    condition.kind = ConditionKind::Equality;
    condition.terms = {expr.children[1].atom, expr.children[2].atom};
    return condition;
  }

  condition.kind = ConditionKind::Fact;
  condition.fact = parseEffectRef(expr);
  return condition;
}

void appendConditionFacts(const ConditionExpression& condition,
                          std::vector<EffectRef>& facts,
                          bool alternative = false) {
  if (condition.kind == ConditionKind::Fact) {
    EffectRef fact = condition.fact;
    fact.negated = fact.negated || condition.negated;
    fact.alternative = alternative || fact.alternative;
    facts.push_back(std::move(fact));
    return;
  }
  const bool child_alternative = alternative ||
      condition.kind == ConditionKind::AnyOf ||
      condition.kind == ConditionKind::AtLeastOne;
  for (const ConditionExpression& child : condition.children) {
    appendConditionFacts(child, facts, child_alternative);
  }
}

bool isSimplePositiveCondition(const ConditionExpression& condition) {
  if (condition.kind == ConditionKind::Fact) {
    return !condition.negated && !condition.fact.negated;
  }
  if (condition.kind != ConditionKind::AllOf) {
    return false;
  }
  return std::all_of(condition.children.begin(), condition.children.end(),
                     isSimplePositiveCondition);
}

FactRef parseFactRef(const Sexpr& expr) {
  const EffectRef effect = parseEffectRef(expr);
  return {effect.predicateName, effect.argNames};
}

void parseGoalExpressionList(const Sexpr& expr, std::vector<FactRef>& refs) {
  if (expr.isList && !expr.children.empty() && isKeyword(expr.children.front(), "and")) {
    for (size_t i = 1; i < expr.children.size(); ++i) {
      refs.push_back(parseFactRef(expr.children[i]));
    }
    return;
  }

  refs.push_back(parseFactRef(expr));
}

void parseEffectExpression(const Sexpr& expr, ActionDef& action) {
  if (expr.isList && !expr.children.empty() && isKeyword(expr.children.front(), "and")) {
    for (size_t i = 1; i < expr.children.size(); ++i) {
      parseEffectExpression(expr.children[i], action);
    }
    return;
  }

  if (expr.isList && !expr.children.empty() && isKeyword(expr.children.front(), "not")) {
    if (expr.children.size() != 2U) {
      throw std::runtime_error("expected (not FACT)");
    }
    action.delEffects.push_back(parseEffectRef(expr.children[1]));
    return;
  }

  action.addEffects.push_back(parseEffectRef(expr));
}

PredicateDef parsePredicate(const Sexpr& expr) {
  if (!expr.isList || expr.children.empty() || expr.children.front().isList) {
    throw std::runtime_error("invalid predicate definition");
  }

  PredicateDef predicate;
  predicate.name = expr.children.front().atom;
  predicate.params = parseParameters(expr, 1U);
  return predicate;
}

// Reads (:confirmed-predicates authorised route-clear) and marks the named
// predicates, so that generating the domain again writes the section back out.
// The core parser tolerates a parenthesised entry such as (authorised), so this
// reader does too.
void applyConfirmedPredicates(const Sexpr& section, ProjectModel& model) {
  for (size_t i = 1; i < section.children.size(); ++i) {
    const Sexpr& entry = section.children[i];
    std::string name;
    if (!entry.isList) {
      name = entry.atom;
    } else if (!entry.children.empty() && !entry.children.front().isList) {
      name = entry.children.front().atom;
    } else {
      throw std::runtime_error("malformed :confirmed-predicates entry");
    }

    const auto it = std::find_if(model.predicates.begin(), model.predicates.end(),
                                 [&name](const PredicateDef& predicate) {
                                   return predicate.name == name;
                                 });
    if (it == model.predicates.end()) {
      throw std::runtime_error(
          "':confirmed-predicates' names an undeclared predicate: " + name);
    }
    it->confirmed = true;
  }
}

ActionDef parseAction(const Sexpr& expr) {
  if (!expr.isList || expr.children.size() < 2U ||
      !isKeyword(expr.children.front(), ":action") || expr.children[1].isList) {
    throw std::runtime_error("invalid action definition");
  }

  ActionDef action;
  action.name = expr.children[1].atom;
  for (size_t i = 2; i < expr.children.size(); ++i) {
    if (expr.children[i].isList) {
      throw std::runtime_error("unexpected list in action definition");
    }

    const std::string keyword = toLower(expr.children[i].atom);
    if (keyword == ":parameters") {
      if (i + 1U >= expr.children.size()) {
        throw std::runtime_error("missing :parameters list");
      }
      action.params = parseParameters(expr.children[++i], 0U, true);
    } else if (keyword == ":precondition") {
      if (i + 1U >= expr.children.size()) {
        throw std::runtime_error("missing :precondition expression");
      }
      const ConditionExpression condition =
          parseCondition(expr.children[++i], action.name);
      appendConditionFacts(condition, action.preconditions);
      action.hasConditionExpression = !isSimplePositiveCondition(condition);
      if (action.hasConditionExpression) {
        action.conditionExpression = condition;
      }
    } else if (keyword == ":effect") {
      if (i + 1U >= expr.children.size()) {
        throw std::runtime_error("missing :effect expression");
      }
      parseEffectExpression(expr.children[++i], action);
    } else {
      throw std::runtime_error("unsupported action section: " + expr.children[i].atom);
    }
  }
  return action;
}

void autoPosition(ProjectModel& model) {
  constexpr float kStartX = 80.0F;
  constexpr float kStepX = 190.0F;
  constexpr float kPredicateY = 80.0F;
  constexpr float kActionY = 300.0F;

  for (size_t i = 0; i < model.predicates.size(); ++i) {
    model.predicates[i].posX = kStartX + kStepX * static_cast<float>(i);
    model.predicates[i].posY = kPredicateY;
  }

  for (size_t i = 0; i < model.actions.size(); ++i) {
    model.actions[i].posX = kStartX + kStepX * static_cast<float>(i);
    model.actions[i].posY = kActionY;
  }
}

void validateDefineRoot(const Sexpr& root) {
  if (!root.isList || root.children.empty() || !isKeyword(root.children.front(), "define")) {
    throw std::runtime_error("expected top-level (define ...)");
  }
}

} // namespace

PddlImportResult PddlImporter::importDomain(const std::string& domainPddl) {
  PddlImportResult result;

  try {
    const Sexpr root = parseDocument(domainPddl);
    validateDefineRoot(root);

    ProjectModel model;
    model.clear();
    model.projectName = readDefineName(root, "domain", "domain");

    if (const Sexpr* types = findListSection(root, ":types")) {
      model.types = parseTypes(*types);
    }

    if (const Sexpr* predicates = findListSection(root, ":predicates")) {
      for (size_t i = 1; i < predicates->children.size(); ++i) {
        model.predicates.push_back(parsePredicate(predicates->children[i]));
      }
    }

    if (const Sexpr* confirmed = findListSection(root, ":confirmed-predicates")) {
      applyConfirmedPredicates(*confirmed, model);
    }

    if (const Sexpr* constants = findListSection(root, ":constants")) {
      model.constants = parseObjects(*constants);
    }

    for (const auto& child : root.children) {
      if (child.isList && !child.children.empty() &&
          isKeyword(child.children.front(), ":action")) {
        model.actions.push_back(parseAction(child));
      }
    }

    autoPosition(model);
    result.ok = true;
    result.model = std::move(model);
  } catch (const std::exception& ex) {
    result.ok = false;
    result.error = ex.what();
  }

  return result;
}

PddlImportResult PddlImporter::importProblem(const ProjectModel& model,
                                             const std::string& problemPddl,
                                             const std::string& scenarioName) {
  PddlImportResult result;

  try {
    const Sexpr root = parseDocument(problemPddl);
    validateDefineRoot(root);

    const std::string problemName = readDefineName(root, "problem", "problem");
    const Sexpr* domain = findListSection(root, ":domain");
    if (domain == nullptr || domain->children.size() < 2U || domain->children[1].isList) {
      throw std::runtime_error("missing problem domain");
    }

    ProjectModel imported = model;
    if (const Sexpr* objects = findListSection(root, ":objects")) {
      const std::vector<ObjectDef> parsedObjects = parseObjects(*objects);
      appendUniqueObjects(imported.objects, parsedObjects);
    }

    ScenarioDef scenario;
    scenario.name = scenarioName.empty() ? problemName : scenarioName;

    if (const Sexpr* init = findListSection(root, ":init")) {
      for (size_t i = 1; i < init->children.size(); ++i) {
        scenario.initialState.push_back(parseFactRef(init->children[i]));
      }
    }

    if (const Sexpr* goal = findListSection(root, ":goal")) {
      if (goal->children.size() < 2U) {
        throw std::runtime_error("missing goal expression");
      }
      const Sexpr& expression = goal->children[1];
      if (expression.isList && !expression.children.empty() &&
          isKeyword(expression.children.front(), "or")) {
        if (expression.children.size() == 1U) {
          throw std::runtime_error("goal says 'any one of these' but has no choices");
        }
        for (size_t i = 1; i < expression.children.size(); ++i) {
          std::vector<FactRef> alternative;
          parseGoalExpressionList(expression.children[i], alternative);
          scenario.goalAlternatives.push_back(std::move(alternative));
        }
        scenario.goals = scenario.goalAlternatives.front();
      } else {
        parseGoalExpressionList(expression, scenario.goals);
      }
    }

    imported.scenarios.push_back(std::move(scenario));

    const std::string problemDomain = domain->children[1].atom;
    if (authoring::slugify(problemDomain) !=
        authoring::slugify(model.projectName)) {
      result.error = "Warning: problem domain '" + problemDomain +
                     "' does not match project domain '" + model.projectName + "'";
    }

    result.ok = true;
    result.model = std::move(imported);
  } catch (const std::exception& ex) {
    result.ok = false;
    result.error = ex.what();
  }

  return result;
}
