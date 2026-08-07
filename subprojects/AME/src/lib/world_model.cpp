#include "ame/world_model.h"

#include <strips_prob.hxx>
#include <fluent.hxx>
#include <action.hxx>
#include <strips_state.hxx>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <utility>

namespace ame {

namespace {

bool isParameterTokenChar(char c) {
  const auto uc = static_cast<unsigned char>(c);
  return std::isalnum(uc) != 0 || c == '_';
}

void requireGroundedTemplate(
  const std::unordered_map<std::string, unsigned>& fluent_index,
  const std::string& schema_name,
  const std::string& tmpl,
  const std::string& key,
  std::vector<unsigned>& output)
{
  auto it = fluent_index.find(key);
  if (it == fluent_index.end()) {
    throw std::runtime_error(
      "WorldModel::groundActionSchema: unresolved template '" + tmpl +
      "' in schema '" + schema_name + "' resolved to '" + key + "'");
  }
  output.push_back(it->second);
}

std::unordered_set<std::string> makeGroundActionSignatures(
  const std::vector<GroundAction>& ground_actions)
{
  std::unordered_set<std::string> signatures;
  signatures.reserve(ground_actions.size());
  for (const auto& action : ground_actions) {
    signatures.insert(action.signature);
  }
  return signatures;
}

}  // namespace

WorldModel::WorldModel(const WorldModel& other)
    : types_(other.types_),
      predicates_(other.predicates_),
      action_schemas_(other.action_schemas_),
      ground_actions_(other.ground_actions_),
      ground_action_signatures_(makeGroundActionSignatures(other.ground_actions_)),
      state_bits_(other.state_bits_),
      fact_metadata_(other.fact_metadata_),
      fluent_names_(other.fluent_names_),
      fluent_index_(other.fluent_index_),
      goal_fluent_ids_(other.goal_fluent_ids_),
      goal_alternatives_(other.goal_alternatives_),
      version_(other.version_),
      agents_(other.agents_) {
  // Note: mutexes are default-constructed (not copied)
  // audit_callback_ is not copied (intentional - caller should set if needed)
}

WorldModel& WorldModel::operator=(const WorldModel& other) {
  if (this != &other) {
    auto ground_action_signatures =
        makeGroundActionSignatures(other.ground_actions_);
    types_ = other.types_;
    predicates_ = other.predicates_;
    confirmed_predicates_ = other.confirmed_predicates_;
    action_schemas_ = other.action_schemas_;
    ground_actions_ = other.ground_actions_;
    ground_action_signatures_ = std::move(ground_action_signatures);
    state_bits_ = other.state_bits_;
    fact_metadata_ = other.fact_metadata_;
    fluent_names_ = other.fluent_names_;
    fluent_index_ = other.fluent_index_;
    goal_fluent_ids_ = other.goal_fluent_ids_;
    goal_alternatives_ = other.goal_alternatives_;
    version_ = other.version_;
    agents_ = other.agents_;
    // Note: mutexes are not copied, audit_callback_ is not copied
  }
  return *this;
}

WorldModel::WorldModel(WorldModel&& other) {
  std::scoped_lock lock(other.state_mutex_, other.mutation_queue_mutex_);
  auto ground_action_signatures =
      makeGroundActionSignatures(other.ground_actions_);
  types_ = std::move(other.types_);
  predicates_ = std::move(other.predicates_);
  confirmed_predicates_ = std::move(other.confirmed_predicates_);
  action_schemas_ = std::move(other.action_schemas_);
  ground_actions_ = std::move(other.ground_actions_);
  ground_action_signatures_ = std::move(ground_action_signatures);
  state_bits_ = std::move(other.state_bits_);
  fact_metadata_ = std::move(other.fact_metadata_);
  fluent_names_ = std::move(other.fluent_names_);
  fluent_index_ = std::move(other.fluent_index_);
  goal_fluent_ids_ = std::move(other.goal_fluent_ids_);
  goal_alternatives_ = std::move(other.goal_alternatives_);
  version_ = other.version_;
  audit_callback_ = std::move(other.audit_callback_);
  mutation_queue_ = std::move(other.mutation_queue_);
  agents_ = std::move(other.agents_);

  other.ground_actions_.clear();
  other.ground_action_signatures_.clear();
  other.version_ = 0;
}

WorldModel& WorldModel::operator=(WorldModel&& other) {
  if (this != &other) {
    std::scoped_lock lock(state_mutex_, other.state_mutex_,
                          mutation_queue_mutex_,
                          other.mutation_queue_mutex_);
    auto ground_action_signatures =
        makeGroundActionSignatures(other.ground_actions_);
    types_ = std::move(other.types_);
    predicates_ = std::move(other.predicates_);
    confirmed_predicates_ = std::move(other.confirmed_predicates_);
    action_schemas_ = std::move(other.action_schemas_);
    ground_actions_ = std::move(other.ground_actions_);
    ground_action_signatures_ = std::move(ground_action_signatures);
    state_bits_ = std::move(other.state_bits_);
    fact_metadata_ = std::move(other.fact_metadata_);
    fluent_names_ = std::move(other.fluent_names_);
    fluent_index_ = std::move(other.fluent_index_);
    goal_fluent_ids_ = std::move(other.goal_fluent_ids_);
    goal_alternatives_ = std::move(other.goal_alternatives_);
    version_ = other.version_;
    audit_callback_ = std::move(other.audit_callback_);
    mutation_queue_ = std::move(other.mutation_queue_);
    agents_ = std::move(other.agents_);

    other.ground_actions_.clear();
    other.ground_action_signatures_.clear();
    other.version_ = 0;
  }
  return *this;
}

void WorldModel::registerPredicate(const std::string& name,
                   const std::vector<std::string>& param_types) {
  predicates_.push_back({name, param_types});
  // Eagerly ground against all existing objects
  groundPredicate(name, param_types);
}

void WorldModel::registerConfirmedPredicate(const std::string& name) {
  confirmed_predicates_.insert(name);
}

bool WorldModel::isConfirmedPredicate(const std::string& name) const {
  return confirmed_predicates_.count(name) != 0;
}

bool WorldModel::isConfirmedFact(const std::string& fact_key) const {
  if (confirmed_predicates_.empty()) {
    return false;
  }
  // fact_key is "(pred arg ...)" or "(pred)"; the predicate is the first token.
  size_t begin = fact_key.find_first_not_of("( \t");
  if (begin == std::string::npos) {
    return false;
  }
  size_t end = fact_key.find_first_of(" \t)", begin);
  if (end == std::string::npos) {
    end = fact_key.size();
  }
  return isConfirmedPredicate(fact_key.substr(begin, end - begin));
}

void WorldModel::addObject(const std::string& name, const std::string& type) {
  types_.addObject(name, type);
  // Eagerly ground: for every predicate, try to create new fluents involving this object
  groundNewObject(name, type);
}

void WorldModel::registerAction(const std::string& name,
                const std::vector<std::string>& param_names,
                const std::vector<std::string>& param_types,
                const std::vector<std::string>& precondition_templates,
                const std::vector<std::string>& add_templates,
                const std::vector<std::string>& del_templates) {
  // Compatibility overload: no negative preconditions.
  registerAction(name, param_names, param_types, precondition_templates,
                 /*neg_precondition_templates=*/{}, add_templates, del_templates);
}

void WorldModel::registerAction(const std::string& name,
                const std::vector<std::string>& param_names,
                const std::vector<std::string>& param_types,
                const std::vector<std::string>& precondition_templates,
                const std::vector<std::string>& neg_precondition_templates,
                const std::vector<std::string>& add_templates,
                const std::vector<std::string>& del_templates) {
  // Forward to the full overload with no equality constraints.
  registerAction(name, param_names, param_types, precondition_templates,
                 neg_precondition_templates, add_templates, del_templates,
                 /*equality_constraints=*/{});
}

void WorldModel::registerAction(const std::string& name,
                const std::vector<std::string>& param_names,
                const std::vector<std::string>& param_types,
                const std::vector<std::string>& precondition_templates,
                const std::vector<std::string>& neg_precondition_templates,
                const std::vector<std::string>& add_templates,
                const std::vector<std::string>& del_templates,
                const std::vector<EqualityConstraint>& equality_constraints) {
  ActionSchemaInternal schema;
  schema.schema.name = name;
  schema.schema.param_names = param_names;
  schema.schema.param_types = param_types;
  schema.precondition_templates = precondition_templates;
  schema.neg_precondition_templates = neg_precondition_templates;
  schema.add_templates = add_templates;
  schema.del_templates = del_templates;
  schema.equality_constraints = equality_constraints;

  unsigned idx = static_cast<unsigned>(action_schemas_.size());
  action_schemas_.push_back(std::move(schema));
  groundActionSchema(idx);
}

void WorldModel::setFact(const std::string& key, bool value,
             const std::string& source, FactAuthority authority) {
  auto it = fluent_index_.find(key);
  if (it == fluent_index_.end()) {
    throw std::runtime_error("WorldModel::setFact: unknown fluent '" + key + "'");
  }
  setFact(it->second, value, source, authority);
}

void WorldModel::setFact(const std::string& key, bool value,
             const std::string& source, FactAuthority authority,
             uint64_t timestamp_us) {
  auto it = fluent_index_.find(key);
  if (it == fluent_index_.end()) {
    throw std::runtime_error("WorldModel::setFact: unknown fluent '" + key + "'");
  }
  setFactAtTimestamp(it->second, value, source, authority, timestamp_us);
}

bool WorldModel::getFact(const std::string& key) const {
  auto it = fluent_index_.find(key);
  if (it == fluent_index_.end()) {
    throw std::runtime_error("WorldModel::getFact: unknown fluent '" + key + "'");
  }
  return getFact(it->second);
}

FactMetadata WorldModel::getFactMetadata(const std::string& key) const {
  auto it = fluent_index_.find(key);
  if (it == fluent_index_.end()) {
    throw std::runtime_error("WorldModel::getFactMetadata: unknown fluent '" + key + "'");
  }
  return getFactMetadata(it->second);
}

void WorldModel::setFact(unsigned id, bool value,
             const std::string& source, FactAuthority authority) {
  auto ts = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  setFactAtTimestamp(id, value, source, authority, static_cast<uint64_t>(ts));
}

void WorldModel::setFactAtTimestamp(unsigned id, bool value,
             const std::string& source, FactAuthority authority,
             uint64_t timestamp_us) {
  std::unique_lock<std::shared_mutex> lock(state_mutex_);

  unsigned word = id / 64;
  unsigned bit = id % 64;
  if (word >= state_bits_.size()) {
    state_bits_.resize(word + 1, 0);
  }
  if (id >= fact_metadata_.size()) {
    fact_metadata_.resize(id + 1);
  }

  bool old_val = (state_bits_[word] >> bit) & 1u;

  // Update metadata regardless of value change
  fact_metadata_[id].authority = authority;
  fact_metadata_[id].timestamp_us = timestamp_us;
  fact_metadata_[id].source = source;

  if (old_val != value) {
    if (value) {
      state_bits_[word] |= (uint64_t(1) << bit);
    } else {
      state_bits_[word] &= ~(uint64_t(1) << bit);
    }
    ++version_;
    if (audit_callback_) {
      audit_callback_(version_, timestamp_us,
               fluent_names_[id], value, source);
    }
  }
}

void WorldModel::setAuditCallback(AuditCallback cb) {
  audit_callback_ = std::move(cb);
}

bool WorldModel::getFact(unsigned id) const {
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  unsigned word = id / 64;
  unsigned bit = id % 64;
  if (word >= state_bits_.size()) return false;
  return (state_bits_[word] >> bit) & 1u;
}

FactMetadata WorldModel::getFactMetadata(unsigned id) const {
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  if (id >= fact_metadata_.size()) {
    return FactMetadata{};
  }
  return fact_metadata_[id];
}

WorldStateSnapshotPtr WorldModel::captureSnapshot() const {
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  auto snapshot = std::make_shared<WorldStateData>();
  snapshot->state_bits = state_bits_;
  snapshot->fact_metadata = fact_metadata_;
  snapshot->version = version_;
  return snapshot;
}

bool WorldModel::hasAuthorityConflict(unsigned id, bool perceived_value) const {
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  unsigned word = id / 64;
  unsigned bit = id % 64;
  bool current_value = (word < state_bits_.size()) ? ((state_bits_[word] >> bit) & 1u) : false;
  if (id < fact_metadata_.size()) {
    const auto& meta = fact_metadata_[id];
    // Conflict if current value is BELIEVED and differs from perception
    if (meta.authority == FactAuthority::BELIEVED && current_value != perceived_value) {
      return true;
    }
  }
  return false;
}

void WorldModel::enqueueMutation(unsigned id, bool value,
                  const std::string& source,
                  FactAuthority authority) {
  std::lock_guard<std::mutex> lock(mutation_queue_mutex_);
  auto ts = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();

  PendingMutation mutation;
  mutation.fluent_id = id;
  mutation.value = value;
  mutation.source = source;
  mutation.authority = authority;
  mutation.timestamp_us = static_cast<uint64_t>(ts);
  mutation_queue_.push_back(std::move(mutation));
}

size_t WorldModel::applyQueuedMutations() {
  std::vector<PendingMutation> mutations;
  {
    std::lock_guard<std::mutex> lock(mutation_queue_mutex_);
    mutations = std::move(mutation_queue_);
    mutation_queue_.clear();
  }

  // Apply all mutations under exclusive state lock
  std::unique_lock<std::shared_mutex> state_lock(state_mutex_);
  size_t applied = 0;

  for (const auto& m : mutations) {
    unsigned word = m.fluent_id / 64;
    unsigned bit = m.fluent_id % 64;

    if (word >= state_bits_.size()) {
      state_bits_.resize(word + 1, 0);
    }
    if (m.fluent_id >= fact_metadata_.size()) {
      fact_metadata_.resize(m.fluent_id + 1);
    }

    bool old_val = (state_bits_[word] >> bit) & 1u;

    // Update metadata
    fact_metadata_[m.fluent_id].authority = m.authority;
    fact_metadata_[m.fluent_id].timestamp_us = m.timestamp_us;
    fact_metadata_[m.fluent_id].source = m.source;

    if (old_val != m.value) {
      if (m.value) {
        state_bits_[word] |= (uint64_t(1) << bit);
      } else {
        state_bits_[word] &= ~(uint64_t(1) << bit);
      }
      ++version_;
      if (audit_callback_) {
        audit_callback_(version_, m.timestamp_us,
                 fluent_names_[m.fluent_id], m.value, m.source);
      }
    }
    ++applied;
  }

  return applied;
}

bool WorldModel::hasPendingMutations() const {
  std::lock_guard<std::mutex> lock(mutation_queue_mutex_);
  return !mutation_queue_.empty();
}

unsigned WorldModel::fluentIndex(const std::string& key) const {
  auto it = fluent_index_.find(key);
  if (it == fluent_index_.end()) {
    throw std::runtime_error("WorldModel::fluentIndex: unknown fluent '" + key + "'");
  }
  return it->second;
}

const std::string& WorldModel::fluentName(unsigned id) const {
  if (id >= fluent_names_.size()) {
    throw std::runtime_error("WorldModel::fluentName: id out of range");
  }
  return fluent_names_[id];
}

void WorldModel::setGoal(const std::vector<std::string>& goal_fluent_keys) {
  goal_fluent_ids_.clear();
  for (auto& key : goal_fluent_keys) {
    goal_fluent_ids_.push_back(fluentIndex(key));
  }
  // A single goal is one alternative.
  goal_alternatives_.assign(1, goal_fluent_ids_);
}

void WorldModel::setGoalAlternatives(
    const std::vector<std::vector<std::string>>& goal_alternatives) {
  goal_alternatives_.clear();
  goal_alternatives_.reserve(goal_alternatives.size());
  for (const auto& alt : goal_alternatives) {
    std::vector<unsigned> ids;
    ids.reserve(alt.size());
    for (const auto& key : alt) {
      ids.push_back(fluentIndex(key));
    }
    goal_alternatives_.push_back(std::move(ids));
  }
  // Mirror the primary alternative for single-goal callers.
  goal_fluent_ids_ = goal_alternatives_.empty() ? std::vector<unsigned>{}
                                                : goal_alternatives_.front();
}

// =========================================================================
// Grounding helpers
// =========================================================================

// Resolve the candidate objects for a parameter type token. A union type from
// an `(either a b)` declaration is encoded as "a|b"; it ranges over the
// deduplicated union of objects of each listed type (each already includes that
// type's subtype objects). A plain token is a single type.
static std::vector<std::string> objectsForTypeToken(const TypeSystem& ts,
                                                    const std::string& token) {
  if (token.find('|') == std::string::npos) {
    return ts.getObjectsOfType(token);
  }
  std::vector<std::string> result;
  std::unordered_set<std::string> seen;
  size_t start = 0;
  while (start <= token.size()) {
    size_t bar = token.find('|', start);
    std::string type = token.substr(
      start, bar == std::string::npos ? std::string::npos : bar - start);
    if (!type.empty()) {
      for (const auto& obj : ts.getObjectsOfType(type)) {
        if (seen.insert(obj).second) result.push_back(obj);
      }
    }
    if (bar == std::string::npos) break;
    start = bar + 1;
  }
  return result;
}

// Helper: generate all combinations of objects matching param_types
static void generateCombinations(
  const TypeSystem& ts,
  const std::vector<std::string>& param_types,
  size_t depth,
  std::vector<std::string>& current,
  std::vector<std::vector<std::string>>& results)
{
  if (depth == param_types.size()) {
    results.push_back(current);
    return;
  }
  auto objects = objectsForTypeToken(ts, param_types[depth]);
  for (auto& obj : objects) {
    current.push_back(obj);
    generateCombinations(ts, param_types, depth + 1, current, results);
    current.pop_back();
  }
}

static std::string makeFluentKey(const std::string& pred_name,
                 const std::vector<std::string>& args) {
  std::string key = "(" + pred_name;
  for (auto& a : args) {
    key += " " + a;
  }
  key += ")";
  return key;
}

void WorldModel::groundPredicate(const std::string& pred_name,
                 const std::vector<std::string>& param_types) {
  if (param_types.empty()) {
    // 0-ary predicate
    std::string key = makeFluentKey(pred_name, {});
    if (fluent_index_.count(key) == 0) {
      unsigned id = static_cast<unsigned>(fluent_names_.size());
      fluent_names_.push_back(key);
      fluent_index_[key] = id;
    }
    return;
  }
  std::vector<std::string> current;
  std::vector<std::vector<std::string>> combos;
  generateCombinations(types_, param_types, 0, current, combos);

  for (auto& args : combos) {
    std::string key = makeFluentKey(pred_name, args);
    if (fluent_index_.count(key) == 0) {
      unsigned id = static_cast<unsigned>(fluent_names_.size());
      fluent_names_.push_back(key);
      fluent_index_[key] = id;
    }
  }
}

void WorldModel::groundNewObject(const std::string& obj_name,
                 const std::string& obj_type) {
  (void)obj_name;
  (void)obj_type;
  // Re-ground all predicates (simple approach -- fine for small domains)
  // We skip duplicates inside groundPredicate
  for (auto& pred : predicates_) {
    groundPredicate(pred.name, pred.param_types);
  }
  // Re-ground all action schemas
  for (unsigned i = 0; i < action_schemas_.size(); ++i) {
    groundActionSchema(i);
  }
}

// Substitute parameter names with actual object names in a template string
// e.g. "(at ?r ?from)" with {?r->uav1, ?from->base} becomes "(at uav1 base)"
static std::string substituteTemplate(
  const std::string& tmpl,
  const std::vector<std::string>& param_names,
  const std::vector<std::string>& args)
{
  std::string result = tmpl;
  for (size_t i = 0; i < param_names.size(); ++i) {
    size_t pos = 0;
    while ((pos = result.find(param_names[i], pos)) != std::string::npos) {
      const size_t end = pos + param_names[i].size();
      if (end < result.size() && isParameterTokenChar(result[end])) {
        pos = end;
        continue;
      }
      result.replace(pos, param_names[i].size(), args[i]);
      pos += args[i].size();
    }
  }
  return result;
}

void WorldModel::groundActionSchema(unsigned schema_index) {
  auto& schema = action_schemas_[schema_index];
  auto& param_types = schema.schema.param_types;
  const auto& param_names = schema.schema.param_names;

  // Resolve an equality-constraint operand to a concrete object name: either
  // the value bound to a parameter, or a literal object name. Fails closed if
  // the operand is neither a parameter nor a known object.
  auto resolveOperand = [&](const std::string& token,
                            const std::vector<std::string>& args) -> std::string {
    for (size_t i = 0; i < param_names.size(); ++i) {
      if (param_names[i] == token) return args[i];
    }
    if (!types_.hasObject(token)) {
      throw std::runtime_error(
        "WorldModel::groundActionSchema: equality operand '" + token +
        "' in schema '" + schema.schema.name +
        "' is neither a parameter nor a known object");
    }
    return token;
  };

  std::vector<std::string> current;
  std::vector<std::vector<std::string>> combos;
  generateCombinations(types_, param_types, 0, current, combos);

  for (auto& args : combos) {
    // Apply equality/inequality binding filters: skip combinations that violate
    // any constraint. These never become fluents.
    bool constraints_ok = true;
    for (const auto& c : schema.equality_constraints) {
      const std::string lhs = resolveOperand(c.lhs, args);
      const std::string rhs = resolveOperand(c.rhs, args);
      if ((lhs == rhs) != c.must_equal) {
        constraints_ok = false;
        break;
      }
    }
    if (!constraints_ok) continue;

    // Build signature
    std::string sig = schema.schema.name + "(";
    for (size_t i = 0; i < args.size(); ++i) {
      if (i > 0) sig += ",";
      sig += args[i];
    }
    sig += ")";

    if (ground_action_signatures_.count(sig) != 0) {
      continue;
    }

    GroundAction ga;
    ga.signature = sig;
    ga.schema_index = schema_index;
    ga.args = args;

    // Resolve preconditions
    for (auto& tmpl : schema.precondition_templates) {
      std::string key = substituteTemplate(tmpl, schema.schema.param_names, args);
      requireGroundedTemplate(fluent_index_, schema.schema.name, tmpl, key,
                              ga.preconditions);
    }

    // Resolve negative preconditions (must resolve to existing fluent IDs;
    // fail closed exactly like positive preconditions and effects).
    for (auto& tmpl : schema.neg_precondition_templates) {
      std::string key = substituteTemplate(tmpl, schema.schema.param_names, args);
      requireGroundedTemplate(fluent_index_, schema.schema.name, tmpl, key,
                              ga.neg_preconditions);
    }

    // Resolve add effects
    for (auto& tmpl : schema.add_templates) {
      std::string key = substituteTemplate(tmpl, schema.schema.param_names, args);
      requireGroundedTemplate(fluent_index_, schema.schema.name, tmpl, key,
                              ga.add_effects);
    }

    // Resolve delete effects
    for (auto& tmpl : schema.del_templates) {
      std::string key = substituteTemplate(tmpl, schema.schema.param_names, args);
      requireGroundedTemplate(fluent_index_, schema.schema.name, tmpl, key,
                              ga.del_effects);
    }

    const auto inserted = ground_action_signatures_.insert(ga.signature);
    if (!inserted.second) {
      continue;
    }
    try {
      ground_actions_.push_back(std::move(ga));
    } catch (...) {
      ground_action_signatures_.erase(inserted.first);
      throw;
    }
  }
}

// =========================================================================
// LAPKT Projection
// =========================================================================

void WorldModel::projectToSTRIPS(aptk::STRIPS_Problem& prob) const {
  std::vector<unsigned> action_order(ground_actions_.size());
  for (unsigned i = 0; i < action_order.size(); ++i) {
    action_order[i] = i;
  }
  projectToSTRIPS(prob, action_order, goal_fluent_ids_);
}

void WorldModel::projectToSTRIPS(aptk::STRIPS_Problem& prob,
                                   const std::vector<unsigned>& action_order) const {
    projectToSTRIPS(prob, action_order, goal_fluent_ids_);
}

void WorldModel::projectToSTRIPS(aptk::STRIPS_Problem& prob,
                                   const std::vector<unsigned>& action_order,
                                   const std::vector<unsigned>& goal_ids) const {
    const unsigned n = static_cast<unsigned>(fluent_names_.size());

    // Add all normal AME fluents first; their LAPKT index equals the AME id.
    for (unsigned i = 0; i < n; ++i)
        aptk::STRIPS_Problem::add_fluent(prob, fluent_names_[i]);

    // Negative preconditions are compiled away with transient complement
    // fluents that exist ONLY inside this projection (never in the WorldModel).
    // The complement of fluent p lives at index n + p, so a single offset maps
    // any fluent to its complement. We only reserve them when at least one
    // grounded action actually has a negative precondition.
    bool needs_complements = false;
    for (unsigned idx : action_order) {
        if (idx >= ground_actions_.size()) continue;
        if (!ground_actions_[idx].neg_preconditions.empty()) {
            needs_complements = true;
            break;
        }
    }

    if (needs_complements) {
        for (unsigned i = 0; i < n; ++i) {
            // Name is for LAPKT diagnostics only.
            aptk::STRIPS_Problem::add_fluent(prob, "(not " + fluent_names_[i] + ")");
        }
    }

    aptk::Conditional_Effect_Vec no_ceffs;
    for (unsigned idx : action_order) {
        if (idx >= ground_actions_.size()) continue;
        const auto& ga = ground_actions_[idx];

        aptk::Fluent_Vec pre(ga.preconditions.begin(), ga.preconditions.end());
        // Negative precondition: require the transient complement fluent.
        if (needs_complements) {
            for (unsigned neg_id : ga.neg_preconditions)
                pre.push_back(n + neg_id);
        }

        aptk::Fluent_Vec add(ga.add_effects.begin(), ga.add_effects.end());
        aptk::Fluent_Vec del(ga.del_effects.begin(), ga.del_effects.end());
        if (needs_complements) {
            // add p  -> add p, delete not-p ; del p -> delete p, add not-p
            // so the complement always tracks the logical negation of p.
            for (unsigned add_id : ga.add_effects)
                del.push_back(n + add_id);
            for (unsigned del_id : ga.del_effects)
                add.push_back(n + del_id);
        }
        aptk::STRIPS_Problem::add_action(prob, ga.signature, pre, add, del, no_ceffs);
    }

    aptk::Fluent_Vec init;
    for (unsigned i = 0; i < n; ++i) {
        if (getFact(i)) {
            init.push_back(i);
        } else if (needs_complements) {
            // Omitted/false fact -> its transient complement is true.
            init.push_back(n + i);
        }
    }
    aptk::STRIPS_Problem::set_init(prob, init);

    aptk::Fluent_Vec goal(goal_ids.begin(), goal_ids.end());
    aptk::STRIPS_Problem::set_goal(prob, goal);

    prob.make_action_tables();
}

aptk::State* WorldModel::currentStateAsSTRIPS(const aptk::STRIPS_Problem& prob) const {
  auto* state = new aptk::State(prob);
  const unsigned n = static_cast<unsigned>(fluent_names_.size());
  // If the projection reserved transient complement fluents (2N total), the
  // initial state must set the complement for every false fact, mirroring
  // projectToSTRIPS::set_init.
  const bool has_complements = prob.num_fluents() >= 2u * n && n > 0;
  for (unsigned i = 0; i < n; ++i) {
    if (getFact(i)) {
      state->set(i);
    } else if (has_complements) {
      state->set(n + i);
    }
  }
  state->update_hash();
  return state;
}

// =========================================================================
// Agent Management
// =========================================================================

void WorldModel::registerAgent(const std::string& id, const std::string& type) {
  // Check for duplicate
  for (const auto& agent : agents_) {
    if (agent.id == id) {
      return; // Already registered
    }
  }
  agents_.push_back({id, type, true});
}

AgentInfo* WorldModel::getAgent(const std::string& id) {
  for (auto& agent : agents_) {
    if (agent.id == id) {
      return &agent;
    }
  }
  return nullptr;
}

const AgentInfo* WorldModel::getAgent(const std::string& id) const {
  for (const auto& agent : agents_) {
    if (agent.id == id) {
      return &agent;
    }
  }
  return nullptr;
}

std::vector<std::string> WorldModel::agentIds() const {
  std::vector<std::string> ids;
  ids.reserve(agents_.size());
  for (const auto& agent : agents_) {
    ids.push_back(agent.id);
  }
  return ids;
}

std::vector<std::string> WorldModel::availableAgentIds() const {
  std::vector<std::string> ids;
  for (const auto& agent : agents_) {
    if (agent.available) {
      ids.push_back(agent.id);
    }
  }
  return ids;
}

} // namespace ame
