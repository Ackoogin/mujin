#include "ame/world_model.h"

#include <strips_prob.hxx>
#include <fluent.hxx>
#include <action.hxx>
#include <strips_state.hxx>

#include <algorithm>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>

namespace ame {

WorldModel::WorldModel(const WorldModel& other)
    : types_(other.types_),
      predicates_(other.predicates_),
      action_schemas_(other.action_schemas_),
      ground_actions_(other.ground_actions_),
      state_bits_(other.state_bits_),
      fact_metadata_(other.fact_metadata_),
      fluent_names_(other.fluent_names_),
      fluent_index_(other.fluent_index_),
      goal_fluent_ids_(other.goal_fluent_ids_),
      version_(other.version_),
      agents_(other.agents_) {
  // Note: mutexes are default-constructed (not copied)
  // audit_callback_ is not copied (intentional - caller should set if needed)
}

WorldModel& WorldModel::operator=(const WorldModel& other) {
  if (this != &other) {
    types_ = other.types_;
    predicates_ = other.predicates_;
    action_schemas_ = other.action_schemas_;
    ground_actions_ = other.ground_actions_;
    state_bits_ = other.state_bits_;
    fact_metadata_ = other.fact_metadata_;
    fluent_names_ = other.fluent_names_;
    fluent_index_ = other.fluent_index_;
    goal_fluent_ids_ = other.goal_fluent_ids_;
    version_ = other.version_;
    agents_ = other.agents_;
    // Note: mutexes are not copied, audit_callback_ is not copied
  }
  return *this;
}

void WorldModel::registerPredicate(const std::string& name,
                   const std::vector<std::string>& param_types) {
  predicates_.push_back({name, param_types});
  // Eagerly ground against all existing objects
  groundPredicate(name, param_types);
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
  ActionSchemaInternal schema;
  schema.schema.name = name;
  schema.schema.param_names = param_names;
  schema.schema.param_types = param_types;
  schema.precondition_templates = precondition_templates;
  schema.add_templates = add_templates;
  schema.del_templates = del_templates;

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
  auto ts = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();

  // Update metadata regardless of value change
  fact_metadata_[id].authority = authority;
  fact_metadata_[id].timestamp_us = static_cast<uint64_t>(ts);
  fact_metadata_[id].source = source;

  if (old_val != value) {
    if (value) {
      state_bits_[word] |= (uint64_t(1) << bit);
    } else {
      state_bits_[word] &= ~(uint64_t(1) << bit);
    }
    ++version_;
    if (audit_callback_) {
      audit_callback_(version_, static_cast<uint64_t>(ts),
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
}

// =========================================================================
// Grounding helpers
// =========================================================================

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
  auto objects = ts.getObjectsOfType(param_types[depth]);
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
  // Re-ground all predicates (simple approach — fine for small domains)
  // We skip duplicates inside groundPredicate
  for (auto& pred : predicates_) {
    groundPredicate(pred.name, pred.param_types);
  }
  // Re-ground all action schemas
  // Simple: clear and re-ground all (duplicates are avoided by signature check)
  ground_actions_.clear();
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
    // Replace all occurrences of param_names[i] with args[i]
    size_t pos = 0;
    while ((pos = result.find(param_names[i], pos)) != std::string::npos) {
      result.replace(pos, param_names[i].size(), args[i]);
      pos += args[i].size();
    }
  }
  return result;
}

void WorldModel::groundActionSchema(unsigned schema_index) {
  auto& schema = action_schemas_[schema_index];
  auto& param_types = schema.schema.param_types;

  std::vector<std::string> current;
  std::vector<std::vector<std::string>> combos;
  generateCombinations(types_, param_types, 0, current, combos);

  for (auto& args : combos) {
    // Build signature
    std::string sig = schema.schema.name + "(";
    for (size_t i = 0; i < args.size(); ++i) {
      if (i > 0) sig += ",";
      sig += args[i];
    }
    sig += ")";

    // Check for duplicate
    bool duplicate = false;
    for (auto& existing : ground_actions_) {
      if (existing.signature == sig) { duplicate = true; break; }
    }
    if (duplicate) continue;

    GroundAction ga;
    ga.signature = sig;
    ga.schema_index = schema_index;
    ga.args = args;

    // Resolve preconditions
    for (auto& tmpl : schema.precondition_templates) {
      std::string key = substituteTemplate(tmpl, schema.schema.param_names, args);
      auto it = fluent_index_.find(key);
      if (it != fluent_index_.end()) {
        ga.preconditions.push_back(it->second);
      }
    }

    // Resolve add effects
    for (auto& tmpl : schema.add_templates) {
      std::string key = substituteTemplate(tmpl, schema.schema.param_names, args);
      auto it = fluent_index_.find(key);
      if (it != fluent_index_.end()) {
        ga.add_effects.push_back(it->second);
      }
    }

    // Resolve delete effects
    for (auto& tmpl : schema.del_templates) {
      std::string key = substituteTemplate(tmpl, schema.schema.param_names, args);
      auto it = fluent_index_.find(key);
      if (it != fluent_index_.end()) {
        ga.del_effects.push_back(it->second);
      }
    }

    ground_actions_.push_back(std::move(ga));
  }
}

// =========================================================================
// LAPKT Projection
// =========================================================================

void WorldModel::projectToSTRIPS(aptk::STRIPS_Problem& prob) const {
  // Add fluents
  for (unsigned i = 0; i < fluent_names_.size(); ++i) {
    aptk::STRIPS_Problem::add_fluent(prob, fluent_names_[i]);
  }

  // Add actions
  aptk::Conditional_Effect_Vec no_ceffs;
  for (auto& ga : ground_actions_) {
    aptk::Fluent_Vec pre(ga.preconditions.begin(), ga.preconditions.end());
    aptk::Fluent_Vec add(ga.add_effects.begin(), ga.add_effects.end());
    aptk::Fluent_Vec del(ga.del_effects.begin(), ga.del_effects.end());

    aptk::STRIPS_Problem::add_action(prob, ga.signature, pre, add, del, no_ceffs);
  }

  // Set initial state (all currently-true fluents)
  aptk::Fluent_Vec init;
  for (unsigned i = 0; i < fluent_names_.size(); ++i) {
    if (getFact(i)) {
      init.push_back(i);
    }
  }
  aptk::STRIPS_Problem::set_init(prob, init);

  // Set goal
  aptk::Fluent_Vec goal(goal_fluent_ids_.begin(), goal_fluent_ids_.end());
  aptk::STRIPS_Problem::set_goal(prob, goal);

  // Finalize
  prob.make_action_tables();
}

aptk::State* WorldModel::currentStateAsSTRIPS(const aptk::STRIPS_Problem& prob) const {
  auto* state = new aptk::State(prob);
  for (unsigned i = 0; i < fluent_names_.size(); ++i) {
    if (getFact(i)) {
      state->set(i);
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
