#include "mujin/world_model.h"
#include <algorithm>
#include <stdexcept>

namespace mujin {

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

void WorldModel::setFact(const std::string& key, bool value) {
    auto it = fluent_index_.find(key);
    if (it == fluent_index_.end()) {
        throw std::runtime_error("WorldModel::setFact: unknown fluent '" + key + "'");
    }
    setFact(it->second, value);
}

bool WorldModel::getFact(const std::string& key) const {
    auto it = fluent_index_.find(key);
    if (it == fluent_index_.end()) {
        throw std::runtime_error("WorldModel::getFact: unknown fluent '" + key + "'");
    }
    return getFact(it->second);
}

void WorldModel::setFact(unsigned id, bool value) {
    unsigned word = id / 64;
    unsigned bit = id % 64;
    if (word >= state_bits_.size()) {
        state_bits_.resize(word + 1, 0);
    }
    bool old_val = (state_bits_[word] >> bit) & 1u;
    if (old_val != value) {
        if (value) {
            state_bits_[word] |= (uint64_t(1) << bit);
        } else {
            state_bits_[word] &= ~(uint64_t(1) << bit);
        }
        ++version_;
    }
}

bool WorldModel::getFact(unsigned id) const {
    unsigned word = id / 64;
    unsigned bit = id % 64;
    if (word >= state_bits_.size()) return false;
    return (state_bits_[word] >> bit) & 1u;
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

void WorldModel::groundNewObject(const std::string& /*obj_name*/,
                                 const std::string& /*obj_type*/) {
    // Re-ground all predicates (simple approach — fine for small domains)
    // We skip duplicates inside groundPredicate
    for (auto& pred : predicates_) {
        groundPredicate(pred.name, pred.param_types);
    }
}

} // namespace mujin
