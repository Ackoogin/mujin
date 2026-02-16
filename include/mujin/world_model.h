#pragma once

#include "mujin/type_system.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace mujin {

class WorldModel {
public:
    TypeSystem& typeSystem() { return types_; }
    const TypeSystem& typeSystem() const { return types_; }

    // Predicate schema registration
    void registerPredicate(const std::string& name,
                           const std::vector<std::string>& param_types);

    // Object addition (triggers eager grounding)
    void addObject(const std::string& name, const std::string& type);

    // Fact access by string key (e.g. "(at robot base)")
    void setFact(const std::string& key, bool value);
    bool getFact(const std::string& key) const;

    // Fact access by fluent index
    void setFact(unsigned id, bool value);
    bool getFact(unsigned id) const;

    // Fluent index <-> string key mapping
    unsigned fluentIndex(const std::string& key) const;
    const std::string& fluentName(unsigned id) const;
    unsigned numFluents() const { return static_cast<unsigned>(fluent_names_.size()); }

    // Version counter (incremented on any state change)
    uint64_t version() const { return version_; }

private:
    void groundPredicate(const std::string& pred_name,
                         const std::vector<std::string>& param_types);
    void groundNewObject(const std::string& obj_name,
                         const std::string& obj_type);

    TypeSystem types_;

    // Predicate schemas: name -> param types
    struct PredicateSchema {
        std::string name;
        std::vector<std::string> param_types;
    };
    std::vector<PredicateSchema> predicates_;

    // Grounded fluent storage (bitset as vector<uint64_t>)
    std::vector<uint64_t> state_bits_;

    // BiMap: fluent index <-> string key
    std::vector<std::string> fluent_names_;           // index -> name
    std::unordered_map<std::string, unsigned> fluent_index_;  // name -> index

    uint64_t version_ = 0;
};

} // namespace mujin
