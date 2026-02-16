#pragma once

#include "mujin/type_system.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

// Forward-declare LAPKT types to avoid header leak
namespace aptk {
class STRIPS_Problem;
class State;
}

namespace mujin {

// Action schema: describes a grounded PDDL action stored in the WorldModel
struct ActionSchema {
    std::string name;                    // e.g. "move"
    std::vector<std::string> param_names; // e.g. {"?r", "?from", "?to"}
    std::vector<std::string> param_types; // e.g. {"robot", "location", "location"}
};

// A fully grounded action instance
struct GroundAction {
    std::string signature;                // e.g. "move(uav1,base,sector_a)"
    unsigned schema_index;                // index into WorldModel::action_schemas_
    std::vector<std::string> args;        // e.g. {"uav1", "base", "sector_a"}
    std::vector<unsigned> preconditions;  // fluent IDs
    std::vector<unsigned> add_effects;    // fluent IDs
    std::vector<unsigned> del_effects;    // fluent IDs
};

class WorldModel {
public:
    TypeSystem& typeSystem() { return types_; }
    const TypeSystem& typeSystem() const { return types_; }

    // Predicate schema registration
    void registerPredicate(const std::string& name,
                           const std::vector<std::string>& param_types);

    // Object addition (triggers eager grounding)
    void addObject(const std::string& name, const std::string& type);

    // Action schema registration
    // precondition/add/del templates use fluent key templates with parameter names
    // e.g. "(at ?r ?from)" where ?r, ?from are parameter names from the schema
    void registerAction(const std::string& name,
                        const std::vector<std::string>& param_names,
                        const std::vector<std::string>& param_types,
                        const std::vector<std::string>& precondition_templates,
                        const std::vector<std::string>& add_templates,
                        const std::vector<std::string>& del_templates);

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

    // Ground actions
    const std::vector<GroundAction>& groundActions() const { return ground_actions_; }
    unsigned numGroundActions() const { return static_cast<unsigned>(ground_actions_.size()); }

    // Version counter (incremented on any state change)
    uint64_t version() const { return version_; }

    // LAPKT projection
    void projectToSTRIPS(aptk::STRIPS_Problem& prob) const;
    aptk::State* currentStateAsSTRIPS(const aptk::STRIPS_Problem& prob) const;

private:
    void groundPredicate(const std::string& pred_name,
                         const std::vector<std::string>& param_types);
    void groundNewObject(const std::string& obj_name,
                         const std::string& obj_type);
    void groundActionSchema(unsigned schema_index);

    TypeSystem types_;

    // Predicate schemas: name -> param types
    struct PredicateSchema {
        std::string name;
        std::vector<std::string> param_types;
    };
    std::vector<PredicateSchema> predicates_;

    // Action schemas and their grounded instances
    struct ActionSchemaInternal {
        ActionSchema schema;
        std::vector<std::string> precondition_templates;
        std::vector<std::string> add_templates;
        std::vector<std::string> del_templates;
    };
    std::vector<ActionSchemaInternal> action_schemas_;
    std::vector<GroundAction> ground_actions_;

    // Grounded fluent storage (bitset as vector<uint64_t>)
    std::vector<uint64_t> state_bits_;

    // BiMap: fluent index <-> string key
    std::vector<std::string> fluent_names_;           // index -> name
    std::unordered_map<std::string, unsigned> fluent_index_;  // name -> index

    // Goal state (fluent IDs that must be true)
    std::vector<unsigned> goal_fluent_ids_;

    uint64_t version_ = 0;

public:
    // Goal management
    void setGoal(const std::vector<std::string>& goal_fluent_keys);
    const std::vector<unsigned>& goalFluentIds() const { return goal_fluent_ids_; }
};

} // namespace mujin
