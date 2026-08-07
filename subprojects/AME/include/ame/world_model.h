#pragma once

#include "ame/fact_authority.h"
#include "ame/type_system.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward-declare LAPKT types to avoid header leak
namespace aptk {
class STRIPS_Problem;
class State;
}

namespace ame {

/// Metadata associated with each fact.
struct FactMetadata {
    FactAuthority authority = FactAuthority::BELIEVED;
    uint64_t timestamp_us = 0;   ///< Microseconds since epoch when last set
    std::string source;          ///< Originator identifier
};

/// Agent information for multi-agent planning.
struct AgentInfo {
    std::string id;           ///< Agent identifier, e.g., "uav1"
    std::string type;         ///< Agent type, e.g., "uav"
    bool available = true;    ///< Whether agent can accept new tasks
};

/// Immutable snapshot of world state for thread-safe reads.
/// BT executor captures a snapshot at tick start and reads from it.
class WorldStateData {
public:
    std::vector<uint64_t> state_bits;
    std::vector<FactMetadata> fact_metadata;
    uint64_t version = 0;

    bool getFact(unsigned id) const {
        unsigned word = id / 64;
        unsigned bit = id % 64;
        if (word >= state_bits.size()) return false;
        return (state_bits[word] >> bit) & 1u;
    }

    const FactMetadata& getMetadata(unsigned id) const {
        static const FactMetadata empty{};
        if (id >= fact_metadata.size()) return empty;
        return fact_metadata[id];
    }
};

using WorldStateSnapshotPtr = std::shared_ptr<const WorldStateData>;

/// A pending fact mutation queued for batch application.
struct PendingMutation {
    unsigned fluent_id;
    bool value;
    std::string source;
    FactAuthority authority;
    uint64_t timestamp_us;
};

// Action schema: describes a grounded PDDL action stored in the WorldModel
struct ActionSchema {
    std::string name;                    // e.g. "move"
    std::vector<std::string> param_names; // e.g. {"?r", "?from", "?to"}
    std::vector<std::string> param_types; // e.g. {"robot", "location", "location"}
};

// A grounding-time binding filter: (= a b) or (not (= a b)).
// Each side is either a parameter name (e.g. "?from") or a literal object name.
// Constraints never become fluents; they only prune which parameter
// combinations are grounded into actions.
struct EqualityConstraint {
    std::string lhs;
    std::string rhs;
    bool must_equal;   // true for (= a b); false for (not (= a b))
};

// A fully grounded action instance
struct GroundAction {
    std::string signature;                  // e.g. "move(uav1,base,sector_a)"
    unsigned schema_index;                  // index into WorldModel::action_schemas_
    std::vector<std::string> args;          // e.g. {"uav1", "base", "sector_a"}
    std::vector<unsigned> preconditions;    // fluent IDs that must be true
    std::vector<unsigned> neg_preconditions;// fluent IDs that must be false
    std::vector<unsigned> add_effects;      // fluent IDs
    std::vector<unsigned> del_effects;      // fluent IDs
};

class WorldModel {
public:
    WorldModel() = default;
    ~WorldModel() = default;

    /// Copy constructor: creates independent copy with fresh mutexes.
    WorldModel(const WorldModel& other);
    WorldModel& operator=(const WorldModel& other);

    /// Move constructor: transfers state into fresh mutexes.
    WorldModel(WorldModel&& other);
    WorldModel& operator=(WorldModel&& other);

    TypeSystem& typeSystem() { return types_; }
    const TypeSystem& typeSystem() const { return types_; }

    // Predicate schema registration
    void registerPredicate(const std::string& name,
                           const std::vector<std::string>& param_types);

    /// \brief Declare a predicate whose facts only count when observed.
    ///
    /// A precondition on such a predicate is satisfied by a CONFIRMED fact
    /// alone; a value predicted by a plan effect does not satisfy it. This is
    /// how a domain states that a gate, a clearance or any other
    /// evidence-bearing condition must be established outside the plan. The
    /// declaration is per predicate rather than per action, so an action can
    /// mix predicted and observed preconditions.
    void registerConfirmedPredicate(const std::string& name);

    /// \brief Whether a predicate was declared confirmed-only.
    bool isConfirmedPredicate(const std::string& name) const;

    /// \brief Whether a grounded fact key, e.g. "(authorised strike-gate)",
    /// names a confirmed-only predicate.
    bool isConfirmedFact(const std::string& fact_key) const;

    const std::set<std::string>& confirmedPredicates() const {
        return confirmed_predicates_;
    }

    // Object addition (triggers eager grounding)
    void addObject(const std::string& name, const std::string& type);

    // Action schema registration
    // precondition/add/del templates use fluent key templates with parameter names
    // e.g. "(at ?r ?from)" where ?r, ?from are parameter names from the schema
    //
    // Compatibility overload: forwards an empty negative-precondition list to the
    // signed overload below. Existing callers that only have positive
    // preconditions need no change.
    void registerAction(const std::string& name,
                        const std::vector<std::string>& param_names,
                        const std::vector<std::string>& param_types,
                        const std::vector<std::string>& precondition_templates,
                        const std::vector<std::string>& add_templates,
                        const std::vector<std::string>& del_templates);

    // Signed overload: negative preconditions are tracked separately. Each
    // neg_precondition_template is a fluent key template (same form as the
    // positive templates) that must be FALSE for the grounded action to apply.
    // They are never materialised as persistent complement facts in the
    // WorldModel; they are compiled away only at the LAPKT projection boundary.
    void registerAction(const std::string& name,
                        const std::vector<std::string>& param_names,
                        const std::vector<std::string>& param_types,
                        const std::vector<std::string>& precondition_templates,
                        const std::vector<std::string>& neg_precondition_templates,
                        const std::vector<std::string>& add_templates,
                        const std::vector<std::string>& del_templates);

    // Full overload: additionally takes grounding-time equality/inequality
    // binding filters. A parameter type may be a union encoded as
    // "typeA|typeB" (from an `(either ...)` declaration), in which case the
    // parameter ranges over the deduplicated union of objects of those types.
    // The other overloads forward empty constraint lists to this one.
    void registerAction(const std::string& name,
                        const std::vector<std::string>& param_names,
                        const std::vector<std::string>& param_types,
                        const std::vector<std::string>& precondition_templates,
                        const std::vector<std::string>& neg_precondition_templates,
                        const std::vector<std::string>& add_templates,
                        const std::vector<std::string>& del_templates,
                        const std::vector<EqualityConstraint>& equality_constraints);

    // Fact access by string key (e.g. "(at robot base)")
    // The optional source tag identifies the originator (e.g. "PlannedAction:node_name").
    // authority: BELIEVED for plan effects, CONFIRMED for perception-sourced facts.
    void setFact(const std::string& key, bool value,
                 const std::string& source = "",
                 FactAuthority authority = FactAuthority::BELIEVED);
    void setFact(const std::string& key, bool value,
                 const std::string& source,
                 FactAuthority authority,
                 uint64_t timestamp_us);
    bool getFact(const std::string& key) const;
    FactMetadata getFactMetadata(const std::string& key) const;

    // Fact access by fluent index
    void setFact(unsigned id, bool value,
                 const std::string& source = "",
                 FactAuthority authority = FactAuthority::BELIEVED);
    bool getFact(unsigned id) const;
    FactMetadata getFactMetadata(unsigned id) const;

    /// Capture an immutable snapshot of the current state for thread-safe reads.
    /// BT executor should call this at tick start.
    WorldStateSnapshotPtr captureSnapshot() const;

    /// Check if a CONFIRMED fact conflicts with current BELIEVED value.
    /// Returns true if there's a conflict (perception disagrees with plan prediction).
    bool hasAuthorityConflict(unsigned id, bool perceived_value) const;

    /// Queue a mutation for deferred application (thread-safe).
    /// Use this from perception callbacks to batch updates between BT ticks.
    void enqueueMutation(unsigned id, bool value,
                         const std::string& source,
                         FactAuthority authority = FactAuthority::CONFIRMED);

    /// Apply all queued mutations atomically. Call this between BT ticks.
    /// Returns number of mutations applied.
    size_t applyQueuedMutations();

    /// Check if there are pending mutations.
    bool hasPendingMutations() const;

    // Fluent index <-> string key mapping
    unsigned fluentIndex(const std::string& key) const;
    const std::string& fluentName(unsigned id) const;
    unsigned numFluents() const { return static_cast<unsigned>(fluent_names_.size()); }

    // Ground actions
    const std::vector<GroundAction>& groundActions() const { return ground_actions_; }
    unsigned numGroundActions() const { return static_cast<unsigned>(ground_actions_.size()); }

    // Version counter (incremented on any state change)
    uint64_t version() const { return version_; }

    // Audit callback: fired on every fact state change.
    // Parameters: (wm_version, timestamp_us, fact_name, value, source)
    using AuditCallback = std::function<void(uint64_t, uint64_t,
                                             const std::string&, bool,
                                             const std::string&)>;
    void setAuditCallback(AuditCallback cb);

    // LAPKT projection
    void projectToSTRIPS(aptk::STRIPS_Problem& prob) const;
    // Overload: project actions in a caller-specified order (permutation of [0,N)).
    // Allows heuristic hooks to bias LAPKT action traversal without mutating the WorldModel.
    void projectToSTRIPS(aptk::STRIPS_Problem& prob,
                         const std::vector<unsigned>& action_order) const;
    /// \brief Project using caller-supplied action order and goal fluent IDs.
    void projectToSTRIPS(aptk::STRIPS_Problem& prob,
                         const std::vector<unsigned>& action_order,
                         const std::vector<unsigned>& goal_ids) const;
    aptk::State* currentStateAsSTRIPS(const aptk::STRIPS_Problem& prob) const;

private:
    void setFactAtTimestamp(unsigned id, bool value,
                            const std::string& source,
                            FactAuthority authority,
                            uint64_t timestamp_us);
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

    // Predicates whose facts only satisfy a precondition when observed.
    std::set<std::string> confirmed_predicates_;

    // Action schemas and their grounded instances
    struct ActionSchemaInternal {
        ActionSchema schema;
        std::vector<std::string> precondition_templates;
        std::vector<std::string> neg_precondition_templates;
        std::vector<std::string> add_templates;
        std::vector<std::string> del_templates;
        std::vector<EqualityConstraint> equality_constraints;
    };
    std::vector<ActionSchemaInternal> action_schemas_;
    std::vector<GroundAction> ground_actions_;
    std::unordered_set<std::string> ground_action_signatures_;

    // Grounded fluent storage (bitset as vector<uint64_t>)
    std::vector<uint64_t> state_bits_;
    std::vector<FactMetadata> fact_metadata_;

    // Thread safety: protects state_bits_ and fact_metadata_
    mutable std::shared_mutex state_mutex_;

    // BiMap: fluent index <-> string key
    std::vector<std::string> fluent_names_;           // index -> name
    std::unordered_map<std::string, unsigned> fluent_index_;  // name -> index

    // Goal state (fluent IDs that must be true). goal_fluent_ids_ mirrors the
    // primary (first) alternative for single-goal callers.
    std::vector<unsigned> goal_fluent_ids_;
    std::vector<std::vector<unsigned>> goal_alternatives_;

    uint64_t version_ = 0;
    AuditCallback audit_callback_;

    // Mutation queue for batched perception updates
    mutable std::mutex mutation_queue_mutex_;
    std::vector<PendingMutation> mutation_queue_;

public:
    // Goal management
    void setGoal(const std::vector<std::string>& goal_fluent_keys);
    const std::vector<unsigned>& goalFluentIds() const { return goal_fluent_ids_; }

    // Disjunctive goals: a goal `(or A B)` lowers to a set of positive goal
    // alternatives, any one of which counts as success. setGoal stores a single
    // alternative; goalFluentIds() always returns the primary (first)
    // alternative for callers that expect one conjunction.
    void setGoalAlternatives(
        const std::vector<std::vector<std::string>>& goal_alternatives);
    const std::vector<std::vector<unsigned>>& goalAlternatives() const {
        return goal_alternatives_;
    }

    // Agent management for multi-agent planning
    void registerAgent(const std::string& id, const std::string& type);
    const std::vector<AgentInfo>& agents() const { return agents_; }
    AgentInfo* getAgent(const std::string& id);
    const AgentInfo* getAgent(const std::string& id) const;
    std::vector<std::string> agentIds() const;
    std::vector<std::string> availableAgentIds() const;
    size_t numAgents() const { return agents_.size(); }

private:
    std::vector<AgentInfo> agents_;
};

} // namespace ame
