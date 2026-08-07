#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

// Resolved BT implementation for a grounded action
struct ActionImpl {
    std::string xml;           // Resolved BT XML fragment (subtree or single node)
    std::string node_type;     // Registered node type for a simple implementation
    bool is_subtree = false;   // True when xml came from a subtree template
    bool reactive = false;     // Recheck preconditions while the action is running
    std::vector<std::string> param_bindings; // Actual parameter values
};

class ActionRegistry {
public:
    // Register a simple BT node type for a PDDL action
    void registerAction(const std::string& pddl_name,
                        const std::string& bt_node_type,
                        bool reactive = false);

    // Register a subtree XML template with {param0}, {param1}, ... placeholders
    void registerActionSubTree(const std::string& pddl_name,
                               const std::string& subtree_xml_template,
                               bool reactive = false);

    // Resolve: given an action name and parameter values, produce the ActionImpl
    ActionImpl resolve(const std::string& action_name,
                       const std::vector<std::string>& params) const;

    // Check if an action is registered
    bool hasAction(const std::string& pddl_name) const;

    // Return all registered PDDL action names.
    std::vector<std::string> registeredNames() const;

    // Get the reactive flag for an action
    bool isReactive(const std::string& pddl_name) const;

private:
    struct Registration {
        enum Kind { SimpleNode, SubTreeTemplate };
        Kind kind = SimpleNode;
        std::string bt_node_type;         // for SimpleNode
        std::string xml_template;         // for SubTreeTemplate
        bool reactive = false;
    };

    std::unordered_map<std::string, Registration> registry_;
};

} // namespace ame
