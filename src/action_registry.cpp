#include "mujin/action_registry.h"

#include <stdexcept>

namespace mujin {

void ActionRegistry::registerAction(const std::string& pddl_name,
                                    const std::string& bt_node_type,
                                    bool reactive) {
    Registration reg;
    reg.kind = Registration::SimpleNode;
    reg.bt_node_type = bt_node_type;
    reg.reactive = reactive;
    registry_[pddl_name] = std::move(reg);
}

void ActionRegistry::registerActionSubTree(const std::string& pddl_name,
                                           const std::string& subtree_xml_template,
                                           bool reactive) {
    Registration reg;
    reg.kind = Registration::SubTreeTemplate;
    reg.xml_template = subtree_xml_template;
    reg.reactive = reactive;
    registry_[pddl_name] = std::move(reg);
}

ActionImpl ActionRegistry::resolve(const std::string& action_name,
                                   const std::vector<std::string>& params) const {
    auto it = registry_.find(action_name);
    if (it == registry_.end()) {
        throw std::runtime_error("ActionRegistry::resolve: unknown action '" + action_name + "'");
    }

    ActionImpl impl;
    impl.reactive = it->second.reactive;
    impl.param_bindings = params;

    if (it->second.kind == Registration::SimpleNode) {
        // Emit a simple node XML: <NodeType param0="val0" param1="val1" .../>
        std::string xml = "<" + it->second.bt_node_type;
        for (size_t i = 0; i < params.size(); ++i) {
            xml += " param" + std::to_string(i) + "=\"" + params[i] + "\"";
        }
        xml += "/>";
        impl.xml = xml;
    } else {
        // Substitute {param0}, {param1}, ... in the template
        std::string xml = it->second.xml_template;
        for (size_t i = 0; i < params.size(); ++i) {
            std::string placeholder = "{param" + std::to_string(i) + "}";
            size_t pos = 0;
            while ((pos = xml.find(placeholder, pos)) != std::string::npos) {
                xml.replace(pos, placeholder.size(), params[i]);
                pos += params[i].size();
            }
        }
        impl.xml = xml;
    }

    return impl;
}

bool ActionRegistry::hasAction(const std::string& pddl_name) const {
    return registry_.count(pddl_name) > 0;
}

bool ActionRegistry::isReactive(const std::string& pddl_name) const {
    auto it = registry_.find(pddl_name);
    if (it == registry_.end()) {
        throw std::runtime_error("ActionRegistry::isReactive: unknown action '" + pddl_name + "'");
    }
    return it->second.reactive;
}

} // namespace mujin
