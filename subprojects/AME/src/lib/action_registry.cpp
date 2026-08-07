#include "ame/action_registry.h"
#include "ame/detail/escape.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <tinyxml2.h>

namespace ame {

static std::string substituteEscapedParams(const std::string& xml_template,
                                           const std::vector<std::string>& params) {
    std::string xml = xml_template;
    for (size_t i = 0; i < params.size(); ++i) {
        const std::string placeholder = "{param" + std::to_string(i) + "}";
        const std::string replacement = detail::xmlAttrEscape(params[i]);
        size_t pos = 0;
        while ((pos = xml.find(placeholder, pos)) != std::string::npos) {
            xml.replace(pos, placeholder.size(), replacement);
            pos += replacement.size();
        }
    }
    return xml;
}

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

void ActionRegistry::registerActionFile(const std::string& pddl_name,
                                        const std::string& path,
                                        bool reactive) {
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input) {
        throw std::runtime_error("ActionRegistry::registerActionFile: failed to open file '" +
                                 path + "' for action '" + pddl_name + "'");
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    if (input.bad()) {
        throw std::runtime_error("ActionRegistry::registerActionFile: failed to read file '" +
                                 path + "' for action '" + pddl_name + "'");
    }

    const std::string xml = buffer.str();
    tinyxml2::XMLDocument doc;
    const auto parse_result = doc.Parse(xml.c_str(), xml.size());
    if (parse_result != tinyxml2::XML_SUCCESS || doc.RootElement() == nullptr) {
        throw std::runtime_error("ActionRegistry::registerActionFile: invalid XML in file '" +
                                 path + "' for action '" + pddl_name + "': " +
                                 doc.ErrorStr());
    }

    Registration reg;
    reg.kind = Registration::PreAuthoredFile;
    reg.xml_template = xml;
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
        impl.node_type = it->second.bt_node_type;
        impl.is_subtree = false;
        // Emit a simple node XML: <NodeType param0="val0" param1="val1" .../>
        std::string xml = "<" + it->second.bt_node_type;
        for (size_t i = 0; i < params.size(); ++i) {
            xml += " param" + std::to_string(i) + "=\"" +
                   detail::xmlAttrEscape(params[i]) + "\"";
        }
        xml += "/>";
        impl.xml = xml;
    } else {
        impl.is_subtree = true;
        // Substitute {param0}, {param1}, ... in the template
        impl.xml = substituteEscapedParams(it->second.xml_template, params);
    }

    return impl;
}

bool ActionRegistry::hasAction(const std::string& pddl_name) const {
    return registry_.count(pddl_name) > 0;
}

std::vector<std::string> ActionRegistry::registeredNames() const {
    std::vector<std::string> names;
    names.reserve(registry_.size());
    for (const auto& entry : registry_) {
        names.push_back(entry.first);
    }
    std::sort(names.begin(), names.end());
    return names;
}

bool ActionRegistry::isReactive(const std::string& pddl_name) const {
    auto it = registry_.find(pddl_name);
    if (it == registry_.end()) {
        throw std::runtime_error("ActionRegistry::isReactive: unknown action '" + pddl_name + "'");
    }
    return it->second.reactive;
}

} // namespace ame
