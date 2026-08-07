#include "ame/type_system.h"
#include <stdexcept>
#include <unordered_set>

namespace ame {

void TypeSystem::addType(const std::string& name, const std::string& parent) {
    types_[name] = parent;
}

bool TypeSystem::hasType(const std::string& name) const {
    return types_.count(name) > 0;
}

bool TypeSystem::isSubtype(const std::string& child, const std::string& parent) const {
    std::unordered_set<std::string> visited;
    std::string current = child;
    while (true) {
        if (current == parent) return true;
        if (!visited.insert(current).second) {
            throw std::runtime_error("TypeSystem::isSubtype: cyclic type hierarchy at '" +
                                     current + "'");
        }
        auto it = types_.find(current);
        if (it == types_.end() || it->second.empty()) return false;
        current = it->second;
    }
}

std::vector<std::string> TypeSystem::getObjectsOfType(const std::string& type) const {
    std::vector<std::string> result;
    for (auto& [obj, obj_type] : objects_) {
        if (isSubtype(obj_type, type)) {
            result.push_back(obj);
        }
    }
    return result;
}

void TypeSystem::addObject(const std::string& name, const std::string& type) {
    if (!hasType(type)) {
        throw std::runtime_error("TypeSystem::addObject: unknown type '" + type + "'");
    }
    const bool is_new_object = !hasObject(name);
    objects_[name] = type;
    if (is_new_object) {
        object_order_.push_back(name);
    }
}

bool TypeSystem::hasObject(const std::string& name) const {
    return objects_.count(name) > 0;
}

std::string TypeSystem::getObjectType(const std::string& name) const {
    auto it = objects_.find(name);
    if (it == objects_.end()) {
        throw std::runtime_error("TypeSystem::getObjectType: unknown object '" + name + "'");
    }
    return it->second;
}

} // namespace ame
