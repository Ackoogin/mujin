#include "mujin/type_system.h"
#include <stdexcept>

namespace mujin {

void TypeSystem::addType(const std::string& name, const std::string& parent) {
    types_[name] = parent;
}

bool TypeSystem::hasType(const std::string& name) const {
    return types_.count(name) > 0;
}

bool TypeSystem::isSubtype(const std::string& child, const std::string& parent) const {
    if (child == parent) return true;
    auto it = types_.find(child);
    if (it == types_.end() || it->second.empty()) return false;
    return isSubtype(it->second, parent);
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
    objects_[name] = type;
    object_order_.push_back(name);
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

} // namespace mujin
