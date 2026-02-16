#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace mujin {

class TypeSystem {
public:
    void addType(const std::string& name, const std::string& parent = "");
    bool hasType(const std::string& name) const;
    bool isSubtype(const std::string& child, const std::string& parent) const;
    std::vector<std::string> getObjectsOfType(const std::string& type) const;

    void addObject(const std::string& name, const std::string& type);
    bool hasObject(const std::string& name) const;
    std::string getObjectType(const std::string& name) const;
    const std::vector<std::string>& getAllObjects() const { return object_order_; }

private:
    // type_name -> parent_type (empty string for root types)
    std::unordered_map<std::string, std::string> types_;
    // object_name -> type_name
    std::unordered_map<std::string, std::string> objects_;
    // insertion-ordered object list
    std::vector<std::string> object_order_;
};

} // namespace mujin
