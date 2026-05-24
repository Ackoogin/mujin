#include "project_model.h"

#include <fstream>

void to_json(nlohmann::json& j, const TypeDef& value) {
    j = nlohmann::json{{"name", value.name}, {"parent", value.parent}};
}

void from_json(const nlohmann::json& j, TypeDef& value) {
    j.at("name").get_to(value.name);
    j.at("parent").get_to(value.parent);
}

void to_json(nlohmann::json& j, const Parameter& value) {
    j = nlohmann::json{{"name", value.name}, {"type", value.type}};
}

void from_json(const nlohmann::json& j, Parameter& value) {
    j.at("name").get_to(value.name);
    j.at("type").get_to(value.type);
}

void to_json(nlohmann::json& j, const PredicateDef& value) {
    j = nlohmann::json{
        {"name", value.name},
        {"params", value.params},
        {"posX", value.posX},
        {"posY", value.posY},
    };
}

void from_json(const nlohmann::json& j, PredicateDef& value) {
    j.at("name").get_to(value.name);
    j.at("params").get_to(value.params);
    j.at("posX").get_to(value.posX);
    j.at("posY").get_to(value.posY);
}

void to_json(nlohmann::json& j, const EffectRef& value) {
    j = nlohmann::json{{"predicateName", value.predicateName}, {"argNames", value.argNames}};
}

void from_json(const nlohmann::json& j, EffectRef& value) {
    j.at("predicateName").get_to(value.predicateName);
    j.at("argNames").get_to(value.argNames);
}

void to_json(nlohmann::json& j, const ActionDef& value) {
    j = nlohmann::json{
        {"name", value.name},
        {"params", value.params},
        {"preconditions", value.preconditions},
        {"addEffects", value.addEffects},
        {"delEffects", value.delEffects},
        {"posX", value.posX},
        {"posY", value.posY},
    };
}

void from_json(const nlohmann::json& j, ActionDef& value) {
    j.at("name").get_to(value.name);
    j.at("params").get_to(value.params);
    j.at("preconditions").get_to(value.preconditions);
    j.at("addEffects").get_to(value.addEffects);
    j.at("delEffects").get_to(value.delEffects);
    j.at("posX").get_to(value.posX);
    j.at("posY").get_to(value.posY);
}

void to_json(nlohmann::json& j, const ObjectDef& value) {
    j = nlohmann::json{{"name", value.name}, {"type", value.type}};
}

void from_json(const nlohmann::json& j, ObjectDef& value) {
    j.at("name").get_to(value.name);
    j.at("type").get_to(value.type);
}

void to_json(nlohmann::json& j, const FactRef& value) {
    j = nlohmann::json{{"predicateName", value.predicateName}, {"objectNames", value.objectNames}};
}

void from_json(const nlohmann::json& j, FactRef& value) {
    j.at("predicateName").get_to(value.predicateName);
    j.at("objectNames").get_to(value.objectNames);
}

void to_json(nlohmann::json& j, const ScenarioDef& value) {
    j = nlohmann::json{
        {"name", value.name},
        {"initialState", value.initialState},
        {"goals", value.goals},
    };
}

void from_json(const nlohmann::json& j, ScenarioDef& value) {
    j.at("name").get_to(value.name);
    j.at("initialState").get_to(value.initialState);
    j.at("goals").get_to(value.goals);
}

void to_json(nlohmann::json& j, const ProjectModel& value) {
    j = nlohmann::json{
        {"version", value.version},
        {"projectName", value.projectName},
        {"types", value.types},
        {"predicates", value.predicates},
        {"actions", value.actions},
        {"objects", value.objects},
        {"scenarios", value.scenarios},
    };
}

void from_json(const nlohmann::json& j, ProjectModel& value) {
    j.at("version").get_to(value.version);
    j.at("projectName").get_to(value.projectName);
    j.at("types").get_to(value.types);
    j.at("predicates").get_to(value.predicates);
    j.at("actions").get_to(value.actions);
    j.at("objects").get_to(value.objects);
    j.at("scenarios").get_to(value.scenarios);
}

bool ProjectModel::save(const std::string& path) const {
    std::fstream file(path, std::ios::out);
    if (!file.is_open()) {
        return false;
    }

    file << nlohmann::json(*this).dump(2);
    return static_cast<bool>(file);
}

bool ProjectModel::load(const std::string& path) {
    try {
        std::fstream file(path, std::ios::in);
        if (!file.is_open()) {
            return false;
        }

        nlohmann::json j;
        file >> j;
        *this = j.get<ProjectModel>();
        return version == 1;
    } catch (...) {
        return false;
    }
}

void ProjectModel::clear() {
    *this = ProjectModel{};
}
