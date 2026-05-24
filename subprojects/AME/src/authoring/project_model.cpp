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

void to_json(nlohmann::json& j, const CausalLink& value) {
    j = nlohmann::json{
        {"fromAction", value.fromAction},
        {"fromAddEffectIdx", value.fromAddEffectIdx},
        {"toAction", value.toAction},
        {"toPreconditionIdx", value.toPreconditionIdx},
    };
}

void from_json(const nlohmann::json& j, CausalLink& value) {
    j.at("fromAction").get_to(value.fromAction);
    j.at("fromAddEffectIdx").get_to(value.fromAddEffectIdx);
    j.at("toAction").get_to(value.toAction);
    j.at("toPreconditionIdx").get_to(value.toPreconditionIdx);
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
        {"causalLinks", value.causalLinks},
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
    value.causalLinks.clear();
    if (j.contains("causalLinks")) {
        j.at("causalLinks").get_to(value.causalLinks);
    }
    j.at("objects").get_to(value.objects);
    j.at("scenarios").get_to(value.scenarios);
}

bool causalLinkCompatible(const ProjectModel& model, const CausalLink& link) {
    if (link.fromAction < 0 ||
        link.toAction < 0 ||
        link.fromAddEffectIdx < 0 ||
        link.toPreconditionIdx < 0) {
        return false;
    }
    if (link.fromAction == link.toAction) {
        return false;
    }
    if (link.fromAction >= static_cast<int>(model.actions.size()) ||
        link.toAction >= static_cast<int>(model.actions.size())) {
        return false;
    }

    const ActionDef& fromAction = model.actions[static_cast<size_t>(link.fromAction)];
    const ActionDef& toAction = model.actions[static_cast<size_t>(link.toAction)];
    if (link.fromAddEffectIdx >= static_cast<int>(fromAction.addEffects.size()) ||
        link.toPreconditionIdx >= static_cast<int>(toAction.preconditions.size())) {
        return false;
    }

    const EffectRef& effect = fromAction.addEffects[static_cast<size_t>(link.fromAddEffectIdx)];
    const EffectRef& precondition =
        toAction.preconditions[static_cast<size_t>(link.toPreconditionIdx)];
    return effect.predicateName == precondition.predicateName &&
           effect.argNames.size() == precondition.argNames.size();
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
