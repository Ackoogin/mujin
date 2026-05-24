#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

struct TypeDef { std::string name, parent; };
struct Parameter { std::string name, type; };
struct PredicateDef { std::string name; std::vector<Parameter> params; float posX=0,posY=0; };
struct EffectRef { std::string predicateName; std::vector<std::string> argNames; };
struct ActionDef {
    std::string name;
    std::vector<Parameter> params;
    std::vector<EffectRef> preconditions, addEffects, delEffects;
    float posX=0,posY=0;
};
struct ObjectDef { std::string name, type; };
struct FactRef { std::string predicateName; std::vector<std::string> objectNames; };
struct ScenarioDef { std::string name; std::vector<FactRef> initialState, goals; };

struct ProjectModel {
    int version = 1;
    std::string projectName = "[Untitled]";
    std::vector<TypeDef> types;
    std::vector<PredicateDef> predicates;
    std::vector<ActionDef> actions;
    std::vector<ObjectDef> objects;
    std::vector<ScenarioDef> scenarios;

    bool save(const std::string& path) const;
    bool load(const std::string& path);
    void clear();
};

void to_json(nlohmann::json&, const TypeDef&);
void from_json(const nlohmann::json&, TypeDef&);
void to_json(nlohmann::json&, const Parameter&);
void from_json(const nlohmann::json&, Parameter&);
void to_json(nlohmann::json&, const PredicateDef&);
void from_json(const nlohmann::json&, PredicateDef&);
void to_json(nlohmann::json&, const EffectRef&);
void from_json(const nlohmann::json&, EffectRef&);
void to_json(nlohmann::json&, const ActionDef&);
void from_json(const nlohmann::json&, ActionDef&);
void to_json(nlohmann::json&, const ObjectDef&);
void from_json(const nlohmann::json&, ObjectDef&);
void to_json(nlohmann::json&, const FactRef&);
void from_json(const nlohmann::json&, FactRef&);
void to_json(nlohmann::json&, const ScenarioDef&);
void from_json(const nlohmann::json&, ScenarioDef&);
void to_json(nlohmann::json&, const ProjectModel&);
void from_json(const nlohmann::json&, ProjectModel&);
