#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

struct TypeDef { std::string name, parent; };
struct Parameter { std::string name, type; };
// `confirmed` marks a fact that an action may only act on once it has been
// observed. It becomes an entry in the domain's (:confirmed-predicates ...)
// section, which the plan compiler turns into a confirmed precondition. It is
// declared last so that existing brace initialisers, which supply name, params
// and the two canvas coordinates, keep compiling.
struct PredicateDef {
    std::string name;
    std::vector<Parameter> params;
    float posX=0,posY=0;
    bool confirmed=false;
};
struct EffectRef { std::string predicateName; std::vector<std::string> argNames; };
struct BtBinding {
    std::string nodeType;
    std::string subtreeXml;
    bool reactive = false;
};
struct ActionDef {
    std::string name;
    std::vector<Parameter> params;
    std::vector<EffectRef> preconditions, addEffects, delEffects;
    float posX=0,posY=0;
    BtBinding btBinding;
};
struct StateGroupDef {
    std::string name;
    std::string type;
    std::vector<std::string> predicateNames;
};
struct ObjectDef { std::string name, type; };
struct FactRef { std::string predicateName; std::vector<std::string> objectNames; };
struct ScenarioExpectation {
    bool shouldSucceed = true;
    int minPlanSteps = 0;
    int maxPlanSteps = 0;
    std::vector<std::string> expectedActions;
    std::vector<std::string> forbiddenActions;
};
struct ScenarioDef {
    std::string name;
    std::vector<FactRef> initialState, goals;
    ScenarioExpectation expectation;
};

struct ProjectModel {
    int version = 1;
    std::string projectName = "[Untitled]";
    std::vector<TypeDef> types;
    std::vector<PredicateDef> predicates;
    std::vector<ActionDef> actions;
    std::vector<StateGroupDef> stateGroups;
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
void to_json(nlohmann::json&, const BtBinding&);
void from_json(const nlohmann::json&, BtBinding&);
void to_json(nlohmann::json&, const ActionDef&);
void from_json(const nlohmann::json&, ActionDef&);
void to_json(nlohmann::json&, const StateGroupDef&);
void from_json(const nlohmann::json&, StateGroupDef&);
void to_json(nlohmann::json&, const ObjectDef&);
void from_json(const nlohmann::json&, ObjectDef&);
void to_json(nlohmann::json&, const FactRef&);
void from_json(const nlohmann::json&, FactRef&);
void to_json(nlohmann::json&, const ScenarioExpectation&);
void from_json(const nlohmann::json&, ScenarioExpectation&);
void to_json(nlohmann::json&, const ScenarioDef&);
void from_json(const nlohmann::json&, ScenarioDef&);
void to_json(nlohmann::json&, const ProjectModel&);
void from_json(const nlohmann::json&, ProjectModel&);
