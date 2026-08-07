#include "project_model.h"

#include <algorithm>
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
        {"confirmed", value.confirmed},
    };
}

void from_json(const nlohmann::json& j, PredicateDef& value) {
    value = PredicateDef{};
    j.at("name").get_to(value.name);
    j.at("params").get_to(value.params);
    j.at("posX").get_to(value.posX);
    j.at("posY").get_to(value.posY);
    // Projects saved before confirmed predicates existed have no such key.
    if (j.contains("confirmed")) {
        j.at("confirmed").get_to(value.confirmed);
    }
}

void to_json(nlohmann::json& j, const EffectRef& value) {
    j = nlohmann::json{{"predicateName", value.predicateName}, {"argNames", value.argNames}};
}

void from_json(const nlohmann::json& j, EffectRef& value) {
    j.at("predicateName").get_to(value.predicateName);
    j.at("argNames").get_to(value.argNames);
}

void to_json(nlohmann::json& j, const BtBinding& value) {
    j = nlohmann::json{
        {"nodeType", value.nodeType},
        {"subtreeXml", value.subtreeXml},
        {"reactive", value.reactive},
    };
}

void from_json(const nlohmann::json& j, BtBinding& value) {
    value = BtBinding{};
    if (j.contains("nodeType")) {
        j.at("nodeType").get_to(value.nodeType);
    }
    if (j.contains("subtreeXml")) {
        j.at("subtreeXml").get_to(value.subtreeXml);
    }
    if (j.contains("reactive")) {
        j.at("reactive").get_to(value.reactive);
    }
}

void to_json(nlohmann::json& j, const SimulationSettings& value) {
    j = nlohmann::json{
        {"ticks", value.ticks},
        {"succeeds", value.succeeds},
        {"failureChance", value.failureChance},
    };
}

void from_json(const nlohmann::json& j, SimulationSettings& value) {
    value = SimulationSettings{};
    if (j.contains("ticks")) {
        j.at("ticks").get_to(value.ticks);
    }
    if (j.contains("succeeds")) {
        j.at("succeeds").get_to(value.succeeds);
    }
    if (j.contains("failureChance")) {
        j.at("failureChance").get_to(value.failureChance);
    }
    if (value.ticks < 1) {
        value.ticks = 1;
    }
    if (value.failureChance < 0.0) {
        value.failureChance = 0.0;
    }
    if (value.failureChance > 1.0) {
        value.failureChance = 1.0;
    }
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
        {"btBinding", value.btBinding},
        {"simulation", value.simulation},
    };
}

void from_json(const nlohmann::json& j, ActionDef& value) {
    value = ActionDef{};
    j.at("name").get_to(value.name);
    j.at("params").get_to(value.params);
    j.at("preconditions").get_to(value.preconditions);
    j.at("addEffects").get_to(value.addEffects);
    j.at("delEffects").get_to(value.delEffects);
    j.at("posX").get_to(value.posX);
    j.at("posY").get_to(value.posY);
    if (j.contains("btBinding")) {
        j.at("btBinding").get_to(value.btBinding);
    }
    // Projects saved before simulation runs existed have no such key, and take
    // the defaults: four ticks, always works.
    if (j.contains("simulation")) {
        j.at("simulation").get_to(value.simulation);
    }
}

void to_json(nlohmann::json& j, const StateGroupDef& value) {
    j = nlohmann::json{
        {"name", value.name},
        {"type", value.type},
        {"predicateNames", value.predicateNames},
    };
}

void from_json(const nlohmann::json& j, StateGroupDef& value) {
    j.at("name").get_to(value.name);
    j.at("type").get_to(value.type);
    j.at("predicateNames").get_to(value.predicateNames);
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

void to_json(nlohmann::json& j, const ForcedActionFailure& value) {
    j = nlohmann::json{{"actionName", value.actionName},
                       {"attempt", value.attempt}};
}

void from_json(const nlohmann::json& j, ForcedActionFailure& value) {
    value = ForcedActionFailure{};
    if (j.contains("actionName")) {
        j.at("actionName").get_to(value.actionName);
    }
    if (j.contains("attempt")) {
        j.at("attempt").get_to(value.attempt);
    }
    value.attempt = std::max(1U, value.attempt);
}

void to_json(nlohmann::json& j, const ScheduledFactChange& value) {
    j = nlohmann::json{{"fact", value.fact},
                       {"value", value.value},
                       {"tick", value.tick}};
}

void from_json(const nlohmann::json& j, ScheduledFactChange& value) {
    value = ScheduledFactChange{};
    if (j.contains("fact")) {
        j.at("fact").get_to(value.fact);
    }
    if (j.contains("value")) {
        j.at("value").get_to(value.value);
    }
    if (j.contains("tick")) {
        j.at("tick").get_to(value.tick);
    }
    value.tick = std::max(1U, value.tick);
}

void to_json(nlohmann::json& j, const RunFaultSet& value) {
    j = nlohmann::json{{"name", value.name},
                       {"actionFailures", value.actionFailures},
                       {"factChanges", value.factChanges}};
}

void from_json(const nlohmann::json& j, RunFaultSet& value) {
    value = RunFaultSet{};
    if (j.contains("name")) {
        j.at("name").get_to(value.name);
    }
    if (j.contains("actionFailures")) {
        j.at("actionFailures").get_to(value.actionFailures);
    }
    if (j.contains("factChanges")) {
        j.at("factChanges").get_to(value.factChanges);
    }
}

void to_json(nlohmann::json& j, const ScenarioExpectation& value) {
    j = nlohmann::json{
        {"shouldSucceed", value.shouldSucceed},
        {"minPlanSteps", value.minPlanSteps},
        {"maxPlanSteps", value.maxPlanSteps},
        {"expectedActions", value.expectedActions},
        {"forbiddenActions", value.forbiddenActions},
        {"shouldReachGoal", value.shouldReachGoal},
        {"minRunActions", value.minRunActions},
        {"maxRunActions", value.maxRunActions},
        {"requiredRunActions", value.requiredRunActions},
        {"forbiddenRunActions", value.forbiddenRunActions},
        {"maxReplans", value.maxReplans},
        {"runFault", value.runFault},
    };
}

void from_json(const nlohmann::json& j, ScenarioExpectation& value) {
    value = ScenarioExpectation{};
    if (j.contains("shouldSucceed")) {
        j.at("shouldSucceed").get_to(value.shouldSucceed);
    }
    if (j.contains("minPlanSteps")) {
        j.at("minPlanSteps").get_to(value.minPlanSteps);
    }
    if (j.contains("maxPlanSteps")) {
        j.at("maxPlanSteps").get_to(value.maxPlanSteps);
    }
    if (j.contains("expectedActions")) {
        j.at("expectedActions").get_to(value.expectedActions);
    }
    if (j.contains("forbiddenActions")) {
        j.at("forbiddenActions").get_to(value.forbiddenActions);
    }
    if (j.contains("shouldReachGoal")) {
        j.at("shouldReachGoal").get_to(value.shouldReachGoal);
    } else {
        // Before execution expectations existed, a feasible scenario implied
        // that its run should reach the goal.
        value.shouldReachGoal = value.shouldSucceed;
    }
    if (j.contains("minRunActions")) {
        j.at("minRunActions").get_to(value.minRunActions);
    }
    if (j.contains("maxRunActions")) {
        j.at("maxRunActions").get_to(value.maxRunActions);
    }
    if (j.contains("requiredRunActions")) {
        j.at("requiredRunActions").get_to(value.requiredRunActions);
    }
    if (j.contains("forbiddenRunActions")) {
        j.at("forbiddenRunActions").get_to(value.forbiddenRunActions);
    }
    if (j.contains("maxReplans")) {
        j.at("maxReplans").get_to(value.maxReplans);
    }
    if (j.contains("runFault")) {
        j.at("runFault").get_to(value.runFault);
    }
}

void to_json(nlohmann::json& j, const ContingencyDeclaration& value) {
    j = nlohmann::json{
        {"contingencyPredicates", value.contingencyPredicates},
        {"safeState", value.safeState},
    };
}

void from_json(const nlohmann::json& j, ContingencyDeclaration& value) {
    value = ContingencyDeclaration{};
    if (j.contains("contingencyPredicates")) {
        j.at("contingencyPredicates").get_to(value.contingencyPredicates);
    }
    if (j.contains("safeState")) {
        j.at("safeState").get_to(value.safeState);
    }
}

void to_json(nlohmann::json& j, const PresentationGroup& value) {
    j = nlohmann::json{
        {"name", value.name},
        {"predicateNames", value.predicateNames},
        {"actionNames", value.actionNames},
        {"collapsed", value.collapsed},
    };
}

void from_json(const nlohmann::json& j, PresentationGroup& value) {
    value = PresentationGroup{};
    if (j.contains("name")) { j.at("name").get_to(value.name); }
    if (j.contains("predicateNames")) {
        j.at("predicateNames").get_to(value.predicateNames);
    }
    if (j.contains("actionNames")) {
        j.at("actionNames").get_to(value.actionNames);
    }
    if (j.contains("collapsed")) { j.at("collapsed").get_to(value.collapsed); }
}

void to_json(nlohmann::json& j, const SavedView& value) {
    j = nlohmann::json{
        {"name", value.name},
        {"focusPredicate", value.focusPredicate},
        {"focusAction", value.focusAction},
        {"depth", value.depth},
        {"relationshipFilter", value.relationshipFilter},
        {"viewMode", value.viewMode},
    };
}

void from_json(const nlohmann::json& j, SavedView& value) {
    value = SavedView{};
    if (j.contains("name")) { j.at("name").get_to(value.name); }
    if (j.contains("focusPredicate")) {
        j.at("focusPredicate").get_to(value.focusPredicate);
    }
    if (j.contains("focusAction")) {
        j.at("focusAction").get_to(value.focusAction);
    }
    if (j.contains("depth")) { j.at("depth").get_to(value.depth); }
    if (j.contains("relationshipFilter")) {
        j.at("relationshipFilter").get_to(value.relationshipFilter);
    }
    if (j.contains("viewMode")) { j.at("viewMode").get_to(value.viewMode); }
}

void to_json(nlohmann::json& j, const ScenarioDef& value) {
    j = nlohmann::json{
        {"name", value.name},
        {"initialState", value.initialState},
        {"goals", value.goals},
        {"expectation", value.expectation},
        {"contingency", value.contingency},
    };
}

void from_json(const nlohmann::json& j, ScenarioDef& value) {
    value = ScenarioDef{};
    j.at("name").get_to(value.name);
    j.at("initialState").get_to(value.initialState);
    j.at("goals").get_to(value.goals);
    if (j.contains("expectation")) {
        j.at("expectation").get_to(value.expectation);
    }
    // Scenarios saved before contingencies could be declared have none, and
    // fall back to the tool inferring them.
    if (j.contains("contingency")) {
        j.at("contingency").get_to(value.contingency);
    }
}

void to_json(nlohmann::json& j, const ProjectModel& value) {
    j = nlohmann::json{
        {"version", value.version},
        {"projectName", value.projectName},
        {"types", value.types},
        {"predicates", value.predicates},
        {"actions", value.actions},
        {"stateGroups", value.stateGroups},
        {"objects", value.objects},
        {"scenarios", value.scenarios},
        {"presentationGroups", value.presentationGroups},
        {"savedViews", value.savedViews},
        {"simulationSeed", value.simulationSeed},
    };
}

void from_json(const nlohmann::json& j, ProjectModel& value) {
    j.at("version").get_to(value.version);
    j.at("projectName").get_to(value.projectName);
    j.at("types").get_to(value.types);
    j.at("predicates").get_to(value.predicates);
    j.at("actions").get_to(value.actions);
    value.stateGroups.clear();
    if (j.contains("stateGroups")) {
        j.at("stateGroups").get_to(value.stateGroups);
    }
    j.at("objects").get_to(value.objects);
    j.at("scenarios").get_to(value.scenarios);
    // Projects saved before groups and saved views existed have neither.
    value.presentationGroups.clear();
    if (j.contains("presentationGroups")) {
        j.at("presentationGroups").get_to(value.presentationGroups);
    }
    value.savedViews.clear();
    if (j.contains("savedViews")) {
        j.at("savedViews").get_to(value.savedViews);
    }
    value.simulationSeed = ProjectModel{}.simulationSeed;
    if (j.contains("simulationSeed")) {
        j.at("simulationSeed").get_to(value.simulationSeed);
    }
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
