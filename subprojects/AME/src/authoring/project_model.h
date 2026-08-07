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
// How an action behaves when a simulated run reaches it. The defaults are
// chosen so that a project which has never been near the run screen still
// simulates, and so that what it simulates can be watched: an action takes four
// ticks, which is one second at the run screen's default speed, and works.
//
// The duration is not one tick on purpose. A behaviour tree walks a sequence of
// actions that each finish immediately within a single tick, so a whole mission
// with one-tick actions starts and ends on tick one: correct, but with nothing
// to see and no order visible on a timeline. Nothing a real vehicle does
// finishes that fast either. Give an action its own duration where the mission
// depends on how long it takes.
//
// `failureChance` is a probability between 0 and 1 drawn against the run's seed,
// so a run with a random element repeats exactly when the seed is unchanged.
struct SimulationSettings {
    int ticks = 4;
    bool succeeds = true;
    double failureChance = 0.0;
};
struct ActionDef {
    std::string name;
    std::vector<Parameter> params;
    std::vector<EffectRef> preconditions, addEffects, delEffects;
    float posX=0,posY=0;
    BtBinding btBinding;
    SimulationSettings simulation;
};
struct StateGroupDef {
    std::string name;
    std::string type;
    std::vector<std::string> predicateNames;
};
struct ObjectDef { std::string name, type; };
struct FactRef { std::string predicateName; std::vector<std::string> objectNames; };
/// \brief Make one action invocation fail, then let later invocations behave normally.
struct ForcedActionFailure {
    std::string actionName;
    unsigned attempt = 1;
};
/// \brief Change one grounded fact at a selected tick of a simulated run.
struct ScheduledFactChange {
    FactRef fact;
    bool value = true;
    unsigned tick = 1;
};
/// \brief The two supported ways a simulated run can be made to go wrong.
///
/// This belongs to a run, not to the PDDL domain. A scenario may name one for
/// regression testing, and the Run tab keeps its current value across Reset.
struct RunFaultSet {
    std::string name;
    std::vector<ForcedActionFailure> actionFailures;
    std::vector<ScheduledFactChange> factChanges;
};
struct ScenarioExpectation {
    bool shouldSucceed = true;
    int minPlanSteps = 0;
    int maxPlanSteps = 0;
    std::vector<std::string> expectedActions;
    std::vector<std::string> forbiddenActions;
    bool shouldReachGoal = true;
    int minRunActions = 0;
    int maxRunActions = 0;
    std::vector<std::string> requiredRunActions;
    std::vector<std::string> forbiddenRunActions;
    int maxReplans = -1;
    RunFaultSet runFault;
};
// A named set of facts and actions drawn as one labelled box on the whole-domain
// canvas, which can be collapsed to a single node. It is a way of talking about
// a domain in a review, not something the generated PDDL knows about.
struct PresentationGroup {
    std::string name;
    std::vector<std::string> predicateNames;
    std::vector<std::string> actionNames;
    bool collapsed = false;
};
// A picture somebody wants to come back to: what was in focus, how far out from
// it the view reached, and which kinds of relationship were shown. Saved with
// the project so it survives reopening, and so it can be shown to somebody else.
struct SavedView {
    std::string name;
    std::string focusPredicate;   // one of these two is set, or neither
    std::string focusAction;
    int depth = 1;
    unsigned relationshipFilter = 0;
    int viewMode = 0;
};
// What this scenario treats as a contingency, and what counts as having
// recovered from it. Left empty, the tool infers both, which is what it did
// before anyone could say. Filled in, the user's answer is used: they know
// which facts represent something going wrong and which represent being safe
// again, and the model cannot tell them apart on its own.
struct ContingencyDeclaration {
    std::vector<std::string> contingencyPredicates;
    std::vector<FactRef> safeState;

    bool isEmpty() const {
        return contingencyPredicates.empty() && safeState.empty();
    }
};
struct ScenarioDef {
    std::string name;
    std::vector<FactRef> initialState, goals;
    ScenarioExpectation expectation;
    ContingencyDeclaration contingency;
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
    std::vector<PresentationGroup> presentationGroups;
    std::vector<SavedView> savedViews;
    // Seed for every random draw a simulated run makes. Saved with the project
    // so that a run someone else repeats draws the same numbers.
    unsigned simulationSeed = 4471;

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
void to_json(nlohmann::json&, const SimulationSettings&);
void from_json(const nlohmann::json&, SimulationSettings&);
void to_json(nlohmann::json&, const ActionDef&);
void from_json(const nlohmann::json&, ActionDef&);
void to_json(nlohmann::json&, const StateGroupDef&);
void from_json(const nlohmann::json&, StateGroupDef&);
void to_json(nlohmann::json&, const ObjectDef&);
void from_json(const nlohmann::json&, ObjectDef&);
void to_json(nlohmann::json&, const FactRef&);
void from_json(const nlohmann::json&, FactRef&);
void to_json(nlohmann::json&, const ForcedActionFailure&);
void from_json(const nlohmann::json&, ForcedActionFailure&);
void to_json(nlohmann::json&, const ScheduledFactChange&);
void from_json(const nlohmann::json&, ScheduledFactChange&);
void to_json(nlohmann::json&, const RunFaultSet&);
void from_json(const nlohmann::json&, RunFaultSet&);
void to_json(nlohmann::json&, const ScenarioExpectation&);
void from_json(const nlohmann::json&, ScenarioExpectation&);
void to_json(nlohmann::json&, const PresentationGroup&);
void from_json(const nlohmann::json&, PresentationGroup&);
void to_json(nlohmann::json&, const SavedView&);
void from_json(const nlohmann::json&, SavedView&);
void to_json(nlohmann::json&, const ContingencyDeclaration&);
void from_json(const nlohmann::json&, ContingencyDeclaration&);
void to_json(nlohmann::json&, const ScenarioDef&);
void from_json(const nlohmann::json&, ScenarioDef&);
void to_json(nlohmann::json&, const ProjectModel&);
void from_json(const nlohmann::json&, ProjectModel&);
