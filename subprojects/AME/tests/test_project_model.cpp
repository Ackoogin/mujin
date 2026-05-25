#include <gtest/gtest.h>
#include "project_model.h"
#include <fstream>
#include <cstdio>

TEST(ProjectModel, RoundTrip) {
    ProjectModel m;
    m.projectName = "Test";
    m.types.push_back({"robot", "object"});
    m.predicates.push_back({"at", {{"?r","robot"},{"?l","location"}}, 10,20});
    ActionDef a;
    a.name = "move";
    a.params = {{"?r","robot"}, {"?from","location"}, {"?to","location"}};
    a.posX = 50;
    a.posY = 75;
    a.preconditions.push_back({"at",{"?r","?from"}});
    a.addEffects.push_back({"at",{"?r","?to"}});
    a.delEffects.push_back({"at",{"?r","?from"}});
    a.btBinding.nodeType = "MoveToLocation";
    a.btBinding.subtreeXml = "<MoveTo goal=\"{param2}\"/>";
    a.btBinding.reactive = true;
    m.actions.push_back(a);
    ActionDef search;
    search.name = "search";
    search.params = {{"?r","robot"}, {"?where","location"}};
    search.preconditions.push_back({"at",{"?r","?where"}});
    m.actions.push_back(search);
    m.causalLinks.push_back({0, 0, 1, 0});
    m.objects.push_back({"uav1","robot"});
    ScenarioDef s; s.name="nominal"; s.goals.push_back({"at",{"uav1","base"}});
    s.expectation.shouldSucceed = true;
    s.expectation.minPlanSteps = 1;
    s.expectation.maxPlanSteps = 10;
    s.expectation.expectedActions = {"move"};
    s.expectation.forbiddenActions = {"explode"};
    m.scenarios.push_back(s);

    const char* path = "test_project_model_tmp.json";
    ASSERT_TRUE(m.save(path));

    ProjectModel m2;
    ASSERT_TRUE(m2.load(path));
    EXPECT_EQ(m2.projectName, "Test");
    ASSERT_EQ(m2.types.size(), 1u); EXPECT_EQ(m2.types[0].name, "robot");
    ASSERT_EQ(m2.predicates.size(), 1u); EXPECT_EQ(m2.predicates[0].posX, 10.0f);
    ASSERT_EQ(m2.actions.size(), 2u);
    EXPECT_EQ(m2.actions[0].name, "move");
    ASSERT_EQ(m2.actions[0].params.size(), 3u);
    EXPECT_EQ(m2.actions[0].params[0].name, "?r");
    EXPECT_EQ(m2.actions[0].params[0].type, "robot");
    EXPECT_EQ(m2.actions[0].params[1].name, "?from");
    EXPECT_EQ(m2.actions[0].params[1].type, "location");
    EXPECT_EQ(m2.actions[0].params[2].name, "?to");
    EXPECT_EQ(m2.actions[0].params[2].type, "location");
    EXPECT_EQ(m2.actions[0].posX, 50.0f);
    EXPECT_EQ(m2.actions[0].posY, 75.0f);
    ASSERT_EQ(m2.actions[0].preconditions.size(), 1u);
    EXPECT_EQ(m2.actions[0].preconditions[0].predicateName, "at");
    ASSERT_EQ(m2.actions[0].preconditions[0].argNames.size(), 2u);
    EXPECT_EQ(m2.actions[0].preconditions[0].argNames[0], "?r");
    EXPECT_EQ(m2.actions[0].preconditions[0].argNames[1], "?from");
    ASSERT_EQ(m2.actions[0].addEffects.size(), 1u);
    EXPECT_EQ(m2.actions[0].addEffects[0].predicateName, "at");
    ASSERT_EQ(m2.actions[0].addEffects[0].argNames.size(), 2u);
    EXPECT_EQ(m2.actions[0].addEffects[0].argNames[0], "?r");
    EXPECT_EQ(m2.actions[0].addEffects[0].argNames[1], "?to");
    ASSERT_EQ(m2.actions[0].delEffects.size(), 1u);
    EXPECT_EQ(m2.actions[0].delEffects[0].predicateName, "at");
    ASSERT_EQ(m2.actions[0].delEffects[0].argNames.size(), 2u);
    EXPECT_EQ(m2.actions[0].delEffects[0].argNames[0], "?r");
    EXPECT_EQ(m2.actions[0].delEffects[0].argNames[1], "?from");
    EXPECT_EQ(m2.actions[0].btBinding.nodeType, "MoveToLocation");
    EXPECT_EQ(m2.actions[0].btBinding.subtreeXml, "<MoveTo goal=\"{param2}\"/>");
    EXPECT_TRUE(m2.actions[0].btBinding.reactive);
    EXPECT_EQ(m2.actions[1].name, "search");
    ASSERT_EQ(m2.actions[1].preconditions.size(), 1u);
    EXPECT_EQ(m2.actions[1].preconditions[0].predicateName, "at");
    ASSERT_EQ(m2.causalLinks.size(), 1u);
    EXPECT_EQ(m2.causalLinks[0].fromAction, 0);
    EXPECT_EQ(m2.causalLinks[0].fromAddEffectIdx, 0);
    EXPECT_EQ(m2.causalLinks[0].toAction, 1);
    EXPECT_EQ(m2.causalLinks[0].toPreconditionIdx, 0);
    ASSERT_EQ(m2.objects.size(), 1u); EXPECT_EQ(m2.objects[0].type, "robot");
    ASSERT_EQ(m2.scenarios.size(), 1u);
    EXPECT_TRUE(m2.scenarios[0].expectation.shouldSucceed);
    EXPECT_EQ(m2.scenarios[0].expectation.minPlanSteps, 1);
    EXPECT_EQ(m2.scenarios[0].expectation.maxPlanSteps, 10);
    EXPECT_EQ(m2.scenarios[0].expectation.expectedActions,
              std::vector<std::string>{"move"});
    EXPECT_EQ(m2.scenarios[0].expectation.forbiddenActions,
              std::vector<std::string>{"explode"});
    std::remove(path);
}

TEST(ProjectModel, LoadOldActionWithoutBindingDefaults) {
    const char* path = "test_old_action_tmp.json";
    std::ofstream f(path);
    f << R"json({
  "version": 1,
  "projectName": "OldActionProject",
  "types": [],
  "predicates": [],
  "actions": [
    {
      "name": "move",
      "params": [],
      "preconditions": [],
      "addEffects": [],
      "delEffects": [],
      "posX": 0.0,
      "posY": 0.0
    }
  ],
  "causalLinks": [],
  "objects": [],
  "scenarios": []
})json";
    f.close();

    ProjectModel m;
    ASSERT_TRUE(m.load(path));
    ASSERT_EQ(m.actions.size(), 1u);
    EXPECT_TRUE(m.actions[0].btBinding.nodeType.empty());
    EXPECT_TRUE(m.actions[0].btBinding.subtreeXml.empty());
    EXPECT_FALSE(m.actions[0].btBinding.reactive);
    std::remove(path);
}

TEST(ProjectModel, LoadOldScenarioWithoutExpectationDefaults) {
    const char* path = "test_old_scenario_tmp.json";
    std::ofstream f(path);
    f << R"json({
  "version": 1,
  "projectName": "OldScenarioProject",
  "types": [],
  "predicates": [],
  "actions": [],
  "causalLinks": [],
  "objects": [],
  "scenarios": [
    {
      "name": "nominal",
      "initialState": [],
      "goals": []
    }
  ]
})json";
    f.close();

    ProjectModel m;
    ASSERT_TRUE(m.load(path));
    ASSERT_EQ(m.scenarios.size(), 1u);
    EXPECT_TRUE(m.scenarios[0].expectation.shouldSucceed);
    EXPECT_EQ(m.scenarios[0].expectation.minPlanSteps, 0);
    EXPECT_EQ(m.scenarios[0].expectation.maxPlanSteps, 0);
    EXPECT_TRUE(m.scenarios[0].expectation.expectedActions.empty());
    EXPECT_TRUE(m.scenarios[0].expectation.forbiddenActions.empty());
    std::remove(path);
}

TEST(ProjectModel, CausalLinkValidation) {
    ProjectModel m;
    ActionDef move;
    move.name = "move";
    move.preconditions.push_back({"at",{"?r","?from"}});
    move.addEffects.push_back({"at",{"?r","?to"}});
    m.actions.push_back(move);

    ActionDef search;
    search.name = "search";
    search.preconditions.push_back({"at",{"?agent","?where"}});
    search.preconditions.push_back({"connected",{"?from","?to"}});
    search.preconditions.push_back({"at",{"?agent"}});
    m.actions.push_back(search);

    EXPECT_TRUE(causalLinkCompatible(m, {0, 0, 1, 0}));
    EXPECT_FALSE(causalLinkCompatible(m, {0, 0, 1, 1}));
    EXPECT_FALSE(causalLinkCompatible(m, {0, 0, 1, 2}));
    EXPECT_FALSE(causalLinkCompatible(m, {0, 0, 0, 0}));
    EXPECT_FALSE(causalLinkCompatible(m, {2, 0, 1, 0}));
}

TEST(ProjectModel, MissingCausalLinksDefaultsEmpty) {
    nlohmann::json j = {
        {"version", 1},
        {"projectName", "OldProject"},
        {"types", nlohmann::json::array()},
        {"predicates", nlohmann::json::array()},
        {"actions", nlohmann::json::array()},
        {"objects", nlohmann::json::array()},
        {"scenarios", nlohmann::json::array()},
    };

    ProjectModel m = j.get<ProjectModel>();
    EXPECT_TRUE(m.causalLinks.empty());
}

TEST(ProjectModel, LoadMissingFile) {
    ProjectModel m;
    EXPECT_FALSE(m.load("__nonexistent_file_xyz__.json"));
}

TEST(ProjectModel, LoadBadJson) {
    const char* path = "test_bad_json_tmp.json";
    std::ofstream f(path); f << "not json"; f.close();
    ProjectModel m;
    EXPECT_FALSE(m.load(path));
    std::remove(path);
}

TEST(ProjectModel, ClearResetsVersion) {
    ProjectModel m;
    m.version = 99; m.types.push_back({"x",""});
    m.clear();
    EXPECT_EQ(m.version, 1);
    EXPECT_TRUE(m.types.empty());
}
