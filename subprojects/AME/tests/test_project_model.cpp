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
    m.actions.push_back(a);
    m.objects.push_back({"uav1","robot"});
    ScenarioDef s; s.name="nominal"; s.goals.push_back({"at",{"uav1","base"}});
    m.scenarios.push_back(s);

    const char* path = "test_project_model_tmp.json";
    ASSERT_TRUE(m.save(path));

    ProjectModel m2;
    ASSERT_TRUE(m2.load(path));
    EXPECT_EQ(m2.projectName, "Test");
    ASSERT_EQ(m2.types.size(), 1u); EXPECT_EQ(m2.types[0].name, "robot");
    ASSERT_EQ(m2.predicates.size(), 1u); EXPECT_EQ(m2.predicates[0].posX, 10.0f);
    ASSERT_EQ(m2.actions.size(), 1u);
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
    ASSERT_EQ(m2.objects.size(), 1u); EXPECT_EQ(m2.objects[0].type, "robot");
    ASSERT_EQ(m2.scenarios.size(), 1u);
    std::remove(path);
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
