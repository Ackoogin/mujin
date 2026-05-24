#include <gtest/gtest.h>
#include "ame/pddl_parser.h"
#include "ame/planner.h"
#include "ame/world_model.h"

#include <string>

static const std::string VEHICLE_DIR =
    std::string(PROJECT_SOURCE_DIR) + "/domains/vehicle_autonomy/";
static const std::string MISSION_DIR =
    std::string(PROJECT_SOURCE_DIR) + "/domains/mission_autonomy/";

static ame::WorldModel loadVehicle(const std::string& problem) {
    ame::WorldModel wm;
    ame::PddlParser::parse(
        VEHICLE_DIR + "domain.pddl", VEHICLE_DIR + problem, wm);
    return wm;
}

static ame::WorldModel loadMission(const std::string& problem) {
    ame::WorldModel wm;
    ame::PddlParser::parse(
        MISSION_DIR + "domain.pddl", MISSION_DIR + problem, wm);
    return wm;
}

static bool planHas(const ame::WorldModel& wm,
                    const ame::PlanResult& r,
                    const std::string& substr) {
    for (auto& s : r.steps) {
        auto& ga = wm.groundActions()[s.action_index];
        if (ga.signature.find(substr) != std::string::npos) return true;
    }
    return false;
}

// =========================================================================
// Vehicle Autonomy — nominal
// =========================================================================

TEST(VehicleContingency, NominalMission) {
    auto wm = loadVehicle("problem_nominal.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "preflight-check"));
    EXPECT_TRUE(planHas(wm, r, "takeoff"));
    EXPECT_TRUE(planHas(wm, r, "execute-mission"));
    EXPECT_TRUE(planHas(wm, r, "land"));
    EXPECT_FALSE(planHas(wm, r, "abort"));
    EXPECT_FALSE(planHas(wm, r, "ditch"));
}

// =========================================================================
// Vehicle Autonomy — pre-takeoff aborts
// =========================================================================

TEST(VehicleContingency, NavFailPreTakeoff) {
    auto wm = loadVehicle("problem_nav_fail_pre_takeoff.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success) << "Must have a valid abort plan";
    EXPECT_TRUE(planHas(wm, r, "abort-on-ground"));
    EXPECT_FALSE(planHas(wm, r, "takeoff"));
    EXPECT_EQ(r.steps.size(), 1u);
}

TEST(VehicleContingency, EngineFailPreTakeoff) {
    auto wm = loadVehicle("problem_engine_fail_pre_takeoff.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "abort-on-ground"));
    EXPECT_EQ(r.steps.size(), 1u);
}

// =========================================================================
// Vehicle Autonomy — airborne emergency returns
// =========================================================================

TEST(VehicleContingency, NavFailEnRoute) {
    auto wm = loadVehicle("problem_nav_fail_en_route.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "emergency-return"));
    EXPECT_FALSE(planHas(wm, r, "fly"));
}

TEST(VehicleContingency, CommsLostAirborne) {
    auto wm = loadVehicle("problem_comms_lost_airborne.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "emergency-return"));
    EXPECT_FALSE(planHas(wm, r, "fly"));
}

TEST(VehicleContingency, NavCommsLost_GpsAvailable) {
    auto wm = loadVehicle("problem_nav_comms_lost.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "emergency-return"));
}

TEST(VehicleContingency, TotalNavLoss_InertialOnly) {
    auto wm = loadVehicle("problem_total_nav_loss.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "emergency-return-inertial"));
}

// =========================================================================
// Vehicle Autonomy — engine failure / total failure (forced landing)
// =========================================================================

TEST(VehicleContingency, EngineFailAirborne) {
    auto wm = loadVehicle("problem_engine_fail_airborne.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "emergency-land"));
    EXPECT_FALSE(planHas(wm, r, "emergency-return"));
    EXPECT_EQ(r.steps.size(), 1u);
}

TEST(VehicleContingency, AllFailedAirborne) {
    auto wm = loadVehicle("problem_all_failed_airborne.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success) << "Must always have a last-resort option";
    EXPECT_TRUE(planHas(wm, r, "emergency-land"));
    EXPECT_EQ(r.steps.size(), 1u);
}

// =========================================================================
// Vehicle Autonomy — plan lengths should form a sensible ordering
// =========================================================================

TEST(VehicleContingency, PlanLengthOrdering) {
    ame::Planner planner;

    auto r_nominal = planner.solve(loadVehicle("problem_nominal.pddl"));
    auto r_abort = planner.solve(loadVehicle("problem_nav_fail_pre_takeoff.pddl"));
    auto r_emret = planner.solve(loadVehicle("problem_nav_fail_en_route.pddl"));
    auto r_land = planner.solve(loadVehicle("problem_all_failed_airborne.pddl"));

    ASSERT_TRUE(r_nominal.success);
    ASSERT_TRUE(r_abort.success);
    ASSERT_TRUE(r_emret.success);
    ASSERT_TRUE(r_land.success);

    EXPECT_GT(r_nominal.steps.size(), r_emret.steps.size());
    EXPECT_GT(r_emret.steps.size(), r_abort.steps.size());
    EXPECT_EQ(r_land.steps.size(), 1u);
    EXPECT_EQ(r_abort.steps.size(), 1u);
}

// =========================================================================
// Vehicle Autonomy — critical mission priority (ditch doctrine)
// =========================================================================

TEST(VehicleContingency, CriticalNominal) {
    auto wm = loadVehicle("problem_critical_nominal.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "preflight-check"));
    EXPECT_TRUE(planHas(wm, r, "takeoff"));
    EXPECT_TRUE(planHas(wm, r, "execute-mission"));
    EXPECT_TRUE(planHas(wm, r, "land"));
    EXPECT_FALSE(planHas(wm, r, "ditch"));
    EXPECT_FALSE(planHas(wm, r, "terminal"));
}

TEST(VehicleContingency, CriticalPushThroughDitch) {
    auto wm = loadVehicle("problem_critical_push_through_ditch.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success) << "Critical mission must push through degradation";
    EXPECT_TRUE(planHas(wm, r, "execute-mission-critical"));
    EXPECT_TRUE(planHas(wm, r, "ditch"));
    EXPECT_FALSE(planHas(wm, r, "emergency-return"));
}

TEST(VehicleContingency, CriticalNavFailDitch) {
    auto wm = loadVehicle("problem_critical_nav_fail_ditch.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "execute-mission-critical"));
    EXPECT_TRUE(planHas(wm, r, "ditch"));
}

TEST(VehicleContingency, CriticalSevereDegradeDitch) {
    auto wm = loadVehicle("problem_critical_severe_degrade_ditch.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "execute-mission-critical"));
    EXPECT_TRUE(planHas(wm, r, "ditch"));
}

TEST(VehicleContingency, CriticalTotalFailureDitch) {
    auto wm = loadVehicle("problem_critical_total_failure_ditch.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success) << "Terminal ditch must always work for critical missions";
    EXPECT_TRUE(planHas(wm, r, "execute-mission-critical"));
    EXPECT_TRUE(planHas(wm, r, "terminal-ditch"));
    EXPECT_EQ(r.steps.size(), 2u);
}

TEST(VehicleContingency, CriticalEngineFailDitch) {
    auto wm = loadVehicle("problem_critical_engine_fail_ditch.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(planHas(wm, r, "terminal-ditch"));
    EXPECT_EQ(r.steps.size(), 1u);
}

// =========================================================================
// Mission Autonomy — nominal
// =========================================================================

TEST(MissionContingency, NominalMultiAgent) {
    auto wm = loadMission("problem_nominal.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success);
    EXPECT_FALSE(planHas(wm, r, "reallocate"));
    EXPECT_FALSE(planHas(wm, r, "mark-task-failed"));
}

// =========================================================================
// Mission Autonomy — task reallocation
// =========================================================================

TEST(MissionContingency, AgentLost_TaskReallocated) {
    auto wm = loadMission("problem_agent_lost.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success) << "Must reallocate tasks from failed agent";
    EXPECT_TRUE(planHas(wm, r, "mark-task-failed"));
    EXPECT_TRUE(planHas(wm, r, "reallocate-task"));
}

TEST(MissionContingency, SensorFail_WithdrawAndReallocate) {
    auto wm = loadMission("problem_sensor_fail.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success) << "Sensor fail must be recoverable via reallocation";
    EXPECT_TRUE(planHas(wm, r, "withdraw-agent"));
    EXPECT_TRUE(planHas(wm, r, "mark-task-failed"));
    EXPECT_TRUE(planHas(wm, r, "reallocate-task"));
}

// =========================================================================
// Mission Autonomy — unsolvable scenarios (design boundary)
// =========================================================================

TEST(MissionContingency, BothSensorsDegraded_Unsolvable) {
    auto wm = loadMission("problem_both_sensors_degraded.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    EXPECT_FALSE(r.success)
        << "Cannot classify without any working sensor";
}

TEST(MissionContingency, CommsLost_PreAssignedTasksStillWork) {
    auto wm = loadMission("problem_comms_lost_no_realloc.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    ASSERT_TRUE(r.success)
        << "Pre-assigned tasks should work without comms";
    EXPECT_FALSE(planHas(wm, r, "reallocate"));
}

TEST(MissionContingency, CommsLost_ReallocationNeeded_Unsolvable) {
    auto wm = loadMission("problem_comms_lost_realloc_needed.pddl");
    ame::Planner planner;
    auto r = planner.solve(wm);

    EXPECT_FALSE(r.success)
        << "Cannot reallocate without comms";
}
