#include <gtest/gtest.h>
#include "ame/pddl_parser.h"
#include "ame/planner.h"
#include "ame/world_model.h"

#include <bitset>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// =========================================================================
// Exhaustive contingency verification
// =========================================================================
// Proves that every possible combination of system health states has a
// valid plan to reach safe-state. Enumerates all 2^N combinations of
// health predicates for each mission phase and solves for the safety
// goal. Any unsolvable combination is a design gap.
//
// Output: a human-readable report suitable for safety review, showing
// every combination, its solvability, plan length, and recovery actions.
// =========================================================================

static const std::string VEHICLE_DIR =
    std::string(PROJECT_SOURCE_DIR) + "/domains/vehicle_autonomy/";
static const std::string MISSION_DIR =
    std::string(PROJECT_SOURCE_DIR) + "/domains/mission_autonomy/";

// --- Vehicle domain health predicates (0-ary, simple on/off) ---

struct HealthPredicate {
    std::string fluent_key;
    std::string short_name;
};

static const std::vector<HealthPredicate> VEHICLE_HEALTH = {
    {"(nav-operational)",        "NAV"},
    {"(engines-operational)",    "ENG"},
    {"(comms-available)",        "COM"},
    {"(gps-available)",          "GPS"},
    {"(inertial-nav-available)", "INS"},
};

struct PhaseConfig {
    std::string name;
    std::vector<std::string> structural_facts;
    std::vector<std::string> goal_facts;
};

static const std::vector<PhaseConfig> VEHICLE_PHASES = {
    {
        "ON_GROUND",
        {"(at uav1 home)", "(on-ground uav1)"},
        {"(safe-state uav1)"},
    },
    {
        "AIRBORNE",
        {"(at uav1 wp1)", "(airborne uav1)", "(preflight-done uav1)"},
        {"(safe-state uav1)", "(landed-safe uav1)"},
    },
};

static const std::vector<std::string> VEHICLE_ODD_FACTS = {
    "(airspace-clear home)", "(airspace-clear wp1)",
    "(weather-ok home)", "(weather-ok wp1)",
};

// --- Helpers ---

static std::string comboLabel(const std::vector<HealthPredicate>& preds,
                              unsigned combo) {
    std::string label;
    for (size_t i = 0; i < preds.size(); ++i) {
        if (!label.empty()) label += " ";
        bool on = (combo >> i) & 1;
        label += preds[i].short_name;
        label += on ? "=ON" : "=off";
    }
    return label;
}

static std::string planActions(const ame::WorldModel& wm,
                               const ame::PlanResult& r) {
    if (!r.success) return "UNSOLVABLE";
    if (r.steps.empty()) return "(already safe)";
    std::string acts;
    for (auto& s : r.steps) {
        if (!acts.empty()) acts += " -> ";
        acts += wm.groundActions()[s.action_index].signature;
    }
    return acts;
}

static std::string actionNames(const ame::WorldModel& wm,
                                const ame::PlanResult& r) {
    if (!r.success) return "";
    std::string names;
    for (auto& s : r.steps) {
        auto& sig = wm.groundActions()[s.action_index].signature;
        auto paren = sig.find('(');
        std::string name = (paren != std::string::npos)
                               ? sig.substr(0, paren)
                               : sig;
        if (names.find(name) == std::string::npos) {
            if (!names.empty()) names += ", ";
            names += name;
        }
    }
    return names;
}

// =========================================================================
// Vehicle Autonomy — exhaustive safe-state reachability
// =========================================================================

class VehicleExhaustive : public ::testing::Test {
protected:
    std::string domain_pddl_;

    void SetUp() override {
        std::ifstream f(VEHICLE_DIR + "domain.pddl");
        ASSERT_TRUE(f.good()) << "Cannot open vehicle domain";
        std::stringstream ss;
        ss << f.rdbuf();
        domain_pddl_ = ss.str();
    }

    ame::WorldModel buildScenario(const PhaseConfig& phase, unsigned combo) {
        std::ostringstream prob;
        prob << "(define (problem exhaustive-check)\n"
             << "  (:domain vehicle-autonomy)\n"
             << "  (:objects uav1 - robot home - base wp1 - waypoint)\n"
             << "  (:init\n";

        for (auto& f : phase.structural_facts)
            prob << "    " << f << "\n";
        for (auto& f : VEHICLE_ODD_FACTS)
            prob << "    " << f << "\n";

        for (size_t i = 0; i < VEHICLE_HEALTH.size(); ++i) {
            if ((combo >> i) & 1)
                prob << "    " << VEHICLE_HEALTH[i].fluent_key << "\n";
        }

        prob << "  )\n"
             << "  (:goal (and\n";
        for (auto& g : phase.goal_facts)
            prob << "    " << g << "\n";
        prob << "  ))\n)\n";

        ame::WorldModel wm;
        ame::PddlParser::parseFromString(domain_pddl_, prob.str(), wm);
        return wm;
    }
};

TEST_F(VehicleExhaustive, AllHealthCombos_ReachSafeState) {
    ame::Planner planner;
    unsigned total = 0, passed = 0, failed = 0;
    std::vector<std::string> failures;

    std::cout << "\n=========================================="
              << "==========================================\n"
              << "VEHICLE AUTONOMY — EXHAUSTIVE SAFE-STATE REACHABILITY\n"
              << "=========================================="
              << "==========================================\n";

    for (auto& phase : VEHICLE_PHASES) {
        unsigned num_combos = 1u << VEHICLE_HEALTH.size();

        std::cout << "\n--- Phase: " << phase.name
                  << " (" << num_combos << " combinations) ---\n\n";
        std::cout << std::left
                  << std::setw(40) << "Health Configuration"
                  << std::setw(10) << "Result"
                  << std::setw(6)  << "Steps"
                  << "Recovery Path\n";
        std::cout << std::string(100, '-') << "\n";

        for (unsigned combo = 0; combo < num_combos; ++combo) {
            auto wm = buildScenario(phase, combo);
            auto result = planner.solve(wm);
            total++;

            std::string label = comboLabel(VEHICLE_HEALTH, combo);
            std::string status = result.success ? "SAFE" : "GAP";
            std::string steps = result.success
                                    ? std::to_string(result.steps.size())
                                    : "-";
            std::string actions = actionNames(wm, result);

            std::cout << std::left
                      << std::setw(40) << label
                      << std::setw(10) << status
                      << std::setw(6)  << steps
                      << actions << "\n";

            if (result.success) {
                passed++;
            } else {
                failed++;
                failures.push_back(phase.name + " | " + label);
            }
        }
    }

    std::cout << "\n=========================================="
              << "==========================================\n"
              << "SUMMARY: " << passed << "/" << total << " safe, "
              << failed << " gaps\n";

    if (!failures.empty()) {
        std::cout << "\nDESIGN GAPS (no path to safe-state):\n";
        for (auto& f : failures)
            std::cout << "  * " << f << "\n";
    }

    std::cout << "=========================================="
              << "==========================================\n\n";

    EXPECT_EQ(failed, 0u)
        << failed << " system state(s) have no path to safe-state";
}

// =========================================================================
// Vehicle Autonomy — airborne return-to-base redundancy ladder
// =========================================================================
// Stronger goal: return to base AND land. This differentiates between
// recovery paths (nav vs GPS vs inertial vs can't-return). Any combo
// where engines work should be able to return via some nav method.

TEST_F(VehicleExhaustive, AirborneReturnToBase_RedundancyLadder) {
    ame::Planner planner;
    unsigned num_combos = 1u << VEHICLE_HEALTH.size();
    unsigned total = 0, returned = 0, landed_only = 0, failed = 0;
    std::vector<std::string> unexpected_failures;

    PhaseConfig airborne_rtb = {
        "AIRBORNE_RTB",
        {"(at uav1 wp1)", "(airborne uav1)", "(preflight-done uav1)"},
        {"(safe-state uav1)", "(landed-safe uav1)", "(at uav1 home)"},
    };

    std::cout << "\n=========================================="
              << "==========================================\n"
              << "VEHICLE AUTONOMY — RETURN-TO-BASE REDUNDANCY LADDER\n"
              << "(Airborne at wp1, goal: land safely at home base)\n"
              << "=========================================="
              << "==========================================\n\n";
    std::cout << std::left
              << std::setw(40) << "Health Configuration"
              << std::setw(12) << "Result"
              << std::setw(6)  << "Steps"
              << "Recovery Path\n";
    std::cout << std::string(100, '-') << "\n";

    for (unsigned combo = 0; combo < num_combos; ++combo) {
        auto wm = buildScenario(airborne_rtb, combo);
        auto result = planner.solve(wm);
        total++;

        bool engines_on = (combo >> 1) & 1;
        bool has_any_nav = ((combo >> 0) & 1)   // nav
                         || ((combo >> 3) & 1)   // gps
                         || ((combo >> 4) & 1);  // inertial

        std::string label = comboLabel(VEHICLE_HEALTH, combo);
        std::string status, actions;

        if (result.success) {
            returned++;
            status = "RETURNED";
            actions = actionNames(wm, result);
        } else if (!engines_on || !has_any_nav) {
            landed_only++;
            status = "NO_RTB(ok)";
            actions = "engines/nav insufficient for return";
        } else {
            failed++;
            status = "GAP";
            actions = "UNEXPECTED: engines+nav available but can't return";
            unexpected_failures.push_back(label);
        }

        std::cout << std::left
                  << std::setw(40) << label
                  << std::setw(12) << status
                  << std::setw(6)  << (result.success ? std::to_string(result.steps.size()) : "-")
                  << actions << "\n";
    }

    std::cout << "\n=========================================="
              << "==========================================\n"
              << "SUMMARY: " << returned << " can return to base, "
              << landed_only << " cannot return (expected), "
              << failed << " unexpected gaps\n"
              << "=========================================="
              << "==========================================\n\n";

    EXPECT_EQ(failed, 0u)
        << "Unexpected failures: engines+nav available but can't return to base";
}

// =========================================================================
// Mission Autonomy — exhaustive task-completion reachability
// =========================================================================

struct MissionHealthVar {
    std::string on_fact;
    std::string off_fact;
    std::string short_name;
};

static const std::vector<MissionHealthVar> MISSION_HEALTH = {
    {"(agent-available uav1)", "(agent-unavailable uav1)", "UAV1_AVAIL"},
    {"(agent-available uav2)", "(agent-unavailable uav2)", "UAV2_AVAIL"},
    {"(sensor-operational uav1)", "(sensor-degraded uav1)", "UAV1_SENS"},
    {"(sensor-operational uav2)", "(sensor-degraded uav2)", "UAV2_SENS"},
    {"(comms-available)", "", "COMMS"},
};

static std::string missionComboLabel(unsigned combo) {
    std::string label;
    for (size_t i = 0; i < MISSION_HEALTH.size(); ++i) {
        if (!label.empty()) label += " ";
        bool on = (combo >> i) & 1;
        label += MISSION_HEALTH[i].short_name;
        label += on ? "=ON" : "=off";
    }
    return label;
}

class MissionExhaustive : public ::testing::Test {
protected:
    std::string domain_pddl_;

    void SetUp() override {
        std::ifstream f(MISSION_DIR + "domain.pddl");
        ASSERT_TRUE(f.good()) << "Cannot open mission domain";
        std::stringstream ss;
        ss << f.rdbuf();
        domain_pddl_ = ss.str();
    }

    ame::WorldModel buildScenario(unsigned combo) {
        std::ostringstream prob;
        prob << "(define (problem mission-exhaustive)\n"
             << "  (:domain mission-autonomy)\n"
             << "  (:objects\n"
             << "    uav1 uav2 - agent\n"
             << "    base - location\n"
             << "    sector_a sector_b - sector\n"
             << "  )\n"
             << "  (:init\n"
             << "    (at uav1 base)\n"
             << "    (at uav2 base)\n"
             << "    (task-assigned sector_a uav1)\n"
             << "    (task-assigned sector_b uav2)\n";

        for (size_t i = 0; i < MISSION_HEALTH.size(); ++i) {
            bool on = (combo >> i) & 1;
            if (on) {
                prob << "    " << MISSION_HEALTH[i].on_fact << "\n";
            } else if (!MISSION_HEALTH[i].off_fact.empty()) {
                prob << "    " << MISSION_HEALTH[i].off_fact << "\n";
            }
        }

        prob << "  )\n"
             << "  (:goal (and\n"
             << "    (searched sector_a)\n"
             << "    (classified sector_a)\n"
             << "    (searched sector_b)\n"
             << "    (classified sector_b)\n"
             << "  ))\n)\n";

        ame::WorldModel wm;
        ame::PddlParser::parseFromString(domain_pddl_, prob.str(), wm);
        return wm;
    }
};

TEST_F(MissionExhaustive, AllHealthCombos_MissionCompletion) {
    ame::Planner planner;
    unsigned num_combos = 1u << MISSION_HEALTH.size();
    unsigned total = 0, solvable = 0, unsolvable = 0;
    std::vector<std::string> solvable_list, unsolvable_list;

    std::cout << "\n=========================================="
              << "==========================================\n"
              << "MISSION AUTONOMY — EXHAUSTIVE MISSION COMPLETION\n"
              << "(" << num_combos << " combinations, 2 sectors, 2 agents)\n"
              << "=========================================="
              << "==========================================\n\n";

    std::cout << std::left
              << std::setw(55) << "Health Configuration"
              << std::setw(12) << "Result"
              << std::setw(6)  << "Steps"
              << "Key Actions\n";
    std::cout << std::string(110, '-') << "\n";

    for (unsigned combo = 0; combo < num_combos; ++combo) {
        auto wm = buildScenario(combo);
        auto result = planner.solve(wm);
        total++;

        std::string label = missionComboLabel(combo);
        std::string status = result.success ? "COMPLETE" : "INCOMPLETE";
        std::string steps = result.success
                                ? std::to_string(result.steps.size())
                                : "-";
        std::string actions = actionNames(wm, result);

        std::cout << std::left
                  << std::setw(55) << label
                  << std::setw(12) << status
                  << std::setw(6)  << steps
                  << actions << "\n";

        if (result.success) {
            solvable++;
            solvable_list.push_back(label);
        } else {
            unsolvable++;
            unsolvable_list.push_back(label);
        }
    }

    std::cout << "\n=========================================="
              << "==========================================\n"
              << "SUMMARY: " << solvable << "/" << total
              << " can complete full mission, "
              << unsolvable << " cannot\n";

    if (!unsolvable_list.empty()) {
        std::cout << "\nCannot complete mission (expected for degraded states):\n";
        for (auto& u : unsolvable_list)
            std::cout << "  - " << u << "\n";
    }

    std::cout << "=========================================="
              << "==========================================\n\n";

    // This is informational — not all combos should be solvable.
    // The critical assertion is that we KNOW which are unsolvable.
    // Record both sets for review.
    RecordProperty("solvable_count", solvable);
    RecordProperty("unsolvable_count", unsolvable);
    RecordProperty("total_combinations", total);
}
