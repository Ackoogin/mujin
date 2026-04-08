#include <gtest/gtest.h>

#include "ame/action_registry.h"
#include "ame/autonomy_backend.h"
#include "ame/current_ame_backend_adapter.h"
#include "ame/world_model.h"

namespace {

ame::WorldModel buildDomain() {
  ame::WorldModel wm;
  auto& ts = wm.typeSystem();
  ts.addType("object");
  ts.addType("location", "object");
  ts.addType("sector", "location");
  ts.addType("robot", "object");

  wm.addObject("uav1", "robot");
  wm.addObject("base", "location");
  wm.addObject("sector_a", "sector");

  wm.registerPredicate("at", {"robot", "location"});
  wm.registerPredicate("searched", {"sector"});

  wm.registerAction("move",
                    {"?r", "?from", "?to"},
                    {"robot", "location", "location"},
                    {"(at ?r ?from)"},
                    {"(at ?r ?to)"},
                    {"(at ?r ?from)"});
  wm.registerAction("search",
                    {"?r", "?s"},
                    {"robot", "sector"},
                    {"(at ?r ?s)"},
                    {"(searched ?s)"},
                    {});

  return wm;
}

ame::ActionRegistry buildRegistry() {
  ame::ActionRegistry registry;
  registry.registerActionSubTree(
      "move",
      "<InvokeService service_name=\"mobility\" operation=\"move\" "
      "param_names=\"?robot;?from;?to\" "
      "param_values=\"{param0};{param1};{param2}\" timeout_ms=\"0\"/>");
  registry.registerActionSubTree(
      "search",
      "<InvokeService service_name=\"imaging\" operation=\"search\" "
      "param_names=\"?robot;?sector\" "
      "param_values=\"{param0};{param1}\" timeout_ms=\"0\"/>");
  return registry;
}

}  // namespace

TEST(AutonomyBackend, EmitsCommandsAndDecisionRecordFromCurrentStack) {
  auto wm = buildDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  auto registry = buildRegistry();

  ame::CurrentAmeBackendAdapter backend(wm, registry);
  backend.start({"session-1", {{"(searched sector_a)"}}, {3}});
  backend.step();

  auto records = backend.pullDecisionRecords();
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records[0].session_id, "session-1");
  EXPECT_TRUE(records[0].plan_success);
  EXPECT_FALSE(records[0].compiled_bt_xml.empty());
  EXPECT_GE(records[0].planned_action_signatures.size(), 2u);

  auto commands = backend.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].service_name, "mobility");
  EXPECT_EQ(commands[0].operation, "move");
  EXPECT_EQ(commands[0].request_fields.at("robot"), "uav1");
  EXPECT_EQ(commands[0].request_fields.at("from"), "base");
  EXPECT_EQ(commands[0].request_fields.at("to"), "sector_a");

  auto snapshot = backend.readSnapshot();
  EXPECT_EQ(snapshot.state, ame::AutonomyBackendState::WAITING_FOR_RESULTS);
  EXPECT_EQ(snapshot.outstanding_commands.size(), 1u);
}

TEST(AutonomyBackend, SuccessfulResultsAdvanceWorldStateAndCompleteSession) {
  auto wm = buildDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  auto registry = buildRegistry();

  ame::CurrentAmeBackendAdapter backend(wm, registry);
  backend.start({"session-2", {{"(searched sector_a)"}}, {3}});
  backend.step();

  auto commands = backend.pullCommands();
  ASSERT_EQ(commands.size(), 1u);

  backend.pushCommandResult({commands[0].command_id, ame::CommandStatus::SUCCEEDED, {}, "dispatcher:move"});
  backend.step();

  auto second_commands = backend.pullCommands();
  ASSERT_EQ(second_commands.size(), 1u);
  EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));
  EXPECT_FALSE(wm.getFact("(at uav1 base)"));
  EXPECT_EQ(second_commands[0].service_name, "imaging");
  EXPECT_EQ(second_commands[0].operation, "search");

  backend.pushCommandResult({second_commands[0].command_id, ame::CommandStatus::SUCCEEDED, {}, "dispatcher:search"});

  backend.step();
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_EQ(backend.readSnapshot().state, ame::AutonomyBackendState::COMPLETE);
}

TEST(AutonomyBackend, ConfirmedObservedUpdatesOverridePredictedEffects) {
  auto wm = buildDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  auto registry = buildRegistry();

  ame::CurrentAmeBackendAdapter backend(wm, registry);
  backend.start({"session-3", {{"(searched sector_a)"}}, {3}});
  backend.step();

  auto commands = backend.pullCommands();
  ASSERT_EQ(commands.size(), 1u);

  ame::CommandResult move_result;
  move_result.command_id = commands[0].command_id;
  move_result.status = ame::CommandStatus::SUCCEEDED;
  move_result.observed_updates = {
      {"(at uav1 base)", false, "perception:gps", ame::FactAuthorityLevel::CONFIRMED},
      {"(at uav1 sector_a)", true, "perception:gps", ame::FactAuthorityLevel::CONFIRMED},
  };
  backend.pushCommandResult(move_result);

  auto metadata = wm.getFactMetadata("(at uav1 sector_a)");
  EXPECT_EQ(metadata.authority, ame::FactAuthority::CONFIRMED);
  EXPECT_EQ(metadata.source, "perception:gps");
}

TEST(AutonomyBackend, FailedCommandTriggersReplanAndNewDecisionRecord) {
  auto wm = buildDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  auto registry = buildRegistry();

  ame::CurrentAmeBackendAdapter backend(wm, registry);
  backend.start({"session-4", {{"(searched sector_a)"}}, {4}});
  backend.step();

  auto commands = backend.pullCommands();
  ASSERT_EQ(commands.size(), 1u);

  backend.pushCommandResult({commands[0].command_id, ame::CommandStatus::FAILED_TRANSIENT, {}, "dispatcher:move"});
  backend.step();
  EXPECT_EQ(backend.readSnapshot().state, ame::AutonomyBackendState::READY);

  backend.step();
  auto records = backend.pullDecisionRecords();
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[1].replan_count, 1u);

  auto replanned_commands = backend.pullCommands();
  ASSERT_EQ(replanned_commands.size(), 1u);
  EXPECT_EQ(replanned_commands[0].service_name, "mobility");
  EXPECT_EQ(replanned_commands[0].operation, "move");
}

TEST(AutonomyBackend, EmitsGoalDispatchesWhenDelegationEnabled) {
  auto wm = buildDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  auto registry = buildRegistry();

  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::SessionRequest request;
  request.session_id = "dispatch-session";
  request.intent.goal_fluents = {"(searched sector_a)"};
  request.policy.max_replans = 3;
  request.policy.enable_goal_dispatch = true;
  request.available_agents = {
      {"uav1", "uav", true},
      {"uav2", "uav", true},
  };
  backend.start(request);

  backend.step();

  auto dispatches = backend.pullGoalDispatches();
  ASSERT_EQ(dispatches.size(), 1u);
  EXPECT_EQ(dispatches[0].agent_id, "uav1");
  EXPECT_EQ(dispatches[0].goals.size(), 1u);
  EXPECT_EQ(dispatches[0].goals[0], "(searched sector_a)");
  auto snapshot = backend.readSnapshot();
  ASSERT_EQ(snapshot.agent_states.size(), 2u);
  EXPECT_EQ(snapshot.agent_states[0].agent_id, "uav1");
  EXPECT_FALSE(snapshot.agent_states[0].available);
  EXPECT_EQ(snapshot.agent_states[1].agent_id, "uav2");
  EXPECT_TRUE(snapshot.agent_states[1].available);
  ASSERT_EQ(snapshot.outstanding_goal_dispatches.size(), 1u);
  EXPECT_EQ(snapshot.state, ame::AutonomyBackendState::WAITING_FOR_RESULTS);

  ame::DispatchResult result;
  result.dispatch_id = dispatches[0].dispatch_id;
  result.status = ame::CommandStatus::SUCCEEDED;
  backend.pushDispatchResult(result);

  auto done_snapshot = backend.readSnapshot();
  ASSERT_EQ(done_snapshot.agent_states.size(), 2u);
  EXPECT_TRUE(done_snapshot.agent_states[0].available);
  EXPECT_TRUE(done_snapshot.agent_states[1].available);
  EXPECT_EQ(done_snapshot.outstanding_goal_dispatches.size(), 0u);
}

TEST(AutonomyBackend, FailedGoalDispatchRestoresAgentAvailability) {
  auto wm = buildDomain();
  auto registry = buildRegistry();

  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::SessionRequest request;
  request.session_id = "dispatch-fail";
  request.intent.goal_fluents = {"(searched sector_a)"};
  request.policy.max_replans = 3;
  request.policy.enable_goal_dispatch = true;
  request.available_agents = {
      {"uav1", "uav", true},
  };
  backend.start(request);
  backend.step();

  auto dispatches = backend.pullGoalDispatches();
  ASSERT_EQ(dispatches.size(), 1u);
  ASSERT_EQ(backend.readSnapshot().agent_states.size(), 1u);
  EXPECT_FALSE(backend.readSnapshot().agent_states[0].available);

  ame::DispatchResult result;
  result.dispatch_id = dispatches[0].dispatch_id;
  result.status = ame::CommandStatus::FAILED_TRANSIENT;
  backend.pushDispatchResult(result);

  auto snapshot = backend.readSnapshot();
  ASSERT_EQ(snapshot.agent_states.size(), 1u);
  EXPECT_TRUE(snapshot.agent_states[0].available);
  EXPECT_EQ(snapshot.state, ame::AutonomyBackendState::READY);
}
