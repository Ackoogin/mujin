#include <gtest/gtest.h>

#include "ame/action_registry.h"
#include "ame/current_ame_backend_adapter.h"
#include "ame/execution_sink.h"
#include "ame/world_model.h"

#include <memory>
#include <string>
#include <vector>

namespace {

ame::ActionCommand makeMoveCommand() {
  ame::ActionCommand command;
  command.command_id = "session/svc/1";
  command.action_name = "move";
  command.signature = "move(uav1, base, sector_a)";
  command.service_name = "mobility";
  command.operation = "move";
  command.request_fields = {
      {"robot", "uav1"},
      {"from", "base"},
      {"to", "sector_a"},
  };
  return command;
}

ame::ExecutionBinding moveBinding() {
  ame::ExecutionBinding binding;
  binding.action_name = "move";
  binding.service_name = "mobility";
  binding.operation = "move";
  binding.target_component = "mobility";
  binding.target_service = "Move_Service";
  binding.target_type = "pyramid.data_model.mobility.MoveRequirement";
  return binding;
}

ame::ExecutionBinding searchBinding() {
  ame::ExecutionBinding binding;
  binding.action_name = "search";
  binding.service_name = "imaging";
  binding.operation = "search";
  binding.target_component = "sensor_tasking";
  binding.target_service = "Area_Search_Service";
  binding.target_type = "pyramid.data_model.sensor.AreaSearchRequirement";
  return binding;
}

ame::CommandResult succeeded(const std::string& command_id,
                             std::vector<ame::FactUpdate> updates = {}) {
  ame::CommandResult result;
  result.command_id = command_id;
  result.status = ame::CommandStatus::SUCCEEDED;
  result.observed_updates = std::move(updates);
  result.source = "test";
  return result;
}

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

const ame::RequirementPlacementRecord* findPlacement(
    const std::vector<ame::RequirementPlacementRecord>& placements,
    const std::string& action_name) {
  for (const auto& placement : placements) {
    if (placement.action_name == action_name) {
      return &placement;
    }
  }
  return nullptr;
}

}  // namespace

TEST(ExecutionSink, CommandQueueExposesCommandsAndWaitsForResults) {
  ame::CommandQueueExecutionSink sink;
  sink.reset("session");

  const auto command = makeMoveCommand();
  const auto submission = sink.submit(command);
  EXPECT_TRUE(submission.accepted);
  EXPECT_TRUE(submission.command_egress_visible);
  EXPECT_FALSE(submission.placement.has_value());

  auto commands = sink.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front().command_id, command.command_id);
  EXPECT_TRUE(sink.pullCommands().empty());
  EXPECT_TRUE(sink.isPending(command.command_id));
  EXPECT_FALSE(sink.resultFor(command.command_id).has_value());

  sink.pushResult(succeeded(command.command_id));
  ASSERT_TRUE(sink.resultFor(command.command_id).has_value());
  EXPECT_EQ(sink.resultFor(command.command_id)->status,
            ame::CommandStatus::SUCCEEDED);
  EXPECT_FALSE(sink.isPending(command.command_id));
}

TEST(ExecutionSink, RequirementBindingConvertsBoundCommandToPlacement) {
  ame::RequirementBindingExecutionSink sink(
      ame::ExecutionBindingPolicy::RequireTypedPlacement, {moveBinding()});
  sink.reset("session");

  const auto command = makeMoveCommand();
  const auto submission = sink.submit(command);
  EXPECT_TRUE(submission.accepted);
  EXPECT_FALSE(submission.command_egress_visible);
  ASSERT_TRUE(submission.placement.has_value());
  EXPECT_EQ(submission.placement->target_component, "mobility");
  EXPECT_EQ(submission.placement->target_service, "Move_Service");
  EXPECT_EQ(submission.placement->target_type,
            "pyramid.data_model.mobility.MoveRequirement");
  EXPECT_TRUE(sink.pullCommands().empty());

  auto placements = sink.readPlacements();
  ASSERT_EQ(placements.size(), 1u);
  EXPECT_EQ(placements.front().state, ame::RequirementPlacementState::Running);
  EXPECT_FALSE(placements.front().target_requirement_id.empty());

  sink.pushResult(succeeded(command.command_id));
  placements = sink.readPlacements();
  ASSERT_EQ(placements.size(), 1u);
  EXPECT_EQ(placements.front().state, ame::RequirementPlacementState::Completed);
  ASSERT_TRUE(sink.resultFor(command.command_id).has_value());
  EXPECT_EQ(sink.resultFor(command.command_id)->status,
            ame::CommandStatus::SUCCEEDED);
  EXPECT_FALSE(sink.isPending(command.command_id));
}

TEST(ExecutionSink, PreferTypedPlacementFallsBackToCommandWhenUnbound) {
  ame::RequirementBindingExecutionSink sink(
      ame::ExecutionBindingPolicy::PreferTypedPlacement, {});
  sink.reset("session");

  const auto command = makeMoveCommand();
  const auto submission = sink.submit(command);
  EXPECT_TRUE(submission.accepted);
  EXPECT_TRUE(submission.command_egress_visible);
  EXPECT_FALSE(submission.placement.has_value());
  EXPECT_TRUE(sink.readPlacements().empty());

  auto commands = sink.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front().command_id, command.command_id);
}

TEST(ExecutionSink, RequireTypedPlacementRejectsUnboundCommand) {
  ame::RequirementBindingExecutionSink sink(
      ame::ExecutionBindingPolicy::RequireTypedPlacement, {});
  sink.reset("session");

  const auto command = makeMoveCommand();
  const auto submission = sink.submit(command);
  EXPECT_FALSE(submission.accepted);
  EXPECT_FALSE(submission.command_egress_visible);
  EXPECT_FALSE(submission.rejection_reason.empty());
  EXPECT_TRUE(sink.pullCommands().empty());
  EXPECT_TRUE(sink.readPlacements().empty());
  ASSERT_TRUE(sink.resultFor(command.command_id).has_value());
  EXPECT_EQ(sink.resultFor(command.command_id)->status,
            ame::CommandStatus::FAILED_PERMANENT);
}

TEST(ExecutionSink, BackendCanExecuteThroughTypedPlacementsWithoutCommandEgress) {
  auto wm = buildDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  auto registry = buildRegistry();
  auto sink = std::make_shared<ame::RequirementBindingExecutionSink>(
      ame::ExecutionBindingPolicy::RequireTypedPlacement,
      std::vector<ame::ExecutionBinding>{moveBinding(), searchBinding()});

  ame::CurrentAmeBackendAdapter backend(
      wm, registry, ame::Planner(), ame::PlanCompiler(), sink);
  backend.start({"session", {{"(searched sector_a)"}}, {3}});

  backend.step();
  EXPECT_TRUE(backend.pullCommands().empty());
  auto placements = sink->readPlacements();
  ASSERT_EQ(placements.size(), 1u);
  auto* move = findPlacement(placements, "move");
  ASSERT_NE(move, nullptr);
  EXPECT_EQ(move->state, ame::RequirementPlacementState::Running);
  EXPECT_EQ(move->target_component, "mobility");
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::WAITING_FOR_RESULTS);

  backend.pushCommandResult(succeeded(
      move->command_id,
      {
          {"(at uav1 base)", false, "mobility", ame::FactAuthorityLevel::CONFIRMED},
          {"(at uav1 sector_a)", true, "mobility", ame::FactAuthorityLevel::CONFIRMED},
      }));

  for (int i = 0; i < 4 && sink->readPlacements().size() < 2; ++i) {
    backend.step();
  }

  EXPECT_TRUE(backend.pullCommands().empty());
  placements = sink->readPlacements();
  ASSERT_EQ(placements.size(), 2u);
  auto* search = findPlacement(placements, "search");
  ASSERT_NE(search, nullptr);
  EXPECT_EQ(search->state, ame::RequirementPlacementState::Running);
  EXPECT_EQ(search->target_component, "sensor_tasking");
  EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));

  backend.pushCommandResult(succeeded(
      search->command_id,
      {{"(searched sector_a)",
        true,
        "sensor_tasking",
        ame::FactAuthorityLevel::CONFIRMED}}));
  backend.step();

  EXPECT_EQ(backend.readSnapshot().state, ame::AutonomyBackendState::COMPLETE);
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  placements = sink->readPlacements();
  ASSERT_EQ(placements.size(), 2u);
  for (const auto& placement : placements) {
    EXPECT_EQ(placement.state, ame::RequirementPlacementState::Completed);
  }
}
