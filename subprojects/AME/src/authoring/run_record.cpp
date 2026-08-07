#include "run_record.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>

namespace {

namespace fs = std::filesystem;

constexpr const char* kBtFilename = "ame_bt_events.jsonl";
constexpr const char* kWmFilename = "ame_wm_audit.jsonl";
constexpr const char* kPlanFilename = "ame_plan_audit.jsonl";
constexpr const char* kManifestFilename = "run.json";

std::vector<RunJsonEvent> readJsonLines(const fs::path& path) {
  std::vector<RunJsonEvent> lines;
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty()) {
      lines.push_back({0U, line});
    }
  }
  return lines;
}

bool writeJsonLines(const fs::path& path,
                    const std::vector<RunJsonEvent>& lines) {
  std::ofstream output(path);
  if (!output) {
    return false;
  }
  for (const RunJsonEvent& line : lines) {
    nlohmann::json json = nlohmann::json::parse(line.json);
    json["tick"] = line.tick;
    output << json.dump() << '\n';
  }
  return static_cast<bool>(output);
}

std::string actionName(const std::string& signature) {
  const size_t paren = signature.find('(');
  return paren == std::string::npos ? signature : signature.substr(0, paren);
}

RunNodeStatus statusFromText(const std::string& status) {
  if (status == "RUNNING") {
    return RunNodeStatus::Happening;
  }
  if (status == "SUCCESS") {
    return RunNodeStatus::Finished;
  }
  if (status == "FAILURE") {
    return RunNodeStatus::WentWrong;
  }
  return RunNodeStatus::Waiting;
}

std::vector<std::string> actionDifference(
    const std::vector<RunActionStep>& left,
    const std::vector<RunActionStep>& right) {
  std::map<std::string, int> counts;
  for (const RunActionStep& step : left) {
    if (step.startTick != 0U) {
      ++counts[step.signature];
    }
  }
  for (const RunActionStep& step : right) {
    if (step.startTick != 0U) {
      --counts[step.signature];
    }
  }
  std::vector<std::string> difference;
  for (const auto& entry : counts) {
    for (int i = 0; i < entry.second; ++i) {
      difference.push_back(entry.first);
    }
  }
  return difference;
}

std::map<std::string, bool> factValues(const RunState& state) {
  std::map<std::string, bool> values;
  for (const RunFactState& fact : state.facts) {
    values[fact.fact] = fact.value;
  }
  return values;
}

}  // namespace

RecordedRun RecordedRun::fromSimulation(const ProjectModel& model,
                                        const SimulationEngine& simulation) {
  RecordedRun run;
  run.manifest_.project = model.projectName;
  run.manifest_.scenario = simulation.scenarioName();
  run.manifest_.seed = simulation.seed();
  run.manifest_.faults = simulation.faults();
  run.manifest_.simulated = true;
  run.manifest_.timeBasis = "simulated_tick_time";
  run.manifest_.tickPeriodSeconds = 1.0 / simulation.ticksPerSecond();
  run.parseEvents(simulation.btAuditEvents(), simulation.wmAuditEvents(),
                  simulation.planAuditEvents());
  run.addMissingInitialWorldEvents();
  run.useSimulatedTickTimestamps();
  return run;
}

bool RecordedRun::load(const std::string& folder) {
  *this = RecordedRun{};
  folder_ = folder;
  const fs::path root(folder);
  if (!fs::is_directory(root)) {
    error_ = "the recorded run folder does not exist";
    return false;
  }
  for (const char* filename : {kBtFilename, kWmFilename, kPlanFilename}) {
    if (!fs::is_regular_file(root / filename)) {
      error_ = std::string("the recorded run is missing ") + filename;
      return false;
    }
  }

  const fs::path manifest_path = root / kManifestFilename;
  if (fs::is_regular_file(manifest_path)) {
    try {
      nlohmann::json json;
      std::ifstream input(manifest_path);
      input >> json;
      manifest_.version = json.value("version", 1);
      manifest_.project = json.value("project", "");
      manifest_.scenario = json.value("scenario", "");
      manifest_.seed = json.value("seed", 0U);
      manifest_.simulated = json.value("simulated", false);
      manifest_.timeBasis = json.value("timeBasis", "");
      manifest_.tickPeriodSeconds = json.value("tickPeriodSeconds", 0.0);
      if (json.contains("faults")) {
        manifest_.faults = json.at("faults").get<RunFaultSet>();
      }
    } catch (const std::exception& exception) {
      error_ = std::string("the run manifest could not be read: ") +
               exception.what();
      return false;
    }
  } else {
    // Runtime recordings made before manifests existed are real runs unless
    // their data explicitly says otherwise. This is the safe direction: the
    // tool must never label field evidence as a simulation by guesswork.
    manifest_.simulated = false;
  }

  return parseEvents(readJsonLines(root / kBtFilename),
                     readJsonLines(root / kWmFilename),
                     readJsonLines(root / kPlanFilename));
}

bool RecordedRun::save(const std::string& folder) const {
  if (!loaded_) {
    return false;
  }
  try {
    const fs::path root(folder);
    fs::create_directories(root);
    nlohmann::json manifest = {
        {"version", manifest_.version},
        {"project", manifest_.project},
        {"scenario", manifest_.scenario},
        {"seed", manifest_.seed},
        {"faults", manifest_.faults},
        {"simulated", manifest_.simulated},
        {"timeBasis", manifest_.timeBasis},
        {"tickPeriodSeconds", manifest_.tickPeriodSeconds}};
    std::ofstream manifest_output(root / kManifestFilename);
    manifest_output << manifest.dump(2) << '\n';
    if (!manifest_output) {
      return false;
    }

    std::vector<RunJsonEvent> bt;
    for (const BtEvent& event : bt_events_) {
      nlohmann::json json = {{"ts_us", event.tsUs},
                             {"node", event.node},
                             {"type", event.type},
                             {"prev", event.previous},
                             {"status", event.status},
                             {"tree_id", event.treeId},
                             {"wm_version", event.wmVersion}};
      bt.push_back({event.tick, json.dump()});
    }
    std::vector<RunJsonEvent> wm;
    for (const WmEvent& event : wm_events_) {
      nlohmann::json json = {{"wm_version", event.wmVersion},
                             {"ts_us", event.tsUs},
                             {"fact", event.fact},
                             {"value", event.value},
                             {"source", event.source}};
      wm.push_back({event.tick, json.dump()});
    }
    std::vector<RunJsonEvent> plan;
    for (const PlanEvent& event : plan_events_) {
      nlohmann::json json = nlohmann::json::parse(event.json);
      json["ts_us"] = event.tsUs;
      plan.push_back({event.tick, json.dump()});
    }
    return writeJsonLines(root / kBtFilename, bt) &&
           writeJsonLines(root / kWmFilename, wm) &&
           writeJsonLines(root / kPlanFilename, plan);
  } catch (const std::exception&) {
    return false;
  }
}

bool RecordedRun::parseEvents(const std::vector<RunJsonEvent>& bt,
                              const std::vector<RunJsonEvent>& wm,
                              const std::vector<RunJsonEvent>& plan) {
  try {
    std::set<uint64_t> timestamps;
    const auto collect_timestamp = [&timestamps](const RunJsonEvent& line) {
      const nlohmann::json json = nlohmann::json::parse(line.json);
      timestamps.insert(json.value("ts_us", uint64_t{0}));
    };
    for (const auto& line : bt) {
      collect_timestamp(line);
    }
    for (const auto& line : wm) {
      collect_timestamp(line);
    }
    std::map<uint64_t, unsigned> inferred_tick;
    unsigned ordinal = 0;
    for (const uint64_t timestamp : timestamps) {
      inferred_tick[timestamp] = ordinal++;
    }

    for (const RunJsonEvent& line : bt) {
      const nlohmann::json json = nlohmann::json::parse(line.json);
      BtEvent event;
      event.tsUs = json.value("ts_us", uint64_t{0});
      event.tick = json.value("tick", line.tick != 0U
                                          ? line.tick
                                          : inferred_tick[event.tsUs]);
      event.node = json.value("node", "");
      event.type = json.value("type", "");
      event.previous = json.value("prev", "");
      event.status = json.value("status", "");
      event.treeId = json.value("tree_id", "");
      event.wmVersion = json.value("wm_version", uint64_t{0});
      bt_events_.push_back(std::move(event));
    }
    for (const RunJsonEvent& line : wm) {
      const nlohmann::json json = nlohmann::json::parse(line.json);
      WmEvent event;
      event.wmVersion = json.value("wm_version", uint64_t{0});
      event.tsUs = json.value("ts_us", uint64_t{0});
      event.tick = json.value("tick", line.tick != 0U
                                          ? line.tick
                                          : inferred_tick[event.tsUs]);
      event.fact = json.value("fact", "");
      event.value = json.value("value", false);
      event.source = json.value("source", "");
      wm_events_.push_back(std::move(event));
    }
    for (const RunJsonEvent& line : plan) {
      const nlohmann::json json = nlohmann::json::parse(line.json);
      PlanEvent event;
      event.tsUs = json.value("ts_us", uint64_t{0});
      event.tick = json.value("tick", line.tick);
      event.initFacts = json.value("init_facts", std::vector<std::string>{});
      event.goals = json.value("goal_fluents", std::vector<std::string>{});
      event.actions = json.value("plan_actions", std::vector<std::string>{});
      event.btXml = json.value("bt_xml", "");
      event.json = json.dump();
      plan_events_.push_back(std::move(event));
    }
  } catch (const std::exception& exception) {
    error_ = std::string("a recorded event could not be read: ") +
             exception.what();
    return false;
  }
  loaded_ = true;
  deriveViews();
  return true;
}

void RecordedRun::addMissingInitialWorldEvents() {
  if (!manifest_.simulated || plan_events_.empty()) {
    return;
  }

  std::set<std::string> recorded_at_tick_zero;
  for (const WmEvent& event : wm_events_) {
    if (event.tick == 0U && event.value) {
      recorded_at_tick_zero.insert(event.fact);
    }
  }

  std::vector<WmEvent> initial_events;
  for (const std::string& fact : plan_events_.front().initFacts) {
    if (recorded_at_tick_zero.count(fact) != 0U) {
      continue;
    }
    WmEvent event;
    event.tsUs = plan_events_.front().tsUs;
    event.tick = 0U;
    event.fact = fact;
    event.value = true;
    event.source = "scenario";
    initial_events.push_back(std::move(event));
  }
  wm_events_.insert(wm_events_.begin(), initial_events.begin(),
                    initial_events.end());
}

void RecordedRun::useSimulatedTickTimestamps() {
  if (!manifest_.simulated || manifest_.tickPeriodSeconds <= 0.0) {
    return;
  }

  uint64_t start_us = std::numeric_limits<uint64_t>::max();
  for (const BtEvent& event : bt_events_) {
    start_us = std::min(start_us, event.tsUs);
  }
  for (const WmEvent& event : wm_events_) {
    start_us = std::min(start_us, event.tsUs);
  }
  for (const PlanEvent& event : plan_events_) {
    start_us = std::min(start_us, event.tsUs);
  }
  if (start_us == std::numeric_limits<uint64_t>::max()) {
    start_us = 0U;
  }

  const auto timestamp_for_tick = [&](unsigned tick) {
    const double offset_us = static_cast<double>(tick) *
                             manifest_.tickPeriodSeconds * 1000000.0;
    return start_us + static_cast<uint64_t>(std::llround(offset_us));
  };
  for (BtEvent& event : bt_events_) {
    event.tsUs = timestamp_for_tick(event.tick);
  }
  for (WmEvent& event : wm_events_) {
    event.tsUs = timestamp_for_tick(event.tick);
  }
  for (PlanEvent& event : plan_events_) {
    event.tsUs = timestamp_for_tick(event.tick);
  }
}

void RecordedRun::deriveViews() {
  std::set<std::string> planned_actions;
  for (const PlanEvent& episode : plan_events_) {
    planned_actions.insert(episode.actions.begin(), episode.actions.end());
    goal_facts_ = episode.goals;
    compiled_xml_ = episode.btXml;
    max_tick_ = std::max(max_tick_, episode.tick);
  }
  if (!plan_events_.empty()) {
    initial_facts_ = plan_events_.front().initFacts;
  }

  std::map<std::string, size_t> action_index;
  std::set<std::string> seen_nodes;
  for (const BtEvent& event : bt_events_) {
    max_tick_ = std::max(max_tick_, event.tick);
    if (seen_nodes.insert(event.node).second) {
      const bool is_action = planned_actions.count(event.node) != 0U ||
                             event.type == "SimulatedAction";
      RunNode node;
      node.name = event.node;
      node.nodeType = event.type;
      node.isAction = is_action;
      nodes_.push_back(std::move(node));
      if (is_action) {
        action_index[event.node] = action_steps_.size();
        RunActionStep step;
        step.signature = event.node;
        step.actionName = actionName(event.node);
        action_steps_.push_back(std::move(step));
      }
    }
    const auto found = action_index.find(event.node);
    if (found == action_index.end()) {
      continue;
    }
    const RunNodeStatus status = statusFromText(event.status);
    size_t step_index = found->second;
    if (status == RunNodeStatus::Happening &&
        action_steps_[step_index].endTick != 0U) {
      RunActionStep next;
      next.signature = event.node;
      next.actionName = actionName(event.node);
      step_index = action_steps_.size();
      action_steps_.push_back(std::move(next));
      action_index[event.node] = step_index;
    }
    RunActionStep& step = action_steps_[step_index];
    if (status == RunNodeStatus::Happening && step.startTick == 0U) {
      step.startTick = event.tick;
    } else if (status == RunNodeStatus::Finished ||
               status == RunNodeStatus::WentWrong) {
      step.endTick = event.tick;
    }
    step.status = status;
  }

  for (const WmEvent& event : wm_events_) {
    max_tick_ = std::max(max_tick_, event.tick);
    if (event.tick == 0U || event.source == "scenario" ||
        event.source == "planner_init") {
      if (event.value &&
          std::find(initial_facts_.begin(), initial_facts_.end(), event.fact) ==
              initial_facts_.end()) {
        initial_facts_.push_back(event.fact);
      }
      continue;
    }
    fact_changes_.push_back(
        {event.tick, event.fact, event.value, event.source});
  }
}

RunState RecordedRun::stateAtTick(unsigned requested_tick) const {
  RunState state;
  state.tick = std::min(requested_tick, max_tick_);
  std::map<std::string, RunFactState> facts;
  for (const std::string& fact : initial_facts_) {
    facts[fact] = {fact, true, 0U, false};
  }
  for (const WmEvent& event : wm_events_) {
    if (event.tick > state.tick) {
      continue;
    }
    RunFactState& fact = facts[event.fact];
    fact.fact = event.fact;
    fact.value = event.value;
    fact.lastChangedTick = event.tick;
    fact.changedDuringRun = event.tick != 0U;
  }
  for (const auto& entry : facts) {
    state.facts.push_back(entry.second);
  }

  state.actionSteps = action_steps_;
  for (RunActionStep& step : state.actionSteps) {
    if (step.startTick == 0U || step.startTick > state.tick) {
      step.status = RunNodeStatus::Waiting;
    } else if (step.endTick == 0U || step.endTick > state.tick) {
      step.status = RunNodeStatus::Happening;
    }
  }
  return state;
}

std::vector<RunGoal> RecordedRun::goalsAtTick(unsigned tick) const {
  const RunState state = stateAtTick(tick);
  std::vector<RunGoal> goals;
  for (const std::string& name : goal_facts_) {
    const auto found = std::find_if(
        state.facts.begin(), state.facts.end(),
        [&name](const RunFactState& fact) { return fact.fact == name; });
    goals.push_back({name, found != state.facts.end() && found->value});
  }
  return goals;
}

std::map<std::string, std::string> RecordedRun::treeStateAtTick(
    unsigned requested_tick) const {
  std::map<std::string, std::string> state;
  for (const BtEvent& event : bt_events_) {
    if (event.tick <= requested_tick && event.status != "IDLE") {
      state[event.node] = event.status;
    }
  }
  return state;
}

std::string RecordedRun::summaryLine() const {
  const RunState state = stateAtTick(max_tick_);
  const size_t finished = static_cast<size_t>(std::count_if(
      state.actionSteps.begin(), state.actionSteps.end(),
      [](const RunActionStep& step) {
        return step.status == RunNodeStatus::Finished;
      }));
  const std::vector<RunGoal> goals = goalsAtTick(max_tick_);
  const size_t met = static_cast<size_t>(std::count_if(
      goals.begin(), goals.end(), [](const RunGoal& goal) { return goal.met; }));
  std::ostringstream text;
  text << "recorded run · tick " << max_tick_ << " · " << finished << " of "
       << action_steps_.size() << " actions done · " << met << " of "
       << goals.size() << " goals met";
  return text.str();
}

RunComparison compareRuns(const RecordedRun& first,
                          const RecordedRun& second) {
  RunComparison comparison;
  const unsigned last_tick = std::max(first.tick(), second.tick());
  for (unsigned tick = 0; tick <= last_tick; ++tick) {
    if (first.treeStateAtTick(tick) != second.treeStateAtTick(tick)) {
      comparison.treesDiffer = true;
      comparison.firstDifferentTick = tick;
      break;
    }
  }
  comparison.actionsOnlyInFirst =
      actionDifference(first.actionSteps(), second.actionSteps());
  comparison.actionsOnlyInSecond =
      actionDifference(second.actionSteps(), first.actionSteps());

  const auto first_facts = factValues(first.stateAtTick(first.tick()));
  const auto second_facts = factValues(second.stateAtTick(second.tick()));
  std::set<std::string> names;
  for (const auto& fact : first_facts) {
    names.insert(fact.first);
  }
  for (const auto& fact : second_facts) {
    names.insert(fact.first);
  }
  for (const std::string& name : names) {
    const auto first_value = first_facts.find(name);
    const auto second_value = second_facts.find(name);
    const bool left = first_value != first_facts.end() && first_value->second;
    const bool right = second_value != second_facts.end() && second_value->second;
    if (left != right) {
      comparison.endFactDifferences.push_back(
          name + ": " + (left ? "true" : "false") + " -> " +
          (right ? "true" : "false"));
    }
  }

  std::ostringstream summary;
  if (!comparison.treesDiffer && comparison.actionsOnlyInFirst.empty() &&
      comparison.actionsOnlyInSecond.empty() &&
      comparison.endFactDifferences.empty()) {
    summary << "The two runs followed the same tree and ended with the same facts.";
  } else if (comparison.treesDiffer) {
    summary << "The two runs are the same until tick "
            << comparison.firstDifferentTick << ".";
  } else {
    summary << "The trees match, but the runs ended with "
            << comparison.endFactDifferences.size() << " different fact"
            << (comparison.endFactDifferences.size() == 1U ? "." : "s.");
  }
  if (!comparison.actionsOnlyInSecond.empty()) {
    summary << " The second run added " << comparison.actionsOnlyInSecond.size()
            << " action"
            << (comparison.actionsOnlyInSecond.size() == 1U ? "." : "s.");
  }
  comparison.summary = summary.str();
  return comparison;
}
