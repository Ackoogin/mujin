#include "plan_graph_panel.h"

#include <ame/planner.h>
#include <ame/world_model.h>
#include "imgui.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int kNodeBase = 7000;
constexpr int kInputPinBase = 8000;
constexpr int kAddPinBase = 9000;
constexpr int kDelPinBase = 10000;
constexpr int kLinkBase = 11000;
constexpr int kPinStride = 100;

ed::PinId inputPinId(int stepIdx, int slotIdx) {
  return kInputPinBase + stepIdx * kPinStride + slotIdx;
}

ed::PinId addPinId(int stepIdx, int slotIdx) {
  return kAddPinBase + stepIdx * kPinStride + slotIdx;
}

ed::PinId delPinId(int stepIdx, int slotIdx) {
  return kDelPinBase + stepIdx * kPinStride + slotIdx;
}

template <typename T>
bool containsValue(const std::vector<T>& values, const T& value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

PlanGraphPanel::PlanGraphPanel() {
  ed::Config cfg;
  cfg.SettingsFile = nullptr;
  m_context = ed::CreateEditor(&cfg);
}

PlanGraphPanel::~PlanGraphPanel() {
  ed::DestroyEditor(m_context);
}

void PlanGraphPanel::clear() {
  if (m_context != nullptr) {
    ed::SetCurrentEditor(m_context);
    ed::ClearSelection();
    ed::SetCurrentEditor(nullptr);
  }
  m_stepData.clear();
  m_fluentNames.clear();
  m_edges.clear();
  m_scenarioName.clear();
  m_selectedStep = -1;
  m_layoutDone = false;
}

void PlanGraphPanel::setPlan(const ame::PlanResult& plan,
                             const ame::WorldModel& wm,
                             const std::string& scenarioName) {
  clear();
  if (!plan.success) {
    return;
  }

  m_scenarioName = scenarioName;
  m_fluentNames.reserve(wm.numFluents());
  for (unsigned i = 0; i < wm.numFluents(); ++i) {
    m_fluentNames.push_back(wm.fluentName(i));
  }

  const auto& groundActions = wm.groundActions();
  m_stepData.reserve(plan.steps.size());
  for (const auto& step : plan.steps) {
    StepData data;
    if (step.action_index < groundActions.size()) {
      const ame::GroundAction& action = groundActions[step.action_index];
      data.label = action.signature;
      data.preconditions = action.preconditions;
      data.add_effects = action.add_effects;
      data.del_effects = action.del_effects;
    } else {
      data.label = "action #" + std::to_string(step.action_index);
    }
    m_stepData.push_back(std::move(data));
  }

  for (int i = 0; i < static_cast<int>(m_stepData.size()); ++i) {
    const StepData& source = m_stepData[static_cast<size_t>(i)];
    for (int j = i + 1; j < static_cast<int>(m_stepData.size()); ++j) {
      const StepData& target = m_stepData[static_cast<size_t>(j)];
      for (const unsigned fluentId : source.add_effects) {
        if (containsValue(target.preconditions, fluentId)) {
          m_edges.push_back({i, j, fluentId});
        }
      }
    }
  }

  for (const EdgeData& edge : m_edges) {
    StepData& target = m_stepData[static_cast<size_t>(edge.toStep)];
    const StepData& source = m_stepData[static_cast<size_t>(edge.fromStep)];
    target.layer = std::max(target.layer, source.layer + 1);
  }

  m_layoutDone = false;
}

std::string PlanGraphPanel::fluentLabel(unsigned fluentId) const {
  if (fluentId < m_fluentNames.size()) {
    return m_fluentNames[fluentId];
  }
  return "fluent #" + std::to_string(fluentId);
}

void PlanGraphPanel::render() {
  ed::SetCurrentEditor(m_context);
  ed::Begin("##PlanGraph");

  if (!m_layoutDone) {
    int maxLayer = 0;
    for (const StepData& step : m_stepData) {
      maxLayer = std::max(maxLayer, step.layer);
    }
    std::vector<int> layerCounts(static_cast<size_t>(maxLayer + 1), 0);
    for (int i = 0; i < static_cast<int>(m_stepData.size()); ++i) {
      const int layer = std::max(0, m_stepData[static_cast<size_t>(i)].layer);
      const int ySlot = layerCounts[static_cast<size_t>(layer)]++;
      ed::SetNodePosition(kNodeBase + i,
                          ImVec2(80.0f + static_cast<float>(layer) * 240.0f,
                                 80.0f + static_cast<float>(ySlot) * 140.0f));
    }
    m_layoutDone = true;
  }

  ed::PushStyleColor(ed::StyleColor_NodeBg, ImVec4(0.20f, 0.10f, 0.40f, 1.0f));
  ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.62f, 0.42f, 1.0f, 0.95f));

  for (int i = 0; i < static_cast<int>(m_stepData.size()); ++i) {
    const StepData& step = m_stepData[static_cast<size_t>(i)];
    const ed::NodeId nodeId = kNodeBase + i;

    ed::BeginNode(nodeId);

    ImGui::TextColored(ImVec4(0.75f, 0.58f, 1.0f, 1.0f), "Step %d", i + 1);
    ImGui::TextWrapped("%s", step.label.empty() ? "(unnamed action)" : step.label.c_str());

    if (!step.preconditions.empty()) {
      ImGui::Separator();
      ImGui::TextDisabled("Preconditions");
    }
    for (int pi = 0; pi < static_cast<int>(step.preconditions.size()); ++pi) {
      ed::BeginPin(inputPinId(i, pi), ed::PinKind::Input);
      const std::string label = fluentLabel(step.preconditions[static_cast<size_t>(pi)]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }

    if (!step.add_effects.empty() || !step.del_effects.empty()) {
      ImGui::Separator();
      ImGui::TextDisabled("Effects");
    }
    for (int ai = 0; ai < static_cast<int>(step.add_effects.size()); ++ai) {
      ed::BeginPin(addPinId(i, ai), ed::PinKind::Output);
      const std::string label = "+" + fluentLabel(step.add_effects[static_cast<size_t>(ai)]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }
    for (int di = 0; di < static_cast<int>(step.del_effects.size()); ++di) {
      ed::BeginPin(delPinId(i, di), ed::PinKind::Output);
      const std::string label = "-" + fluentLabel(step.del_effects[static_cast<size_t>(di)]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }

    ed::EndNode();
  }

  ed::PopStyleColor(2);

  for (int i = 0; i < static_cast<int>(m_edges.size()); ++i) {
    const EdgeData& edge = m_edges[static_cast<size_t>(i)];
    const StepData& sourceStep = m_stepData[static_cast<size_t>(edge.fromStep)];
    const StepData& targetStep = m_stepData[static_cast<size_t>(edge.toStep)];

    const auto sourceIt =
        std::find(sourceStep.add_effects.begin(), sourceStep.add_effects.end(), edge.fluentId);
    const auto targetIt =
        std::find(targetStep.preconditions.begin(), targetStep.preconditions.end(), edge.fluentId);
    if (sourceIt == sourceStep.add_effects.end() ||
        targetIt == targetStep.preconditions.end()) {
      continue;
    }

    const int sourceSlot =
        static_cast<int>(std::distance(sourceStep.add_effects.begin(), sourceIt));
    const int targetSlot =
        static_cast<int>(std::distance(targetStep.preconditions.begin(), targetIt));
    ed::Link(kLinkBase + i,
             addPinId(edge.fromStep, sourceSlot),
             inputPinId(edge.toStep, targetSlot),
             ImVec4(0.0f, 0.8f, 1.0f, 1.0f));
  }

  ed::End();

  m_selectedStep = -1;
  const int total = ed::GetSelectedObjectCount();
  if (total > 0) {
    std::vector<ed::NodeId> selectedNodes(static_cast<size_t>(total));
    const int count = ed::GetSelectedNodes(selectedNodes.data(), total);
    for (int i = 0; i < count; ++i) {
      const int id = static_cast<int>(selectedNodes[static_cast<size_t>(i)].Get());
      if (id >= kNodeBase &&
          id < kNodeBase + static_cast<int>(m_stepData.size())) {
        m_selectedStep = id - kNodeBase;
      }
    }
  }

  ed::SetCurrentEditor(nullptr);
}
