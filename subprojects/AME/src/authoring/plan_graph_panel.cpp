#include "plan_graph_panel.h"

#include <ame/planner.h>
#include <ame/world_model.h>
#include "imgui.h"

#include <algorithm>
#include <cmath>
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
constexpr float kNodeBodyWidth = 230.0f;
constexpr float kMinLayerGap = 110.0f;
constexpr float kNodeGapY = 46.0f;

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

float fallbackNodeHeight(const std::vector<unsigned>& preconditions,
                         const std::vector<unsigned>& addEffects,
                         const std::vector<unsigned>& delEffects) {
  const int factRows = static_cast<int>(preconditions.size() +
                                        addEffects.size() +
                                        delEffects.size());
  const int sectionRows = (preconditions.empty() ? 0 : 1) +
                          (addEffects.empty() && delEffects.empty() ? 0 : 1);
  return 58.0f + static_cast<float>(factRows + sectionRows) * 22.0f;
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
  m_navigateToContent = false;
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
    m_stepData[static_cast<size_t>(i)].order = i;
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
  m_navigateToContent = true;
}

std::string PlanGraphPanel::fluentLabel(unsigned fluentId) const {
  if (fluentId < m_fluentNames.size()) {
    return m_fluentNames[fluentId];
  }
  return "fluent #" + std::to_string(fluentId);
}

std::string PlanGraphPanel::selectedActionSchemaName() const {
  if (m_selectedStep < 0 ||
      m_selectedStep >= static_cast<int>(m_stepData.size())) {
    return {};
  }

  const std::string& label = m_stepData[static_cast<size_t>(m_selectedStep)].label;
  const size_t paren = label.find('(');
  if (paren == std::string::npos) {
    return label;
  }
  return label.substr(0, paren);
}

void PlanGraphPanel::setSelectedStepForTest(int stepIdx) {
  if (stepIdx < 0 || stepIdx >= static_cast<int>(m_stepData.size())) {
    m_selectedStep = -1;
    if (m_context != nullptr) {
      ed::SetCurrentEditor(m_context);
      ed::ClearSelection();
      ed::SetCurrentEditor(nullptr);
    }
    return;
  }

  m_selectedStep = stepIdx;
  if (m_context != nullptr) {
    ed::SetCurrentEditor(m_context);
    ed::SelectNode(kNodeBase + stepIdx, false);
    ed::SetCurrentEditor(nullptr);
  }
}

void PlanGraphPanel::orderLayersForFewerCrossings() {
  if (m_stepData.empty()) {
    return;
  }

  int maxLayer = 0;
  for (const StepData& step : m_stepData) {
    maxLayer = std::max(maxLayer, step.layer);
  }

  std::vector<std::vector<int>> layers(static_cast<size_t>(maxLayer + 1));
  for (int i = 0; i < static_cast<int>(m_stepData.size()); ++i) {
    const int layer = std::max(0, m_stepData[static_cast<size_t>(i)].layer);
    layers[static_cast<size_t>(layer)].push_back(i);
  }

  auto refreshLayerOrders = [&]() {
    for (std::vector<int>& layerNodes : layers) {
      std::stable_sort(layerNodes.begin(), layerNodes.end(), [&](int lhs, int rhs) {
        const int lhsOrder = m_stepData[static_cast<size_t>(lhs)].order;
        const int rhsOrder = m_stepData[static_cast<size_t>(rhs)].order;
        return lhsOrder == rhsOrder ? lhs < rhs : lhsOrder < rhsOrder;
      });
      for (int order = 0; order < static_cast<int>(layerNodes.size()); ++order) {
        m_stepData[static_cast<size_t>(layerNodes[static_cast<size_t>(order)])].order = order;
      }
    }
  };

  auto neighbourAverage = [&](int nodeIdx, bool incoming) {
    float total = 0.0f;
    int count = 0;
    for (const EdgeData& edge : m_edges) {
      if (incoming && edge.toStep == nodeIdx) {
        total += static_cast<float>(m_stepData[static_cast<size_t>(edge.fromStep)].order);
        ++count;
      } else if (!incoming && edge.fromStep == nodeIdx) {
        total += static_cast<float>(m_stepData[static_cast<size_t>(edge.toStep)].order);
        ++count;
      }
    }
    if (count == 0) {
      return static_cast<float>(m_stepData[static_cast<size_t>(nodeIdx)].order);
    }
    return total / static_cast<float>(count);
  };

  auto sortLayerByNeighbours = [&](int layer, bool incoming) {
    std::vector<int>& layerNodes = layers[static_cast<size_t>(layer)];
    std::stable_sort(layerNodes.begin(), layerNodes.end(), [&](int lhs, int rhs) {
      const float lhsAverage = neighbourAverage(lhs, incoming);
      const float rhsAverage = neighbourAverage(rhs, incoming);
      if (std::fabs(lhsAverage - rhsAverage) > 0.001f) {
        return lhsAverage < rhsAverage;
      }
      const int lhsOrder = m_stepData[static_cast<size_t>(lhs)].order;
      const int rhsOrder = m_stepData[static_cast<size_t>(rhs)].order;
      return lhsOrder == rhsOrder ? lhs < rhs : lhsOrder < rhsOrder;
    });
    for (int order = 0; order < static_cast<int>(layerNodes.size()); ++order) {
      m_stepData[static_cast<size_t>(layerNodes[static_cast<size_t>(order)])].order = order;
    }
  };

  refreshLayerOrders();
  for (int sweep = 0; sweep < 6; ++sweep) {
    for (int layer = 1; layer <= maxLayer; ++layer) {
      sortLayerByNeighbours(layer, true);
    }
    for (int layer = maxLayer - 1; layer >= 0; --layer) {
      sortLayerByNeighbours(layer, false);
    }
  }
}

void PlanGraphPanel::applyLayout() {
  if (m_stepData.empty()) {
    m_layoutDone = true;
    return;
  }

  orderLayersForFewerCrossings();

  int maxLayer = 0;
  for (const StepData& step : m_stepData) {
    maxLayer = std::max(maxLayer, step.layer);
  }

  std::vector<std::vector<int>> layers(static_cast<size_t>(maxLayer + 1));
  std::vector<float> layerWidths(static_cast<size_t>(maxLayer + 1), 0.0f);
  std::vector<float> layerHeights(static_cast<size_t>(maxLayer + 1), 0.0f);

  for (int i = 0; i < static_cast<int>(m_stepData.size()); ++i) {
    const StepData& step = m_stepData[static_cast<size_t>(i)];
    const int layer = std::max(0, step.layer);
    layers[static_cast<size_t>(layer)].push_back(i);

    const ImVec2 fallbackSize(
        kNodeBodyWidth + 28.0f,
        fallbackNodeHeight(step.preconditions, step.add_effects, step.del_effects));
    const ImVec2 size = step.size.x > 1.0f && step.size.y > 1.0f ? step.size : fallbackSize;
    layerWidths[static_cast<size_t>(layer)] =
        std::max(layerWidths[static_cast<size_t>(layer)], size.x);
    layerHeights[static_cast<size_t>(layer)] += size.y;
  }

  float maxLayerHeight = 0.0f;
  for (int layer = 0; layer <= maxLayer; ++layer) {
    std::vector<int>& layerNodes = layers[static_cast<size_t>(layer)];
    std::stable_sort(layerNodes.begin(), layerNodes.end(), [&](int lhs, int rhs) {
      const int lhsOrder = m_stepData[static_cast<size_t>(lhs)].order;
      const int rhsOrder = m_stepData[static_cast<size_t>(rhs)].order;
      return lhsOrder == rhsOrder ? lhs < rhs : lhsOrder < rhsOrder;
    });
    if (layerNodes.size() > 1U) {
      layerHeights[static_cast<size_t>(layer)] +=
          kNodeGapY * static_cast<float>(layerNodes.size() - 1U);
    }
    maxLayerHeight = std::max(maxLayerHeight, layerHeights[static_cast<size_t>(layer)]);
  }

  float x = 80.0f;
  for (int layer = 0; layer <= maxLayer; ++layer) {
    const std::vector<int>& layerNodes = layers[static_cast<size_t>(layer)];
    float y = 80.0f + (maxLayerHeight - layerHeights[static_cast<size_t>(layer)]) * 0.5f;
    for (const int nodeIdx : layerNodes) {
      const StepData& step = m_stepData[static_cast<size_t>(nodeIdx)];
      const ImVec2 fallbackSize(
          kNodeBodyWidth + 28.0f,
          fallbackNodeHeight(step.preconditions, step.add_effects, step.del_effects));
      const ImVec2 size = step.size.x > 1.0f && step.size.y > 1.0f ? step.size : fallbackSize;
      const float centeredX = x + (layerWidths[static_cast<size_t>(layer)] - size.x) * 0.5f;
      ed::SetNodePosition(kNodeBase + nodeIdx, ImVec2(centeredX, y));
      y += size.y + kNodeGapY;
    }
    x += layerWidths[static_cast<size_t>(layer)] + kMinLayerGap;
  }

  m_layoutDone = true;
  if (m_navigateToContent) {
    ed::NavigateToContent(0.0f);
    m_navigateToContent = false;
  }
}

bool PlanGraphPanel::updateMeasuredNodeSizes() {
  bool changed = false;
  for (int i = 0; i < static_cast<int>(m_stepData.size()); ++i) {
    const ImVec2 size = ed::GetNodeSize(kNodeBase + i);
    if (size.x <= 1.0f || size.y <= 1.0f) {
      continue;
    }

    ImVec2& previous = m_stepData[static_cast<size_t>(i)].size;
    if (std::fabs(previous.x - size.x) > 1.0f ||
        std::fabs(previous.y - size.y) > 1.0f) {
      previous = size;
      changed = true;
    }
  }
  return changed;
}

int PlanGraphPanel::selectedStepFromEditor() const {
  const int total = ed::GetSelectedObjectCount();
  if (total <= 0) {
    return -1;
  }

  std::vector<ed::NodeId> selectedNodes(static_cast<size_t>(total));
  const int count = ed::GetSelectedNodes(selectedNodes.data(), total);
  for (int i = 0; i < count; ++i) {
    const int id = static_cast<int>(selectedNodes[static_cast<size_t>(i)].Get());
    if (id >= kNodeBase &&
        id < kNodeBase + static_cast<int>(m_stepData.size())) {
      return id - kNodeBase;
    }
  }
  return -1;
}

void PlanGraphPanel::render() {
  ed::SetCurrentEditor(m_context);
  ed::Begin("##PlanGraph");

  if (!m_layoutDone) {
    applyLayout();
  }

  ed::PushStyleVar(ed::StyleVar_LinkStrength, 80.0f);
  ed::PushStyleVar(ed::StyleVar_SourceDirection, ImVec2(1.0f, 0.0f));
  ed::PushStyleVar(ed::StyleVar_TargetDirection, ImVec2(-1.0f, 0.0f));
  ed::PushStyleVar(ed::StyleVar_SnapLinkToPinDir, 1.0f);
  ed::PushStyleColor(ed::StyleColor_NodeBg, ImVec4(0.20f, 0.10f, 0.40f, 1.0f));
  ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.62f, 0.42f, 1.0f, 0.95f));

  for (int i = 0; i < static_cast<int>(m_stepData.size()); ++i) {
    const StepData& step = m_stepData[static_cast<size_t>(i)];
    const ed::NodeId nodeId = kNodeBase + i;

    ed::BeginNode(nodeId);

    ImGui::TextColored(ImVec4(0.75f, 0.58f, 1.0f, 1.0f), "Step %d", i + 1);
    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + kNodeBodyWidth);
    ImGui::TextWrapped("%s", step.label.empty() ? "(unnamed action)" : step.label.c_str());
    ImGui::PopTextWrapPos();

    if (!step.preconditions.empty()) {
      // Separators stretch across the full node-editor canvas inside
      // ed::BeginNode; spacing keeps this section break local to the node.
      ImGui::Spacing();
      ImGui::TextDisabled("Preconditions");
    }
    for (int pi = 0; pi < static_cast<int>(step.preconditions.size()); ++pi) {
      ed::BeginPin(inputPinId(i, pi), ed::PinKind::Input);
      const std::string label = fluentLabel(step.preconditions[static_cast<size_t>(pi)]);
      ImGui::Text("%s", label.c_str());
      ed::EndPin();
    }

    if (!step.add_effects.empty() || !step.del_effects.empty()) {
      ImGui::Spacing();
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

  const int selectedStepForLinks = selectedStepFromEditor();
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
    const bool selectedLink =
        selectedStepForLinks >= 0 &&
        (edge.fromStep == selectedStepForLinks || edge.toStep == selectedStepForLinks);
    const bool mutedLink = selectedStepForLinks >= 0 && !selectedLink;
    const ImVec4 color = selectedLink
                             ? ImVec4(0.30f, 0.95f, 1.0f, 1.0f)
                             : mutedLink ? ImVec4(0.20f, 0.35f, 0.45f, 0.30f)
                                         : ImVec4(0.0f, 0.72f, 0.95f, 0.72f);
    const float thickness = selectedLink ? 2.5f : 1.15f;
    ed::Link(kLinkBase + i,
             addPinId(edge.fromStep, sourceSlot),
             inputPinId(edge.toStep, targetSlot),
             color,
             thickness);
  }

  if (updateMeasuredNodeSizes()) {
    m_layoutDone = false;
    m_navigateToContent = true;
  }

  ed::PopStyleVar(4);
  ed::End();

  m_selectedStep = selectedStepFromEditor();

  ed::SetCurrentEditor(nullptr);
}
