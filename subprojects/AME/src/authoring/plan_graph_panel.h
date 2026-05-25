#pragma once

#include <imgui_node_editor.h>

#include <string>
#include <vector>

namespace ed = ax::NodeEditor;

namespace ame {
struct PlanResult;
class WorldModel;
}

/// \brief Read-only view of a plan as a causal dependency graph.
class PlanGraphPanel {
public:
  PlanGraphPanel();
  ~PlanGraphPanel();

  /// \brief Push a new plan to display.
  ///
  /// Snapshots all data needed for rendering so the WorldModel does not need
  /// to outlive this call.
  void setPlan(const ame::PlanResult& plan,
               const ame::WorldModel& wm,
               const std::string& scenarioName);

  /// \brief Clear the displayed plan.
  void clear();

  /// \brief Render the current plan inside the caller's ImGui scope.
  void render();

  /// \brief Index of the currently selected plan step, or -1 if none.
  int selectedStepIndex() const { return m_selectedStep; }

  /// \brief Number of plan steps currently displayed.
  size_t stepCount() const { return m_stepData.size(); }

  /// \brief Label rendered as the step body.
  const std::string& stepLabel(size_t i) const { return m_stepData[i].label; }

  /// \brief Preconditions for a displayed step.
  const std::vector<unsigned>& stepPreconditions(size_t i) const {
    return m_stepData[i].preconditions;
  }

  /// \brief Add effects for a displayed step.
  const std::vector<unsigned>& stepAddEffects(size_t i) const {
    return m_stepData[i].add_effects;
  }

  /// \brief Delete effects for a displayed step.
  const std::vector<unsigned>& stepDelEffects(size_t i) const {
    return m_stepData[i].del_effects;
  }

  /// \brief Human-readable fluent label captured from the source WorldModel.
  std::string fluentLabel(unsigned fluentId) const;

private:
  struct StepData {
    std::string label;
    std::vector<unsigned> preconditions;
    std::vector<unsigned> add_effects;
    std::vector<unsigned> del_effects;
    int layer = 0;
  };

  struct EdgeData {
    int fromStep;
    int toStep;
    unsigned fluentId;
  };

  std::vector<StepData> m_stepData;
  std::vector<std::string> m_fluentNames;
  std::vector<EdgeData> m_edges;
  std::string m_scenarioName;
  ed::EditorContext* m_context = nullptr;
  int m_selectedStep = -1;
  bool m_layoutDone = false;
};
