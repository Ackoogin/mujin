#pragma once

#include "project_model.h"

#include <cstddef>
#include <deque>
#include <functional>
#include <string>

/// \brief Snapshot-based undo/redo command stack for authoring operations.
class CommandStack {
public:
  explicit CommandStack(size_t maxDepth = 100);

  bool execute(ProjectModel& model,
               const std::string& label,
               const std::function<void(ProjectModel&)>& mutation);

  bool canUndo() const;
  bool canRedo() const;
  bool undo(ProjectModel& model);
  bool redo(ProjectModel& model);

  std::string topUndoLabel() const;
  std::string topRedoLabel() const;
  size_t maxDepth() const { return m_maxDepth; }
  size_t undoDepth() const { return m_undo.size(); }
  void clear();

private:
  struct Entry {
    ProjectModel before;
    ProjectModel after;
    std::string label;
  };

  std::deque<Entry> m_undo;
  std::deque<Entry> m_redo;
  size_t m_maxDepth;
};
