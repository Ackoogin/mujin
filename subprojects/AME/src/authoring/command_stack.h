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

  /// \brief Record an edit that continues the one before it.
  ///
  /// Typing a name is one intention and many keystrokes. Each keystroke calls
  /// this with the same key — something identifying the field being typed in,
  /// such as "action:3:name" — and they fold into a single undoable step, so
  /// one press of undo puts back the name the user started with rather than
  /// removing one letter.
  bool executeCoalescing(ProjectModel& model,
                         const std::string& label,
                         const std::string& coalesceKey,
                         const std::function<void(ProjectModel&)>& mutation);

  /// \brief End the run of edits being folded together.
  ///
  /// Called when the user moves away from a field, so that returning to it
  /// later starts a new undoable step rather than extending the old one.
  void endCoalescing() { m_coalesceKey.clear(); }

  bool canUndo() const;
  bool canRedo() const;
  bool undo(ProjectModel& model);
  bool redo(ProjectModel& model);

  /// \brief How many edits this stack has recorded, ever.
  ///
  /// Always moves forward, unlike the undo depth, which is capped and does not
  /// change when a run of keystrokes folds into the step before it. Anything
  /// asking "has the project changed since I last looked" has to use this.
  size_t editCount() const { return m_editCount; }

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

  std::string m_coalesceKey;
  size_t m_editCount = 0;

  std::deque<Entry> m_undo;
  std::deque<Entry> m_redo;
  size_t m_maxDepth;
};
