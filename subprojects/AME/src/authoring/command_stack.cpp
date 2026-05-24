#include "command_stack.h"

#include <utility>

CommandStack::CommandStack(size_t maxDepth)
  : m_maxDepth(maxDepth) {
}

bool CommandStack::execute(ProjectModel& model,
                           const std::string& label,
                           const std::function<void(ProjectModel&)>& mutation) {
  if (!mutation) {
    return false;
  }

  Entry entry;
  entry.before = model;
  entry.label = label;
  mutation(model);
  entry.after = model;

  // First cut intentionally records every command, including no-op mutations.
  m_undo.push_back(std::move(entry));
  while (m_undo.size() > m_maxDepth) {
    m_undo.pop_front();
  }
  m_redo.clear();
  return true;
}

bool CommandStack::canUndo() const {
  return !m_undo.empty();
}

bool CommandStack::canRedo() const {
  return !m_redo.empty();
}

bool CommandStack::undo(ProjectModel& model) {
  if (!canUndo()) {
    return false;
  }

  Entry entry = std::move(m_undo.back());
  m_undo.pop_back();
  model = entry.before;
  m_redo.push_back(std::move(entry));
  return true;
}

bool CommandStack::redo(ProjectModel& model) {
  if (!canRedo()) {
    return false;
  }

  Entry entry = std::move(m_redo.back());
  m_redo.pop_back();
  model = entry.after;
  m_undo.push_back(std::move(entry));
  return true;
}

std::string CommandStack::topUndoLabel() const {
  return canUndo() ? m_undo.back().label : std::string();
}

std::string CommandStack::topRedoLabel() const {
  return canRedo() ? m_redo.back().label : std::string();
}

void CommandStack::clear() {
  m_undo.clear();
  m_redo.clear();
}
