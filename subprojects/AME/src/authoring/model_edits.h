#pragma once

#include "project_model.h"

#include <string>
#include <vector>

/// \brief Something copied, waiting to be pasted.
///
/// A fact and an action are the two things worth copying, so the clipboard
/// holds one of each rather than a variant: whichever was copied last is the
/// one that pastes.
struct ElementClipboard {
  enum class Kind { Nothing, Fact, Action };

  Kind kind = Kind::Nothing;
  PredicateDef fact;
  ActionDef action;

  bool holdsSomething() const { return kind != Kind::Nothing; }
  /// \brief What a menu entry should say, e.g. "Paste 'move'".
  std::string description() const;
};

/// \brief Edits to a project that touch more than the thing being edited.
///
/// Renaming a type is the clearest case: the name appears on the type, on
/// every parameter that uses it, on every object of it, and on any type that
/// has it as a parent. Doing that in the drawing code would mean doing it
/// again the next time another screen offered the same edit, so it lives here,
/// where it is one function with tests around it.
class ModelEdits {
public:
  /// \brief Rename a type everywhere it is used.
  /// \return False when the name is empty, unchanged, or already taken.
  static bool renameType(ProjectModel& model,
                         const std::string& oldName,
                         const std::string& newName);

  /// \brief Why a rename would be refused, for showing before it is attempted.
  static std::string whyTypeCannotBeRenamed(const ProjectModel& model,
                                            const std::string& oldName,
                                            const std::string& newName);

  /// \brief Move one of an action's parameters earlier or later.
  ///
  /// The conditions and outcomes refer to parameters by name, so they follow
  /// the move without being touched. What changes is the order the action is
  /// written in, and so the order its arguments are given in.
  static bool moveActionParameter(ProjectModel& model,
                                  size_t actionIndex,
                                  size_t parameterIndex,
                                  bool later);

  /// \brief A name like the one given that nothing else is using.
  static std::string unusedName(const ProjectModel& model,
                                const std::string& wanted,
                                bool forAction);

  /// \brief Copy a fact into the clipboard.
  static bool copyFact(const ProjectModel& model,
                       size_t factIndex,
                       ElementClipboard& clipboard);

  /// \brief Copy an action into the clipboard.
  static bool copyAction(const ProjectModel& model,
                         size_t actionIndex,
                         ElementClipboard& clipboard);

  /// \brief Paste whatever the clipboard holds, under a name nothing is using.
  /// \return The name the pasted element was given, or empty when nothing was
  /// pasted.
  static std::string paste(ProjectModel& model,
                           const ElementClipboard& clipboard);
};
