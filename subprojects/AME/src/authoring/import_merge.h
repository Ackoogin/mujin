#pragma once

#include "project_model.h"

#include <string>
#include <vector>

/// \brief What kind of thing an import would bring in.
enum class MergeKind { Type, Fact, Action, Object };

/// \brief What would happen to one element if the import went ahead.
enum class MergeDisposition {
  Added,      // the project has nothing by this name
  Replaced,   // the project has one by this name, and it is different
  Unchanged,  // the project has one by this name, and it is the same
};

/// \brief One element the import would bring in, and what it would do.
struct MergeItem {
  MergeKind kind = MergeKind::Fact;
  std::string name;
  MergeDisposition disposition = MergeDisposition::Added;
  /// What the user would lose, for something being replaced: a short phrase
  /// such as "3 conditions and 2 outcomes".
  std::string whatWouldBeLost;
};

/// \brief Whether each kind of thing may be replaced by the import.
///
/// Adding is never a loss, so it is not a choice. Replacing is, so it is.
struct MergeChoices {
  bool replaceTypes = false;
  bool replaceFacts = false;
  bool replaceActions = false;
  bool replaceObjects = false;

  bool replaces(MergeKind kind) const;
};

/// \brief Everything an import would do, before it does any of it.
struct MergePlan {
  std::vector<MergeItem> items;

  size_t countAdded() const;
  size_t countReplaced() const;
  size_t countUnchanged() const;
  /// \brief The elements this import would overwrite, by name.
  std::vector<std::string> wouldOverwrite() const;
  bool anythingWouldBeOverwritten() const { return countReplaced() > 0; }
};

/// \brief Bringing a second domain into a project that already has one.
///
/// Importing used to replace the project, which loses everything the user had
/// done. This works out what the import would do first, so the user sees what
/// would be overwritten before committing, and then does only the parts they
/// agreed to.
class ImportMerge {
public:
  /// \brief What the import would do to the project as it stands.
  static MergePlan plan(const ProjectModel& current,
                        const ProjectModel& incoming);

  /// \brief Carry out the parts of the plan the choices allow.
  static ProjectModel apply(const ProjectModel& current,
                            const ProjectModel& incoming,
                            const MergeChoices& choices);

  /// \brief Lay elements out by what they have to do with each other.
  ///
  /// An import used to arrange everything in two long rows, facts along one and
  /// actions along the other, which says nothing about the domain. This puts
  /// each action beside the facts it uses, so the first thing a user sees after
  /// an import is the shape of what they imported.
  static void layoutByRelationships(ProjectModel& model);
};
