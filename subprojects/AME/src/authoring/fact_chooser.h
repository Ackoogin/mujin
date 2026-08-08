#pragma once

#include "project_model.h"

#include <string>
#include <vector>

/// \brief One choice offered for a fact, and whether it can be taken.
///
/// A choice that is not legal stays in the list, greyed, with the reason beside
/// it, so a user learns the rule rather than wondering where the option went.
struct FactChoice {
  std::string name;
  bool allowed = true;
  std::string reason;   // why not, when it is not allowed
};

/// \brief Builds the choices for saying which fact is true, without typing it.
///
/// The user picks the fact, then picks each object it involves from a list
/// holding only objects of the right type. That is what makes a fact the
/// project does not have impossible to express rather than reported afterwards.
class FactChooser {
public:
  /// \brief The facts a scenario can say something about.
  static std::vector<FactChoice> facts(const ProjectModel& model);

  /// \brief The objects allowed in one position of a fact.
  ///
  /// Objects of the wrong type are returned too, marked as not allowed and
  /// carrying the reason, so the list can show them greyed.
  static std::vector<FactChoice> objectsFor(const ProjectModel& model,
                                            const std::string& factName,
                                            size_t position);

  /// \brief How many objects the named fact involves.
  static size_t arity(const ProjectModel& model, const std::string& factName);

  /// \brief Whether a chosen fact and its objects make something the project has.
  /// \return Empty when it does; otherwise the reason it does not.
  static std::string whyNotValid(const ProjectModel& model,
                                 const FactRef& fact);

  /// \brief Read a typed fact, "at uav1 base" or "(at uav1 base)".
  ///
  /// The typed path stays for users who prefer it, and is held to the same
  /// rules as the chosen path.
  static FactRef parse(const std::string& text);

  /// \brief Whether one type is the same as, or a kind of, another.
  static bool typeMatches(const ProjectModel& model,
                          const std::string& objectType,
                          const std::string& requiredType);
};
