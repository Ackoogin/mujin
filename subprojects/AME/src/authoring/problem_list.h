#pragma once

#include <string>
#include <vector>

struct ProjectModel;
struct StructuralReport;
struct ValidationReport;

/// \brief What kind of element a problem is about, so it can be revealed.
enum class ProblemTarget { None, Fact, Action, Type, Object };

/// \brief One thing wrong with the project, in words a reader can act on.
///
/// `sentence` is what the list shows. `detail` is the raw message underneath it,
/// which is empty unless something else produced text worth keeping: the PDDL
/// parser's own wording, for instance, which names positions in generated text
/// rather than things in the project.
struct ProblemEntry {
  bool isError = true;
  std::string sentence;
  std::string detail;
  ProblemTarget target = ProblemTarget::None;
  std::string targetName;
  /// Index into the project's facts, actions, types or objects, or -1 when the
  /// element named no longer exists.
  int targetIndex = -1;

  bool canReveal() const {
    return target != ProblemTarget::None && targetIndex >= 0;
  }
};

/// \brief Turns the checkers' reports into a list a reader can work through.
///
/// The checkers report what is wrong; this decides how to say it and what to
/// point at. It holds no user-interface state, so the wording and the targeting
/// are testable without opening a window.
class ProblemList {
public:
  /// \brief Every problem the two checkers found, worst first.
  static std::vector<ProblemEntry> build(const ProjectModel& model,
                                         const StructuralReport& structural,
                                         const ValidationReport& validation);

  /// \brief Rewrite planning vocabulary into the words the screens use.
  ///
  /// The checkers speak of predicates because that is what the generated PDDL
  /// calls them. Every screen in this tool says "fact", and a problem list a
  /// non-specialist is expected to work through cannot be the one place that
  /// does not.
  static std::string inPlainWords(const std::string& message);
};
