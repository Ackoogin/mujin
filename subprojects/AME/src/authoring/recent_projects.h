#pragma once

#include <string>
#include <vector>

/// \brief The projects this user opened last, most recent first.
///
/// Kept beside the tool's other settings rather than in any project, because
/// it belongs to the person rather than to the work. A path that no longer
/// exists is dropped when the list is read, so the menu never offers something
/// that cannot be opened.
class RecentProjects {
public:
  /// \brief How many are remembered.
  static constexpr size_t kMaxEntries = 8;

  /// \brief Read the list, dropping anything that is no longer there.
  static std::vector<std::string> load(const std::string& settingsPath);

  /// \brief Put a project at the top of the list and write it back.
  static bool remember(const std::string& settingsPath,
                       const std::string& projectPath);

  /// \brief Where the list is kept when the caller has no preference.
  static std::string defaultSettingsPath();

  /// \brief The recovery copy written beside a project while it has unsaved
  /// changes, and removed once it is saved.
  static std::string recoveryPathFor(const std::string& projectPath);
};
