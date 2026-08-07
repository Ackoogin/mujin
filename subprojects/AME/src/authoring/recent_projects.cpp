#include "recent_projects.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

}  // namespace

std::string RecentProjects::defaultSettingsPath() {
  // The user's own configuration directory where the platform has one, and the
  // working directory otherwise, so the tool never fails to start over this.
  const char* home = std::getenv("HOME");
  if (home != nullptr && home[0] != '\0') {
    return (fs::path(home) / ".ame_authoring_recent.json").string();
  }
  const char* appdata = std::getenv("APPDATA");
  if (appdata != nullptr && appdata[0] != '\0') {
    return (fs::path(appdata) / "ame_authoring_recent.json").string();
  }
  return "ame_authoring_recent.json";
}

std::string RecentProjects::recoveryPathFor(const std::string& projectPath) {
  if (projectPath.empty()) {
    return "";
  }
  return projectPath + ".recovery";
}

std::vector<std::string> RecentProjects::load(const std::string& settingsPath) {
  std::vector<std::string> paths;
  std::ifstream in(settingsPath);
  if (!in.good()) {
    return paths;
  }

  try {
    nlohmann::json json;
    in >> json;
    if (!json.contains("recent") || !json["recent"].is_array()) {
      return paths;
    }
    for (const auto& entry : json["recent"]) {
      if (!entry.is_string()) {
        continue;
      }
      const std::string path = entry.get<std::string>();
      std::error_code error;
      if (path.empty() || !fs::exists(path, error)) {
        continue;
      }
      if (std::find(paths.begin(), paths.end(), path) == paths.end()) {
        paths.push_back(path);
      }
      if (paths.size() >= kMaxEntries) {
        break;
      }
    }
  } catch (const std::exception&) {
    // A settings file this tool cannot read is not worth stopping over: the
    // list is a convenience, and it will be written afresh on the next open.
    paths.clear();
  }
  return paths;
}

bool RecentProjects::remember(const std::string& settingsPath,
                              const std::string& projectPath) {
  if (projectPath.empty()) {
    return false;
  }

  std::vector<std::string> paths = load(settingsPath);
  paths.erase(std::remove(paths.begin(), paths.end(), projectPath), paths.end());
  paths.insert(paths.begin(), projectPath);
  if (paths.size() > kMaxEntries) {
    paths.resize(kMaxEntries);
  }

  nlohmann::json json;
  json["recent"] = paths;
  std::ofstream out(settingsPath);
  if (!out.good()) {
    return false;
  }
  out << json.dump(2) << '\n';
  return static_cast<bool>(out);
}
