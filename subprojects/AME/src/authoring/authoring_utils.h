#pragma once

#include "project_model.h"

#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace authoring {

inline std::string slugify(const std::string& text) {
  std::string out;
  bool lastWasDash = false;

  for (unsigned char ch : text) {
    const char lower = static_cast<char>(std::tolower(ch));
    const bool isSlugChar =
        (lower >= 'a' && lower <= 'z') || (lower >= '0' && lower <= '9') ||
        lower == '-';
    if (isSlugChar) {
      if (lower == '-') {
        if (!out.empty() && !lastWasDash) {
          out.push_back(lower);
        }
        lastWasDash = true;
      } else {
        out.push_back(lower);
        lastWasDash = false;
      }
    } else if (!out.empty() && !lastWasDash) {
      out.push_back('-');
      lastWasDash = true;
    }
  }

  while (!out.empty() && out.back() == '-') {
    out.pop_back();
  }

  if (out.empty()) {
    return "untitled";
  }
  return out;
}

inline std::string projectFilePath(const std::string& projectName) {
  return "./" + slugify(projectName) + ".ameproj.json";
}

inline std::string importDomainPath() {
  return "./import_domain.pddl";
}

inline std::string importProblemPath() {
  return "./import_problem.pddl";
}

inline std::string domainPddlPath(const std::string& projectName) {
  return "./" + slugify(projectName) + "-domain.pddl";
}

inline std::string problemPddlPath(const std::string& projectName,
                                   const std::string& scenarioName) {
  return "./" + slugify(projectName) + "-problem-" +
         slugify(scenarioName) + ".pddl";
}

inline std::string regressionReportPath(const std::string& projectName) {
  return "./" + slugify(projectName) + "-regression-report.json";
}

inline std::string formatEffectRef(const EffectRef& ref) {
  std::ostringstream out;
  out << "(" << ref.predicateName;
  for (const auto& arg : ref.argNames) {
    out << " " << arg;
  }
  out << ")";
  return out.str();
}

inline std::string formatFactRef(const FactRef& fact) {
  std::ostringstream out;
  out << "(" << fact.predicateName;
  for (const auto& arg : fact.objectNames) {
    out << " " << arg;
  }
  out << ")";
  return out.str();
}

inline std::string formatArgList(const std::vector<std::string>& args) {
  std::ostringstream out;
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0U) {
      out << " ";
    }
    out << args[i];
  }
  return out.str();
}

} // namespace authoring
