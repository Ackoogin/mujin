#pragma once

#include <string>

namespace mujin {

class WorldModel;

// Minimal STRIPS-level PDDL parser.
// Parses domain.pddl and problem.pddl files into a WorldModel.
// Supports: :strips, :typing requirements. No ADL, no conditional effects.
class PddlParser {
public:
    // Parse a domain file and problem file, populating the given WorldModel.
    // Throws std::runtime_error on parse errors.
    static void parse(const std::string& domain_path,
                      const std::string& problem_path,
                      WorldModel& wm);

    // Parse from string content (for testing without files)
    static void parseFromString(const std::string& domain_pddl,
                                const std::string& problem_pddl,
                                WorldModel& wm);
};

} // namespace mujin
