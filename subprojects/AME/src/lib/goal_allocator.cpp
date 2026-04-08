#include "ame/goal_allocator.h"
#include "ame/world_model.h"

#include <algorithm>
#include <map>
#include <sstream>

namespace ame {

std::string GoalAllocator::extractSector(const std::string& goal) {
    // Parse fluent like "(searched sector_a)" or "(at uav1 sector_b)"
    // Extract the last token before the closing paren as the sector/location
    
    if (goal.empty()) return "";
    
    // Find content between parens
    size_t start = goal.find('(');
    size_t end = goal.rfind(')');
    if (start == std::string::npos || end == std::string::npos || end <= start) {
        return "";
    }
    
    std::string content = goal.substr(start + 1, end - start - 1);
    
    // Tokenize by whitespace
    std::istringstream iss(content);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    
    // Return last token as sector (common pattern: predicate args... sector)
    if (tokens.size() >= 2) {
        return tokens.back();
    }
    
    return "";
}

bool GoalAllocator::goalsShareSector(const std::string& g1, const std::string& g2) {
    std::string s1 = extractSector(g1);
    std::string s2 = extractSector(g2);
    return !s1.empty() && s1 == s2;
}

std::vector<AgentGoalAssignment> GoalAllocator::allocate(
    const std::vector<std::string>& goals,
    const WorldModel& wm) const {
    auto agents = wm.availableAgentIds();
    if (agents.empty() || goals.empty()) {
        return {};
    }
    
    // Group goals by sector
    std::map<std::string, std::vector<std::string>> sector_goals;
    std::vector<std::string> no_sector_goals;  // Goals without identifiable sector
    
    for (const auto& goal : goals) {
        std::string sector = extractSector(goal);
        if (sector.empty()) {
            no_sector_goals.push_back(goal);
        } else {
            sector_goals[sector].push_back(goal);
        }
    }
    
    // Initialize assignments for each agent
    std::vector<AgentGoalAssignment> assignments;
    assignments.reserve(agents.size());
    for (const auto& agent_id : agents) {
        assignments.push_back({agent_id, {}});
    }
    
    // Round-robin assign sector groups to agents
    size_t agent_idx = 0;
    for (const auto& [sector, sg] : sector_goals) {
        for (const auto& g : sg) {
            assignments[agent_idx].goals.push_back(g);
        }
        agent_idx = (agent_idx + 1) % agents.size();
    }
    
    // Distribute remaining no-sector goals round-robin
    for (const auto& g : no_sector_goals) {
        assignments[agent_idx].goals.push_back(g);
        agent_idx = (agent_idx + 1) % agents.size();
    }
    
    // Remove empty assignments
    assignments.erase(
        std::remove_if(assignments.begin(), assignments.end(),
                       [](const AgentGoalAssignment& a) { return a.goals.empty(); }),
        assignments.end());
    
    return assignments;
}

} // namespace ame
