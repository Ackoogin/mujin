#pragma once

#include <map>
#include <string>
#include <vector>

namespace ame {

class WorldModel;

/// Assignment of goals to a specific agent.
struct AgentGoalAssignment {
    std::string agent_id;
    std::vector<std::string> goals;  // PDDL goal fluents
};

/// \brief Interface for assigning mission goals to available agents.
///
/// Production allocators should derive grouping vocabulary from the loaded
/// domain rather than hard-coded predicate names, argument positions, or sector
/// string parsing.
class IGoalAllocator {
public:
    virtual ~IGoalAllocator() = default;

    /// \brief Allocate goals to available agents.
    virtual std::vector<AgentGoalAssignment> allocate(
        const std::vector<std::string>& goals,
        const WorldModel& wm) const = 0;
};

/// Allocates mission goals to available agents.
///
/// The GoalAllocator supports the PYRAMID leader-delegation pattern where
/// a leader agent plans at the goal level and delegates sub-goals to agents.
///
/// This concrete stub allocator bakes in a same-token sector grouping policy
/// and round-robin sector assignment over agents.
class GoalAllocator : public IGoalAllocator {
public:
    /// Allocate goals to available agents.
    ///
    /// Strategy: Groups goals by sector (extracted from fluent), then
    /// round-robin assigns sector groups to agents. Goals on the same
    /// sector go to the same agent for efficiency.
    ///
    /// \param goals Vector of goal fluent strings, e.g., "(searched sector_a)"
    /// \param wm WorldModel with registered agents
    /// \return Vector of per-agent goal assignments
    std::vector<AgentGoalAssignment> allocate(
        const std::vector<std::string>& goals,
        const WorldModel& wm) const override;

    /// Extract sector/location from a goal fluent string.
    /// E.g., "(searched sector_a)" -> "sector_a"
    /// E.g., "(at uav1 sector_b)" -> "sector_b"
    static std::string extractSector(const std::string& goal);

    /// Check if two goals share the same sector (should be assigned together).
    static bool goalsShareSector(const std::string& g1, const std::string& g2);
};

} // namespace ame
