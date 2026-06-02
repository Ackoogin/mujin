#pragma once

#include "ame/neuro/authority_view.h"
#include "ame/neuro/verifier.h"
#include "ame/plan_compiler.h"

#include <string>
#include <vector>

namespace ame::neuro {

// Proposal type: a sequence of plan steps plus the goal fluents to achieve.
struct PlanProposal {
    std::vector<ame::PlanStep> steps;
    std::vector<std::string> goal_fluent_keys;
};

// Forward-simulation verifier for proposed plan sequences.
//
// Applies each step to a *copy* of the world state, checking preconditions and
// effects in sequence, then confirms the goal is satisfied.  Operates on a
// WorldStateData snapshot — never mutates authoritative state.
//
// The AuthorityView parameter controls which facts are trusted; see WI-2.3.
class ForwardSimVerifier : public IVerifier<PlanProposal> {
public:
    explicit ForwardSimVerifier(AuthorityView view = AuthorityView::All);

    Verdict verify(const PlanProposal& proposal,
                   const ame::WorldModel& wm) const override;

private:
    AuthorityView view_;

    // Returns true (and flags evidence) if the fluent should be trusted given view_.
    bool fact_trusted(unsigned id, const ame::WorldStateData& snap,
                      std::vector<std::string>& evidence_out) const;
};

} // namespace ame::neuro
