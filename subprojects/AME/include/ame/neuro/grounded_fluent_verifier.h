#pragma once

#include "ame/neuro/verifier.h"

#include <functional>
#include <string>
#include <vector>

namespace ame::neuro {

// Proposal type: a set of ground fluent keys (e.g. proposed goal fluents).
struct FluentProposal {
    std::vector<std::string> fluent_keys;
};

// Verifies that every proposed fluent key resolves to a grounded entry
// in the WorldModel. Unknown keys are rejected with each invalid key cited.
//
// An optional allow-list hook provides "goal authorization" governance:
// even grounded keys can be blocked if they violate deployment policy.
class GroundedFluentVerifier : public IVerifier<FluentProposal> {
public:
    using AllowListFn = std::function<bool(const std::string&)>;

    // allow_list: if provided, each grounded key is additionally checked.
    // Returning false blocks the key even if it is grounded.
    explicit GroundedFluentVerifier(AllowListFn allow_list = nullptr);

    Verdict verify(const FluentProposal& proposal,
                   const ame::WorldModel& wm) const override;

private:
    AllowListFn allow_list_;
};

} // namespace ame::neuro
