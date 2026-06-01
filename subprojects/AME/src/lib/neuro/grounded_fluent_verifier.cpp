#include "ame/neuro/grounded_fluent_verifier.h"

#include "ame/world_model.h"

namespace ame::neuro {

GroundedFluentVerifier::GroundedFluentVerifier(AllowListFn allow_list)
    : allow_list_(std::move(allow_list)) {}

IVerifier<FluentProposal>::Verdict
GroundedFluentVerifier::verify(const FluentProposal& proposal,
                                const ame::WorldModel& wm) const {
    Verdict v;
    v.accepted = true;

    const unsigned num = wm.numFluents();
    for (const auto& key : proposal.fluent_keys) {
        // fluentIndex throws for unknown keys; treat exceptions as "not grounded".
        unsigned idx = 0;
        bool grounded = false;
        try {
            idx = wm.fluentIndex(key);
            grounded = (idx < num) && (wm.fluentName(idx) == key);
        } catch (...) {
            grounded = false;
        }

        if (!grounded) {
            v.accepted = false;
            v.evidence.push_back(key);
            continue;
        }

        if (allow_list_ && !allow_list_(key)) {
            v.accepted = false;
            v.evidence.push_back("disallowed:" + key);
        }
    }

    if (!v.accepted) {
        v.reason = "unknown_or_disallowed_fluents";
    } else {
        v.reason = "all_fluents_grounded";
    }
    return v;
}

} // namespace ame::neuro
