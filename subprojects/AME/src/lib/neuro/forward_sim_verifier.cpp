#include "ame/neuro/forward_sim_verifier.h"

#include "ame/world_model.h"

namespace ame::neuro {

ForwardSimVerifier::ForwardSimVerifier(AuthorityView view) : view_(view) {}

bool ForwardSimVerifier::fact_trusted(unsigned id,
                                       const ame::WorldStateData& snap,
                                       std::vector<std::string>& evidence_out) const {
    bool raw_value = snap.getFact(id);
    if (!raw_value) return false; // false regardless of authority

    const auto& meta = snap.getMetadata(id);
    switch (view_) {
        case AuthorityView::All:
            return true;

        case AuthorityView::ConfirmedThenBelieved:
            if (meta.authority == ame::FactAuthority::BELIEVED) {
                evidence_out.push_back("believed:" + std::to_string(id));
            }
            return true;

        case AuthorityView::ConfirmedOnly:
            return (meta.authority == ame::FactAuthority::CONFIRMED);
    }
    return true;
}

IVerifier<PlanProposal>::Verdict
ForwardSimVerifier::verify(const PlanProposal& proposal,
                            const ame::WorldModel& wm) const {
    Verdict v;

    if (proposal.steps.empty() && proposal.goal_fluent_keys.empty()) {
        v.accepted = true;
        v.reason = "empty_plan_accepted";
        return v;
    }

    // Take a snapshot — never mutates authoritative state.
    auto snap_ptr = wm.captureSnapshot();
    const auto& snap = *snap_ptr;

    // Copy the mutable state bits for simulation.
    std::vector<uint64_t> sim_bits = snap.state_bits;
    std::vector<ame::FactMetadata> sim_meta = snap.fact_metadata;

    const auto& ground_actions = wm.groundActions();
    const unsigned num_fluents = wm.numFluents();

    // Helper: get sim fact value.
    auto sim_get = [&](unsigned id) -> bool {
        unsigned word = id / 64;
        unsigned bit  = id % 64;
        if (word >= sim_bits.size()) return false;
        return (sim_bits[word] >> bit) & 1u;
    };
    // Helper: check authority for simulation state.
    auto sim_meta_get = [&](unsigned id) -> const ame::FactMetadata& {
        static const ame::FactMetadata empty{};
        if (id >= sim_meta.size()) return empty;
        return sim_meta[id];
    };
    auto sim_trusted = [&](unsigned id, std::vector<std::string>& ev) -> bool {
        if (!sim_get(id)) return false;
        const auto& m = sim_meta_get(id);
        switch (view_) {
            case AuthorityView::All:                 return true;
            case AuthorityView::ConfirmedThenBelieved:
                if (m.authority == ame::FactAuthority::BELIEVED)
                    ev.push_back("believed:" + std::to_string(id));
                return true;
            case AuthorityView::ConfirmedOnly:
                return (m.authority == ame::FactAuthority::CONFIRMED);
        }
        return true;
    };
    // Helper: set sim fact.
    auto sim_set = [&](unsigned id, bool val) {
        if (id >= num_fluents) return;
        unsigned word = id / 64;
        unsigned bit  = id % 64;
        while (sim_bits.size() <= word) sim_bits.push_back(0);
        if (val)
            sim_bits[word] |=  (uint64_t(1) << bit);
        else
            sim_bits[word] &= ~(uint64_t(1) << bit);
        // Effects of plan steps are BELIEVED (plan-applied predictions).
        if (sim_meta.size() <= id) sim_meta.resize(id + 1);
        sim_meta[id].authority = ame::FactAuthority::BELIEVED;
    };

    for (unsigned step_idx = 0; step_idx < proposal.steps.size(); ++step_idx) {
        const auto& step = proposal.steps[step_idx];
        if (step.action_index >= ground_actions.size()) {
            v.accepted = false;
            v.reason = "invalid_action_index_at_step_" + std::to_string(step_idx);
            v.evidence.push_back("step:" + std::to_string(step_idx));
            return v;
        }
        const auto& ga = ground_actions[step.action_index];

        // Check preconditions.
        for (unsigned pre_id : ga.preconditions) {
            std::vector<std::string> ev;
            if (!sim_trusted(pre_id, ev)) {
                v.accepted = false;
                v.reason = "precondition_failed_at_step_" + std::to_string(step_idx);
                v.evidence.push_back("step:" + std::to_string(step_idx));
                v.evidence.push_back("precondition:" + wm.fluentName(pre_id));
                v.evidence.insert(v.evidence.end(), ev.begin(), ev.end());
                return v;
            }
            v.evidence.insert(v.evidence.end(), ev.begin(), ev.end());
        }

        // Apply effects.
        for (unsigned add_id : ga.add_effects) sim_set(add_id, true);
        for (unsigned del_id : ga.del_effects)  sim_set(del_id, false);
    }

    // Check goal satisfaction.
    for (const auto& goal_key : proposal.goal_fluent_keys) {
        unsigned gid = 0;
        try {
            gid = wm.fluentIndex(goal_key);
        } catch (...) {
            v.accepted = false;
            v.reason = "unknown_goal_fluent";
            v.evidence.push_back("goal:" + goal_key);
            return v;
        }
        std::vector<std::string> ev;
        if (!sim_trusted(gid, ev)) {
            v.accepted = false;
            v.reason = "goal_not_achieved";
            v.evidence.push_back("goal:" + goal_key);
            return v;
        }
        v.evidence.insert(v.evidence.end(), ev.begin(), ev.end());
    }

    v.accepted = true;
    v.reason = "plan_verified";
    return v;
}

} // namespace ame::neuro
