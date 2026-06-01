#pragma once

#include "ame/world_model.h"

#include <string>
#include <vector>

namespace ame::neuro {

// Symbolic gate contract: every proposal type ships a verifier.
// Verifiers MUST be: deterministic, side-effect-free, operate on a WorldModel snapshot.
template <class Proposal>
class IVerifier {
public:
    struct Verdict {
        bool accepted = false;
        std::string reason;
        std::vector<std::string> evidence; // machine-usable rejection keys
    };
    virtual ~IVerifier() = default;
    virtual Verdict verify(const Proposal& proposal, const ame::WorldModel& wm) const = 0;
};

// Reference verifiers for testing.
template <class Proposal>
class AlwaysReject : public IVerifier<Proposal> {
public:
    explicit AlwaysReject(std::string reason = "always_reject")
        : reason_(std::move(reason)) {}
    typename IVerifier<Proposal>::Verdict
    verify(const Proposal&, const ame::WorldModel&) const override {
        return {false, reason_, {}};
    }
private:
    std::string reason_;
};

template <class Proposal>
class AlwaysAccept : public IVerifier<Proposal> {
public:
    typename IVerifier<Proposal>::Verdict
    verify(const Proposal&, const ame::WorldModel&) const override {
        return {true, "always_accept", {}};
    }
};

} // namespace ame::neuro
