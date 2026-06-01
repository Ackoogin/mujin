#pragma once

namespace ame::neuro {

// Determines which facts a verifier trusts from a WorldStateData snapshot.
// Each fluent carries a single (value, authority) pair — no shadow store.
//
// See doc/plans/AME/neuro_symbolic_infrastructure_plan.md WI-2.3 for the
// complete semantics and the "out of scope / future" note on dual shadow stores.
enum class AuthorityView {
    // Trust each fluent's stored value regardless of authority tag.
    All,

    // Trust stored value; annotate Verdict.evidence when a BELIEVED fact was
    // relied on, so the caller knows the result leaned on a prediction.
    ConfirmedThenBelieved,

    // A fluent counts as true only if its value is true AND authority == CONFIRMED.
    // BELIEVED facts are treated as not-yet-established for verification purposes.
    ConfirmedOnly
};

} // namespace ame::neuro
