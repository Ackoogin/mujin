// Phase 2 tests: GroundedFluentVerifier, ForwardSimVerifier, AuthorityView
// WI-2.1, WI-2.2, WI-2.3

#include "ame/neuro/authority_view.h"
#include "ame/neuro/forward_sim_verifier.h"
#include "ame/neuro/grounded_fluent_verifier.h"
#include "ame/world_model.h"

#include <climits>
#include <gtest/gtest.h>

using namespace ame::neuro;

// ---------------------------------------------------------------------------
// Fixture: simple two-sector search domain
// ---------------------------------------------------------------------------
class VerifierFixture : public ::testing::Test {
protected:
    void SetUp() override {
        auto& ts = wm_.typeSystem();
        ts.addType("object");
        ts.addType("robot", "object");
        ts.addType("sector", "object");

        wm_.addObject("uav1", "robot");
        wm_.addObject("s1", "sector");
        wm_.addObject("s2", "sector");

        wm_.registerPredicate("at",       {"robot", "sector"});
        wm_.registerPredicate("searched", {"sector"});

        // move: at(?r,?from) -> at(?r,?to), -at(?r,?from)
        wm_.registerAction("move",
            {"?r", "?from", "?to"},
            {"robot", "sector", "sector"},
            {"(at ?r ?from)"},
            {"(at ?r ?to)"},
            {"(at ?r ?from)"});

        // search: at(?r,?s) -> searched(?s)
        wm_.registerAction("search",
            {"?r", "?s"},
            {"robot", "sector"},
            {"(at ?r ?s)"},
            {"(searched ?s)"},
            {});

        // Initial state: uav1 at s1
        wm_.setFact("(at uav1 s1)", true, "test", ame::FactAuthority::CONFIRMED);
    }

    ame::WorldModel wm_;
};

// ---------------------------------------------------------------------------
// GroundedFluentVerifier (WI-2.1)
// ---------------------------------------------------------------------------

TEST_F(VerifierFixture, GroundedFluent_AcceptsKnownKeys) {
    GroundedFluentVerifier v;
    FluentProposal p;
    p.fluent_keys = {"(at uav1 s1)", "(searched s2)"};
    auto verdict = v.verify(p, wm_);
    EXPECT_TRUE(verdict.accepted);
    EXPECT_TRUE(verdict.evidence.empty());
}

TEST_F(VerifierFixture, GroundedFluent_RejectsUnknownKey) {
    GroundedFluentVerifier v;
    FluentProposal p;
    p.fluent_keys = {"(at uav1 s1)", "(nonexistent pred)"};
    auto verdict = v.verify(p, wm_);
    EXPECT_FALSE(verdict.accepted);
    // Evidence must name every invalid key
    ASSERT_FALSE(verdict.evidence.empty());
    bool found = false;
    for (const auto& e : verdict.evidence)
        if (e.find("nonexistent") != std::string::npos) found = true;
    EXPECT_TRUE(found);
}

TEST_F(VerifierFixture, GroundedFluent_AllowListBlocks) {
    GroundedFluentVerifier v([](const std::string& key) {
        return key == "(at uav1 s1)"; // only allow at uav1 s1
    });
    FluentProposal p;
    p.fluent_keys = {"(at uav1 s1)", "(searched s1)"};
    auto verdict = v.verify(p, wm_);
    EXPECT_FALSE(verdict.accepted);
    bool found = false;
    for (const auto& e : verdict.evidence)
        if (e.find("disallowed") != std::string::npos) found = true;
    EXPECT_TRUE(found);
}

TEST_F(VerifierFixture, GroundedFluent_EmptyProposalAccepted) {
    GroundedFluentVerifier v;
    FluentProposal p;
    auto verdict = v.verify(p, wm_);
    EXPECT_TRUE(verdict.accepted);
}

// ---------------------------------------------------------------------------
// ForwardSimVerifier (WI-2.2)
// ---------------------------------------------------------------------------

// Helper: find action index by signature prefix
static unsigned find_action(const ame::WorldModel& wm, const std::string& sig_prefix) {
    const auto& gas = wm.groundActions();
    for (unsigned i = 0; i < gas.size(); ++i) {
        if (gas[i].signature.rfind(sig_prefix, 0) == 0) return i;
    }
    return UINT_MAX;
}

TEST_F(VerifierFixture, ForwardSim_ValidPlanAccepted) {
    // Plan: move(uav1,s1,s2), search(uav1,s2)
    ForwardSimVerifier v;
    PlanProposal p;
    unsigned move_idx   = find_action(wm_, "move(uav1,s1,s2)");
    unsigned search_idx = find_action(wm_, "search(uav1,s2)");
    ASSERT_NE(move_idx, UINT_MAX);
    ASSERT_NE(search_idx, UINT_MAX);

    p.steps = {{move_idx}, {search_idx}};
    p.goal_fluent_keys = {"(searched s2)"};
    auto verdict = v.verify(p, wm_);
    EXPECT_TRUE(verdict.accepted) << verdict.reason;
}

TEST_F(VerifierFixture, ForwardSim_ViolatedPreconditionRejected) {
    // Try search(uav1,s2) without moving there first — precondition (at uav1 s2) fails
    ForwardSimVerifier v;
    PlanProposal p;
    unsigned search_s2 = find_action(wm_, "search(uav1,s2)");
    ASSERT_NE(search_s2, UINT_MAX);

    p.steps = {{search_s2}};
    p.goal_fluent_keys = {"(searched s2)"};
    auto verdict = v.verify(p, wm_);
    EXPECT_FALSE(verdict.accepted);
    EXPECT_NE(verdict.reason.find("precondition_failed"), std::string::npos);
    // Evidence must cite the failing step
    bool found = false;
    for (const auto& e : verdict.evidence)
        if (e.find("step:0") != std::string::npos) found = true;
    EXPECT_TRUE(found);
}

TEST_F(VerifierFixture, ForwardSim_GoalNotAchievedRejected) {
    // Plan that achieves s2 but goal asks for s1 — goal not met
    ForwardSimVerifier v;
    PlanProposal p;
    unsigned move_idx   = find_action(wm_, "move(uav1,s1,s2)");
    unsigned search_idx = find_action(wm_, "search(uav1,s2)");
    ASSERT_NE(move_idx, UINT_MAX);
    ASSERT_NE(search_idx, UINT_MAX);

    p.steps = {{move_idx}, {search_idx}};
    p.goal_fluent_keys = {"(searched s1)"}; // wrong goal
    auto verdict = v.verify(p, wm_);
    EXPECT_FALSE(verdict.accepted);
    EXPECT_NE(verdict.reason.find("goal_not_achieved"), std::string::npos);
}

// ---------------------------------------------------------------------------
// ForwardSimVerifier: AuthorityView (WI-2.3)
// Three views on the same snapshot with a BELIEVED precondition
// ---------------------------------------------------------------------------

TEST_F(VerifierFixture, AuthorityView_ThreeVerdictsFromSameSnapshot) {
    // Add a new action that requires (searched s1) as precondition.
    wm_.registerAction("classify",
        {"?r", "?s"},
        {"robot", "sector"},
        {"(at ?r ?s)", "(searched ?s)"},
        {},
        {});

    // Set (searched s1) as BELIEVED only
    wm_.setFact("(searched s1)", true, "plan", ame::FactAuthority::BELIEVED);

    // Move uav1 to s1 first (already there, so use a plan that starts from s1)
    // Plan: classify(uav1, s1)
    unsigned classify_idx = find_action(wm_, "classify(uav1,s1)");
    ASSERT_NE(classify_idx, UINT_MAX);

    PlanProposal p;
    p.steps = {{classify_idx}};
    p.goal_fluent_keys = {}; // just check precondition satisfaction

    // All: accepts (trusts BELIEVED)
    {
        ForwardSimVerifier v(AuthorityView::All);
        auto verdict = v.verify(p, wm_);
        EXPECT_TRUE(verdict.accepted) << "All should accept: " << verdict.reason;
    }

    // ConfirmedThenBelieved: accepts but annotates evidence
    {
        ForwardSimVerifier v(AuthorityView::ConfirmedThenBelieved);
        auto verdict = v.verify(p, wm_);
        EXPECT_TRUE(verdict.accepted) << "ConfirmedThenBelieved should accept";
        bool has_believed_note = false;
        for (const auto& e : verdict.evidence)
            if (e.find("believed:") != std::string::npos) has_believed_note = true;
        EXPECT_TRUE(has_believed_note) << "should annotate BELIEVED reliance";
    }

    // ConfirmedOnly: rejects because (searched s1) is BELIEVED
    {
        ForwardSimVerifier v(AuthorityView::ConfirmedOnly);
        auto verdict = v.verify(p, wm_);
        EXPECT_FALSE(verdict.accepted) << "ConfirmedOnly should reject BELIEVED facts";
    }
}
