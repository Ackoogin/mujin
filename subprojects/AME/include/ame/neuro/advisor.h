#pragma once

#include "ame/neuro/backend_registry.h"
#include "ame/neuro/codec.h"
#include "ame/neuro/fallback_policy.h"
#include "ame/neuro/neuro_audit_log.h"
#include "ame/neuro/verifier.h"
#include "ame/world_model.h"

#include <chrono>
#include <future>
#include <optional>
#include <string>
#include <thread>

namespace ame::neuro {

// Propose-verify-fallback orchestrator.
//
// Given a Request, the Advisor:
//  1. Encodes it via RequestCodec<Request>::encode()
//  2. Submits to a BackendExecutor, waiting at most (remaining budget)
//  3. Decodes the response via ProposalCodec<Proposal>::decode()
//  4. Runs IVerifier<Proposal>::verify() against the WorldModel snapshot
//  5. Returns Accepted or a fell-back outcome
//  6. Emits exactly one NeuroAuditRecord per call
//
// Options A-G each supply a Request type, Proposal type, verifier, and codecs.
// They inherit all timing, retry, fallback, and audit logic from this template.
template <class Request, class Proposal>
class Advisor {
public:
    enum class Outcome {
        Accepted,
        RejectedFellBack,
        TimedOutFellBack,
        UnavailableFellBack,
        ErroredFellBack,
        Disabled
    };

    struct Result {
        Outcome outcome = Outcome::Disabled;
        std::optional<Proposal> proposal;  // present iff Accepted
        typename IVerifier<Proposal>::Verdict verdict;
        double total_latency_ms = 0.0;
        unsigned retries = 0;
    };

    Advisor(std::string kind,
            BackendRegistry& registry,
            IVerifier<Proposal>& verifier,
            FallbackPolicy policy,
            NeuroAuditLog* audit = nullptr,
            ClockFn clock = &default_clock_ms)
        : kind_(std::move(kind))
        , registry_(registry)
        , verifier_(verifier)
        , policy_(std::move(policy))
        , audit_(audit)
        , clock_(clock)
    {}

    Result advise(const Request& req, const ame::WorldModel& wm) {
        const double start_ms = clock_();
        Result result;

        if (!policy_.enabled) {
            emit_audit(req, {}, result, 0.0, 0, start_ms);
            return result;
        }

        BackendExecutor* exec = registry_.find(policy_.backend_id);
        if (!exec || !exec->available()) {
            result.outcome = Outcome::UnavailableFellBack;
            emit_audit(req, {}, result, clock_() - start_ms, 0, start_ms);
            return result;
        }

        double remaining_ms = policy_.latency_budget_ms;
        unsigned attempt = 0;
        const unsigned max_attempts = 1 + policy_.max_retries;

        while (attempt < max_attempts && remaining_ms > 0.0) {
            const double attempt_start = clock_();

            // Per-attempt abandoned flag: set to true BEFORE on_abandoned() so
            // the worker thread can detect that its late ok=true result must not
            // reset consecutive_failures (on_abandoned() already counted the timeout).
            auto abandoned_flag = std::make_shared<std::atomic<bool>>(false);

            CancelSource cs;
            NeuralRequest neural_req;
            try {
                neural_req = RequestCodec<Request>::encode(req, kind_);
            } catch (...) {
                result.outcome = Outcome::ErroredFellBack;
                emit_audit(req, {}, result, clock_() - start_ms, attempt, start_ms);
                return result;
            }
            std::future<NeuralResponse> fut = exec->submit(neural_req, cs.token(), abandoned_flag);

            // Each attempt waits up to the full remaining budget so an in-budget
            // first response is never cancelled early by a pre-allocated retry share.
            const auto budget_dur = std::chrono::microseconds(
                static_cast<long long>(remaining_ms * 1000.0));
            const auto wait_status = fut.wait_for(budget_dur);

            cs.request_cancel(); // cooperative signal; backend may ignore it

            double elapsed = clock_() - attempt_start;
            remaining_ms -= elapsed;

            if (wait_status != std::future_status::ready) {
                // Budget elapsed: signal abandonment then count the failure.
                // Order matters: set the flag before on_abandoned() so the worker
                // sees is_abandoned=true if it completes during/after on_abandoned().
                abandoned_flag->store(true);
                exec->on_abandoned();
                ++attempt;
                // If retries remain and budget remains, retry the request.
                if (attempt < max_attempts && remaining_ms > 0.0) {
                    if (policy_.retry_backoff_ms > 0.0) {
                        double backoff = std::min(policy_.retry_backoff_ms, remaining_ms);
                        std::this_thread::sleep_for(std::chrono::microseconds(
                            static_cast<long long>(backoff * 1000.0)));
                        remaining_ms -= backoff;
                    }
                    continue;
                }
                result.outcome = Outcome::TimedOutFellBack;
                emit_audit(req, {}, result, clock_() - start_ms, attempt - 1, start_ms);
                return result;
            }

            NeuralResponse resp;
            try {
                resp = fut.get();
            } catch (...) {
                resp.ok = false;
                resp.error = "exception_on_get";
            }

            if (!resp.ok) {
                // Pool-full/circuit-open responses from BackendExecutor are
                // availability issues — report UnavailableFellBack immediately
                // rather than retrying or misreporting as ErroredFellBack.
                if (resp.error == "pool_saturated" || resp.error == "circuit_open") {
                    result.outcome = Outcome::UnavailableFellBack;
                    emit_audit(req, {}, result, clock_() - start_ms, attempt, start_ms);
                    return result;
                }
                ++attempt;
                if (attempt >= max_attempts) {
                    result.outcome = Outcome::ErroredFellBack;
                    emit_audit(req, {}, result, clock_() - start_ms, attempt - 1, start_ms);
                    return result;
                }
                if (policy_.retry_backoff_ms > 0.0 && remaining_ms > 0.0) {
                    double backoff = std::min(policy_.retry_backoff_ms, remaining_ms);
                    std::this_thread::sleep_for(std::chrono::microseconds(
                        static_cast<long long>(backoff * 1000.0)));
                    remaining_ms -= backoff;
                }
                continue;
            }

            std::string decode_err;
            std::optional<Proposal> proposal_opt;
            try {
                proposal_opt = ProposalCodec<Proposal>::decode(resp.payload, decode_err);
            } catch (...) {
                result.outcome = Outcome::ErroredFellBack;
                emit_audit(req, {}, result, clock_() - start_ms, attempt, start_ms);
                return result;
            }

            if (!proposal_opt) {
                ++attempt;
                result.outcome = Outcome::ErroredFellBack;
                emit_audit(req, {}, result, clock_() - start_ms, attempt - 1, start_ms);
                return result;
            }

            typename IVerifier<Proposal>::Verdict verdict;
            try {
                verdict = verifier_.verify(*proposal_opt, wm);
            } catch (...) {
                result.outcome = Outcome::ErroredFellBack;
                emit_audit(req, {}, result, clock_() - start_ms, attempt, start_ms);
                return result;
            }

            if (!verdict.accepted) {
                if (policy_.on_reject == OnRejectAction::FallBack ||
                    attempt + 1 >= max_attempts) {
                    result.outcome = Outcome::RejectedFellBack;
                    result.verdict = std::move(verdict);
                    emit_audit(req, proposal_opt, result, clock_() - start_ms, attempt, start_ms, resp.payload);
                    return result;
                }
                ++attempt;
                if (policy_.retry_backoff_ms > 0.0 && remaining_ms > 0.0) {
                    double backoff = std::min(policy_.retry_backoff_ms, remaining_ms);
                    std::this_thread::sleep_for(std::chrono::microseconds(
                        static_cast<long long>(backoff * 1000.0)));
                    remaining_ms -= backoff;
                }
                continue;
            }

            result.outcome = Outcome::Accepted;
            result.proposal = std::move(proposal_opt);
            result.verdict = std::move(verdict);
            result.total_latency_ms = clock_() - start_ms;
            result.retries = attempt;
            emit_audit(req, result.proposal, result, result.total_latency_ms, attempt, start_ms, resp.payload);
            return result;
        }

        result.outcome = Outcome::ErroredFellBack;
        emit_audit(req, {}, result, clock_() - start_ms, attempt, start_ms);
        return result;
    }

private:
    std::string kind_;
    BackendRegistry& registry_;
    IVerifier<Proposal>& verifier_;
    FallbackPolicy policy_;
    NeuroAuditLog* audit_;
    ClockFn clock_;

    void emit_audit(const Request& req,
                    const std::optional<Proposal>& proposal,
                    const Result& result,
                    double latency_ms,
                    unsigned retries,
                    double start_ms,
                    const std::string& raw_payload = {}) {
        if (!audit_) return;

        NeuroAuditRecord rec;
        // Use the request's start time, not the emission time, so the record
        // lines up with the BT/plan event that triggered it in AuditIndex windows.
        rec.ts_us = static_cast<uint64_t>(start_ms * 1000.0);
        rec.integration_kind = kind_;
        rec.backend_id = policy_.backend_id;
        rec.model_id = policy_.model_id;
        // When backend_id is unpinned (empty), BackendRegistry::find("") routes to
        // the first registered executor.  Resolve its actual ID so audit records
        // carry useful provenance for "use default backend" policies.
        if (rec.backend_id.empty()) {
            const BackendExecutor* exec = registry_.find({});
            if (exec) rec.backend_id = exec->info().id;
        }
        try {
            rec.request_digest = RequestCodec<Request>::digest(req);
        } catch (...) {
            // Digest threw; leave empty rather than propagating.
        }
        try {
            rec.proposal_digest = proposal
                ? ProposalCodec<Proposal>::digest(*proposal) : "";
        } catch (...) {
            // Digest threw; leave empty rather than propagating.
        }
        rec.outcome = outcome_str(result.outcome);
        rec.verdict_reason = result.verdict.reason;
        rec.evidence = result.verdict.evidence;
        rec.latency_ms = latency_ms;
        rec.retries = retries;
        rec.affected_behaviour = (result.outcome == Outcome::Accepted);
        if (policy_.verbosity >= 1) {
            try {
                rec.raw_request = RequestCodec<Request>::encode(req, kind_).payload;
            } catch (...) {
                // Encoder threw; leave raw_request empty rather than propagating.
            }
            if (proposal) rec.raw_proposal = raw_payload;
        }
        audit_->append(std::move(rec));
    }

    static std::string outcome_str(Outcome o) {
        switch (o) {
            case Outcome::Accepted:            return "Accepted";
            case Outcome::RejectedFellBack:    return "RejectedFellBack";
            case Outcome::TimedOutFellBack:    return "TimedOutFellBack";
            case Outcome::UnavailableFellBack: return "UnavailableFellBack";
            case Outcome::ErroredFellBack:     return "ErroredFellBack";
            case Outcome::Disabled:            return "Disabled";
        }
        return "Unknown";
    }
};

} // namespace ame::neuro
