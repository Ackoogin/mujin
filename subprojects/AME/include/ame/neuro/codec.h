#pragma once

#include "ame/neuro/neural_backend.h"

#include <optional>
#include <string>

namespace ame::neuro {

// Traits seam: each integration specialises these two templates for its own
// Request/Proposal types. The only obligation besides a verifier is to supply
// encode/decode/digest — no orchestration code.

template <class Request>
struct RequestCodec {
    // Encode a Request to a NeuralRequest.
    static NeuralRequest encode(const Request& req, const std::string& kind);
    // Short digest for audit (not required to be cryptographic).
    static std::string digest(const Request& req);
};

template <class Proposal>
struct ProposalCodec {
    // Decode a NeuralResponse payload to a Proposal. Returns nullopt on failure.
    static std::optional<Proposal> decode(const std::string& payload, std::string& error);
    // Short digest for audit.
    static std::string digest(const Proposal& p);
};

// ---------------------------------------------------------------------------
// Default specialisations for std::string (used by infrastructure tests)
// ---------------------------------------------------------------------------

template <>
struct RequestCodec<std::string> {
    static NeuralRequest encode(const std::string& req, const std::string& kind) {
        return {kind, req};
    }
    static std::string digest(const std::string& req) {
        // Simple length+prefix digest for tests
        return "str:" + std::to_string(req.size()) + ":" +
               req.substr(0, std::min<size_t>(req.size(), 8));
    }
};

template <>
struct ProposalCodec<std::string> {
    static std::optional<std::string> decode(const std::string& payload, std::string& /*error*/) {
        return payload;  // passthrough
    }
    static std::string digest(const std::string& p) {
        return "str:" + std::to_string(p.size()) + ":" +
               p.substr(0, std::min<size_t>(p.size(), 8));
    }
};

} // namespace ame::neuro
