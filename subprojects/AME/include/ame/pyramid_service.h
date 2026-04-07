#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace ame {

/// \brief Generic key-value payload for PYRAMID service requests/responses.
/// Concrete PYRAMID SDK types map to/from this bag at the adapter layer so
/// `ame` remains SDK-agnostic.
struct ServiceMessage {
    std::unordered_map<std::string, std::string> fields;

    void set(const std::string& key, const std::string& value) { fields[key] = value; }
    std::string get(const std::string& key, const std::string& default_val = "") const {
        auto it = fields.find(key);
        return (it != fields.end()) ? it->second : default_val;
    }
    bool has(const std::string& key) const { return fields.count(key) > 0; }
};

/// \brief Result status for an async service call.
enum class AsyncCallStatus {
    PENDING,    // Call is still in progress
    SUCCESS,    // Call completed successfully
    FAILURE,    // Call completed with failure
    CANCELLED   // Call was cancelled
};

/// \brief Abstract PYRAMID service interface.
/// Supports both synchronous and asynchronous service invocation.
/// Implement this to connect to a real PYRAMID SDK backend.
class IPyramidService {
public:
    virtual ~IPyramidService() = default;

    /// \brief Executes a PYRAMID service operation synchronously.
    /// Blocks until response or timeout; returns true on success.
    virtual bool call(const std::string& service_name,
                      const std::string& operation,
                      const ServiceMessage& request,
                      ServiceMessage& response) = 0;

    /// \brief Initiates an asynchronous service call.
    /// Returns a request ID that can be used to poll or cancel.
    virtual uint64_t callAsync(const std::string& service_name,
                               const std::string& operation,
                               const ServiceMessage& request) = 0;

    /// \brief Polls the status of an async call.
    /// If the call is complete (SUCCESS or FAILURE), populates response.
    virtual AsyncCallStatus pollResult(uint64_t request_id,
                                       ServiceMessage& response) = 0;

    /// \brief Cancels a pending async call. No-op if already complete.
    virtual void cancelCall(uint64_t request_id) = 0;
};

/// \brief No-op mock service for tests and simulation.
/// Async calls complete immediately on the next poll.
class MockPyramidService : public IPyramidService {
public:
    bool call(const std::string&, const std::string&,
              const ServiceMessage&, ServiceMessage&) override {
        return true;
    }

    uint64_t callAsync(const std::string&, const std::string&,
                       const ServiceMessage&) override {
        return ++next_id_;
    }

    AsyncCallStatus pollResult(uint64_t, ServiceMessage&) override {
        return AsyncCallStatus::SUCCESS;
    }

    void cancelCall(uint64_t) override {}

private:
    uint64_t next_id_ = 0;
};

} // namespace ame
