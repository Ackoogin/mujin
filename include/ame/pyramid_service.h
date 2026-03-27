#pragma once

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

/// \brief Abstract PYRAMID service interface.
/// Implement this to connect to a real PYRAMID SDK backend.
class IPyramidService {
public:
    virtual ~IPyramidService() = default;

    /// \brief Executes a PYRAMID service operation.
    /// Should block until response or timeout; returns true on success.
    virtual bool call(const std::string& service_name,
                      const std::string& operation,
                      const ServiceMessage& request,
                      ServiceMessage& response) = 0;
};

/// \brief No-op mock service for tests and simulation.
class MockPyramidService : public IPyramidService {
public:
    bool call(const std::string& service_name,
              const std::string& operation,
              const ServiceMessage& request,
              ServiceMessage& response) override {
        (void)service_name;
        (void)operation;
        (void)request;
        (void)response;
        return true;
    }
};

} // namespace ame
