#pragma once

#include <string>
#include <unordered_map>

namespace ame {

// ServiceMessage: generic key-value payload for PYRAMID service requests and
// responses.  Concrete PYRAMID SDK types map to/from this bag at the adapter
// layer so that ame_core stays SDK-agnostic.
struct ServiceMessage {
    std::unordered_map<std::string, std::string> fields;

    void set(const std::string& key, const std::string& value) { fields[key] = value; }
    std::string get(const std::string& key, const std::string& default_val = "") const {
        auto it = fields.find(key);
        return (it != fields.end()) ? it->second : default_val;
    }
    bool has(const std::string& key) const { return fields.count(key) > 0; }
};

// Abstract PYRAMID service interface.
//
// Implement this to connect to a real PYRAMID SDK backend.  A mock
// implementation is provided for testing (MockPyramidService).
//
// call() should block until the service responds or times out.
// Returns true on success, false on failure/timeout.
class IPyramidService {
public:
    virtual ~IPyramidService() = default;

    virtual bool call(const std::string& service_name,
                      const std::string& operation,
                      const ServiceMessage& request,
                      ServiceMessage& response) = 0;
};

// No-op mock: always succeeds with an empty response.
// Useful for unit tests and simulation runs without PYRAMID hardware.
class MockPyramidService : public IPyramidService {
public:
    bool call(const std::string& /*service_name*/,
              const std::string& /*operation*/,
              const ServiceMessage& /*request*/,
              ServiceMessage& /*response*/) override {
        return true;
    }
};

} // namespace ame
