#pragma once

#include "ame/neuro/backend_executor.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame::neuro {

// Deployment tier: hot=local ONNX/fast model, warm=local LLM/fast cloud, cold=full cloud API
enum class BackendTier { Hot, Warm, Cold };

struct BackendEntry {
    std::shared_ptr<BackendExecutor> executor;
    BackendTier tier = BackendTier::Warm;
};

// Named lookup + ownership store for backend executors.
class BackendRegistry {
public:
    // Register a backend (registry takes ownership to keep it alive).
    void add(std::shared_ptr<INeuralBackend> backend,
             BackendTier tier = BackendTier::Warm,
             BackendExecutorConfig cfg = {});

    // Lookup by id. If id is empty, returns first registered executor.
    BackendExecutor* find(const std::string& id);
    const BackendExecutor* find(const std::string& id) const;

    const std::vector<BackendEntry>& all() const { return entries_; }

private:
    std::vector<BackendEntry> entries_;
    std::unordered_map<std::string, size_t> index_;
    std::vector<std::shared_ptr<INeuralBackend>> owned_; // keep backends alive
};

} // namespace ame::neuro
