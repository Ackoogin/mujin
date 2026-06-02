#include "ame/neuro/backend_registry.h"

namespace ame::neuro {

void BackendRegistry::add(std::shared_ptr<INeuralBackend> backend,
                          BackendTier tier,
                          BackendExecutorConfig cfg) {
    const std::string id = backend->info().id;
    // Pass the shared_ptr to BackendExecutor so detached threads keep the
    // backend alive independently of the registry's lifetime.
    auto executor = std::make_shared<BackendExecutor>(backend, cfg);
    index_[id] = entries_.size();
    entries_.push_back(BackendEntry{std::move(executor), tier});
    owned_.push_back(std::move(backend)); // belt-and-suspenders: registry also holds a ref
}

BackendExecutor* BackendRegistry::find(const std::string& id) {
    if (id.empty()) {
        return entries_.empty() ? nullptr : entries_.front().executor.get();
    }
    auto it = index_.find(id);
    return (it == index_.end()) ? nullptr : entries_[it->second].executor.get();
}

const BackendExecutor* BackendRegistry::find(const std::string& id) const {
    if (id.empty()) {
        return entries_.empty() ? nullptr : entries_.front().executor.get();
    }
    auto it = index_.find(id);
    return (it == index_.end()) ? nullptr : entries_[it->second].executor.get();
}

} // namespace ame::neuro
