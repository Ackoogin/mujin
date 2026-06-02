#include "ame/neuro/fallback_policy.h"

#include <chrono>

namespace ame::neuro {

double default_clock_ms() noexcept {
    using namespace std::chrono;
    return duration<double, std::milli>(
        system_clock::now().time_since_epoch()).count();
}

} // namespace ame::neuro
