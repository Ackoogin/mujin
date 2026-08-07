#pragma once

namespace ame {

/// State authority classification for facts.
/// Used to distinguish perception-sourced facts from plan-applied predictions.
///
/// This lives in its own header so that code which only needs to talk about the
/// authority of a fact, such as the world-state access interface used by
/// planned action nodes, does not have to include the whole world model.
enum class FactAuthority {
    BELIEVED,   ///< Fact value derived from plan effects (predicted)
    CONFIRMED   ///< Fact value derived from perception (observed)
};

}  // namespace ame
