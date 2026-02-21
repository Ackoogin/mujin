# Full snapshot of the WorldModel boolean state.
# Published on /world_state on every fact change (debounced).
std_msgs/Header  header
uint64           wm_version
WorldFact[]      facts           # all fluents that are currently TRUE
string[]         goal_fluents    # fluent keys in the current goal
