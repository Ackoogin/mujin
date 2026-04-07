# A single boolean fluent in the WorldModel.
# key is a PDDL-style fact string, e.g. "(at uav1 base)"
string  key
bool    value
string  source        # originator tag, e.g. "perception", "SetWorldPredicate:node_name"
uint64  wm_version    # WorldModel::version() at time of this fact
