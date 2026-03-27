# A perception detection that maps to one or more world model facts.
# Published on /detections by perception nodes.
std_msgs/Header header

# Detected entity identifier (e.g., object ID or entity name)
string entity_id

# Entity type for type-checking against PDDL domain (e.g., "robot", "location")
string entity_type

# Detected properties as key-value pairs mapped to PDDL predicates
# Each property generates a setFact call with CONFIRMED authority.
# Format: key = predicate template (e.g., "at"), value = argument(s)
# Example: key="at", value="base" → generates "(at entity_id base)"
string[] property_keys
string[] property_values

# Confidence score [0.0, 1.0] for filtering low-confidence detections
float32 confidence

# Sensor source identifier for audit trail
string sensor_source
