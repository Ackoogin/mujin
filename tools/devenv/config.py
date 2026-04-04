"""Connection settings and application defaults."""

from dataclasses import dataclass, field


@dataclass
class ConnectionConfig:
    """Network connection settings."""
    foxglove_url: str = "ws://localhost:8765"
    ros2_node_name: str = "ame_devenv"
    wm_node_prefix: str = "/world_model_node"
    plan_action_name: str = "/planner_node/plan"
    world_state_topic: str = "/world_state"
    bt_events_topic: str = "/executor/bt_events"
    # Backend selection: "ros2", "pcl", or "none"
    backend: str = "ros2"
    # PDDL paths for PCL backend
    domain_path: str = ""
    problem_path: str = ""


@dataclass
class UIConfig:
    """UI layout and behaviour settings."""
    window_title: str = "AME Dev Environment"
    window_width: int = 1600
    window_height: int = 1000
    event_buffer_size: int = 10_000
    stream_auto_scroll: bool = True
    bt_tick_hz: int = 60


@dataclass
class PathConfig:
    """Default file paths."""
    domains_dir: str = "domains"
    bt_events_jsonl: str = "ame_bt_events.jsonl"
    wm_audit_jsonl: str = "ame_wm_audit.jsonl"
    plan_audit_jsonl: str = "ame_plan_audit.jsonl"


@dataclass
class AppConfig:
    """Top-level application configuration."""
    connection: ConnectionConfig = field(default_factory=ConnectionConfig)
    ui: UIConfig = field(default_factory=UIConfig)
    paths: PathConfig = field(default_factory=PathConfig)


# BT node status colours (R, G, B, A)
STATUS_COLOURS = {
    "SUCCESS":  (52, 199, 105, 255),
    "RUNNING":  (210, 170,  30, 255),
    "FAILURE":  (210,  55,  55, 255),
    "IDLE":     (90,  110,  95, 255),
    "SKIPPED":  (90,  140, 180, 255),
}

# Source tag colours for WM audit
SOURCE_COLOURS = {
    "perception":        (90,  180, 210, 255),
    "SetWorldPredicate": (52,  199, 105, 255),
    "planner_init":      (160, 130, 220, 255),
    "devenv":            (210, 160,  40, 255),
}

# PDDL syntax keywords for editor highlighting
PDDL_KEYWORDS = [
    "define", "domain", "problem",
    ":requirements", ":typing", ":strips", ":equality",
    ":fluents", ":durative-actions", ":duration-inequalities",
    ":types", ":constants", ":predicates", ":functions",
    ":action", ":parameters", ":precondition", ":effect",
    ":objects", ":init", ":goal",
    "and", "or", "not", "when", "forall", "exists", "imply",
    "increase", "decrease", "assign",
]
