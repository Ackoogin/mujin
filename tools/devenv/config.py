"""Connection settings and application defaults."""

from dataclasses import dataclass, field


@dataclass
class ConnectionConfig:
    """Network connection settings."""
    foxglove_url: str = "ws://localhost:8765"
    ros2_node_name: str = "ame_devenv"
    wm_node_prefix: str = "/world_model_node"
    plan_action_name: str = "/ame/plan"
    world_state_topic: str = "/world_state"
    bt_events_topic: str = "/executor/bt_events"


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
    "SUCCESS":  (80, 200, 80, 255),
    "RUNNING":  (220, 180, 40, 255),
    "FAILURE":  (220, 60, 60, 255),
    "IDLE":     (120, 120, 130, 255),
    "SKIPPED":  (100, 140, 200, 255),
}

# Source tag colours for WM audit
SOURCE_COLOURS = {
    "perception":        (100, 180, 255, 255),
    "SetWorldPredicate": (80, 200, 80, 255),
    "planner_init":      (180, 140, 255, 255),
    "devenv":            (255, 180, 60, 255),
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
