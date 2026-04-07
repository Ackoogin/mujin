"""Data classes for BT events, WM audit entries, and plan episodes."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class BTEvent:
    """A single BT node state transition (Layer 2)."""
    ts_us: int = 0
    node: str = ""
    node_type: str = ""
    prev_status: str = ""
    status: str = ""
    tree_id: str = ""
    wm_version: int = 0

    @classmethod
    def from_json(cls, raw: str | dict) -> BTEvent:
        d = json.loads(raw) if isinstance(raw, str) else raw
        return cls(
            ts_us=d.get("ts_us", 0),
            node=d.get("node", ""),
            node_type=d.get("type", ""),
            prev_status=d.get("prev", ""),
            status=d.get("status", ""),
            tree_id=d.get("tree_id", ""),
            wm_version=d.get("wm_version", 0),
        )

    @property
    def ts_sec(self) -> float:
        return self.ts_us / 1_000_000.0


@dataclass
class WMAuditEntry:
    """A single world model fact change (Layer 3)."""
    wm_version: int = 0
    ts_us: int = 0
    fact: str = ""
    value: bool = False
    source: str = ""

    @classmethod
    def from_json(cls, raw: str | dict) -> WMAuditEntry:
        d = json.loads(raw) if isinstance(raw, str) else raw
        return cls(
            wm_version=d.get("wm_version", 0),
            ts_us=d.get("ts_us", 0),
            fact=d.get("fact", ""),
            value=d.get("value", False),
            source=d.get("source", ""),
        )

    @property
    def ts_sec(self) -> float:
        return self.ts_us / 1_000_000.0


@dataclass
class PlanEpisode:
    """A planning episode record (Layer 5)."""
    ts_us: int = 0
    init_facts: list[str] = field(default_factory=list)
    goal_fluents: list[str] = field(default_factory=list)
    solver: str = ""
    solve_time_ms: float = 0.0
    success: bool = False
    expanded: int = 0
    generated: int = 0
    cost: float = 0.0
    plan_actions: list[str] = field(default_factory=list)
    bt_xml: str = ""

    @classmethod
    def from_json(cls, raw: str | dict) -> PlanEpisode:
        d = json.loads(raw) if isinstance(raw, str) else raw
        return cls(
            ts_us=d.get("ts_us", 0),
            init_facts=d.get("init_facts", []),
            goal_fluents=d.get("goal_fluents", []),
            solver=d.get("solver", ""),
            solve_time_ms=d.get("solve_time_ms", 0.0),
            success=d.get("success", False),
            expanded=d.get("expanded", 0),
            generated=d.get("generated", 0),
            cost=d.get("cost", 0.0),
            plan_actions=d.get("plan_actions", []),
            bt_xml=d.get("bt_xml", ""),
        )


@dataclass
class WorldFact:
    """A single fact in the world model."""
    key: str = ""
    value: bool = False
    authority: str = "BELIEVED"
    source: str = ""
    timestamp_us: int = 0
    wm_version: int = 0


@dataclass
class WorldSnapshot:
    """A snapshot of the full world model state."""
    wm_version: int = 0
    facts: list[WorldFact] = field(default_factory=list)
    goal_fluents: list[str] = field(default_factory=list)
