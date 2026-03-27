"""JSONL file loader and time-scrubbing for offline replay.

Loads ame_bt_events.jsonl, ame_wm_audit.jsonl, and ame_plan_audit.jsonl
files and provides time-windowed access for the UI time slider.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from .events import BTEvent, WMAuditEntry, PlanEpisode


class ReplaySession:
    """Holds events loaded from JSONL files for offline replay.

    Usage::

        session = ReplaySession()
        session.load_bt_events("ame_bt_events.jsonl")
        session.load_wm_events("ame_wm_audit.jsonl")
        session.load_plan_episodes("ame_plan_audit.jsonl")

        # Get time range
        t0, t1 = session.time_range_sec

        # Scrub to a point in time
        bt_evts = session.bt_events_up_to(t0 + 5.0)
        wm_evts = session.wm_events_up_to(t0 + 5.0)
    """

    def __init__(self):
        self.bt_events: list[BTEvent] = []
        self.wm_events: list[WMAuditEntry] = []
        self.plan_episodes: list[PlanEpisode] = []
        self._min_ts: float = 0.0
        self._max_ts: float = 0.0

    @property
    def time_range_sec(self) -> tuple[float, float]:
        return (self._min_ts, self._max_ts)

    @property
    def duration_sec(self) -> float:
        return self._max_ts - self._min_ts

    @property
    def loaded(self) -> bool:
        return bool(self.bt_events or self.wm_events or self.plan_episodes)

    def load_bt_events(self, path: str | Path) -> int:
        """Load BT events from a JSONL file. Returns count loaded."""
        self.bt_events = _load_jsonl(path, BTEvent.from_json)
        self._update_time_range()
        return len(self.bt_events)

    def load_wm_events(self, path: str | Path) -> int:
        """Load WM audit entries from a JSONL file. Returns count loaded."""
        self.wm_events = _load_jsonl(path, WMAuditEntry.from_json)
        self._update_time_range()
        return len(self.wm_events)

    def load_plan_episodes(self, path: str | Path) -> int:
        """Load plan episodes from a JSONL file. Returns count loaded."""
        self.plan_episodes = _load_jsonl(path, PlanEpisode.from_json)
        return len(self.plan_episodes)

    def load_all(self, directory: str | Path) -> dict[str, int]:
        """Load all three JSONL files from a directory. Returns counts."""
        d = Path(directory)
        counts = {}
        for name, loader in [
            ("ame_bt_events.jsonl", self.load_bt_events),
            ("ame_wm_audit.jsonl", self.load_wm_events),
            ("ame_plan_audit.jsonl", self.load_plan_episodes),
        ]:
            p = d / name
            if p.exists():
                counts[name] = loader(p)
            else:
                counts[name] = 0
        return counts

    def bt_events_up_to(self, t_sec: float) -> list[BTEvent]:
        """Return BT events with timestamp <= t_sec."""
        return [e for e in self.bt_events if e.ts_sec <= t_sec]

    def bt_events_in_range(self, t0: float, t1: float) -> list[BTEvent]:
        """Return BT events within [t0, t1] seconds."""
        return [e for e in self.bt_events if t0 <= e.ts_sec <= t1]

    def wm_events_up_to(self, t_sec: float) -> list[WMAuditEntry]:
        """Return WM events with timestamp <= t_sec."""
        return [e for e in self.wm_events if e.ts_sec <= t_sec]

    def wm_events_in_range(self, t0: float, t1: float) -> list[WMAuditEntry]:
        """Return WM events within [t0, t1] seconds."""
        return [e for e in self.wm_events if t0 <= e.ts_sec <= t1]

    def fact_history(self, fact_key: str) -> list[WMAuditEntry]:
        """Return all WM events for a specific fact key, in time order."""
        return [e for e in self.wm_events if e.fact == fact_key]

    def all_fact_keys(self) -> list[str]:
        """Return sorted list of unique fact keys seen in WM events."""
        return sorted({e.fact for e in self.wm_events})

    def _update_time_range(self) -> None:
        all_ts = []
        if self.bt_events:
            all_ts.extend(e.ts_sec for e in self.bt_events)
        if self.wm_events:
            all_ts.extend(e.ts_sec for e in self.wm_events)
        if all_ts:
            self._min_ts = min(all_ts)
            self._max_ts = max(all_ts)


def _load_jsonl(path: str | Path, parser) -> list:
    """Load a JSONL file, skipping malformed lines."""
    items = []
    p = Path(path)
    if not p.exists():
        return items
    with p.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                items.append(parser(line))
            except (json.JSONDecodeError, KeyError, TypeError):
                continue
    return items
