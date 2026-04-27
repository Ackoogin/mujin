"""Observability tab -- live event stream, WM plots, fact history, JSONL replay."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from ..config import STATUS_COLOURS, SOURCE_COLOURS
from ._plot_utils import set_integer_y_axis as _set_integer_y_axis
from ..models.events import BTEvent, WMAuditEntry, PlanEpisode
from ..models.replay import ReplaySession

if TYPE_CHECKING:
    from .app import App


class ObservabilityTab:
    """Builds and manages the Observability tab."""

    def __init__(self, app: App):
        self._app = app

        # Event buffers (live + replay)
        self._bt_events: list[BTEvent] = []
        self._wm_events: list[WMAuditEntry] = []
        self._combined_log: list[tuple[float, str, str, tuple]] = []
        self._max_log_lines = 2000
        self._auto_scroll = True
        self._filter_text: str = ""

        # Track event counts at last plot refresh -- only refit when new data arrived
        self._wm_count_at_refresh: int = 0

        # Replay
        self._replay = ReplaySession()
        self._replay_time: float = 0.0

        # Widget IDs
        self._log_panel: int = 0
        self._wm_plot: int = 0
        self._wm_plot_x: int = 0
        self._wm_plot_y: int = 0
        self._fact_combo: int = 0
        self._fact_plot: int = 0
        self._fact_plot_x: int = 0
        self._fact_plot_y: int = 0
        self._replay_slider: int = 0
        self._replay_info: int = 0
        self._auto_scroll_check: int = 0
        self._filter_input: int = 0
        self._event_count_text: int = 0

    def build(self, parent: int) -> None:
        with dpg.group(parent=parent):
            with dpg.tab_bar():
                # --- Live Stream sub-tab ---
                with dpg.tab(label="Live Stream"):
                    self._build_live_stream_tab()

                # --- WM Version Plot sub-tab ---
                with dpg.tab(label="WM Version"):
                    self._build_wm_plot_tab()

                # --- Fact History sub-tab ---
                with dpg.tab(label="Fact History"):
                    self._build_fact_history_tab()

                # --- JSONL Replay sub-tab ---
                with dpg.tab(label="JSONL Replay"):
                    self._build_replay_tab()

    def _build_live_stream_tab(self) -> None:
        with dpg.group(horizontal=True):
            dpg.add_text("Filter:")
            self._filter_input = dpg.add_input_text(
                width=250, callback=self._on_filter_changed,
                hint="node name, fact key, or source..."
            )
            dpg.add_spacer(width=10)
            self._auto_scroll_check = dpg.add_checkbox(
                label="Auto-scroll", default_value=True,
                callback=lambda s, v, u: setattr(self, '_auto_scroll', v),
            )
            dpg.add_spacer(width=10)
            self._event_count_text = dpg.add_text(
                "0 events", color=(120, 120, 130)
            )
            dpg.add_spacer(width=10)
            dpg.add_button(
                label="Clear", callback=self._on_clear_log, width=60
            )

        dpg.add_spacer(height=4)
        self._log_panel = dpg.add_child_window(height=-5, border=True)

    def _build_wm_plot_tab(self) -> None:
        dpg.add_text(
            "World model version over time. Shows rate of state change.",
            color=(160, 160, 170),
        )
        dpg.add_spacer(height=6)
        self._wm_plot = dpg.add_plot(
            label="", height=-5, width=-1, anti_aliased=True,
        )
        self._wm_plot_x = dpg.add_plot_axis(
            dpg.mvXAxis, label="Time (s)", parent=self._wm_plot,
        )
        self._wm_plot_y = dpg.add_plot_axis(
            dpg.mvYAxis, label="WM Version", parent=self._wm_plot,
        )

    def _build_fact_history_tab(self) -> None:
        with dpg.group(horizontal=True):
            dpg.add_text("Fact:")
            self._fact_combo = dpg.add_combo(
                items=[], width=400,
                callback=self._on_fact_selected,
            )
            dpg.add_spacer(width=10)
            dpg.add_button(
                label="Refresh Facts List",
                callback=self._on_refresh_fact_list, width=130,
            )

        dpg.add_spacer(height=6)
        self._fact_plot = dpg.add_plot(
            label="", height=250, width=-1, anti_aliased=True,
        )
        self._fact_plot_x = dpg.add_plot_axis(
            dpg.mvXAxis, label="Time (s)", parent=self._fact_plot,
        )
        self._fact_plot_y = dpg.add_plot_axis(
            dpg.mvYAxis, label="Value", parent=self._fact_plot,
        )

        dpg.add_spacer(height=6)
        dpg.add_text("Change Log:", color=(180, 180, 200))
        self._fact_history_panel = dpg.add_child_window(
            height=-5, border=True
        )

    def _build_replay_tab(self) -> None:
        dpg.add_text(
            "Load JSONL audit files for offline analysis.",
            color=(160, 160, 170),
        )
        dpg.add_spacer(height=6)

        with dpg.group(horizontal=True):
            dpg.add_button(
                label="Load Directory...",
                callback=self._on_load_directory, width=140,
            )
            dpg.add_spacer(width=10)
            self._replay_info = dpg.add_text("No files loaded.", color=(120, 120, 130))

        dpg.add_spacer(height=8)

        dpg.add_text("Time Scrubber:")
        self._replay_slider = dpg.add_slider_float(
            min_value=0.0, max_value=1.0,
            default_value=0.0, width=-1,
            callback=self._on_replay_scrub,
            format="%.2f s",
        )

        dpg.add_spacer(height=8)
        self._replay_log_panel = dpg.add_child_window(height=-5, border=True)

    # -- Public update methods -----------------------------------------------

    def add_bt_event(self, event: BTEvent) -> None:
        self._bt_events.append(event)
        ts = event.ts_sec
        color = STATUS_COLOURS.get(event.status, (180, 180, 190, 255))
        text = (
            f"[BT] {event.node}: {event.prev_status} -> {event.status}"
            f"  ({event.node_type}, v{event.wm_version})"
        )
        self._add_log_entry(ts, "BT", text, color)

    def add_wm_event(self, event: WMAuditEntry) -> None:
        self._wm_events.append(event)
        ts = event.ts_sec
        val_str = "TRUE" if event.value else "FALSE"
        color = SOURCE_COLOURS.get("perception", (180, 180, 190, 255))
        for prefix, c in SOURCE_COLOURS.items():
            if event.source.startswith(prefix):
                color = c
                break
        text = (
            f"[WM] {event.fact} = {val_str}"
            f"  (src: {event.source}, v{event.wm_version})"
        )
        self._add_log_entry(ts, "WM", text, color)

    def refresh_plots(self) -> None:
        """Update WM version plot. Only refits axes when new data has arrived."""
        n = len(self._wm_events)
        if not n:
            return
        new_data = n > self._wm_count_at_refresh
        self._wm_count_at_refresh = n

        t0 = self._wm_events[0].ts_sec
        xs = [e.ts_sec - t0 for e in self._wm_events]
        ys = [float(e.wm_version) for e in self._wm_events]

        children = dpg.get_item_children(self._wm_plot_y, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        dpg.add_stair_series(xs, ys, label="WM Version", parent=self._wm_plot_y)

        if new_data:
            dpg.fit_axis_data(self._wm_plot_x)
            _set_integer_y_axis(self._wm_plot_y, ys)

    # -- Internal ------------------------------------------------------------

    def _add_log_entry(self, ts: float, tag: str, text: str,
                        color: tuple) -> None:
        self._combined_log.append((ts, tag, text, color))
        if len(self._combined_log) > self._max_log_lines:
            self._combined_log = self._combined_log[-self._max_log_lines:]

        # Apply filter
        if self._filter_text and self._filter_text.lower() not in text.lower():
            return

        dpg.add_text(text, parent=self._log_panel, color=color)

        dpg.set_value(
            self._event_count_text,
            f"{len(self._combined_log)} events"
        )

        if self._auto_scroll:
            dpg.set_y_scroll(self._log_panel, dpg.get_y_scroll_max(self._log_panel))

    def _on_filter_changed(self, sender, value, user_data) -> None:
        self._filter_text = value
        self._rebuild_log()

    def _on_clear_log(self, sender=None, value=None, user_data=None) -> None:
        self._combined_log.clear()
        self._bt_events.clear()
        self._wm_events.clear()
        children = dpg.get_item_children(self._log_panel, 1)
        if children:
            for child in children:
                dpg.delete_item(child)
        dpg.set_value(self._event_count_text, "0 events")

    def _rebuild_log(self) -> None:
        children = dpg.get_item_children(self._log_panel, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        ft = self._filter_text.lower()
        for ts, tag, text, color in self._combined_log:
            if ft and ft not in text.lower():
                continue
            dpg.add_text(text, parent=self._log_panel, color=color)

    def _on_refresh_fact_list(self, sender=None, value=None,
                               user_data=None) -> None:
        keys = sorted({e.fact for e in self._wm_events})
        dpg.configure_item(self._fact_combo, items=keys)

    def _on_fact_selected(self, sender, value, user_data) -> None:
        fact_key = value
        history = [e for e in self._wm_events if e.fact == fact_key]

        # Update fact plot
        children = dpg.get_item_children(self._fact_plot_y, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        if history:
            t0 = history[0].ts_sec
            xs = [e.ts_sec - t0 for e in history]
            ys = [1.0 if e.value else 0.0 for e in history]
            dpg.add_stair_series(
                xs, ys, label=fact_key, parent=self._fact_plot_y,
            )
            dpg.fit_axis_data(self._fact_plot_x)
            dpg.set_axis_limits(self._fact_plot_y, -0.2, 1.2)
            dpg.set_axis_ticks(
                self._fact_plot_y,
                (("FALSE", 0.0), ("TRUE", 1.0)),
            )

        # Update change log panel
        children = dpg.get_item_children(self._fact_history_panel, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        for e in history:
            val_color = (80, 200, 80, 255) if e.value else (220, 60, 60, 255)
            dpg.add_text(
                f"v{e.wm_version}  {'TRUE' if e.value else 'FALSE':>5s}"
                f"  src: {e.source}",
                parent=self._fact_history_panel,
                color=val_color,
            )

    # -- Replay --------------------------------------------------------------

    def _on_load_directory(self, sender=None, value=None,
                            user_data=None) -> None:
        dpg.add_file_dialog(
            directory_selector=True,
            callback=self._on_directory_selected,
            width=600, height=400,
        )

    def _on_directory_selected(self, sender, app_data, user_data=None) -> None:
        path = app_data.get("file_path_name", "")
        if not path:
            return

        counts = self._replay.load_all(path)
        total = sum(counts.values())
        dpg.set_value(
            self._replay_info,
            f"Loaded {total} entries from {path}"
        )

        t0, t1 = self._replay.time_range_sec
        duration = t1 - t0
        if duration > 0:
            dpg.configure_item(
                self._replay_slider,
                min_value=0.0, max_value=duration,
                default_value=0.0,
                format=f"%.2f / {duration:.2f} s",
            )

        # Load into fact combo
        keys = self._replay.all_fact_keys()
        dpg.configure_item(self._fact_combo, items=keys)

        # Show all replay events in the log
        self._replay_scrub_to(duration)

    def _on_replay_scrub(self, sender, value, user_data=None) -> None:
        self._replay_scrub_to(value)

    def _replay_scrub_to(self, offset_sec: float) -> None:
        t0, t1 = self._replay.time_range_sec
        target = t0 + offset_sec

        children = dpg.get_item_children(self._replay_log_panel, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        bt_evts = self._replay.bt_events_up_to(target)
        wm_evts = self._replay.wm_events_up_to(target)

        # Merge and display chronologically (last 500)
        entries = []
        for e in bt_evts:
            color = STATUS_COLOURS.get(e.status, (180, 180, 190, 255))
            text = f"[BT] {e.node}: {e.prev_status} -> {e.status}"
            entries.append((e.ts_sec, text, color))
        for e in wm_evts:
            val = "TRUE" if e.value else "FALSE"
            color = (80, 200, 80, 255) if e.value else (220, 60, 60, 255)
            text = f"[WM] {e.fact} = {val} (src: {e.source})"
            entries.append((e.ts_sec, text, color))

        entries.sort(key=lambda x: x[0])
        for _, text, color in entries[-500:]:
            dpg.add_text(
                text, parent=self._replay_log_panel, color=color
            )
