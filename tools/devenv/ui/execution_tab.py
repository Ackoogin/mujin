"""Execution tab — BT tree visualisation, node status, timeline, tick controls."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from ..config import STATUS_COLOURS
from ._plot_utils import set_integer_y_axis as _set_integer_y_axis
from ..models.events import BTEvent

if TYPE_CHECKING:
    from .app import App


class ExecutionTab:
    """Builds and manages the Execution monitoring tab."""

    def __init__(self, app: App):
        self._app = app

        # BT structure (parsed from XML)
        self._bt_xml: str = ""

        # Live node statuses: node_name -> latest status
        self._node_statuses: dict[str, str] = {}
        # Node statistics: node_name -> list of events
        self._node_events: dict[str, list[BTEvent]] = defaultdict(list)
        # All events for timeline
        self._all_events: list[BTEvent] = []
        self._events_at_last_timeline_refresh: int = 0

        # --- Control bar widget IDs ---
        self._status_badge: int = 0
        self._tick_label: int = 0
        self._mode_combo: int = 0
        self._interval_input: int = 0
        self._authority_combo: int = 0
        self._load_start_btn: int = 0
        self._stop_btn: int = 0
        self._tick_once_btn: int = 0

        # --- Main display widget IDs ---
        self._tree_panel: int = 0
        self._inspector_panel: int = 0
        self._timeline_plot: int = 0
        self._timeline_x: int = 0
        self._timeline_y: int = 0
        self._event_count_text: int = 0
        self._selected_node: str = ""
        self._inspector_text: int = 0

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def build(self, parent: int) -> None:
        with dpg.group(parent=parent):

            # ── Control bar ──────────────────────────────────────────
            with dpg.child_window(height=58, border=True):
                with dpg.group(horizontal=True):
                    dpg.add_text("Status:", color=(140, 140, 160))
                    self._status_badge = dpg.add_text(
                        "IDLE", color=STATUS_COLOURS.get("IDLE", (90, 110, 95, 255))
                    )
                    dpg.add_text("  Ticks:", color=(140, 140, 160))
                    self._tick_label = dpg.add_text("0", color=(180, 180, 190))

                    dpg.add_spacer(width=16)

                    dpg.add_text("Mode:", color=(140, 140, 160))
                    self._mode_combo = dpg.add_combo(
                        items=["Auto-tick", "Manual step"],
                        default_value="Auto-tick",
                        width=130,
                        callback=self._on_mode_changed,
                    )

                    dpg.add_text("Interval (s):", color=(140, 140, 160))
                    self._interval_input = dpg.add_input_float(
                        default_value=0.05,
                        min_value=0.001,
                        max_value=5.0,
                        step=0.01,
                        format="%.3f",
                        width=90,
                    )

                    dpg.add_spacer(width=16)

                    dpg.add_text("Preconditions:", color=(140, 140, 160))
                    self._authority_combo = dpg.add_combo(
                        items=["any (planning)", "confirmed (execution)"],
                        default_value="any (planning)",
                        width=175,
                        callback=None,
                    )

                    dpg.add_spacer(width=16)

                    self._load_start_btn = dpg.add_button(
                        label="Load & Start",
                        callback=self._on_load_start,
                        width=110, height=28,
                    )
                    self._stop_btn = dpg.add_button(
                        label="Stop",
                        callback=self._on_stop,
                        width=60, height=28,
                        enabled=False,
                    )
                    self._tick_once_btn = dpg.add_button(
                        label="Tick Once",
                        callback=self._on_tick_once,
                        width=80, height=28,
                        enabled=False,
                    )

                    dpg.add_spacer(width=16)
                    self._event_count_text = dpg.add_text(
                        "Events: 0", color=(100, 100, 115)
                    )
                    dpg.add_spacer(width=8)
                    dpg.add_button(
                        label="Clear", callback=self._on_clear, width=55, height=28
                    )

            dpg.add_spacer(height=4)

            # ── Tree + Inspector ──────────────────────────────────────
            with dpg.group(horizontal=True):
                with dpg.child_window(width=500, height=-250):
                    dpg.add_text("Behaviour Tree", color=(180, 180, 200))
                    dpg.add_separator()
                    self._tree_panel = dpg.add_child_window(
                        height=-5, border=False
                    )

                with dpg.child_window(width=-1, height=-250):
                    dpg.add_text("Node Inspector", color=(180, 180, 200))
                    dpg.add_separator()
                    self._inspector_panel = dpg.add_child_window(
                        height=-5, border=False
                    )
                    self._inspector_text = dpg.add_text(
                        "Select a node from the tree to inspect.",
                        parent=self._inspector_panel,
                        wrap=-1, color=(160, 160, 170),
                    )

            # ── Timeline ─────────────────────────────────────────────
            with dpg.child_window(height=240):
                dpg.add_text("Event Timeline", color=(180, 180, 200))
                dpg.add_separator()
                self._timeline_plot = dpg.add_plot(
                    label="", height=-5, width=-1,
                    anti_aliased=True,
                )
                self._timeline_x = dpg.add_plot_axis(
                    dpg.mvXAxis, label="Time (s)",
                    parent=self._timeline_plot,
                )
                self._timeline_y = dpg.add_plot_axis(
                    dpg.mvYAxis, label="WM Version",
                    parent=self._timeline_plot,
                )

    # ------------------------------------------------------------------
    # Public API called by App frame loop
    # ------------------------------------------------------------------

    def set_bt_xml(self, xml_str: str) -> None:
        self._bt_xml = xml_str
        self._render_tree()

    def add_bt_event(self, event: BTEvent) -> None:
        self._all_events.append(event)
        self._node_statuses[event.node] = event.status
        self._node_events[event.node].append(event)
        dpg.set_value(
            self._event_count_text,
            f"Events: {len(self._all_events)}"
        )

    def refresh_display(self) -> None:
        """Rebuild tree display and update timeline (called ~1 Hz)."""
        self._render_tree()
        self._update_timeline()

        dpg.set_value(self._event_count_text, f"Events: {len(self._all_events)}")

    def update_controls(self) -> None:
        """Poll PCL client state and refresh status badge + tick counter (every frame)."""
        client = self._app.client
        if not hasattr(client, "last_bt_status") or not client.connected:
            return

        status = client.last_bt_status()
        color = STATUS_COLOURS.get(status, (180, 180, 190, 255))
        dpg.set_value(self._status_badge, status)
        dpg.configure_item(self._status_badge, color=color)
        dpg.set_value(self._tick_label, str(client.tick_count))

        executing = client.is_executing()
        dpg.configure_item(self._stop_btn, enabled=executing)

        mode = dpg.get_value(self._mode_combo)
        manual = (mode == "Manual step")
        dpg.configure_item(
            self._tick_once_btn,
            enabled=(manual and executing),
        )

    # ------------------------------------------------------------------
    # Control bar callbacks
    # ------------------------------------------------------------------

    def _on_mode_changed(self, sender=None, value=None, user_data=None) -> None:
        mode = dpg.get_value(self._mode_combo)
        is_manual = (mode == "Manual step")
        dpg.configure_item(self._interval_input, enabled=not is_manual)

    def _on_load_start(self, sender=None, value=None, user_data=None) -> None:
        bt_xml = self._app.last_bt_xml
        if not bt_xml:
            return

        client = self._app.client
        if not hasattr(client, "load_bt"):
            return

        # Clear display for the new run
        self._on_clear()

        # Parse and display tree structure
        self._bt_xml = bt_xml
        self._render_tree()

        authority_val = dpg.get_value(self._authority_combo)
        authority = "confirmed" if "confirmed" in authority_val else "any"
        bt_xml = self._inject_required_authority(bt_xml, authority)

        mode = dpg.get_value(self._mode_combo)
        client.load_bt(bt_xml)

        if mode == "Auto-tick":
            interval = float(dpg.get_value(self._interval_input))
            client.start_auto_tick(
                interval_s=interval,
                done_callback=self._on_execution_done,
            )
        # Manual: tree is loaded, user drives via Tick Once

    def _on_stop(self, sender=None, value=None, user_data=None) -> None:
        client = self._app.client
        if hasattr(client, "stop_execution"):
            client.stop_execution()

    def _on_tick_once(self, sender=None, value=None, user_data=None) -> None:
        client = self._app.client
        if hasattr(client, "tick_once"):
            client.tick_once()

    def _on_execution_done(self, success: bool, message: str = "") -> None:
        color = STATUS_COLOURS.get("SUCCESS" if success else "FAILURE",
                                   (180, 180, 190, 255))
        status = "SUCCESS" if success else "FAILURE"
        dpg.set_value(self._status_badge, status)
        dpg.configure_item(self._status_badge, color=color)

    def _on_clear(self, sender=None, value=None, user_data=None) -> None:
        self._all_events.clear()
        self._node_statuses.clear()
        self._node_events.clear()
        self._events_at_last_timeline_refresh = 0
        dpg.set_value(self._event_count_text, "Events: 0")
        self.refresh_display()

    # ------------------------------------------------------------------
    # BT XML transforms
    # ------------------------------------------------------------------

    @staticmethod
    def _inject_required_authority(bt_xml: str, authority: str) -> str:
        """Return bt_xml with required_authority set on every CheckWorldPredicate node."""
        try:
            root = ET.fromstring(bt_xml)
            for node in root.iter("CheckWorldPredicate"):
                node.set("required_authority", authority)
            return ET.tostring(root, encoding="unicode")
        except ET.ParseError:
            return bt_xml

    # ------------------------------------------------------------------
    # Tree rendering
    # ------------------------------------------------------------------

    def _render_tree(self) -> None:
        children = dpg.get_item_children(self._tree_panel, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        if self._bt_xml:
            try:
                root = ET.fromstring(self._bt_xml)
                self._render_xml_node(root, self._tree_panel, 0)
            except ET.ParseError:
                dpg.add_text(
                    "Failed to parse BT XML",
                    parent=self._tree_panel,
                    color=(220, 60, 60)
                )
        elif self._node_statuses:
            for name, status in sorted(self._node_statuses.items()):
                color = STATUS_COLOURS.get(status, (180, 180, 190, 255))
                with dpg.group(horizontal=True, parent=self._tree_panel):
                    dpg.add_text(f"[{status:8s}]", color=color)
                    dpg.add_button(
                        label=name,
                        callback=lambda s, a, u: self._select_node(u),
                        user_data=name,
                    )
        else:
            dpg.add_text(
                "No BT data. Use 'Load & Start' or run from the Planning tab.",
                parent=self._tree_panel,
                color=(160, 160, 170), wrap=-1,
            )

    # Attributes that are meaningful to show inline
    _DISPLAY_ATTRS = {"predicate", "value", "from", "to", "target", "agent",
                      "required_authority"}
    # Attributes that are BT-internal / not user-facing
    _SKIP_ATTRS = {"name", "ID", "id"}

    def _render_xml_node(self, elem: ET.Element, parent: int,
                          depth: int) -> None:
        tag = elem.tag
        name_attr = elem.get("name", "")

        # Status lookup key: explicit name attr, else the tag type
        lookup_key = name_attr if name_attr else tag
        status = self._node_statuses.get(lookup_key, "IDLE")
        color = STATUS_COLOURS.get(status, (120, 120, 130, 255))

        # Build display label: tag + any meaningful attributes
        extra_attrs = [
            f"{k}={v}" for k, v in elem.attrib.items()
            if k not in self._SKIP_ATTRS
            and (k in self._DISPLAY_ATTRS or not name_attr)
        ]
        if extra_attrs:
            display = f"{tag}({', '.join(extra_attrs)})"
        elif name_attr and name_attr != tag:
            display = f"{tag}: {name_attr}"
        else:
            display = tag

        has_children = len(elem) > 0

        if has_children:
            with dpg.tree_node(
                label=f"[{status}] {display}",
                parent=parent,
                default_open=(depth < 2),
            ) as node_id:
                dpg.bind_item_theme(
                    node_id,
                    self._get_or_create_color_theme(color)
                )
                for child in elem:
                    self._render_xml_node(child, node_id, depth + 1)
        else:
            with dpg.group(horizontal=True, parent=parent):
                indent = "  " * depth
                dpg.add_text(f"{indent}[{status:8s}]", color=color)
                dpg.add_button(
                    label=display,
                    callback=lambda s, a, u: self._select_node(u),
                    user_data=lookup_key,
                )

    _color_themes: dict[tuple, int] = {}

    def _get_or_create_color_theme(self, color: tuple) -> int:
        if color not in self._color_themes:
            with dpg.theme() as t:
                with dpg.theme_component(dpg.mvAll):
                    dpg.add_theme_color(dpg.mvThemeCol_Text, color)
            self._color_themes[color] = t
        return self._color_themes[color]

    # ------------------------------------------------------------------
    # Node inspector
    # ------------------------------------------------------------------

    def _select_node(self, node_name: str) -> None:
        self._selected_node = node_name
        events = self._node_events.get(node_name, [])

        children = dpg.get_item_children(self._inspector_panel, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        status = self._node_statuses.get(node_name, "IDLE")
        color = STATUS_COLOURS.get(status, (120, 120, 130, 255))

        dpg.add_text(f"Node: {node_name}",
                     parent=self._inspector_panel, color=(220, 220, 230))
        dpg.add_text(f"Status: {status}",
                     parent=self._inspector_panel, color=color)
        dpg.add_spacer(height=6, parent=self._inspector_panel)

        if events:
            total = len(events)
            successes = sum(1 for e in events if e.status == "SUCCESS")
            failures  = sum(1 for e in events if e.status == "FAILURE")
            running   = sum(1 for e in events if e.status == "RUNNING")

            dpg.add_text(
                f"Total transitions: {total}\n"
                f"  SUCCESS: {successes}\n"
                f"  FAILURE: {failures}\n"
                f"  RUNNING: {running}",
                parent=self._inspector_panel,
            )
            dpg.add_spacer(height=8, parent=self._inspector_panel)
            dpg.add_text("Recent Events:", parent=self._inspector_panel,
                         color=(180, 180, 200))
            dpg.add_separator(parent=self._inspector_panel)

            for e in events[-20:]:
                evt_color = STATUS_COLOURS.get(e.status, (180, 180, 190, 255))
                dpg.add_text(
                    f"  {e.prev_status} -> {e.status}  (v{e.wm_version})",
                    parent=self._inspector_panel, color=evt_color,
                )
        else:
            dpg.add_text("No events recorded for this node.",
                         parent=self._inspector_panel, color=(160, 160, 170))

    # ------------------------------------------------------------------
    # Timeline
    # ------------------------------------------------------------------

    def _update_timeline(self) -> None:
        n = len(self._all_events)
        if not n:
            return
        new_data = n > self._events_at_last_timeline_refresh
        self._events_at_last_timeline_refresh = n

        t0 = self._all_events[0].ts_sec
        xs = [e.ts_sec - t0 for e in self._all_events]
        ys = [float(e.wm_version) for e in self._all_events]

        children = dpg.get_item_children(self._timeline_y, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        dpg.add_stair_series(xs, ys, label="WM Version", parent=self._timeline_y)

        if new_data:
            dpg.fit_axis_data(self._timeline_x)
            _set_integer_y_axis(self._timeline_y, ys)
