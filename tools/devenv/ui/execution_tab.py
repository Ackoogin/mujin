"""Execution tab — BT tree visualisation, node status, timeline."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from ..config import STATUS_COLOURS
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

        # Widget IDs
        self._tree_panel: int = 0
        self._inspector_panel: int = 0
        self._timeline_plot: int = 0
        self._timeline_x: int = 0
        self._timeline_y: int = 0
        self._status_text: int = 0
        self._event_count_text: int = 0
        self._selected_node: str = ""
        self._inspector_text: int = 0

    def build(self, parent: int) -> None:
        with dpg.group(parent=parent):
            # Top bar
            with dpg.group(horizontal=True):
                self._status_text = dpg.add_text("BT: No tree loaded")
                dpg.add_spacer(width=20)
                self._event_count_text = dpg.add_text(
                    "Events: 0", color=(120, 120, 130)
                )
                dpg.add_spacer(width=20)
                dpg.add_button(
                    label="Clear", callback=self._on_clear, width=60
                )

            dpg.add_spacer(height=6)

            with dpg.group(horizontal=True):
                # --- Left: BT tree structure ---
                with dpg.child_window(width=500, height=-250):
                    dpg.add_text("Behaviour Tree", color=(180, 180, 200))
                    dpg.add_separator()
                    self._tree_panel = dpg.add_child_window(
                        height=-5, border=False
                    )

                # --- Right: Node inspector ---
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

            # --- Bottom: Timeline plot ---
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

    def set_bt_xml(self, xml_str: str) -> None:
        """Parse BT XML and render the tree structure."""
        self._bt_xml = xml_str
        self._render_tree()

    def add_bt_event(self, event: BTEvent) -> None:
        """Process a live BT event."""
        self._all_events.append(event)
        self._node_statuses[event.node] = event.status
        self._node_events[event.node].append(event)

        # Update event counter
        dpg.set_value(
            self._event_count_text,
            f"Events: {len(self._all_events)}"
        )

    def refresh_display(self) -> None:
        """Refresh the tree display with current statuses and update timeline."""
        self._render_tree()
        self._update_timeline()

        if self._node_statuses:
            running = sum(
                1 for s in self._node_statuses.values() if s == "RUNNING"
            )
            dpg.set_value(
                self._status_text,
                f"BT: {len(self._node_statuses)} nodes, {running} running"
            )

    def _render_tree(self) -> None:
        """Render the BT tree structure with status colours."""
        # Clear existing tree
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
            # No XML but we have events — render flat list
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
                "No BT data. Load a plan or connect to a running executor.",
                parent=self._tree_panel,
                color=(160, 160, 170), wrap=-1,
            )

    def _render_xml_node(self, elem: ET.Element, parent: int,
                          depth: int) -> None:
        """Recursively render an XML element as a tree node."""
        name = elem.get("name", elem.tag)
        node_type = elem.tag
        display = f"{node_type}: {name}" if name != node_type else node_type

        status = self._node_statuses.get(name, "IDLE")
        color = STATUS_COLOURS.get(status, (120, 120, 130, 255))

        has_children = len(elem) > 0

        if has_children:
            # Use a tree node for containers
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
            # Leaf node — clickable button
            with dpg.group(horizontal=True, parent=parent):
                indent = "  " * depth
                status_text = dpg.add_text(
                    f"{indent}[{status:8s}]", color=color
                )
                dpg.add_button(
                    label=display,
                    callback=lambda s, a, u: self._select_node(u),
                    user_data=name,
                )

    _color_themes: dict[tuple, int] = {}

    def _get_or_create_color_theme(self, color: tuple) -> int:
        """Cache and return a text-colour theme."""
        if color not in self._color_themes:
            with dpg.theme() as t:
                with dpg.theme_component(dpg.mvAll):
                    dpg.add_theme_color(dpg.mvThemeCol_Text, color)
            self._color_themes[color] = t
        return self._color_themes[color]

    def _select_node(self, node_name: str) -> None:
        """Show inspector details for the selected node."""
        self._selected_node = node_name
        events = self._node_events.get(node_name, [])

        # Clear inspector
        children = dpg.get_item_children(self._inspector_panel, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        status = self._node_statuses.get(node_name, "IDLE")
        color = STATUS_COLOURS.get(status, (120, 120, 130, 255))

        dpg.add_text(
            f"Node: {node_name}",
            parent=self._inspector_panel,
            color=(220, 220, 230),
        )
        dpg.add_text(
            f"Status: {status}",
            parent=self._inspector_panel,
            color=color,
        )
        dpg.add_spacer(height=6, parent=self._inspector_panel)

        if events:
            total = len(events)
            successes = sum(1 for e in events if e.status == "SUCCESS")
            failures = sum(1 for e in events if e.status == "FAILURE")
            running = sum(1 for e in events if e.status == "RUNNING")

            dpg.add_text(
                f"Total transitions: {total}\n"
                f"  SUCCESS: {successes}\n"
                f"  FAILURE: {failures}\n"
                f"  RUNNING: {running}",
                parent=self._inspector_panel,
            )

            dpg.add_spacer(height=8, parent=self._inspector_panel)
            dpg.add_text(
                "Recent Events:",
                parent=self._inspector_panel,
                color=(180, 180, 200),
            )
            dpg.add_separator(parent=self._inspector_panel)

            # Show last 20 events
            for e in events[-20:]:
                evt_color = STATUS_COLOURS.get(e.status, (180, 180, 190, 255))
                dpg.add_text(
                    f"  {e.prev_status} -> {e.status}  (v{e.wm_version})",
                    parent=self._inspector_panel,
                    color=evt_color,
                )
        else:
            dpg.add_text(
                "No events recorded for this node.",
                parent=self._inspector_panel,
                color=(160, 160, 170),
            )

    def _update_timeline(self) -> None:
        """Update the timeline plot with WM version over time."""
        if not self._all_events:
            return

        # Find the base timestamp
        t0 = self._all_events[0].ts_sec if self._all_events else 0

        xs = [e.ts_sec - t0 for e in self._all_events]
        ys = [float(e.wm_version) for e in self._all_events]

        # Clear existing series
        children = dpg.get_item_children(self._timeline_y, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        dpg.add_line_series(
            xs, ys, label="WM Version",
            parent=self._timeline_y,
        )

        # Auto-fit axes
        dpg.fit_axis_data(self._timeline_x)
        dpg.fit_axis_data(self._timeline_y)

    def _on_clear(self, sender=None, value=None, user_data=None) -> None:
        self._all_events.clear()
        self._node_statuses.clear()
        self._node_events.clear()
        self.refresh_display()
