"""World Model tab -- facts table, objects, types, goals, fact injection."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from ..config import STATUS_COLOURS, SOURCE_COLOURS
from ..models.events import WorldFact, WorldSnapshot

if TYPE_CHECKING:
    from .app import App


class WorldModelTab:
    """Builds and manages the World Model tab contents."""

    def __init__(self, app: App):
        self._app = app
        self._facts: list[WorldFact] = []
        self._goal_fluents: list[str] = []
        self._wm_version: int = 0
        self._filter_text: str = ""

        # Widget IDs
        self._facts_table: int = 0
        self._version_label: int = 0
        self._filter_input: int = 0
        self._inject_key_input: int = 0
        self._inject_value_check: int = 0
        self._inject_source_input: int = 0
        self._goals_list: int = 0
        self._goal_input: int = 0
        self._status_text: int = 0

    def build(self, parent: int) -> None:
        """Create all widgets inside the given parent container."""
        with dpg.group(parent=parent):
            # Top bar: version + filter + refresh
            with dpg.group(horizontal=True):
                self._version_label = dpg.add_text("WM Version: --")
                dpg.add_spacer(width=20)
                dpg.add_text("Filter:")
                self._filter_input = dpg.add_input_text(
                    width=250, callback=self._on_filter_changed,
                    hint="type to filter facts..."
                )
                dpg.add_spacer(width=10)
                dpg.add_button(
                    label="Refresh", callback=self._on_refresh,
                    width=80
                )
                dpg.add_spacer(width=10)
                self._status_text = dpg.add_text("", color=(120, 120, 130))

            dpg.add_spacer(height=6)

            # Main content: left = facts table, right = inject + goals
            with dpg.group(horizontal=True):
                # --- Left: Facts table ---
                with dpg.child_window(width=-350, height=-5):
                    dpg.add_text("Facts", color=(180, 180, 200))
                    dpg.add_separator()
                    self._facts_table = dpg.add_table(
                        header_row=True,
                        resizable=True,
                        sortable=True,
                        borders_innerH=True,
                        borders_outerH=True,
                        borders_innerV=True,
                        borders_outerV=True,
                        row_background=True,
                        scrollY=True,
                        height=-5,
                        policy=dpg.mvTable_SizingStretchProp,
                    )
                    dpg.add_table_column(
                        label="Fluent Key", parent=self._facts_table,
                        width_stretch=True, init_width_or_weight=3.0
                    )
                    dpg.add_table_column(
                        label="Value", parent=self._facts_table,
                        width_stretch=True, init_width_or_weight=0.5
                    )
                    dpg.add_table_column(
                        label="Source", parent=self._facts_table,
                        width_stretch=True, init_width_or_weight=1.5
                    )
                    dpg.add_table_column(
                        label="Authority", parent=self._facts_table,
                        width_stretch=True, init_width_or_weight=0.8
                    )

                # --- Right: Inject + Goals panels ---
                with dpg.child_window(width=330, height=-5):
                    # Inject Fact panel
                    dpg.add_text("Inject Fact", color=(180, 180, 200))
                    dpg.add_separator()
                    dpg.add_spacer(height=4)

                    dpg.add_text("Fluent Key:")
                    self._inject_key_input = dpg.add_input_text(
                        width=-1, hint="(at uav1 base)"
                    )

                    with dpg.group(horizontal=True):
                        dpg.add_text("Value:")
                        self._inject_value_check = dpg.add_checkbox(
                            default_value=True
                        )

                    dpg.add_text("Source:")
                    self._inject_source_input = dpg.add_input_text(
                        width=-1, default_value="devenv"
                    )

                    dpg.add_spacer(height=4)
                    dpg.add_button(
                        label="Set Fact",
                        callback=self._on_inject_fact,
                        width=-1,
                    )

                    dpg.add_spacer(height=16)
                    dpg.add_separator()
                    dpg.add_spacer(height=4)

                    # Goals panel
                    dpg.add_text("Goal Fluents", color=(180, 180, 200))
                    dpg.add_separator()
                    dpg.add_spacer(height=4)

                    self._goals_list = dpg.add_child_window(
                        height=200, border=True
                    )

                    dpg.add_spacer(height=4)
                    self._goal_input = dpg.add_input_text(
                        width=-1, hint="(searched sector_a)"
                    )
                    with dpg.group(horizontal=True):
                        dpg.add_button(
                            label="Add Goal",
                            callback=self._on_add_goal,
                            width=-60,
                        )
                        dpg.add_button(
                            label="Clear",
                            callback=self._on_clear_goals,
                            width=52,
                        )

    def update_from_snapshot(self, snapshot: WorldSnapshot) -> None:
        """Update the tab with a new world state snapshot."""
        self._facts = snapshot.facts
        self._goal_fluents = snapshot.goal_fluents
        self._wm_version = snapshot.wm_version
        self._refresh_display()

    def update_from_wm_event(self, fact_key: str, value: bool,
                              source: str, wm_version: int) -> None:
        """Incrementally update a single fact from a WM audit event."""
        self._wm_version = wm_version
        # Update or add the fact
        for f in self._facts:
            if f.key == fact_key:
                f.value = value
                f.source = source
                f.wm_version = wm_version
                break
        else:
            self._facts.append(WorldFact(
                key=fact_key, value=value, source=source,
                wm_version=wm_version
            ))
        self._refresh_display()

    def _refresh_display(self) -> None:
        """Rebuild the facts table and goals list."""
        dpg.set_value(
            self._version_label,
            f"WM Version: {self._wm_version}"
        )

        # Filter facts
        filter_text = self._filter_text.lower()
        visible_facts = [
            f for f in self._facts
            if not filter_text or filter_text in f.key.lower()
        ]

        # Rebuild table rows
        children = dpg.get_item_children(self._facts_table, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        for fact in visible_facts:
            with dpg.table_row(parent=self._facts_table):
                dpg.add_text(fact.key)

                value_color = (80, 200, 80, 255) if fact.value else (220, 60, 60, 255)
                dpg.add_text(
                    "TRUE" if fact.value else "FALSE",
                    color=value_color
                )

                source_color = (180, 180, 190, 255)
                for prefix, colour in SOURCE_COLOURS.items():
                    if fact.source.startswith(prefix):
                        source_color = colour
                        break
                dpg.add_text(fact.source, color=source_color)

                dpg.add_text(fact.authority)

        count = len(visible_facts)
        total = len(self._facts)
        if filter_text:
            dpg.set_value(self._status_text, f"Showing {count}/{total} facts")
        else:
            dpg.set_value(self._status_text, f"{total} facts")

        # Rebuild goals list
        children = dpg.get_item_children(self._goals_list, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        for g in self._goal_fluents:
            with dpg.group(horizontal=True, parent=self._goals_list):
                dpg.add_text(g)
                dpg.add_button(
                    label="X", width=20,
                    callback=lambda s, a, u: self._on_remove_goal(u),
                    user_data=g,
                )

    # -- Callbacks -----------------------------------------------------------

    def _on_filter_changed(self, sender, value, user_data) -> None:
        self._filter_text = value
        self._refresh_display()

    def _on_refresh(self, sender=None, value=None, user_data=None) -> None:
        """Request a full state refresh from ROS2."""
        dpg.set_value(self._status_text, "Refreshing...")
        self._app.ros2.query_state(callback=self._on_query_result)

    def _on_query_result(self, snapshot: WorldSnapshot) -> None:
        self.update_from_snapshot(snapshot)

    def _on_inject_fact(self, sender=None, value=None, user_data=None) -> None:
        key = dpg.get_value(self._inject_key_input)
        val = dpg.get_value(self._inject_value_check)
        source = dpg.get_value(self._inject_source_input)
        if not key.strip():
            return
        self._app.ros2.set_fact(
            key.strip(), val, source.strip(),
            callback=lambda ok, ver: dpg.set_value(
                self._status_text,
                f"Set OK (v{ver})" if ok else "Set FAILED"
            ),
        )

    def _on_add_goal(self, sender=None, value=None, user_data=None) -> None:
        goal = dpg.get_value(self._goal_input).strip()
        if goal and goal not in self._goal_fluents:
            self._goal_fluents.append(goal)
            dpg.set_value(self._goal_input, "")
            self._refresh_display()

    def _on_remove_goal(self, goal: str) -> None:
        if goal in self._goal_fluents:
            self._goal_fluents.remove(goal)
            self._refresh_display()

    def _on_clear_goals(self, sender=None, value=None, user_data=None) -> None:
        self._goal_fluents.clear()
        self._refresh_display()
