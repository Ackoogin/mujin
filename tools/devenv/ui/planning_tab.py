"""Planning tab — goal editor, plan trigger, results viewer, BT XML."""

from __future__ import annotations

from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from ..models.events import PlanEpisode

if TYPE_CHECKING:
    from .app import App
    from ..comms.ros2_client import PlanResult


class PlanningTab:
    """Builds and manages the Planning tab."""

    def __init__(self, app: App):
        self._app = app
        self._goal_fluents: list[str] = []
        self._plan_history: list[PlanEpisode] = []

        # Widget IDs
        self._goal_input: int = 0
        self._goals_list: int = 0
        self._plan_button: int = 0
        self._feedback_text: int = 0
        self._result_panel: int = 0
        self._steps_table: int = 0
        self._stats_text: int = 0
        self._bt_xml_text: int = 0
        self._history_table: int = 0

    def build(self, parent: int) -> None:
        with dpg.group(parent=parent):
            with dpg.group(horizontal=True):
                # --- Left: Goal editor + Plan ---
                with dpg.child_window(width=400, height=-5):
                    dpg.add_text("Goal Fluents", color=(180, 180, 200))
                    dpg.add_separator()
                    dpg.add_spacer(height=4)

                    self._goals_list = dpg.add_child_window(
                        height=200, border=True
                    )

                    dpg.add_spacer(height=4)
                    self._goal_input = dpg.add_input_text(
                        width=-1, hint="(searched sector_a)",
                        on_enter=True, callback=self._on_add_goal,
                    )
                    with dpg.group(horizontal=True):
                        dpg.add_button(
                            label="Add Goal", callback=self._on_add_goal,
                            width=120,
                        )
                        dpg.add_button(
                            label="Clear All", callback=self._on_clear_goals,
                            width=80,
                        )

                    dpg.add_spacer(height=12)
                    dpg.add_separator()
                    dpg.add_spacer(height=8)

                    self._plan_button = dpg.add_button(
                        label="  Plan  ",
                        callback=self._on_plan,
                        width=-1, height=40,
                    )
                    dpg.add_spacer(height=6)
                    self._feedback_text = dpg.add_text(
                        "", color=(120, 120, 130), wrap=380
                    )

                    dpg.add_spacer(height=16)
                    dpg.add_separator()
                    dpg.add_spacer(height=4)

                    # Statistics
                    dpg.add_text("Last Plan Statistics", color=(180, 180, 200))
                    dpg.add_separator()
                    self._stats_text = dpg.add_text(
                        "No plan yet.", wrap=380, color=(160, 160, 170)
                    )

                # --- Right: Results ---
                with dpg.child_window(width=-1, height=-5):
                    with dpg.tab_bar():
                        # Plan steps
                        with dpg.tab(label="Plan Steps"):
                            self._steps_table = dpg.add_table(
                                header_row=True,
                                resizable=True,
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
                                label="#", parent=self._steps_table,
                                init_width_or_weight=0.3,
                                width_stretch=True,
                            )
                            dpg.add_table_column(
                                label="Action", parent=self._steps_table,
                                init_width_or_weight=3.0,
                                width_stretch=True,
                            )

                        # BT XML
                        with dpg.tab(label="Compiled BT XML"):
                            self._bt_xml_text = dpg.add_input_text(
                                multiline=True, readonly=True,
                                width=-1, height=-5,
                                tab_input=False,
                            )

                        # History
                        with dpg.tab(label="Plan History"):
                            self._history_table = dpg.add_table(
                                header_row=True,
                                resizable=True,
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
                                label="#", parent=self._history_table,
                                init_width_or_weight=0.3,
                                width_stretch=True,
                            )
                            dpg.add_table_column(
                                label="Goals", parent=self._history_table,
                                init_width_or_weight=2.0,
                                width_stretch=True,
                            )
                            dpg.add_table_column(
                                label="Steps", parent=self._history_table,
                                init_width_or_weight=0.5,
                                width_stretch=True,
                            )
                            dpg.add_table_column(
                                label="Time (ms)", parent=self._history_table,
                                init_width_or_weight=0.7,
                                width_stretch=True,
                            )
                            dpg.add_table_column(
                                label="Result", parent=self._history_table,
                                init_width_or_weight=0.5,
                                width_stretch=True,
                            )

    def add_plan_episode(self, episode: PlanEpisode) -> None:
        """Add a plan episode to history (from JSONL replay or live)."""
        self._plan_history.append(episode)
        self._refresh_history()

    def update_feedback(self) -> None:
        """Drain plan feedback from ROS2 client and update display."""
        for fb in self._app.ros2.drain_plan_feedback():
            dpg.set_value(
                self._feedback_text,
                f"Expanded: {fb.nodes_expanded}  "
                f"Generated: {fb.nodes_generated}  "
                f"Elapsed: {fb.elapsed_ms:.0f}ms\n"
                f"{fb.status_msg}"
            )

    # -- Callbacks -----------------------------------------------------------

    def _on_add_goal(self, sender=None, value=None, user_data=None) -> None:
        goal = dpg.get_value(self._goal_input).strip()
        if goal and goal not in self._goal_fluents:
            self._goal_fluents.append(goal)
            dpg.set_value(self._goal_input, "")
            self._refresh_goals_list()

    def _on_clear_goals(self, sender=None, value=None, user_data=None) -> None:
        self._goal_fluents.clear()
        self._refresh_goals_list()

    def _on_plan(self, sender=None, value=None, user_data=None) -> None:
        if not self._goal_fluents:
            dpg.set_value(self._feedback_text, "No goals specified.")
            return

        dpg.set_value(self._feedback_text, "Planning...")
        dpg.configure_item(self._plan_button, enabled=False)
        self._app.ros2.plan(
            self._goal_fluents[:],
            callback=self._on_plan_result,
        )

    def _on_plan_result(self, result: PlanResult) -> None:
        dpg.configure_item(self._plan_button, enabled=True)

        if result.success:
            dpg.set_value(
                self._feedback_text,
                f"Plan found: {len(result.plan_actions)} steps in "
                f"{result.solve_time_ms:.1f}ms"
            )
            dpg.set_value(
                self._stats_text,
                f"Solve time:  {result.solve_time_ms:.1f} ms\n"
                f"Expanded:    {result.expanded}\n"
                f"Generated:   {result.generated}\n"
                f"Steps:       {len(result.plan_actions)}"
            )
        else:
            dpg.set_value(
                self._feedback_text,
                f"Planning FAILED: {result.error_msg}"
            )
            dpg.set_value(self._stats_text, f"Failed: {result.error_msg}")

        # Update plan steps table
        self._clear_table(self._steps_table)
        for i, action in enumerate(result.plan_actions):
            with dpg.table_row(parent=self._steps_table):
                dpg.add_text(str(i + 1))
                dpg.add_text(action)

        # Update BT XML
        dpg.set_value(self._bt_xml_text, result.bt_xml or "(no XML)")

        # Add to history
        episode = PlanEpisode(
            goal_fluents=self._goal_fluents[:],
            solve_time_ms=result.solve_time_ms,
            success=result.success,
            expanded=result.expanded,
            generated=result.generated,
            plan_actions=result.plan_actions,
            bt_xml=result.bt_xml,
        )
        self._plan_history.append(episode)
        self._refresh_history()

    def _refresh_goals_list(self) -> None:
        children = dpg.get_item_children(self._goals_list, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        for g in self._goal_fluents:
            with dpg.group(horizontal=True, parent=self._goals_list):
                dpg.add_text(g)
                dpg.add_button(
                    label="X", width=20,
                    callback=lambda s, a, u: self._remove_goal(u),
                    user_data=g,
                )

    def _remove_goal(self, goal: str) -> None:
        if goal in self._goal_fluents:
            self._goal_fluents.remove(goal)
            self._refresh_goals_list()

    def _refresh_history(self) -> None:
        self._clear_table(self._history_table)
        for i, ep in enumerate(self._plan_history):
            with dpg.table_row(parent=self._history_table):
                dpg.add_text(str(i + 1))
                dpg.add_text(", ".join(ep.goal_fluents[:3]) + (
                    "..." if len(ep.goal_fluents) > 3 else ""
                ))
                dpg.add_text(str(len(ep.plan_actions)))
                dpg.add_text(f"{ep.solve_time_ms:.1f}")
                color = (80, 200, 80, 255) if ep.success else (220, 60, 60, 255)
                dpg.add_text(
                    "OK" if ep.success else "FAIL",
                    color=color
                )

    @staticmethod
    def _clear_table(table_id: int) -> None:
        children = dpg.get_item_children(table_id, 1)
        if children:
            for child in children:
                dpg.delete_item(child)
