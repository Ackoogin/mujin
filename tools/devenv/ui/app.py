"""Top-level application window, tab bar, status bar, and frame loop."""

from __future__ import annotations

import time
from typing import Optional, Union

import dearpygui.dearpygui as dpg

from ..config import AppConfig
from ..comms.foxglove_client import FoxgloveClient
from ..comms.ros2_client import AmeRos2Client
from ..comms.pcl_client import AmePclClient
from ..models.events import BTEvent, WMAuditEntry, WorldSnapshot
from .theme import (
    create_global_theme,
    create_accent_button_theme,
)
from .world_model_tab import WorldModelTab
from .planning_tab import PlanningTab
from .execution_tab import ExecutionTab
from .observability_tab import ObservabilityTab
from .pddl_editor_tab import PddlEditorTab


class App:
    """Main application controller — owns comms clients, tabs, and frame loop."""

    def __init__(self, config: Optional[AppConfig] = None):
        self.config = config or AppConfig()

        # Communication clients
        self.foxglove = FoxgloveClient(
            url=self.config.connection.foxglove_url,
            buffer_size=self.config.ui.event_buffer_size,
        )
        
        # Select backend client based on config
        backend = self.config.connection.backend
        if backend == "pcl":
            self.client: Union[AmeRos2Client, AmePclClient] = AmePclClient()
            self._backend_name = "PCL"
        else:
            self.client = AmeRos2Client(
                node_name=self.config.connection.ros2_node_name,
            )
            self._backend_name = "ROS2"
        
        # Alias for backward compatibility
        self.ros2 = self.client

        # Tabs (created during build)
        self.world_model_tab: Optional[WorldModelTab] = None
        self.planning_tab: Optional[PlanningTab] = None
        self.execution_tab: Optional[ExecutionTab] = None
        self.observability_tab: Optional[ObservabilityTab] = None
        self.pddl_editor_tab: Optional[PddlEditorTab] = None

        # Status bar widgets
        self._foxglove_status: int = 0
        self._ros2_status: int = 0
        self._wm_version_label: int = 0
        self._goals_label: int = 0
        self._fps_label: int = 0

        # Frame timing
        self._last_frame_time = time.monotonic()
        self._frame_count = 0
        self._fps = 0.0
        self._fps_update_time = time.monotonic()

        # Plot refresh throttle
        self._last_plot_refresh = 0.0
        self._plot_refresh_interval = 1.0  # seconds

        # Last compiled BT XML — shared between Planning and Execution tabs
        self.last_bt_xml: str = ""

    def run(self) -> None:
        """Initialise Dear PyGui, build UI, start comms, enter render loop."""
        dpg.create_context()

        # Viewport
        dpg.create_viewport(
            title=self.config.ui.window_title,
            width=self.config.ui.window_width,
            height=self.config.ui.window_height,
            min_width=900,
            min_height=600,
        )

        # Global theme
        theme = create_global_theme()
        dpg.bind_theme(theme)

        # Build the UI
        self._build_ui()

        dpg.setup_dearpygui()
        dpg.show_viewport()

        # Start communication clients
        self.foxglove.start()
        
        # Start backend client with appropriate arguments
        if self.config.connection.backend == "pcl":
            self.client.start(
                domain_path=self.config.connection.domain_path,
                problem_path=self.config.connection.problem_path,
            )
        else:
            self.client.start()

        # Register the per-frame callback on the render loop
        while dpg.is_dearpygui_running():
            self._on_frame()
            dpg.render_dearpygui_frame()

        # Cleanup
        self.foxglove.stop()
        self.ros2.stop()
        dpg.destroy_context()

    def _build_ui(self) -> None:
        """Create the primary window with tab bar, status bar."""
        with dpg.window(tag="primary_window"):
            # Main tab bar
            with dpg.tab_bar(tag="main_tabs"):
                # Tab 1: World Model
                with dpg.tab(label="  World Model  "):
                    self.world_model_tab = WorldModelTab(self)
                    with dpg.child_window(border=False) as container:
                        self.world_model_tab.build(container)

                # Tab 2: Planning
                with dpg.tab(label="  Planning  "):
                    self.planning_tab = PlanningTab(self)
                    with dpg.child_window(border=False) as container:
                        self.planning_tab.build(container)

                # Tab 3: Execution
                with dpg.tab(label="  Execution  "):
                    self.execution_tab = ExecutionTab(self)
                    with dpg.child_window(border=False) as container:
                        self.execution_tab.build(container)

                # Tab 4: Observability
                with dpg.tab(label="  Observability  "):
                    self.observability_tab = ObservabilityTab(self)
                    with dpg.child_window(border=False) as container:
                        self.observability_tab.build(container)

                # Tab 5: PDDL Editor
                with dpg.tab(label="  PDDL Editor  "):
                    self.pddl_editor_tab = PddlEditorTab(self)
                    with dpg.child_window(border=False) as container:
                        self.pddl_editor_tab.build(container)

            # Status bar at bottom
            dpg.add_spacer(height=2)
            dpg.add_separator()
            with dpg.group(horizontal=True):
                self._foxglove_status = dpg.add_text(
                    "Foxglove: --", color=(120, 120, 130)
                )
                dpg.add_text("  |  ", color=(55, 55, 68))
                self._ros2_status = dpg.add_text(
                    "ROS2: --", color=(120, 120, 130)
                )
                dpg.add_text("  |  ", color=(55, 55, 68))
                self._wm_version_label = dpg.add_text(
                    "WM: --", color=(120, 120, 130)
                )
                dpg.add_text("  |  ", color=(55, 55, 68))
                self._goals_label = dpg.add_text(
                    "Goals: --", color=(120, 120, 130)
                )
                dpg.add_spacer()
                self._fps_label = dpg.add_text(
                    "", color=(80, 80, 90)
                )

        dpg.set_primary_window("primary_window", True)

    def _on_frame(self) -> None:
        """Called every frame — drain events, update UI."""
        now = time.monotonic()

        # FPS counter
        self._frame_count += 1
        if now - self._fps_update_time >= 1.0:
            self._fps = self._frame_count / (now - self._fps_update_time)
            self._frame_count = 0
            self._fps_update_time = now
            dpg.set_value(self._fps_label, f"{self._fps:.0f} fps")

        # --- Drain Foxglove events ---
        bt_events = self.foxglove.drain_bt_events()
        wm_events = self.foxglove.drain_wm_events()

        for raw in bt_events:
            evt = BTEvent.from_json(raw)
            if self.execution_tab:
                self.execution_tab.add_bt_event(evt)
            if self.observability_tab:
                self.observability_tab.add_bt_event(evt)

        for raw in wm_events:
            entry = WMAuditEntry.from_json(raw)
            if self.world_model_tab:
                self.world_model_tab.update_from_wm_event(
                    entry.fact, entry.value, entry.source, entry.wm_version
                )
            if self.observability_tab:
                self.observability_tab.add_wm_event(entry)

        # --- Drain PCL in-process BT events ---
        pcl_bt_events: list = []
        if hasattr(self.client, "drain_bt_events"):
            for raw in self.client.drain_bt_events():
                pcl_bt_events.append(raw)
                evt = BTEvent.from_json(raw)
                if self.execution_tab:
                    self.execution_tab.add_bt_event(evt)
                if self.observability_tab:
                    self.observability_tab.add_bt_event(evt)

        # --- Drain PCL in-process WM audit events ---
        pcl_wm_events: list = []
        if hasattr(self.client, "drain_wm_events"):
            pcl_wm_events = self.client.drain_wm_events()
            if pcl_wm_events:
                print(f"[app] drained {len(pcl_wm_events)} PCL WM events")
            for raw in pcl_wm_events:
                entry = WMAuditEntry.from_json(raw)
                if self.world_model_tab:
                    self.world_model_tab.update_from_wm_event(
                        entry.fact, entry.value, entry.source, entry.wm_version
                    )
                if self.observability_tab:
                    self.observability_tab.add_wm_event(entry)

        # --- Drain plan feedback ---
        if self.planning_tab:
            self.planning_tab.update_feedback()

        # --- Update execution tab controls (status label, tick counter) ---
        if self.execution_tab:
            self.execution_tab.update_controls()

        # --- Periodic plot/display refresh (throttled) ---
        have_bt = bool(bt_events or pcl_bt_events)
        if now - self._last_plot_refresh >= self._plot_refresh_interval:
            self._last_plot_refresh = now
            if have_bt and self.execution_tab:
                self.execution_tab.refresh_display()
            if (wm_events or pcl_wm_events) and self.observability_tab:
                self.observability_tab.refresh_plots()

        # --- Update status bar ---
        self._update_status_bar()

    def _update_status_bar(self) -> None:
        # Foxglove connection
        if self.foxglove.connected:
            dpg.set_value(self._foxglove_status, "Foxglove: Connected")
            dpg.configure_item(
                self._foxglove_status, color=(80, 200, 80, 255)
            )
        else:
            dpg.set_value(self._foxglove_status, "Foxglove: Disconnected")
            dpg.configure_item(
                self._foxglove_status, color=(220, 60, 60, 255)
            )

        # ROS2 connection
        if self.ros2.connected:
            dpg.set_value(self._ros2_status, "ROS2: Connected")
            dpg.configure_item(
                self._ros2_status, color=(80, 200, 80, 255)
            )
        elif not self.ros2.available:
            dpg.set_value(self._ros2_status, "ROS2: Not installed")
            dpg.configure_item(
                self._ros2_status, color=(180, 140, 60, 255)
            )
        else:
            dpg.set_value(self._ros2_status, "ROS2: Disconnected")
            dpg.configure_item(
                self._ros2_status, color=(220, 60, 60, 255)
            )

        # WM version from latest snapshot or events
        snap = self.ros2.latest_snapshot
        if snap:
            dpg.set_value(
                self._wm_version_label, f"WM: v{snap.wm_version}"
            )
            dpg.set_value(
                self._goals_label,
                f"Goals: {len(snap.goal_fluents)}"
            )
