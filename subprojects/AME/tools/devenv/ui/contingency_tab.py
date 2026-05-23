"""Contingency Verifier tab -- run exhaustive safe-state reachability analysis."""

from __future__ import annotations

import json
import os
import subprocess
import threading
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import dearpygui.dearpygui as dpg

if TYPE_CHECKING:
    from .app import App


class ContingencyVerifierTab:
    """Runs the contingency_verifier tool and displays results."""

    def __init__(self, app: App):
        self._app = app

        self._domain_path: str = ""
        self._problem_path: str = ""
        self._verifier_binary: str = ""
        self._prune: bool = True

        self._running = False
        self._result_json: Optional[dict] = None
        self._error_msg: str = ""

        # Widget IDs
        self._domain_label: int = 0
        self._problem_label: int = 0
        self._binary_label: int = 0
        self._run_button: int = 0
        self._prune_checkbox: int = 0
        self._status_text: int = 0
        self._progress_text: int = 0
        self._summary_group: int = 0
        self._results_table: int = 0
        self._pruning_info: int = 0

    def build(self, parent: int) -> None:
        with dpg.group(parent=parent):
            dpg.add_text(
                "Contingency Verifier",
                color=(0, 230, 255, 255),
            )
            dpg.add_text(
                "Exhaustive safe-state reachability analysis across all context predicate combinations",
                color=(100, 130, 150),
            )
            dpg.add_spacer(height=8)

            # --- Configuration ---
            with dpg.child_window(height=160, border=True):
                dpg.add_text("Configuration", color=(180, 180, 200))
                dpg.add_separator()
                dpg.add_spacer(height=4)

                # Verifier binary
                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Verifier Binary",
                        callback=self._on_browse_binary,
                        width=120,
                    )
                    self._binary_label = dpg.add_text(
                        "Not set (auto-detect)", color=(180, 140, 60)
                    )
                self._auto_detect_binary()

                # Domain file
                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Domain File",
                        callback=self._on_browse_domain,
                        width=120,
                    )
                    self._domain_label = dpg.add_text(
                        "No domain selected", color=(120, 120, 130)
                    )

                # Template problem file
                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Template Problem",
                        callback=self._on_browse_problem,
                        width=120,
                    )
                    self._problem_label = dpg.add_text(
                        "No problem selected", color=(120, 120, 130)
                    )

                dpg.add_spacer(height=4)
                with dpg.group(horizontal=True):
                    self._prune_checkbox = dpg.add_checkbox(
                        label="Monotone dominance pruning",
                        default_value=True,
                        callback=self._on_prune_toggled,
                    )
                    dpg.add_spacer(width=20)
                    self._run_button = dpg.add_button(
                        label="  Run Verification  ",
                        callback=self._on_run,
                        width=180,
                    )
                    dpg.add_spacer(width=10)
                    self._status_text = dpg.add_text(
                        "", color=(120, 120, 130)
                    )

            dpg.add_spacer(height=6)

            # --- Summary ---
            self._summary_group = dpg.add_group()

            dpg.add_spacer(height=6)

            # --- Pruning info ---
            self._pruning_info = dpg.add_input_text(
                multiline=True, readonly=True,
                width=-1, height=0, show=False,
            )

            # --- Results table ---
            self._results_table = dpg.add_child_window(
                height=-5, border=True,
            )

    def _auto_detect_binary(self) -> None:
        candidates = [
            "build-ame/subprojects/AME/src/contingency_verifier",
            "build/subprojects/AME/src/contingency_verifier",
            "build/subprojects/AME/src/Release/contingency_verifier.exe",
            "build-all-off/subprojects/AME/src/contingency_verifier",
        ]
        for c in candidates:
            full = str(Path.cwd() / c)
            if os.path.isfile(full) and os.access(full, os.X_OK):
                self._verifier_binary = full
                dpg.set_value(self._binary_label, Path(full).name)
                dpg.configure_item(
                    self._binary_label, color=(80, 200, 80, 255)
                )
                return

    # -- Callbacks -----------------------------------------------------------

    def _on_browse_binary(self, sender=None, value=None,
                          user_data=None) -> None:
        dpg.add_file_dialog(
            callback=self._on_binary_selected,
            width=600, height=400,
        )

    def _on_binary_selected(self, sender, app_data,
                            user_data=None) -> None:
        path = app_data.get("file_path_name", "")
        if path and os.path.isfile(path):
            self._verifier_binary = path
            dpg.set_value(self._binary_label, Path(path).name)
            dpg.configure_item(
                self._binary_label, color=(80, 200, 80, 255)
            )

    def _on_browse_domain(self, sender=None, value=None,
                          user_data=None) -> None:
        dpg.add_file_dialog(
            callback=self._on_domain_selected,
            width=600, height=400,
        )

    def _on_domain_selected(self, sender, app_data,
                            user_data=None) -> None:
        path = app_data.get("file_path_name", "")
        if path and os.path.isfile(path):
            self._domain_path = path
            dpg.set_value(
                self._domain_label, Path(path).name
            )
            dpg.configure_item(
                self._domain_label, color=(220, 240, 250, 255)
            )

    def _on_browse_problem(self, sender=None, value=None,
                           user_data=None) -> None:
        dpg.add_file_dialog(
            callback=self._on_problem_selected,
            width=600, height=400,
        )

    def _on_problem_selected(self, sender, app_data,
                             user_data=None) -> None:
        path = app_data.get("file_path_name", "")
        if path and os.path.isfile(path):
            self._problem_path = path
            dpg.set_value(
                self._problem_label, Path(path).name
            )
            dpg.configure_item(
                self._problem_label, color=(220, 240, 250, 255)
            )

    def _on_prune_toggled(self, sender, value, user_data=None) -> None:
        self._prune = value

    def _on_run(self, sender=None, value=None, user_data=None) -> None:
        if self._running:
            return

        if not self._verifier_binary:
            dpg.set_value(
                self._status_text,
                "No verifier binary -- build with -DUNMANNED_BUILD_AME=ON",
            )
            dpg.configure_item(
                self._status_text, color=(210, 55, 55, 255)
            )
            return

        if not self._domain_path or not self._problem_path:
            dpg.set_value(
                self._status_text,
                "Select both domain and template problem files",
            )
            dpg.configure_item(
                self._status_text, color=(210, 55, 55, 255)
            )
            return

        self._running = True
        self._result_json = None
        self._error_msg = ""
        dpg.set_value(self._status_text, "Running...")
        dpg.configure_item(
            self._status_text, color=(210, 170, 30, 255)
        )
        dpg.configure_item(self._run_button, enabled=False)

        thread = threading.Thread(
            target=self._run_verifier, daemon=True
        )
        thread.start()

    def _run_verifier(self) -> None:
        import tempfile

        json_path = tempfile.mktemp(suffix=".json")
        cmd = [
            self._verifier_binary,
            self._domain_path,
            self._problem_path,
            "--json", json_path,
        ]
        if not self._prune:
            cmd.append("--no-prune")

        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=600,
            )

            if os.path.isfile(json_path):
                with open(json_path, "r") as f:
                    self._result_json = json.load(f)
                os.unlink(json_path)
            else:
                self._error_msg = result.stderr or result.stdout or "No output"

        except subprocess.TimeoutExpired:
            self._error_msg = "Verification timed out (10 min limit)"
        except Exception as e:
            self._error_msg = str(e)

        self._running = False

    def update(self) -> None:
        if self._running:
            return

        if self._result_json is not None:
            self._display_results(self._result_json)
            self._result_json = None
            dpg.configure_item(self._run_button, enabled=True)

        if self._error_msg:
            dpg.set_value(self._status_text, self._error_msg)
            dpg.configure_item(
                self._status_text, color=(210, 55, 55, 255)
            )
            self._error_msg = ""
            dpg.configure_item(self._run_button, enabled=True)

    def _display_results(self, data: dict) -> None:
        safe = data.get("safe_count", 0)
        gap = data.get("gap_count", 0)
        total = data.get("total_combinations", 0)
        solver_calls = data.get("solver_calls", 0)
        pruned = data.get("pruned", 0)
        time_ms = data.get("total_time_ms", 0)

        # Status
        if gap == 0:
            dpg.set_value(
                self._status_text,
                f"ALL SAFE ({safe}/{total}) -- {solver_calls} solver calls, "
                f"{pruned} pruned, {time_ms:.1f} ms",
            )
            dpg.configure_item(
                self._status_text, color=(52, 199, 105, 255)
            )
        else:
            dpg.set_value(
                self._status_text,
                f"{gap} GAPS found ({safe}/{total} safe) -- "
                f"{solver_calls} solver calls, {time_ms:.1f} ms",
            )
            dpg.configure_item(
                self._status_text, color=(210, 55, 55, 255)
            )

        # Summary cards
        children = dpg.get_item_children(self._summary_group, 1)
        if children:
            for c in children:
                dpg.delete_item(c)

        with dpg.group(horizontal=True, parent=self._summary_group):
            safe_color = (52, 199, 105, 255) if gap == 0 else (210, 170, 30, 255)
            gap_color = (52, 199, 105, 255) if gap == 0 else (210, 55, 55, 255)

            dpg.add_text(f"SAFE: {safe}/{total}", color=safe_color)
            dpg.add_text("  |  ", color=(55, 55, 68))
            dpg.add_text(f"GAPS: {gap}", color=gap_color)
            dpg.add_text("  |  ", color=(55, 55, 68))
            dpg.add_text(
                f"Solver calls: {solver_calls}/{total}",
                color=(100, 130, 150),
            )
            dpg.add_text("  |  ", color=(55, 55, 68))
            dpg.add_text(
                f"Pruned: {pruned}", color=(100, 130, 150)
            )
            dpg.add_text("  |  ", color=(55, 55, 68))
            dpg.add_text(
                f"Time: {time_ms:.1f} ms", color=(100, 130, 150)
            )

        # Context predicates
        health_vars = data.get("health_variables", [])
        if health_vars:
            with dpg.group(
                horizontal=True, parent=self._summary_group
            ):
                dpg.add_text(
                    "Context predicates: ", color=(120, 120, 130)
                )
                for hv in health_vars:
                    dpg.add_text(
                        hv["name"], color=(0, 180, 210, 230)
                    )
                    dpg.add_text("  ", color=(55, 55, 68))

        # Results table
        children = dpg.get_item_children(self._results_table, 1)
        if children:
            for c in children:
                dpg.delete_item(c)

        combos = data.get("combinations", [])
        if not combos:
            return

        with dpg.table(
            parent=self._results_table,
            header_row=True,
            borders_innerH=True,
            borders_innerV=True,
            borders_outerH=True,
            borders_outerV=True,
            resizable=True,
            scrollY=True,
        ):
            # Columns: one per health var + result + steps + actions
            for hv in health_vars:
                dpg.add_table_column(label=hv["name"], width_fixed=True, init_width_or_weight=50)
            dpg.add_table_column(label="Result", width_fixed=True, init_width_or_weight=110)
            dpg.add_table_column(label="Steps", width_fixed=True, init_width_or_weight=50)
            dpg.add_table_column(label="Time(ms)", width_fixed=True, init_width_or_weight=70)
            dpg.add_table_column(label="Recovery Actions")

            for combo in combos:
                with dpg.table_row():
                    flags = combo.get("flags", {})
                    result = combo.get("result", "")
                    is_safe = "SAFE" in result
                    is_implied = "implied" in result

                    # Health var flags
                    for hv in health_vars:
                        val = flags.get(hv["name"], False)
                        if val:
                            dpg.add_text(
                                "ON",
                                color=(52, 199, 105, 255),
                            )
                        else:
                            dpg.add_text(
                                "off",
                                color=(210, 55, 55, 200),
                            )

                    # Result
                    if is_safe:
                        color = (52, 199, 105, 255) if not is_implied else (90, 160, 110, 200)
                    else:
                        color = (210, 55, 55, 255) if not is_implied else (180, 80, 80, 200)
                    dpg.add_text(result, color=color)

                    # Steps
                    pl = combo.get("plan_length", 0)
                    dpg.add_text(
                        str(pl) if is_safe else "-",
                        color=(180, 180, 200) if is_safe else (100, 100, 110),
                    )

                    # Time
                    t = combo.get("solve_time_ms", 0)
                    dpg.add_text(
                        f"{t:.1f}" if t > 0 else "-",
                        color=(100, 130, 150),
                    )

                    # Actions
                    actions = combo.get("actions", "")
                    dpg.add_text(
                        actions,
                        color=(180, 180, 200) if is_safe and not is_implied else (100, 100, 110),
                    )
