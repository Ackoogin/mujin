"""PDDL Editor tab — file browser, text editor, syntax validation."""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING

import dearpygui.dearpygui as dpg

from ..config import PDDL_KEYWORDS

if TYPE_CHECKING:
    from .app import App


class PddlEditorTab:
    """Builds and manages the PDDL Editor tab."""

    def __init__(self, app: App):
        self._app = app
        self._current_file: str = ""
        self._modified = False
        self._files: list[str] = []

        # Widget IDs
        self._file_list: int = 0
        self._editor: int = 0
        self._file_label: int = 0
        self._status_text: int = 0
        self._validation_text: int = 0

    def build(self, parent: int) -> None:
        with dpg.group(parent=parent):
            with dpg.group(horizontal=True):
                # --- Left: File browser ---
                with dpg.child_window(width=280, height=-5):
                    dpg.add_text("PDDL Files", color=(180, 180, 200))
                    dpg.add_separator()
                    dpg.add_spacer(height=4)

                    with dpg.group(horizontal=True):
                        dpg.add_button(
                            label="Browse...",
                            callback=self._on_browse, width=90,
                        )
                        dpg.add_button(
                            label="Refresh",
                            callback=self._on_refresh_files, width=70,
                        )

                    dpg.add_spacer(height=6)
                    self._file_list = dpg.add_child_window(
                        height=-5, border=True
                    )

                    # Auto-scan domains/ directory
                    self._scan_domains_dir()

                # --- Right: Editor ---
                with dpg.child_window(width=-1, height=-5):
                    # Toolbar
                    with dpg.group(horizontal=True):
                        self._file_label = dpg.add_text(
                            "No file loaded", color=(180, 180, 200)
                        )
                        dpg.add_spacer(width=20)
                        dpg.add_button(
                            label="Save", callback=self._on_save, width=60,
                        )
                        dpg.add_button(
                            label="Validate",
                            callback=self._on_validate, width=80,
                        )
                        dpg.add_button(
                            label="Load to WM",
                            callback=self._on_load_to_wm, width=90,
                        )
                        dpg.add_spacer(width=20)
                        self._status_text = dpg.add_text(
                            "", color=(120, 120, 130)
                        )

                    dpg.add_spacer(height=4)

                    # Editor area
                    self._editor = dpg.add_input_text(
                        multiline=True,
                        width=-1, height=-80,
                        tab_input=True,
                        callback=self._on_editor_changed,
                    )

                    dpg.add_spacer(height=4)

                    # Validation output
                    dpg.add_text("Validation:", color=(180, 180, 200))
                    self._validation_text = dpg.add_input_text(
                        multiline=True, readonly=True,
                        width=-1, height=55,
                    )

    def _scan_domains_dir(self) -> None:
        """Scan the domains/ directory for .pddl files."""
        domains_dir = Path(self._app.config.paths.domains_dir)
        if not domains_dir.exists():
            # Try relative to cwd
            domains_dir = Path.cwd() / "domains"
        if not domains_dir.exists():
            return

        self._files = []
        for root, dirs, files in os.walk(domains_dir):
            for f in sorted(files):
                if f.endswith(".pddl"):
                    self._files.append(str(Path(root) / f))

        self._refresh_file_list()

    def _refresh_file_list(self) -> None:
        children = dpg.get_item_children(self._file_list, 1)
        if children:
            for child in children:
                dpg.delete_item(child)

        for filepath in self._files:
            name = Path(filepath).name
            parent_dir = Path(filepath).parent.name
            display = f"{parent_dir}/{name}"
            dpg.add_button(
                label=display,
                callback=lambda s, a, u: self._load_file(u),
                user_data=filepath,
                width=-1,
                parent=self._file_list,
            )

    def _load_file(self, filepath: str) -> None:
        """Load a PDDL file into the editor."""
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                content = f.read()
            self._current_file = filepath
            dpg.set_value(self._editor, content)
            dpg.set_value(
                self._file_label,
                f"File: {Path(filepath).name}"
            )
            dpg.set_value(self._validation_text, "")
            dpg.set_value(self._status_text, "")
            self._modified = False
        except Exception as e:
            dpg.set_value(self._status_text, f"Error: {e}")

    # -- Callbacks -----------------------------------------------------------

    def _on_browse(self, sender=None, value=None, user_data=None) -> None:
        dpg.add_file_dialog(
            directory_selector=True,
            callback=self._on_directory_selected,
            width=600, height=400,
        )

    def _on_directory_selected(self, sender, app_data,
                                user_data=None) -> None:
        path = app_data.get("file_path_name", "")
        if not path:
            return

        self._files = []
        for root, dirs, files in os.walk(path):
            for f in sorted(files):
                if f.endswith(".pddl"):
                    self._files.append(str(Path(root) / f))

        self._refresh_file_list()
        dpg.set_value(
            self._status_text,
            f"Found {len(self._files)} PDDL files"
        )

    def _on_refresh_files(self, sender=None, value=None,
                           user_data=None) -> None:
        self._scan_domains_dir()

    def _on_editor_changed(self, sender, value, user_data=None) -> None:
        self._modified = True
        dpg.set_value(self._status_text, "Modified")

    def _on_save(self, sender=None, value=None, user_data=None) -> None:
        if not self._current_file:
            dpg.set_value(self._status_text, "No file to save")
            return

        content = dpg.get_value(self._editor)
        try:
            with open(self._current_file, "w", encoding="utf-8") as f:
                f.write(content)
            self._modified = False
            dpg.set_value(self._status_text, "Saved")
        except Exception as e:
            dpg.set_value(self._status_text, f"Save error: {e}")

    def _on_validate(self, sender=None, value=None, user_data=None) -> None:
        """Basic PDDL syntax validation."""
        content = dpg.get_value(self._editor)
        errors = self._validate_pddl(content)

        if not errors:
            dpg.set_value(self._validation_text, "Syntax OK — no errors found.")
            dpg.set_value(self._status_text, "Valid")
        else:
            dpg.set_value(self._validation_text, "\n".join(errors))
            dpg.set_value(self._status_text, f"{len(errors)} issue(s)")

    def _on_load_to_wm(self, sender=None, value=None, user_data=None) -> None:
        """Load parsed PDDL into the running world model."""
        if not self._current_file:
            dpg.set_value(self._status_text, "No file loaded")
            return
        
        # Check if client is connected
        client = self._app.client
        if not client.connected:
            dpg.set_value(self._status_text, "Not connected to backend")
            return
        
        # Determine if this is a domain or problem file
        current_path = Path(self._current_file)
        parent_dir = current_path.parent
        
        # Look for matching domain/problem pair
        domain_path = ""
        problem_path = ""
        
        if "domain" in current_path.name.lower():
            domain_path = str(current_path)
            # Look for problem file in same directory
            for f in parent_dir.glob("*.pddl"):
                if "problem" in f.name.lower():
                    problem_path = str(f)
                    break
        elif "problem" in current_path.name.lower():
            problem_path = str(current_path)
            # Look for domain file in same directory
            for f in parent_dir.glob("*.pddl"):
                if "domain" in f.name.lower():
                    domain_path = str(f)
                    break
        
        if not domain_path or not problem_path:
            dpg.set_value(self._status_text, "Need both domain.pddl and problem.pddl in same folder")
            return
        
        # Reload via client
        if hasattr(client, 'reload_domain'):
            success = client.reload_domain(domain_path, problem_path)
            if success:
                dpg.set_value(self._status_text, f"Loaded: {parent_dir.name}")
            else:
                dpg.set_value(self._status_text, "Failed to load PDDL")
        else:
            dpg.set_value(self._status_text, "Backend does not support reload")

    @staticmethod
    def _validate_pddl(content: str) -> list[str]:
        """Basic structural validation of PDDL content."""
        errors = []

        # Check balanced parentheses
        depth = 0
        for i, ch in enumerate(content):
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
            if depth < 0:
                line = content[:i].count("\n") + 1
                errors.append(f"Line {line}: Unmatched closing parenthesis")
                break
        if depth > 0:
            errors.append(
                f"Unmatched opening parenthesis ({depth} unclosed)"
            )

        # Check for (define ...)
        if "(define" not in content:
            errors.append("Missing (define ...) — not a valid PDDL file")

        # Check for domain or problem declaration
        has_domain = "(domain" in content
        has_problem = "(problem" in content
        if not has_domain and not has_problem:
            errors.append(
                "Missing (domain ...) or (problem ...) declaration"
            )

        return errors
