"""Shared DearPyGui plot helpers.

Axes that use these helpers must be created with ``no_tick_labels=True`` so
ImPlot's auto-generated decimal labels are suppressed.  Custom labels set via
``set_axis_ticks`` are still rendered even when ``no_tick_labels`` is set.
"""
from __future__ import annotations

import dearpygui.dearpygui as dpg


def set_integer_y_axis(axis_id: int, ys: list[float]) -> None:
    """Set integer-only tick labels on a ``no_tick_labels=True`` Y axis."""
    if not ys:
        return
    v_min = int(min(ys))
    v_max = int(max(ys))
    dpg.set_axis_limits(axis_id, v_min - 0.5, v_max + 0.5)
    ticks = tuple((str(v), float(v)) for v in range(v_min, v_max + 1))
    dpg.set_axis_ticks(axis_id, ticks)
