"""Dear PyGui theme definitions for the AME Dev Environment."""

import dearpygui.dearpygui as dpg

# ── Holo-Cyan palette ── dark industrial / cyberpunk dashboard ──────────
#
# Key principle: true-black base, bright cyan accents, panels distinguished
# by subtle background lift — no visible borders.

_BLACK       = (2,   4,   6, 255)     # deepest black (viewport bg)
_BG_WINDOW   = (6,  10,  14, 255)     # main window fill
_BG_PANEL    = (10,  16,  22, 255)    # child panels — just visible lift
_BG_CARD     = (14,  22,  30, 255)    # raised cards / popups / frame bg
_SURFACE     = (18,  30,  40, 255)    # buttons, headers at rest
_SURFACE_HI  = (24,  42,  56, 255)    # hovered surfaces
_SURFACE_ACT = (30,  54,  72, 255)    # active / pressed surfaces

_CYAN        = (0,  230, 255, 255)    # primary accent — vivid
_CYAN_MED    = (0,  180, 210, 230)    # mid accent (active tabs, slider)
_CYAN_DIM    = (0,  120, 150, 200)    # subtle accent (hover tints)
_CYAN_GHOST  = (0,   80, 110, 100)    # ghost accent (resize grips, etc)

_TEXT        = (220, 240, 250, 255)   # primary text — near white
_TEXT_DIM    = (100, 130, 150, 255)   # secondary / disabled
_TEXT_BRIGHT = (255, 255, 255, 255)   # pure white for emphasis

# Font settings
_FONT_SIZE = 15


def setup_font() -> int:
    """Register Consolas (or fallback) and return the font id."""
    with dpg.font_registry():
        font = dpg.add_font("C:/Windows/Fonts/consola.ttf", _FONT_SIZE)
    return font


def create_global_theme() -> int:
    """Create and return the global Holo-Cyan flat theme."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            # ── Backgrounds ──
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg,     _BG_WINDOW)
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg,      _BG_PANEL)
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg,      _BG_CARD)
            dpg.add_theme_color(dpg.mvThemeCol_MenuBarBg,     _BLACK)

            # ── Title bar ──
            dpg.add_theme_color(dpg.mvThemeCol_TitleBg,          _BLACK)
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive,    (4,  8, 12, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgCollapsed,  _BLACK)

            # ── Tabs — active tab gets cyan underline feel ──
            dpg.add_theme_color(dpg.mvThemeCol_Tab,                (10,  18,  26, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered,         _CYAN_DIM)
            dpg.add_theme_color(dpg.mvThemeCol_TabActive,          (0,   60,  80, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocused,       _BG_WINDOW)
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocusedActive, (8,  16,  24, 255))

            # ── Frames / inputs — recessed dark wells ──
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg,        (4,   8,  14, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (10,  20,  30, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive,  (14,  28,  42, 255))

            # ── Buttons ──
            dpg.add_theme_color(dpg.mvThemeCol_Button,        _SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  _SURFACE_HI)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,   _SURFACE_ACT)

            # ── Headers (collapsing, tree, table headers) ──
            dpg.add_theme_color(dpg.mvThemeCol_Header,        (12,  24,  34, 255))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered,  _SURFACE_HI)
            dpg.add_theme_color(dpg.mvThemeCol_HeaderActive,   _SURFACE_ACT)

            # ── Table ──
            dpg.add_theme_color(dpg.mvThemeCol_TableHeaderBg,     (8,  16,  24, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderStrong, (0,  80, 110,  80))
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderLight,  (0,  60,  85,  60))
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBg,        (0,    0,   0,   0))
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBgAlt,     (8,  16,  24, 100))

            # ── Scrollbar ──
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg,          (2,   4,   6, 150))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab,        (20,  50,  65, 180))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabHovered, (30,  80, 100, 220))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabActive,  _CYAN_DIM)

            # ── Borders — fully transparent (flat shading only) ──
            dpg.add_theme_color(dpg.mvThemeCol_Border,       (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_BorderShadow, (0, 0, 0, 0))

            # ── Misc chrome ──
            dpg.add_theme_color(dpg.mvThemeCol_Separator,        (0, 100, 130, 160))
            dpg.add_theme_color(dpg.mvThemeCol_ResizeGrip,        _CYAN_GHOST)
            dpg.add_theme_color(dpg.mvThemeCol_ResizeGripHovered, _CYAN_DIM)
            dpg.add_theme_color(dpg.mvThemeCol_ResizeGripActive,  _CYAN_MED)

            # ── Text ──
            dpg.add_theme_color(dpg.mvThemeCol_Text,          _TEXT)
            dpg.add_theme_color(dpg.mvThemeCol_TextDisabled,   _TEXT_DIM)
            dpg.add_theme_color(dpg.mvThemeCol_TextSelectedBg, (0, 140, 180, 100))

            # ── Interactive accents — bright cyan ──
            dpg.add_theme_color(dpg.mvThemeCol_CheckMark,        _CYAN)
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab,       _CYAN_MED)
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, _CYAN)
            dpg.add_theme_color(dpg.mvThemeCol_NavHighlight,     _CYAN_MED)

            # ── Plot ──
            dpg.add_theme_color(dpg.mvThemeCol_PlotLines,         _CYAN)
            dpg.add_theme_color(dpg.mvThemeCol_PlotLinesHovered,  _TEXT_BRIGHT)
            dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram,     _CYAN_MED)

            # ── Style: flat, zero rounding ──
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,    0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding,   0)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding,    0)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding,      0)
            dpg.add_theme_style(dpg.mvStyleVar_GrabRounding,     0)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 0)
            dpg.add_theme_style(dpg.mvStyleVar_PopupRounding,    0)

            # ── Spacing ──
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing,   8, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding,  8, 5)
            dpg.add_theme_style(dpg.mvStyleVar_CellPadding,   6, 3)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 6, 6)

            # ── Borders OFF ──
            dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize,  0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowBorderSize, 0)
            dpg.add_theme_style(dpg.mvStyleVar_ChildBorderSize,  0)
            dpg.add_theme_style(dpg.mvStyleVar_PopupBorderSize,  0)
            dpg.add_theme_style(dpg.mvStyleVar_TabBorderSize,    0)

            # ── Scrollbar size ──
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarSize, 10)

    return theme


def create_primary_window_theme() -> int:
    """Zero-padding theme for the primary window so it fills the viewport."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 0, 0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowBorderSize, 0)
    return theme


def create_status_theme(r: int, g: int, b: int, a: int = 255) -> int:
    """Create a theme that sets text colour for status indicators."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Text, (r, g, b, a))
    return theme


def create_accent_button_theme() -> int:
    """Create a highlighted button theme for primary actions (cyan accent)."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,       (0,  60,  80, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, _CYAN_DIM)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  _CYAN_MED)
            dpg.add_theme_color(dpg.mvThemeCol_Text,          _CYAN)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 0)
    return theme


def create_danger_button_theme() -> int:
    """Create a red button theme for destructive actions."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,       (80,  10,  10, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (160, 30,  30, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (200, 50,  50, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 0)
    return theme
