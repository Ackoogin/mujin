"""Dear PyGui theme definitions for the AME Dev Environment."""

import dearpygui.dearpygui as dpg

# High-contrast flat dark + green accent palette
_BG0        = (8,   10,   9,  255)   # deepest background (near-black)
_BG1        = (13,  16,  14,  255)   # window bg
_BG2        = (19,  24,  20,  255)   # child / panel bg
_BG3        = (27,  34,  29,  255)   # popup / frame bg
_SURFACE    = (38,  48,  41,  255)   # raised surfaces (buttons, headers)
_BORDER     = (52,  68,  56,  255)   # separators, table borders
_MUTED      = (75,  98,  80,  255)   # disabled / placeholder text

_GREEN      = (58, 220, 112,  255)   # primary accent (bright)
_GREEN_DIM  = (40, 170,  84,  255)   # accent pressed / active
_GREEN_FADE = (30,  95,  55, 190)    # accent hover (translucent)
_GREEN_SOFT = (36, 130,  65,  255)   # subtle accent (tabs active, titles)

_TEXT       = (228, 245, 232,  255)  # primary text (high contrast)
_TEXT_DIM   = (105, 138, 112,  255)  # secondary text


def create_global_theme() -> int:
    """Create and return the global high-contrast flat theme with green accents."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            # --- Backgrounds ---
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg,     _BG1)
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg,      _BG2)
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg,      _BG3)

            # --- Title bar ---
            dpg.add_theme_color(dpg.mvThemeCol_TitleBg,          _BG0)
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive,     _GREEN_SOFT)
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgCollapsed,  _BG0)

            # --- Tabs ---
            dpg.add_theme_color(dpg.mvThemeCol_Tab,                _BG2)
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered,         _GREEN_FADE)
            dpg.add_theme_color(dpg.mvThemeCol_TabActive,          _GREEN_SOFT)
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocused,       _BG1)
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocusedActive, _BG3)

            # --- Frames / inputs ---
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg,        _BG3)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, _SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive,  _SURFACE)

            # --- Buttons ---
            dpg.add_theme_color(dpg.mvThemeCol_Button,        _SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  _GREEN_FADE)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,   _GREEN_DIM)

            # --- Headers ---
            dpg.add_theme_color(dpg.mvThemeCol_Header,        _SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered,  _GREEN_FADE)
            dpg.add_theme_color(dpg.mvThemeCol_HeaderActive,   _GREEN_SOFT)

            # --- Table ---
            dpg.add_theme_color(dpg.mvThemeCol_TableHeaderBg,     _BG3)
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderStrong,  _BORDER)
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderLight,   _BG3)
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBg,         _BG1)
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBgAlt,      _BG2)

            # --- Scrollbar ---
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg,          _BG0)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab,        _SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabHovered, _GREEN_FADE)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabActive,  _GREEN_DIM)

            # --- Misc chrome ---
            dpg.add_theme_color(dpg.mvThemeCol_Separator,        _BORDER)
            dpg.add_theme_color(dpg.mvThemeCol_ResizeGrip,        _SURFACE)
            dpg.add_theme_color(dpg.mvThemeCol_ResizeGripHovered,  _GREEN_FADE)
            dpg.add_theme_color(dpg.mvThemeCol_ResizeGripActive,   _GREEN_DIM)

            # --- Text ---
            dpg.add_theme_color(dpg.mvThemeCol_Text,         _TEXT)
            dpg.add_theme_color(dpg.mvThemeCol_TextDisabled,  _TEXT_DIM)

            # --- Interactive accents ---
            dpg.add_theme_color(dpg.mvThemeCol_CheckMark,        _GREEN)
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab,       _GREEN_SOFT)
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive,  _GREEN)

            # --- Plot lines ---
            dpg.add_theme_color(dpg.mvThemeCol_PlotLines,         _GREEN)
            dpg.add_theme_color(dpg.mvThemeCol_PlotLinesHovered,  _GREEN_DIM)
            dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram,     _GREEN_SOFT)

            # --- Flat style (zero rounding) ---
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,     0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding,    0)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding,     0)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding,       0)
            dpg.add_theme_style(dpg.mvStyleVar_GrabRounding,      0)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding,  0)
            dpg.add_theme_style(dpg.mvStyleVar_PopupRounding,     0)

            # --- Spacing ---
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing,  8, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 8, 4)
            dpg.add_theme_style(dpg.mvStyleVar_CellPadding,  6, 3)

            # --- Thin 1px borders ---
            dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize,  1)
            dpg.add_theme_style(dpg.mvStyleVar_WindowBorderSize, 1)
            dpg.add_theme_style(dpg.mvStyleVar_ChildBorderSize,  1)
            dpg.add_theme_style(dpg.mvStyleVar_PopupBorderSize,  1)

    return theme


def create_status_theme(r: int, g: int, b: int, a: int = 255) -> int:
    """Create a theme that sets text colour for status indicators."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Text, (r, g, b, a))
    return theme


def create_accent_button_theme() -> int:
    """Create a highlighted button theme for primary actions (green accent)."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        _GREEN_SOFT)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  _GREEN)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,   _GREEN_DIM)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 0)
    return theme


def create_danger_button_theme() -> int:
    """Create a red button theme for destructive actions."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (100, 22, 22, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  (155, 35, 35, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,   (80,  15, 15, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 0)
    return theme
