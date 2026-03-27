"""Dear PyGui theme definitions for the AME Dev Environment."""

import dearpygui.dearpygui as dpg


def create_global_theme() -> int:
    """Create and return the global dark theme."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            # Window
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (30, 30, 35, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (35, 35, 42, 255))
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg, (40, 40, 48, 255))

            # Title bar
            dpg.add_theme_color(dpg.mvThemeCol_TitleBg, (25, 25, 30, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, (45, 90, 160, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgCollapsed, (20, 20, 25, 255))

            # Tabs
            dpg.add_theme_color(dpg.mvThemeCol_Tab, (50, 50, 60, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered, (65, 110, 185, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabActive, (45, 90, 160, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocused, (40, 40, 48, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocusedActive, (45, 75, 120, 255))

            # Frames / inputs
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (45, 45, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (55, 55, 68, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (60, 60, 75, 255))

            # Buttons
            dpg.add_theme_color(dpg.mvThemeCol_Button, (55, 55, 68, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (65, 110, 185, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (45, 90, 160, 255))

            # Headers (collapsing headers, tree nodes)
            dpg.add_theme_color(dpg.mvThemeCol_Header, (45, 45, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (55, 90, 140, 255))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, (45, 90, 160, 255))

            # Table
            dpg.add_theme_color(dpg.mvThemeCol_TableHeaderBg, (40, 40, 50, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderStrong, (55, 55, 68, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TableBorderLight, (45, 45, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBg, (30, 30, 35, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TableRowBgAlt, (35, 35, 42, 255))

            # Scrollbar
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg, (25, 25, 30, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab, (60, 60, 75, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabHovered, (80, 80, 100, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabActive, (100, 100, 120, 255))

            # Separator
            dpg.add_theme_color(dpg.mvThemeCol_Separator, (55, 55, 68, 255))

            # Text
            dpg.add_theme_color(dpg.mvThemeCol_Text, (220, 220, 230, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (110, 110, 120, 255))

            # Checkmark
            dpg.add_theme_color(dpg.mvThemeCol_CheckMark, (80, 200, 80, 255))

            # Slider grab
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab, (65, 110, 185, 255))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, (45, 90, 160, 255))

            # Rounding
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_GrabRounding, 3)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_PopupRounding, 4)

            # Spacing
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 8, 4)
            dpg.add_theme_style(dpg.mvStyleVar_CellPadding, 6, 3)

    return theme


def create_status_theme(r: int, g: int, b: int, a: int = 255) -> int:
    """Create a theme that sets text colour for status indicators."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Text, (r, g, b, a))
    return theme


def create_accent_button_theme() -> int:
    """Create a highlighted button theme for primary actions."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (45, 90, 160, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (55, 110, 190, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (35, 75, 140, 255))
    return theme


def create_danger_button_theme() -> int:
    """Create a red button theme for destructive actions."""
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (160, 45, 45, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (190, 55, 55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (140, 35, 35, 255))
    return theme
