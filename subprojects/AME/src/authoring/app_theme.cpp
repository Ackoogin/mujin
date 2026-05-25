// Holo-Cyan theme — ported from subprojects/AME/tools/devenv/ui/theme.py.
// Palette: true-black base, bright cyan accents, flat shading, zero rounding.
#include "app_theme.h"

#include <imgui.h>

static ImVec4 rgb(int r, int g, int b, int a = 255) {
  return ImVec4(r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f);
}

void applyHoloCyanTheme() {
  ImGuiStyle& s   = ImGui::GetStyle();
  ImVec4*     col = s.Colors;

  // -- Backgrounds -------------------------------------------------------
  col[ImGuiCol_WindowBg]          = rgb(  6,  10,  14);
  col[ImGuiCol_ChildBg]           = rgb( 10,  16,  22);
  col[ImGuiCol_PopupBg]           = rgb( 14,  22,  30);
  col[ImGuiCol_MenuBarBg]         = rgb(  2,   4,   6);

  // -- Title bar ---------------------------------------------------------
  col[ImGuiCol_TitleBg]           = rgb(  2,   4,   6);
  col[ImGuiCol_TitleBgActive]     = rgb(  4,   8,  12);
  col[ImGuiCol_TitleBgCollapsed]  = rgb(  2,   4,   6);

  // -- Tabs (ImGui 1.90.7+ names) ----------------------------------------
  col[ImGuiCol_Tab]                       = rgb(  4,   8,  12);
  col[ImGuiCol_TabHovered]                = rgb(  0, 140, 170, 220);
  col[ImGuiCol_TabSelected]               = rgb(  0,  90, 120);   // brighter "lit up" fill
  col[ImGuiCol_TabSelectedOverline]       = rgb(  0, 230, 255, 255);
  col[ImGuiCol_TabDimmed]                 = rgb(  4,   8,  12);
  col[ImGuiCol_TabDimmedSelected]         = rgb(  6,  18,  26);
  col[ImGuiCol_TabDimmedSelectedOverline] = rgb(  0, 140, 170,  90);

  // -- Frames / inputs ---------------------------------------------------
  col[ImGuiCol_FrameBg]           = rgb(  4,   8,  14);
  col[ImGuiCol_FrameBgHovered]    = rgb( 10,  20,  30);
  col[ImGuiCol_FrameBgActive]     = rgb( 14,  28,  42);

  // -- Buttons -----------------------------------------------------------
  col[ImGuiCol_Button]            = rgb( 18,  30,  40);
  col[ImGuiCol_ButtonHovered]     = rgb( 24,  42,  56);
  col[ImGuiCol_ButtonActive]      = rgb( 30,  54,  72);

  // -- Headers (collapsing, tree, table) ---------------------------------
  col[ImGuiCol_Header]            = rgb( 12,  24,  34);
  col[ImGuiCol_HeaderHovered]     = rgb( 24,  42,  56);
  col[ImGuiCol_HeaderActive]      = rgb( 30,  54,  72);

  // -- Table -------------------------------------------------------------
  col[ImGuiCol_TableHeaderBg]     = rgb(  8,  16,  24);
  col[ImGuiCol_TableBorderStrong] = rgb(  0,  80, 110,  80);
  col[ImGuiCol_TableBorderLight]  = rgb(  0,  60,  85,  60);
  col[ImGuiCol_TableRowBg]        = rgb(  0,   0,   0,   0);
  col[ImGuiCol_TableRowBgAlt]     = rgb(  8,  16,  24, 100);

  // -- Scrollbar ---------------------------------------------------------
  col[ImGuiCol_ScrollbarBg]           = rgb(  2,   4,   6, 150);
  col[ImGuiCol_ScrollbarGrab]         = rgb( 20,  50,  65, 180);
  col[ImGuiCol_ScrollbarGrabHovered]  = rgb( 30,  80, 100, 220);
  col[ImGuiCol_ScrollbarGrabActive]   = rgb(  0, 120, 150, 200);

  // -- Borders: crisp cyan outline (HUD chrome) -------------------------
  col[ImGuiCol_Border]            = rgb(  0, 140, 180, 200);
  col[ImGuiCol_BorderShadow]      = rgb(  0,  40,  60, 120);

  // -- Misc chrome -------------------------------------------------------
  col[ImGuiCol_Separator]         = rgb(  0, 180, 220, 200);
  col[ImGuiCol_ResizeGrip]        = rgb(  0,  80, 110, 100);
  col[ImGuiCol_ResizeGripHovered] = rgb(  0, 120, 150, 200);
  col[ImGuiCol_ResizeGripActive]  = rgb(  0, 180, 210, 230);

  // -- Text --------------------------------------------------------------
  col[ImGuiCol_Text]              = rgb(220, 240, 250);
  col[ImGuiCol_TextDisabled]      = rgb(100, 130, 150);
  col[ImGuiCol_TextSelectedBg]    = rgb(  0, 140, 180, 100);

  // -- Interactive accents: bright cyan ----------------------------------
  col[ImGuiCol_CheckMark]         = rgb(  0, 230, 255);
  col[ImGuiCol_SliderGrab]        = rgb(  0, 180, 210, 230);
  col[ImGuiCol_SliderGrabActive]  = rgb(  0, 230, 255);
  col[ImGuiCol_NavCursor]         = rgb(  0, 180, 210, 230); // was NavHighlight

  // -- Docking -----------------------------------------------------------
  col[ImGuiCol_DockingPreview]    = rgb(  0, 180, 210, 180);
  col[ImGuiCol_DockingEmptyBg]    = rgb(  2,   4,   6);

  // -- Plot --------------------------------------------------------------
  col[ImGuiCol_PlotLines]         = rgb(  0, 230, 255);
  col[ImGuiCol_PlotLinesHovered]  = rgb(255, 255, 255);
  col[ImGuiCol_PlotHistogram]     = rgb(  0, 180, 210, 230);
  col[ImGuiCol_PlotHistogramHovered] = rgb(  0, 230, 255);

  // -- Style: flat, zero rounding ----------------------------------------
  s.FrameRounding     = 0.0f;
  s.WindowRounding    = 0.0f;
  s.ChildRounding     = 0.0f;
  s.TabRounding       = 0.0f;
  s.GrabRounding      = 0.0f;
  s.ScrollbarRounding = 0.0f;
  s.PopupRounding     = 0.0f;

  // -- Spacing -----------------------------------------------------------
  s.ItemSpacing   = ImVec2(8.0f, 6.0f);
  s.FramePadding  = ImVec2(8.0f, 5.0f);
  s.CellPadding   = ImVec2(6.0f, 3.0f);
  s.WindowPadding = ImVec2(6.0f, 6.0f);

  // -- Borders ON (HUD framing) -----------------------------------------
  s.FrameBorderSize   = 1.0f;
  s.WindowBorderSize  = 1.0f;
  s.ChildBorderSize   = 1.0f;
  s.PopupBorderSize   = 1.0f;
  s.TabBorderSize     = 1.0f;
  s.TabBarBorderSize  = 2.0f;
  s.TabBarOverlineSize = 2.0f;

  // -- Scrollbar ---------------------------------------------------------
  s.ScrollbarSize = 10.0f;
}
