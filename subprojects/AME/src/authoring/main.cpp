#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "app_shell.h"
#include "app_theme.h"

#include <SDL.h>

#if defined(_WIN32)
#include <SDL_main.h>
#endif

#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <imgui_internal.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Self-test helpers
// ---------------------------------------------------------------------------

static bool captureScreenshot(SDL_Window* window, const char* path) {
  int w = 0;
  int h = 0;
  SDL_GL_GetDrawableSize(window, &w, &h);

  std::vector<unsigned char> pixels(static_cast<size_t>(w) * h * 4);
  glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

  // OpenGL origin is bottom-left; flip vertically for top-down PNG
  std::vector<unsigned char> row(static_cast<size_t>(w) * 4);
  for (int y = 0; y < h / 2; ++y) {
    unsigned char* top = pixels.data() + static_cast<size_t>(y) * w * 4;
    unsigned char* bot = pixels.data() + static_cast<size_t>(h - 1 - y) * w * 4;
    std::memcpy(row.data(), top, static_cast<size_t>(w) * 4);
    std::memcpy(top, bot, static_cast<size_t>(w) * 4);
    std::memcpy(bot, row.data(), static_cast<size_t>(w) * 4);
  }

  return stbi_write_png(path, w, h, 4, pixels.data(), w * 4) != 0;
}

// ---------------------------------------------------------------------------
// Self-test framework
// ---------------------------------------------------------------------------

struct SelfTestAssertion {
  std::string name;
  bool        pass = false;
  std::string detail;
};

struct SelfTestReport {
  std::vector<SelfTestAssertion> assertions;
  std::vector<std::string>       windowsFound;
  int failures = 0;

  void check(const char* name, bool condition, const char* failDetail = "") {
    SelfTestAssertion a;
    a.name   = name;
    a.pass   = condition;
    a.detail = condition ? "" : failDetail;
    if (!condition) { ++failures; }
    assertions.push_back(std::move(a));
  }

  // Call after ImGui::Render() — enumerates active windows this frame.
  void collectWindows() {
    windowsFound.clear();
    ImGuiContext& g = *GImGui;
    for (ImGuiWindow* w : g.Windows) {
      if (w != nullptr && w->Active && w->Name != nullptr && w->Name[0] != '\0') {
        windowsFound.push_back(w->Name);
      }
    }
  }

  bool windowActive(const char* name) const {
    for (const auto& n : windowsFound) {
      if (n == name) { return true; }
    }
    return false;
  }

  void print(const char* screenshotPath, int w, int h) const {
    const char* status = (failures == 0) ? "ok" : "fail";
    std::printf("{\n");
    std::printf("  \"status\": \"%s\",\n", status);
    std::printf("  \"screenshot\": \"%s\",\n", screenshotPath);
    std::printf("  \"width\": %d,\n", w);
    std::printf("  \"height\": %d,\n", h);
    std::printf("  \"failures\": %d,\n", failures);

    // Assertions array
    std::printf("  \"assertions\": [\n");
    for (size_t i = 0; i < assertions.size(); ++i) {
      const auto& a = assertions[i];
      const char* sep = (i + 1 < assertions.size()) ? "," : "";
      if (a.detail.empty()) {
        std::printf("    { \"name\": \"%s\", \"pass\": %s }%s\n",
                    a.name.c_str(), a.pass ? "true" : "false", sep);
      } else {
        std::printf("    { \"name\": \"%s\", \"pass\": %s, \"detail\": \"%s\" }%s\n",
                    a.name.c_str(), a.pass ? "true" : "false",
                    a.detail.c_str(), sep);
      }
    }
    std::printf("  ],\n");

    // Windows found
    std::printf("  \"windows_found\": [");
    for (size_t i = 0; i < windowsFound.size(); ++i) {
      std::printf("\"%s\"%s", windowsFound[i].c_str(),
                  (i + 1 < windowsFound.size()) ? ", " : "");
    }
    std::printf("]\n}\n");
    std::fflush(stdout);
  }
};

static void injectSdlKey(SDL_Keycode key) {
  SDL_Event dn;
  SDL_memset(&dn, 0, sizeof(dn));
  dn.type             = SDL_KEYDOWN;
  dn.key.keysym.sym   = key;
  dn.key.keysym.scancode = SDL_GetScancodeFromKey(key);
  SDL_PushEvent(&dn);

  SDL_Event up = dn;
  up.type = SDL_KEYUP;
  SDL_PushEvent(&up);
}

static void injectSdlMouseClick(int x, int y) {
  SDL_Event dn;
  SDL_memset(&dn, 0, sizeof(dn));
  dn.type         = SDL_MOUSEBUTTONDOWN;
  dn.button.button = SDL_BUTTON_LEFT;
  dn.button.state  = SDL_PRESSED;
  dn.button.x      = x;
  dn.button.y      = y;
  SDL_PushEvent(&dn);

  SDL_Event up = dn;
  up.type          = SDL_MOUSEBUTTONUP;
  up.button.state  = SDL_RELEASED;
  SDL_PushEvent(&up);
}

static void renderAppShellFrame(SDL_Window* window, AppShell& shell,
                                const ImVec4& clearColor) {
  SDL_Event event;
  while (SDL_PollEvent(&event) != 0) {
    ImGui_ImplSDL2_ProcessEvent(&event);
  }
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplSDL2_NewFrame();
  ImGui::NewFrame();

  shell.renderMenuBar();
  shell.renderPanels();
  shell.renderStatusBar();

  ImGui::Render();

  int dw = 0, dh = 0;
  SDL_GL_GetDrawableSize(window, &dw, &dh);
  glViewport(0, 0, dw, dh);
  glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  SDL_GL_SwapWindow(window);
}

static void printSelfTestResult(const char* status, const char* screenshotPath,
                                int w, int h, const char* detail) {
  std::printf(
    "{\n"
    "  \"status\": \"%s\",\n"
    "  \"screenshot\": \"%s\",\n"
    "  \"width\": %d,\n"
    "  \"height\": %d,\n"
    "  \"detail\": \"%s\"\n"
    "}\n",
    status, screenshotPath, w, h, detail);
  std::fflush(stdout);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  // Parse --self-test [output.png]
  bool selfTestMode = false;
  std::string selfTestPath = "ame_authoring_self_test.png";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-test") == 0) {
      selfTestMode = true;
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        selfTestPath = argv[++i];
      }
    }
  }

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
    if (selfTestMode) {
      printSelfTestResult("error", selfTestPath.c_str(), 0, 0, "SDL_Init failed");
    }
    return 1;
  }

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

  Uint32 windowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
  if (selfTestMode) {
    windowFlags |= SDL_WINDOW_HIDDEN;
  }

  SDL_Window* window = SDL_CreateWindow(
    "AME Authoring Tool",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    1280, 720,
    windowFlags);
  if (window == nullptr) {
    if (selfTestMode) {
      printSelfTestResult("error", selfTestPath.c_str(), 0, 0, "SDL_CreateWindow failed");
    }
    SDL_Quit();
    return 1;
  }

  SDL_GLContext gl_context = SDL_GL_CreateContext(window);
  if (gl_context == nullptr) {
    if (selfTestMode) {
      printSelfTestResult("error", selfTestPath.c_str(), 0, 0, "SDL_GL_CreateContext failed");
    }
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  SDL_GL_MakeCurrent(window, gl_context);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  applyHoloCyanTheme();
  // Disable imgui.ini writes during self-test to keep the filesystem clean
  if (selfTestMode) {
    io.IniFilename = nullptr;
  } else {
    io.IniFilename = "ame_authoring_tool.ini";
  }

  ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
  ImGui_ImplOpenGL3_Init("#version 130");

  // ---------------------------------------------------------------------------
  // Self-test: drive the real AppShell, inject actions, validate UI, capture
  // ---------------------------------------------------------------------------
  if (selfTestMode) {
    const ImVec4   clearColor(0.06F, 0.10F, 0.14F, 1.0F);
    AppShell       shell;
    SelfTestReport report;

    // Phase 1: warm-up — let ImGui settle docking/layout
    for (int i = 0; i < 3; ++i) {
      renderAppShellFrame(window, shell, clearColor);
    }

    // Phase 2: inject programmatic actions through the test interface
    shell.selfTestNew();
    shell.selfTestAddType("robot", "object");
    shell.selfTestAddType("location", "object");
    shell.selfTestAddPredicate("at");
    shell.selfTestAddPredicate("connected");

    // Phase 3: inject an SDL key (Escape would quit; pick something benign)
    injectSdlKey(SDLK_F1);

    // Phase 4: render a few more frames so the new state is reflected
    for (int i = 0; i < 3; ++i) {
      renderAppShellFrame(window, shell, clearColor);
    }

    // Phase 5: collect window list after the final render and validate
    report.collectWindows();

    report.check("project_name_set",
                 shell.projectName == "[Untitled]",
                 "expected projectName to be [Untitled] after New");
    report.check("model_has_two_types",
                 shell.selfTestModel().types.size() == 2,
                 "expected 2 types in model");
    report.check("model_has_two_predicates",
                 shell.selfTestModel().predicates.size() == 2,
                 "expected 2 predicates in model");
    report.check("panel_domain_graph",
                 report.windowActive("Domain Graph"),
                 "Domain Graph panel not active");
    report.check("panel_properties",
                 report.windowActive("Properties"),
                 "Properties panel not active");
    report.check("panel_pddl_preview",
                 report.windowActive("PDDL Preview"),
                 "PDDL Preview panel not active");
    report.check("panel_validation_output",
                 report.windowActive("Validation Output"),
                 "Validation Output panel not active");
    report.check("panel_plan_view",
                 report.windowActive("Plan View"),
                 "Plan View panel not active");
    report.check("panel_bt_view",
                 report.windowActive("BT View"),
                 "BT View panel not active");
    report.check("status_bar_present",
                 report.windowActive("##StatusBar"),
                 "Status bar overlay not active");

    // Phase 6: capture and print
    bool ok = captureScreenshot(window, selfTestPath.c_str());
    int dw = 0, dh = 0;
    SDL_GL_GetDrawableSize(window, &dw, &dh);
    if (!ok) {
      report.check("screenshot_written", false, "stbi_write_png failed");
    } else {
      report.check("screenshot_written", true);
    }
    report.print(selfTestPath.c_str(), dw, dh);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return report.failures == 0 ? 0 : 2;
  }

  // ---------------------------------------------------------------------------
  // Normal interactive loop
  // ---------------------------------------------------------------------------
  AppShell shell;
  bool running = true;
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        running = false;
      }
      if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
        running = false;
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    shell.renderMenuBar();
    shell.renderPanels();
    shell.renderStatusBar();
    if (shell.wantsQuit) {
      running = false;
    }

    ImGui::Render();

    int drawable_width = 0;
    int drawable_height = 0;
    SDL_GL_GetDrawableSize(window, &drawable_width, &drawable_height);
    glViewport(0, 0, drawable_width, drawable_height);
    glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_GL_DeleteContext(gl_context);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
