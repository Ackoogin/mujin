#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "app_shell.h"
#include "app_theme.h"
#include "pddl_generator.h"

#include <SDL.h>

#if defined(_WIN32)
#include <SDL_main.h>
#endif

#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <imgui_internal.h>

#include <algorithm>
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
    shell.selfTestAddPredicateParam(0, "?r", "robot");
    shell.selfTestAddPredicateParam(0, "?l", "location");
    shell.selfTestAddPredicate("connected");
    shell.selfTestAddPredicateParam(1, "?from", "location");
    shell.selfTestAddPredicateParam(1, "?to", "location");
    shell.selfTestAddAction("move");
    shell.selfTestAddActionParam(0, "?r", "robot");
    shell.selfTestAddActionParam(0, "?from", "location");
    shell.selfTestAddActionParam(0, "?to", "location");
    shell.selfTestAddActionPrecondition(0, "at", {"?r", "?from"});
    shell.selfTestAddActionAddEffect(0, "at", {"?r", "?to"});
    shell.selfTestAddActionDelEffect(0, "at", {"?r", "?from"});
    shell.selfTestAddAction("search");
    shell.selfTestAddActionParam(1, "?r", "robot");
    shell.selfTestAddActionParam(1, "?where", "location");
    shell.selfTestAddActionPrecondition(1, "at", {"?r", "?where"});
    const bool addCausalLinkOk = shell.selfTestAddCausalLink(0, 0, 1, 0);
    const bool rejectSelfLinkOk = !shell.selfTestAddCausalLink(0, 0, 0, 0);
    const std::string pddl = PddlGenerator::generateDomain(shell.selfTestModel());
    shell.selfTestValidate();
    report.check("validation_ok_for_valid_model",
                 shell.selfTestValidation().ok,
                 "expected valid generated PDDL to parse");
    const GroundingReport& initialGrounding =
        shell.selfTestValidation().grounding;
    report.check("grounding_valid",
                 initialGrounding.valid,
                 "expected grounding report after valid parse");
    report.check("grounding_has_predicate_stats",
                 initialGrounding.predicateStats.size() ==
                     shell.selfTestModel().predicates.size(),
                 "expected one grounding stat per predicate");
    report.check("grounding_has_action_stats",
                 initialGrounding.actionStats.size() ==
                     shell.selfTestModel().actions.size(),
                 "expected one grounding stat per action schema");
    const bool predicateCountsZeroOrPositive =
        std::all_of(initialGrounding.predicateStats.begin(),
                    initialGrounding.predicateStats.end(),
                    [](const GroundingStat& stat) {
                      return static_cast<long long>(stat.count) >= 0LL;
                    });
    report.check("grounding_predicate_counts_zero_or_positive",
                 predicateCountsZeroOrPositive,
                 "expected predicate grounding counts to be zero or positive");

    shell.selfTestRemoveAllObjects();
    shell.selfTestValidate();
    const bool zeroGroundActionsWarns =
        std::any_of(shell.selfTestValidation().grounding.warnings.begin(),
                    shell.selfTestValidation().grounding.warnings.end(),
                    [](const std::string& warning) {
                      return warning.find("no ground actions") != std::string::npos;
                    });
    report.check("grounding_zero_ground_actions_warns",
                 zeroGroundActionsWarns,
                 "expected warning for action schemas with no ground actions");
    shell.selfTestUndo();
    shell.selfTestValidate();

    shell.selfTestCorruptPredicateName(0);
    shell.selfTestValidate();
    report.check("validation_failed_after_corruption",
                 !shell.selfTestValidation().ok,
                 "expected empty predicate name to fail validation");
    report.check("validation_has_at_least_one_error",
                 !shell.selfTestValidation().errors.empty(),
                 "expected validation failure to include at least one error");
    shell.selfTestUndo();
    shell.selfTestValidate();
    report.check("validation_ok_after_undo",
                 shell.selfTestValidation().ok,
                 "expected validation to pass after undoing corruption");

    shell.selfTestAddObject("uav1", "robot");
    shell.selfTestAddObject("base", "location");
    shell.selfTestAddScenario("nominal");
    shell.selfTestAddInitialFact(0, "at", {"uav1", "base"});
    shell.selfTestAddGoal(0, "at", {"uav1", "base"});
    shell.selfTestRunFeasibility("nominal");
    report.check("feasibility_plan_returned",
                 shell.selfTestLastPlan().success,
                 "expected trivially satisfied goal to be feasible");
    report.check("plan_graph_populated",
                 static_cast<long long>(shell.selfTestPlanGraphStepCount()) >= 0LL,
                 "expected feasible plan to populate the plan graph");
    report.check("plan_graph_consistent_with_plan",
                 shell.selfTestPlanGraphStepCount() ==
                     shell.selfTestLastPlan().steps.size(),
                 "expected plan graph step count to match returned plan");
    report.check("feasibility_no_error",
                 shell.selfTestLastPlan().error_msg.empty(),
                 "expected feasible plan to have no error message");

    shell.selfTestAddObject("sector_x", "location");
    shell.selfTestAddScenario("infeasible");
    shell.selfTestAddInitialFact(1, "at", {"uav1", "base"});
    shell.selfTestAddGoal(1, "connected", {"base", "sector_x"});
    shell.selfTestRunFeasibility("infeasible");
    report.check("feasibility_infeasible_returns_false",
                 !shell.selfTestLastPlan().success,
                 "expected unreachable connected goal to be infeasible");

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
    const ProjectModel& model = shell.selfTestModel();
    const bool hasMoveAction = (model.actions.size() >= 1);
    const bool hasSearchAction = (model.actions.size() >= 2);
    report.check("model_has_two_actions",
                 model.actions.size() == 2,
                 "expected 2 actions in model");
    report.check("action_name_is_move",
                 hasMoveAction && model.actions[0].name == "move",
                 "expected action name to be move");
    report.check("action_has_three_params",
                 hasMoveAction && model.actions[0].params.size() == 3,
                 "expected move action to have 3 params");
    report.check("action_has_one_precondition",
                 hasMoveAction && model.actions[0].preconditions.size() == 1,
                 "expected move action to have 1 precondition");
    report.check("action_has_one_add_effect",
                 hasMoveAction && model.actions[0].addEffects.size() == 1,
                 "expected move action to have 1 add-effect");
    report.check("action_has_one_del_effect",
                 hasMoveAction && model.actions[0].delEffects.size() == 1,
                 "expected move action to have 1 del-effect");
    const bool searchPreconditionMatches =
        hasSearchAction &&
        !model.actions[0].addEffects.empty() &&
        model.actions[1].preconditions.size() == 1 &&
        model.actions[1].preconditions[0].predicateName ==
            model.actions[0].addEffects[0].predicateName;
    report.check("search_action_has_matching_precondition",
                 searchPreconditionMatches,
                 "expected search precondition to match move add-effect predicate");
    report.check("causal_link_added",
                 addCausalLinkOk,
                 "expected matching causal link to be accepted");
    report.check("causal_links_size_one",
                 model.causalLinks.size() == 1,
                 "expected exactly 1 causal link");
    report.check("causal_link_endpoints",
                 model.causalLinks.size() == 1 &&
                     model.causalLinks[0].fromAction == 0 &&
                     model.causalLinks[0].toAction == 1,
                 "expected causal link from action 0 to action 1");
    report.check("self_causal_link_rejected",
                 rejectSelfLinkOk,
                 "expected self causal link to be rejected");
    report.check("pddl_contains_define_domain",
                 pddl.find("(define (domain ") != std::string::npos,
                 "expected generated PDDL to contain domain definition");
    report.check("pddl_contains_types_section",
                 pddl.find("(:types") != std::string::npos,
                 "expected generated PDDL to contain types section");
    report.check("pddl_contains_at_predicate",
                 pddl.find("(at ?r - robot") != std::string::npos,
                 "expected generated PDDL to contain typed at predicate");
    report.check("pddl_contains_move_action",
                 pddl.find("(:action move") != std::string::npos,
                 "expected generated PDDL to contain move action");

    const size_t predCountBefore = shell.selfTestModel().predicates.size();
    shell.selfTestAddPredicate("__undo_probe__");
    const bool undoProbeAdded = (shell.selfTestModel().predicates.size() == predCountBefore + 1);
    const bool undoOk = shell.selfTestUndo();
    const bool undoRestored = (shell.selfTestModel().predicates.size() == predCountBefore);
    const bool redoOk = shell.selfTestRedo();
    const bool redoRestored = (shell.selfTestModel().predicates.size() == predCountBefore + 1);
    const size_t undoDepthAfter = shell.selfTestUndoDepth();

    report.check("undo_probe_added", undoProbeAdded, "selfTestAddPredicate failed to grow predicates");
    report.check("undo_call_ok", undoOk, "selfTestUndo returned false");
    report.check("undo_restored_predicate_count", undoRestored, "predicates size mismatch after undo");
    report.check("redo_call_ok", redoOk, "selfTestRedo returned false");
    report.check("redo_restored_predicate_count", redoRestored, "predicates size mismatch after redo");
    report.check("undo_depth_positive", undoDepthAfter > 0, "undo depth should be > 0");

    // Tab labels are declared by AppShell::tabLabels() and rendered by
    // renderPanels(). The main host window must be present in the window list;
    // the per-tab content lives inside it, not as separate windows.
    report.check("host_window_present",
                 report.windowActive("##MainHost"),
                 "Main host window not active");
    report.check("status_bar_present",
                 report.windowActive("##StatusBar"),
                 "Status bar overlay not active");
    renderAppShellFrame(window, shell, clearColor);
    report.check("structural_no_errors_for_valid_model",
                 shell.selfTestStructuralReport().errorCount == 0U,
                 "expected continuous structural validation to report no errors");
    shell.selfTestCorruptPredicateName(0);
    renderAppShellFrame(window, shell, clearColor);
    report.check("structural_detects_empty_predicate_name",
                 shell.selfTestStructuralReport().errorCount >= 1U,
                 "expected empty predicate name to be a structural error");
    shell.selfTestUndo();
    renderAppShellFrame(window, shell, clearColor);
    report.check("structural_clears_after_undo",
                 shell.selfTestStructuralReport().errorCount == 0U,
                 "expected structural errors to clear after undo");
    const auto& labels = AppShell::tabLabels();
    report.check("tabs_in_workflow_order",
                 labels.size() == 4 &&
                     labels[0] == "Domain" && labels[1] == "PDDL" &&
                     labels[2] == "Plan"   && labels[3] == "BT",
                 "AppShell::tabLabels() must be {Domain, PDDL, Plan, BT}");

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
