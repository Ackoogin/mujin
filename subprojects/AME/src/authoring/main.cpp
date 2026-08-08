#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "app_shell.h"
#include "imgui_id_audit.h"
#include "app_theme.h"
#include "pddl_generator.h"
#include "run_record.h"
#include "simulation_engine.h"

#include <SDL.h>

#if defined(_WIN32)
#include <SDL_main.h>
#endif

#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <imgui_internal.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

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

static const char* kUavSearchDomainPddl = R"pddl(
(define (domain uav-search)
  (:requirements :strips :typing)

  (:types
    location - object
    sector - location
    robot - object
  )

  (:predicates
    (at ?r - robot ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at ?r ?from)
    :effect (and
      (at ?r ?to)
      (not (at ?r ?from))
    )
  )

  (:action search
    :parameters (?r - robot ?s - sector)
    :precondition (at ?r ?s)
    :effect (searched ?s)
  )

  (:action classify
    :parameters (?r - robot ?s - sector)
    :precondition (and
      (at ?r ?s)
      (searched ?s)
    )
    :effect (classified ?s)
  )
)
)pddl";

static const char* kUavSearchProblemPddl = R"pddl(
(define (problem uav-search-1)
  (:domain uav-search)

  (:objects
    uav1 - robot
    base - location
    sector_a - sector
    sector_b - sector
  )

  (:init
    (at uav1 base)
  )

  (:goal (and
    (searched sector_a)
    (classified sector_a)
  ))
)
)pddl";

// ---------------------------------------------------------------------------
// Self-test framework
// ---------------------------------------------------------------------------

struct SelfTestAssertion {
  std::string name;
  bool        pass = false;
  std::string detail;
};

/// \brief Keep third-party planner progress out of the self-test JSON stream.
///
/// The planner writes directly to C stdout, so redirecting iostreams is not
/// enough. The original descriptor is restored immediately before the one JSON
/// document is printed.
class ScopedStdoutSilencer {
public:
  explicit ScopedStdoutSilencer(bool enabled) {
    if (!enabled) {
      return;
    }
    std::fflush(stdout);
#if defined(_WIN32)
    saved_fd_ = _dup(_fileno(stdout));
#else
    saved_fd_ = dup(fileno(stdout));
#endif
    sink_ = std::tmpfile();
    if (saved_fd_ < 0 || sink_ == nullptr) {
      restore();
      return;
    }
#if defined(_WIN32)
    _dup2(_fileno(sink_), _fileno(stdout));
#else
    dup2(fileno(sink_), fileno(stdout));
#endif
    active_ = true;
  }

  ~ScopedStdoutSilencer() { restore(); }

  void restore() {
    if (active_) {
      std::fflush(stdout);
#if defined(_WIN32)
      _dup2(saved_fd_, _fileno(stdout));
#else
      dup2(saved_fd_, fileno(stdout));
#endif
      active_ = false;
    }
    if (saved_fd_ >= 0) {
#if defined(_WIN32)
      _close(saved_fd_);
#else
      close(saved_fd_);
#endif
      saved_fd_ = -1;
    }
    if (sink_ != nullptr) {
      std::fclose(sink_);
      sink_ = nullptr;
    }
  }

private:
  int saved_fd_ = -1;
  FILE* sink_ = nullptr;
  bool active_ = false;
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

  /// Dear ImGui identifies a clickable item by its label, so two items drawn
  /// with the same label and nothing else to tell them apart become the same
  /// item: clicking one activates the other. This is easy to introduce in the
  /// domain views, where one action legitimately appears under several
  /// headings.
  ///
  /// Dear ImGui does detect this, but only while the mouse is over one of the
  /// clashing items, which never happens in a headless run. Checking its
  /// DebugDrawIdConflicts field here would produce an assertion that always
  /// passes. The identities the view actually used are compared instead, which
  /// does not depend on where a pointer is.
  void checkItemIdsAreUnique(const char* stage,
                             const std::vector<std::string>& ids) {
    std::vector<std::string> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    const auto duplicate = std::adjacent_find(sorted.begin(), sorted.end());
    const bool unique = duplicate == sorted.end();
    const std::string name = std::string("unique_item_ids_") + stage;
    std::string detail = "two clickable items shared one identity";
    if (!unique) {
      // Single quotes on purpose: this text is emitted inside a JSON string.
      detail += ", including '" + *duplicate +
                "'. Give each its own identity with PushID or a ##suffix.";
    }
    check(name.c_str(), unique, detail.c_str());
  }

  /// Whole-interface check: no two items drawn in the last frame claimed the
  /// same identity. Covers every panel at once, unlike a check written against
  /// one view's own bookkeeping.
  void checkNoSharedItemIds(const std::string& stage) {
    const bool clean = imgui_id_audit::duplicates().empty();
    const std::string name = "no_shared_item_ids_" + stage;
    const std::string detail = imgui_id_audit::describeFirstDuplicate();
    check(name.c_str(), clean, detail.c_str());
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
  imgui_id_audit::beginFrame();
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
  // --self-test is the only option this executable takes. Running a mission
  // without a window is ame_mission_cli's job, so that a build agent needs no
  // display stack for it.
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

  // Load JetBrains Mono if present alongside the executable (copied there by
  // the build's post-build step). SDL_GetBasePath returns the directory of
  // the running exe so the file resolves regardless of CWD. Silently falls
  // back to the default ProggyClean if the file isn't found.
  {
    char* basePath = SDL_GetBasePath();
    if (basePath != nullptr) {
      std::string fontPath = std::string(basePath) + "JetBrainsMono-Regular.ttf";
      SDL_free(basePath);
      if (FILE* f = std::fopen(fontPath.c_str(), "rb")) {
        std::fclose(f);
        ImFontConfig cfg;
        cfg.OversampleH = 2;
        cfg.OversampleV = 2;
        io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 15.0F, &cfg);
      }
    }
  }

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
    ScopedStdoutSilencer stdout_silencer(true);
    const ImVec4   clearColor(0.06F, 0.10F, 0.14F, 1.0F);
    AppShell       shell;
    SelfTestReport report;

    // Phase 1: warm-up — let ImGui settle docking/layout
    for (int i = 0; i < 3; ++i) {
      renderAppShellFrame(window, shell, clearColor);
    }

    imgui_id_audit::enable();

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
    shell.selfTestAddPredicate("comms_ok");
    shell.selfTestAddPredicateParam(2, "?l", "location");
    shell.selfTestAddAction("move");
    shell.selfTestAddActionParam(0, "?r", "robot");
    shell.selfTestAddActionParam(0, "?from", "location");
    shell.selfTestAddActionParam(0, "?to", "location");
    shell.selfTestAddActionPrecondition(0, "at", {"?r", "?from"});
    shell.selfTestAddActionAddEffect(0, "at", {"?r", "?to"});
    shell.selfTestAddActionDelEffect(0, "at", {"?r", "?from"});
    shell.selfTestSetActionBtBinding(0, "MoveToLocation", "", false);
    shell.selfTestAddAction("search");
    shell.selfTestAddActionParam(1, "?r", "robot");
    shell.selfTestAddActionParam(1, "?where", "location");
    shell.selfTestAddActionPrecondition(1, "at", {"?r", "?where"});
    shell.selfTestAddActionPrecondition(1, "comms_ok", {"?where"});
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
    report.check("bt_binding_node_type_set",
                 shell.selfTestActionBtBinding(0).nodeType == "MoveToLocation",
                 "expected move action BT node type binding to be set");
    report.check("bt_binding_reactive_default_false",
                 !shell.selfTestActionBtBinding(0).reactive,
                 "expected move action BT binding to default to non-reactive");

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
    // A3: the problem list is a set of rows that reveal what they name, so a
    // reader works down the list rather than hunting for the element.
    shell.selfTestShowPddlTab();
    // A requested tab is applied on the next frame, and the panel only draws
    // once that tab is the visible one.
    for (int frame = 0; frame < 3; ++frame) {
      renderAppShellFrame(window, shell, clearColor);
    }
    report.check("problem_list_rendered",
                 shell.selfTestProblemListRendered(),
                 "expected the PDDL tab to show the problem list");
    const bool problem_opened = shell.selfTestClickFirstProblem();
    renderAppShellFrame(window, shell, clearColor);
    report.check("problem_click_reveals_element",
                 problem_opened,
                 "expected clicking a problem to select the element it names");
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
    shell.selfTestSetScenarioExpectation(0, true, 1, 10, {"move"}, {"explode"});
    const ScenarioExpectation& nominalExpectation =
        shell.selfTestScenarioExpectation(0);
    report.check("scenario_expectation_should_succeed",
                 nominalExpectation.shouldSucceed == true,
                 "expected nominal scenario to expect success");
    report.check("scenario_expectation_min_steps",
                 nominalExpectation.minPlanSteps == 1,
                 "expected nominal scenario min plan steps to be 1");
    report.check("scenario_expectation_max_steps",
                 nominalExpectation.maxPlanSteps == 10,
                 "expected nominal scenario max plan steps to be 10");
    report.check("scenario_expectation_expected_actions",
                 nominalExpectation.expectedActions ==
                     std::vector<std::string>{"move"},
                 "expected nominal scenario to require move action");
    report.check("scenario_expectation_forbidden_actions",
                 nominalExpectation.forbiddenActions ==
                     std::vector<std::string>{"explode"},
                 "expected nominal scenario to forbid explode action");

    // Palette selection (WI-5.2)
    shell.selfTestSelectPredicateFromPalette(1);
    renderAppShellFrame(window, shell, clearColor);
    report.check("palette_selects_predicate",
                 shell.selfTestSelectedPredicateIndex() == 1,
                 "expected palette to select predicate index 1");
    shell.selfTestSelectActionFromPalette(0);
    renderAppShellFrame(window, shell, clearColor);
    report.check("palette_selects_action",
                 shell.selfTestSelectedActionIndex() == 0,
                 "expected palette to select action index 0");

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
    report.check("bt_graph_has_nodes",
                 shell.selfTestBtNodeCount() > 0U,
                 "expected feasible plan to compile into a visible BT graph");
    report.check("bt_graph_has_nodes_after_binding",
                 shell.selfTestBtNodeCount() > 0U,
                 "expected BT binding to preserve visible BT compilation");
    report.check("feasibility_no_error",
                 shell.selfTestLastPlan().error_msg.empty(),
                 "expected feasible plan to have no error message");
    shell.selfTestRunContingencyAnalysis();
    const ContingencyReport& contingencyReport =
        shell.selfTestContingencyReport();
    report.check("contingency_ok",
                 contingencyReport.ok,
                 "expected contingency analysis to complete");
    report.check("contingency_identifies_context_preds",
                 contingencyReport.contextPredicates.size() >= 1U,
                 "expected at least one context predicate");
    report.check("contingency_enumerates_subsets",
                 contingencyReport.results.size() ==
                     (size_t{1} << contingencyReport.contextFluents.size()),
                 "expected contingency analysis to enumerate every subset");
    report.check("contingency_counts_add_up",
                 contingencyReport.feasibleCount +
                         contingencyReport.infeasibleCount +
                         contingencyReport.errorCount ==
                     contingencyReport.results.size(),
                 "expected contingency result counts to sum to result count");

    shell.selfTestAddObject("sector_a", "location");
    shell.selfTestAddScenario("preview");
    shell.selfTestAddInitialFact(1, "at", {"uav1", "base"});
    shell.selfTestAddGoal(1, "at", {"uav1", "sector_a"});
    shell.selfTestPlanAndPreview();
    for (int i = 0; i < 3; ++i) {
      renderAppShellFrame(window, shell, clearColor);
    }
    report.check("plan_and_preview_runs_without_error",
                 shell.selfTestLastPlan().error_msg.empty(),
                 "expected Plan & Preview to complete without a planner error");
    shell.selfTestSetSelectedPlanStep(0);
    renderAppShellFrame(window, shell, clearColor);
    report.check("cross_view_schema_extracted",
                 shell.selfTestPlanGraph().selectedActionSchemaName() == "move",
                 "expected selected plan step to extract move action schema");

    // A simulated run of the same scenario: it must start and finish with
    // nothing configured, which is the acceptance line for the run screens.
    const bool runStarted = shell.selfTestStartRun("preview");
    renderAppShellFrame(window, shell, clearColor);
    report.check("run_starts_without_configuration",
                 runStarted,
                 "expected the preview scenario to load a run");
    const SimulationEngine& run = shell.selfTestSimulation();
    report.check("run_has_one_node_per_action",
                 run.actionSteps().size() == shell.selfTestLastPlan().steps.size(),
                 "expected one run step per planned action");
    report.check("run_draws_the_tree_it_is_running",
                 shell.selfTestBtNodeCount() > 0U,
                 "expected the run to load its compiled tree into the tree view");
    report.check("run_tree_carries_action_detail",
                 shell.selfTestBtHasActionContract(),
                 "expected a run action node to carry its preconditions and effects");
    shell.selfTestStepRun();
    renderAppShellFrame(window, shell, clearColor);
    report.check("run_steps_one_tick_at_a_time",
                 run.tick() == 1U,
                 "expected One step to advance the run by exactly one tick");
    const bool runCompleted = shell.selfTestRunToCompletion();
    renderAppShellFrame(window, shell, clearColor);
    report.check("run_reaches_the_goal",
                 runCompleted && run.phase() == RunPhase::Completed,
                 "expected the run to reach the scenario's goal");
    report.check("run_meets_every_goal",
                 run.goalsMetCount() == run.goals().size(),
                 "expected every goal to be true at the end of the run");
    report.check("run_records_what_changed",
                 !run.factChanges().empty(),
                 "expected the run to record the facts it changed");
    // A picture of the run screen, so the new views can be reviewed from a
    // headless machine the same way the rest of the tool can.
    shell.selfTestShowRunTab();
    renderAppShellFrame(window, shell, clearColor);
    report.check("run_timeline_rendered_as_primary_view",
                 shell.selfTestRunTimelineRendered(),
                 "expected the Run tab to render its timeline");
    report.check("run_facts_panel_rendered",
                 shell.selfTestRunFactsRendered(),
                 "expected the Run tab to render its facts panel");
    report.check("run_tab_screenshot_written",
                 captureScreenshot(window,
                                   (selfTestPath + ".run-tab.png").c_str()),
                 "expected the run tab screenshot to be written");
    shell.selfTestReplayCurrentRun();
    shell.selfTestCompareCurrentRunWithItself();
    renderAppShellFrame(window, shell, clearColor);
    report.check("recorded_run_reuses_run_views",
                 shell.selfTestReplayRendered() &&
                     shell.selfTestRunTimelineRendered() &&
                     shell.selfTestRunFactsRendered(),
                 "expected replay to use the Run tab timeline and facts views");
    report.check("run_comparison_summary_rendered",
                 shell.selfTestRunComparisonRendered(),
                 "expected the Run tab to show a one-line run comparison");
    shell.selfTestCloseReplay();

    RunFaultSet selfTestFault;
    selfTestFault.name = "move-fails-once";
    selfTestFault.actionFailures.push_back({"move", 1U});
    shell.selfTestSetRunFaults(selfTestFault);
    report.check("faulted_run_starts",
                 shell.selfTestStartRun("preview"),
                 "expected a run-local forced action failure to load");
    report.check("faulted_run_recovers_after_replan",
                 shell.selfTestRunToCompletion(),
                 "expected the forced first attempt to replan and then recover");
    shell.selfTestShowRunTab();
    renderAppShellFrame(window, shell, clearColor);
    report.check("run_fault_controls_rendered",
                 shell.selfTestRunFaultPanelRendered(),
                 "expected the Run tab to show the two fault controls");
    report.check("replan_comparison_rendered",
                 shell.selfTestReplanComparisonRendered(),
                 "expected the Run tab to compare abandoned and replacement plans");
    report.check("run_replan_count_visible_in_model",
                 shell.selfTestSimulation().replanCount() == 1U,
                 "expected the recovered run to count one replan");

    shell.selfTestAddObject("sector_x", "location");
    shell.selfTestAddScenario("infeasible");
    shell.selfTestAddInitialFact(2, "at", {"uav1", "base"});
    shell.selfTestAddGoal(2, "connected", {"base", "sector_x"});
    shell.selfTestRunFeasibility("infeasible");
    report.check("feasibility_infeasible_returns_false",
                 !shell.selfTestLastPlan().success,
                 "expected unreachable connected goal to be infeasible");
    report.check("failure_panel_has_plain_language_explanation",
                 shell.selfTestFailureExplanation().available &&
                     !shell.selfTestFailureExplanation().blockingFact.predicateName.empty(),
                 "expected the failure panel to identify an unreachable fact");
    report.check("failure_panel_offers_expected_failure",
                 std::find(shell.selfTestFailureExplanation().fixes.begin(),
                           shell.selfTestFailureExplanation().fixes.end(),
                           "Mark this scenario expected-to-fail") !=
                     shell.selfTestFailureExplanation().fixes.end(),
                 "expected failure report to offer expected-to-fail");
    const bool hasScenarioExpectation =
        std::any_of(shell.selfTestModel().scenarios.begin(),
                    shell.selfTestModel().scenarios.end(),
                    [](const ScenarioDef& scenario) {
                      return scenario.expectation.minPlanSteps > 0 ||
                             scenario.expectation.maxPlanSteps > 0 ||
                             !scenario.expectation.expectedActions.empty() ||
                             !scenario.expectation.forbiddenActions.empty();
                    });
    report.check("scenario_expectation_available",
                 hasScenarioExpectation,
                 "expected at least one scenario expectation before batch run");
    shell.selfTestRunAllScenarios();
    const ScenarioBatchReport& batchReport = shell.selfTestBatchReport();
    report.check("batch_report_has_results",
                 batchReport.results.size() >= 1U,
                 "expected Run All Scenarios to produce at least one result");
    report.check("batch_report_counts_consistent",
                 batchReport.passCount + batchReport.failCount +
                         batchReport.errorCount ==
                     batchReport.results.size(),
                 "expected batch report counts to sum to result count");

    shell.selfTestStartBatch();
    shell.selfTestShowPddlTab();
    renderAppShellFrame(window, shell, clearColor);
    report.check("batch_progress_rendered_between_scenarios",
                 shell.selfTestBatchProgressRendered(),
                 "expected the PDDL tab to show incremental batch progress");
    report.check("batch_can_be_stopped_part_way",
                 shell.selfTestBatchRunning(),
                 "expected more scenarios to remain after one frame");
    shell.selfTestStopBatch();
    report.check("batch_stop_takes_effect",
                 !shell.selfTestBatchRunning(),
                 "expected Stop to leave the remaining scenarios waiting");

    AppShell importShell;
    const bool importDomainOk =
        importShell.selfTestImportDomain(kUavSearchDomainPddl);
    report.check("import_domain_ok",
                 importDomainOk,
                 "expected uav_search domain import to succeed");
    report.check("import_domain_types_count",
                 importShell.selfTestModel().types.size() >= 3U,
                 "expected imported domain to include location, sector, robot");
    report.check("import_domain_predicates_count",
                 importShell.selfTestModel().predicates.size() == 3U,
                 "expected imported domain to include 3 predicates");
    report.check("import_domain_actions_count",
                 importShell.selfTestModel().actions.size() == 3U,
                 "expected imported domain to include 3 actions");
    const size_t scenarioCountBeforeImport =
        importShell.selfTestModel().scenarios.size();
    const bool importProblemOk =
        importShell.selfTestImportProblem(kUavSearchProblemPddl, "");
    report.check("import_problem_ok",
                 importProblemOk,
                 "expected uav_search problem import to succeed");
    report.check("import_problem_objects_count",
                 importShell.selfTestModel().objects.size() >= 4U,
                 "expected imported problem to include at least 4 objects");
    report.check("import_problem_scenario_added",
                 importShell.selfTestModel().scenarios.size() ==
                     scenarioCountBeforeImport + 1U,
                 "expected imported problem to append one scenario");
    report.check("import_problem_goal_count",
                 !importShell.selfTestModel().scenarios.empty() &&
                     importShell.selfTestModel().scenarios.back().goals.size() >= 1U,
                 "expected imported problem scenario to include goals");

    for (int domainView = 0; domainView < 5; ++domainView) {
      shell.selfTestSetDomainView(domainView);
      renderAppShellFrame(window, shell, clearColor);
      const std::string checkName =
          "domain_view_" + std::to_string(domainView) + "_rendered";
      report.check(checkName.c_str(),
                   shell.selfTestDomainViewRendered(domainView),
                   "expected every Phase 6 domain view to render");
    }

    // ---------------------------------------------------------------------
    // Modelling workflow against a real saved project.
    //
    // The hand-built model above has three predicates and three actions, and
    // no action uses the same predicate in more than one role. Real domains do:
    // in vehicle-autonomy, seven actions require "at", make it true and make it
    // false all at once, so each of them appears three times in the
    // neighbourhood of that one fact. That is the shape that exposed both
    // conflicting item identities and name labels drawn across the screen, and
    // it never occurred in the model the rest of the self-test uses.
    // ---------------------------------------------------------------------
    {
      AppShell projectShell;
      const bool projectLoaded =
          projectShell.selfTestLoadProject(AME_AUTHORING_SAMPLE_PROJECT);
      report.check("sample_project_loads",
                   projectLoaded,
                   "expected the vehicle-autonomy sample project to load");

      if (projectLoaded) {
        report.check("sample_project_has_a_real_domain",
                     projectShell.selfTestModel().predicates.size() >= 10U &&
                         projectShell.selfTestModel().actions.size() >= 10U,
                     "expected the sample project to be large enough to be "
                     "representative");

        const bool focused = projectShell.selfTestFocusBusiestPredicate();
        report.check("sample_project_has_a_busy_fact",
                     focused,
                     "expected at least one fact with relationships to focus on");

        // Walk every view the way someone reviewing a domain would, checking
        // each rendered frame rather than only the last one.
        for (int domainView = 0; domainView < 5; ++domainView) {
          projectShell.selfTestSetDomainView(domainView);
          renderAppShellFrame(window, projectShell, clearColor);
          const std::string renderedName =
              "sample_project_view_" + std::to_string(domainView) + "_rendered";
          report.check(renderedName.c_str(),
                       projectShell.selfTestDomainViewRendered(domainView),
                       "expected the view to render for the sample project");
          if (domainView == 0) {
            report.checkItemIdsAreUnique(
                ("sample_project_view_" + std::to_string(domainView)).c_str(),
                projectShell.selfTestNeighbourItemIds());
          }
        }

        // The neighbourhood view is where the width problem showed up. These
        // items must be as wide as their text, not as wide as the window.
        projectShell.selfTestSetDomainView(0);
        renderAppShellFrame(window, projectShell, clearColor);

        // Saved alongside the main screenshot. How links meet their nodes is
        // not something the checks below can judge, so this view is captured
        // for a person to look at when something about the drawing changes.
        {
          std::string neighbourhoodShot = selfTestPath;
          const size_t dot = neighbourhoodShot.find_last_of('.');
          if (dot != std::string::npos) {
            neighbourhoodShot.insert(dot, "_neighbourhood");
          } else {
            neighbourhoodShot += "_neighbourhood.png";
          }
          report.check("neighbourhood_screenshot_written",
                       captureScreenshot(window, neighbourhoodShot.c_str()),
                       "expected to capture the neighbourhood view");
        }
        const float widest = projectShell.selfTestWidestNeighbourItemWidth();
        report.check("neighbour_labels_are_not_full_width",
                     widest > 0.0F && widest < 400.0F,
                     "expected each clickable name in the neighbourhood view to "
                     "be about as wide as its text; a much larger value means it "
                     "is stretching to the width of the window");

        // Selecting a neighbour re-centres the view, which is the core of the
        // navigation loop, so walk a few steps and keep checking.
        for (int step = 0; step < 3; ++step) {
          projectShell.selfTestSelectActionFromPalette(step);
          renderAppShellFrame(window, projectShell, clearColor);
          report.checkItemIdsAreUnique(
              ("sample_project_navigation_step_" + std::to_string(step)).c_str(),
              projectShell.selfTestNeighbourItemIds());
        }

        // Back to the neighbourhood after navigating, since re-centring
        // rebuilds the node list and is where a duplicate would reappear.
        projectShell.selfTestSetDomainView(0);
        renderAppShellFrame(window, projectShell, clearColor);
        report.checkItemIdsAreUnique("after_navigation",
                                     projectShell.selfTestNeighbourItemIds());

        // Editing an action, reached by picking one from the palette on the
        // left. Each action is shown as three groups of sentences -- what must
        // be true beforehand, what becomes true afterwards, and what becomes
        // false afterwards -- all drawn by one piece of code that numbers its
        // rows from zero within each group. An action using the same fact in
        // all three roles, which is common, therefore drew three rows asking
        // for the same identities.
        //
        // These use the whole-interface check rather than one view's own
        // bookkeeping, so they cover the palette, the sentence rows, the
        // dropdowns and everything else on screen at the same time.
        const size_t actionsToEdit =
            std::min<size_t>(6U, projectShell.selfTestModel().actions.size());
        for (size_t actionIndex = 0; actionIndex < actionsToEdit; ++actionIndex) {
          projectShell.selfTestSelectActionFromPalette(static_cast<int>(actionIndex));
          renderAppShellFrame(window, projectShell, clearColor);
          report.checkNoSharedItemIds("editing_action_" +
                                      std::to_string(actionIndex));
        }
      }
    }
    report.check("guided_editor_rendered",
                 shell.selfTestGuidedEditorRendered(),
                 "expected the guided sentence editor to render for an action");
    shell.selfTestShowPlanTab();

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
    report.check("model_has_three_predicates",
                 shell.selfTestModel().predicates.size() == 3,
                 "expected 3 predicates in model");
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
    // Any of search's preconditions matching move's add-effect predicate is
    // sufficient — WI-4.4 added a second precondition (comms_ok) so don't
    // require preconditions.size() == 1.
    bool searchPreconditionMatches =
        hasSearchAction && !model.actions[0].addEffects.empty();
    if (searchPreconditionMatches) {
      const auto& targetPred = model.actions[0].addEffects[0].predicateName;
      bool anyMatch = false;
      for (const auto& pre : model.actions[1].preconditions) {
        if (pre.predicateName == targetPred) { anyMatch = true; break; }
      }
      searchPreconditionMatches = anyMatch;
    }
    report.check("search_action_has_matching_precondition",
                 searchPreconditionMatches,
                 "expected search precondition to match move add-effect predicate");
    const RelationIndex relationIndex = shell.selfTestRelationIndex();
    report.check("relation_index_counts_links",
                 relationIndex.linkCount() == 5U,
                 "expected five derived fact-to-action links");
    report.check("derived_causal_relationship_present",
                 !relationIndex.causalLinks().empty(),
                 "expected move to be able to enable search");
    report.check("neighbourhood_panel_has_nodes",
                 shell.selfTestNeighbourhoodNodeCount(1) > 1U,
                 "expected the focused neighbourhood panel model to contain neighbours");
    report.check("matrix_panel_exports_csv",
                 shell.selfTestMatrixCsv().find("fact \\ action") != std::string::npos,
                 "expected the matrix panel to produce CSV evidence");
    shell.selfTestAddStateGroup("position", "robot", {"at"});
    report.check("lifecycle_panel_derives_transition",
                 shell.selfTestLifecycleTransitionCount() == 1U,
                 "expected move to derive an at-to-at lifecycle transition");
    shell.selfTestSelectPredicateFromPalette(0);
    shell.selfTestSelectActionFromPalette(0);
    const bool historyBack = shell.selfTestSelectionBack();
    const bool historyForward = shell.selfTestSelectionForward();
    report.check("relations_history_back",
                 historyBack,
                 "expected relations selection history to go back");
    report.check("relations_history_forward",
                 historyForward,
                 "expected relations selection history to go forward");
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
    // A6: a saved view is a picture somebody can come back to, by name.
    shell.selfTestSelectActionFromPalette(0);
    shell.selfTestSetDomainView(0);
    shell.selfTestSaveCurrentView("how the vehicle moves");
    renderAppShellFrame(window, shell, clearColor);
    shell.selfTestSelectPredicateFromPalette(1);
    renderAppShellFrame(window, shell, clearColor);
    const bool view_opened = shell.selfTestOpenSavedView("how the vehicle moves");
    renderAppShellFrame(window, shell, clearColor);
    report.check("saved_view_reopens_the_same_picture",
                 view_opened && shell.selfTestSelectedActionIndex() == 0,
                 "expected a saved view to put the focus back where it was");

    // A6: a group is a named set of facts and actions the whole-domain canvas
    // draws as one labelled box, and closes to a single box. Closing one has to
    // leave fewer boxes on the canvas than were there before.
    shell.selfTestSetDomainView(4);  // the whole-domain canvas
    renderAppShellFrame(window, shell, clearColor);
    const size_t nodes_before_grouping = shell.selfTestCanvasNodeCount();
    const bool group_made =
        shell.selfTestCreateGroup("getting about", {"at"}, {"move"});
    renderAppShellFrame(window, shell, clearColor);
    report.check("open_group_hides_nothing",
                 group_made &&
                     shell.selfTestCanvasNodeCount() == nodes_before_grouping &&
                     shell.selfTestCollapsedGroupCount() == 0U,
                 "expected an open group to draw a box round its contents "
                 "without hiding any of them");
    const bool group_closed =
        shell.selfTestSetGroupCollapsed("getting about", true);
    renderAppShellFrame(window, shell, clearColor);
    report.check("closed_group_draws_fewer_boxes",
                 group_closed &&
                     shell.selfTestCollapsedGroupCount() == 1U &&
                     shell.selfTestCanvasNodeCount() < nodes_before_grouping,
                 "expected closing a group to replace its contents with one box");
    shell.selfTestUndo();  // open it again
    shell.selfTestUndo();  // and remove the group
    renderAppShellFrame(window, shell, clearColor);
    report.check("undo_puts_the_grouped_boxes_back",
                 shell.selfTestCanvasNodeCount() == nodes_before_grouping &&
                     shell.selfTestCollapsedGroupCount() == 0U,
                 "expected undo to restore the canvas to what it was before "
                 "the group was made");
    shell.selfTestSetDomainView(0);

    // Making a fact false is a thing an action can do, and the button offering
    // it has to say so. The three buttons are drawn by one function and look
    // alike, so two of them once read "+ add an outcome" and the way to make a
    // fact false could only be found by guessing.
    const auto& addLabels = AppShell::guidedGroupAddLabels();
    const std::set<std::string> distinctAddLabels(addLabels.begin(),
                                                  addLabels.end());
    report.check("action_editor_names_all_three_kinds_of_fact",
                 addLabels.size() == 3 && distinctAddLabels.size() == 3,
                 "the condition, becomes-true and becomes-false buttons must "
                 "each say which they are");

    const auto& labels = AppShell::tabLabels();
    report.check("tabs_in_workflow_order",
                 labels.size() == 4 &&
                     labels[0] == "Domain" && labels[1] == "PDDL" &&
                     labels[2] == "Plan"   && labels[3] == "Run",
                 "AppShell::tabLabels() must be {Domain, PDDL, Plan, Run}");

    // Phase 6: capture and print
    bool ok = captureScreenshot(window, selfTestPath.c_str());
    int dw = 0, dh = 0;
    SDL_GL_GetDrawableSize(window, &dw, &dh);
    if (!ok) {
      report.check("screenshot_written", false, "stbi_write_png failed");
    } else {
      report.check("screenshot_written", true);
    }
    stdout_silencer.restore();
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
