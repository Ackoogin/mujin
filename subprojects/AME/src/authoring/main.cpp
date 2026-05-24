#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <SDL.h>

#if defined(_WIN32)
#include <SDL_main.h>
#endif

#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

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

// Prints a JSON result object to stdout and flushes — agents read this.
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
  // Disable imgui.ini writes during self-test to keep the filesystem clean
  if (selfTestMode) {
    io.IniFilename = nullptr;
  }

  ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
  ImGui_ImplOpenGL3_Init("#version 130");

  // ---------------------------------------------------------------------------
  // Self-test: render a fixed number of frames, capture, report, exit
  // ---------------------------------------------------------------------------
  if (selfTestMode) {
    const int kFrames = 3;
    bool show_demo = true;
    for (int frame = 0; frame < kFrames; ++frame) {
      // Pump events so ImGui state is consistent
      SDL_Event event;
      while (SDL_PollEvent(&event) != 0) {
        ImGui_ImplSDL2_ProcessEvent(&event);
      }

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplSDL2_NewFrame();
      ImGui::NewFrame();

      ImGui::ShowDemoWindow(&show_demo);
      ImGui::Begin("AME Authoring Tool");
      ImGui::TextUnformatted("Self-test frame");
      ImGui::End();

      ImGui::Render();

      int dw = 0, dh = 0;
      SDL_GL_GetDrawableSize(window, &dw, &dh);
      glViewport(0, 0, dw, dh);
      glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      // Capture on the last frame (before SwapWindow — reads back buffer)
      if (frame == kFrames - 1) {
        bool ok = captureScreenshot(window, selfTestPath.c_str());
        int dw2 = 0, dh2 = 0;
        SDL_GL_GetDrawableSize(window, &dw2, &dh2);
        if (ok) {
          printSelfTestResult("ok", selfTestPath.c_str(), dw2, dh2, "");
        } else {
          printSelfTestResult("error", selfTestPath.c_str(), dw2, dh2,
                              "stbi_write_png failed");
        }
      }

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

  // ---------------------------------------------------------------------------
  // Normal interactive loop
  // ---------------------------------------------------------------------------
  bool running = true;
  bool show_demo_window = true;
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

    ImGui::ShowDemoWindow(&show_demo_window);
    ImGui::Begin("AME Authoring Tool");
    ImGui::End();

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
