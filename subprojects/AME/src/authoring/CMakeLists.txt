cmake_minimum_required(VERSION 3.20)

include(FetchContent)

find_package(OpenGL REQUIRED)

set(SDL_SHARED ON CACHE BOOL "Build SDL2 shared library" FORCE)
set(SDL_STATIC OFF CACHE BOOL "Build SDL2 static library" FORCE)
set(SDL_TEST OFF CACHE BOOL "Build SDL2 tests" FORCE)
set(SDL_TESTS OFF CACHE BOOL "Build SDL2 tests" FORCE)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui
  GIT_TAG v1.91.6-docking
  GIT_SHALLOW TRUE
)

FetchContent_Declare(
  imgui_node_editor
  GIT_REPOSITORY https://github.com/thedmd/imgui-node-editor
  GIT_TAG develop
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR cmake/unused
)

FetchContent_Declare(
  sdl2
  GIT_REPOSITORY https://github.com/libsdl-org/SDL
  GIT_TAG release-2.30.10
  GIT_SHALLOW TRUE
)

# stb — single-header libs; we only need stb_image_write.h for self-test screenshots
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb
  GIT_TAG 5736b15f7ea0ffb08dd38af21067c314d6a3aae9
  GIT_SHALLOW FALSE
)

FetchContent_MakeAvailable(imgui imgui_node_editor sdl2)
FetchContent_GetProperties(stb)
if(NOT stb_POPULATED)
  FetchContent_Populate(stb)
endif()

# nlohmann_json is already provided by subprojects/PYRAMID as an interface target
# (nlohmann_json::nlohmann_json alias) — no FetchContent needed here.

add_library(ame_authoring_project_model STATIC
    project_model.cpp
    pddl_generator.cpp
    pddl_validator.cpp
    scenario_runner.cpp
    structural_validator.cpp
)
target_include_directories(ame_authoring_project_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ame_authoring_project_model
  PUBLIC
    nlohmann_json::nlohmann_json
    ame_core
)

add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  ${imgui_node_editor_SOURCE_DIR}/imgui_node_editor.cpp
  ${imgui_node_editor_SOURCE_DIR}/imgui_node_editor_api.cpp
  ${imgui_node_editor_SOURCE_DIR}/imgui_canvas.cpp
  ${imgui_node_editor_SOURCE_DIR}/crude_json.cpp
)

target_compile_definitions(imgui_lib
  PUBLIC
    IMGUI_ENABLE_DOCKING
)

target_include_directories(imgui_lib
  PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${imgui_node_editor_SOURCE_DIR}
    ${stb_SOURCE_DIR}
)

target_link_libraries(imgui_lib
  PUBLIC
    SDL2::SDL2
    OpenGL::GL
)

# Console subsystem on Windows: keeps stdout/stderr available for --self-test
# JSON reports and lets users see any runtime diagnostic messages. SDL2 still
# brings up the window regardless of subsystem.
add_executable(ame_authoring_tool
  main.cpp
  app_shell.cpp
  bt_graph_panel.cpp
  command_stack.cpp
  domain_graph_panel.cpp
  plan_graph_panel.cpp
  type_hierarchy_panel.cpp
  app_theme.cpp
)

set(AME_AUTHORING_SDL_MAIN_TARGET)
if(WIN32 AND TARGET SDL2::SDL2main)
  set(AME_AUTHORING_SDL_MAIN_TARGET SDL2::SDL2main)
endif()

target_link_libraries(ame_authoring_tool
  PRIVATE
    imgui_lib
    ${AME_AUTHORING_SDL_MAIN_TARGET}
    SDL2::SDL2
    OpenGL::GL
    ame_core
    ame_authoring_project_model
)

set_target_properties(ame_authoring_tool PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/..
)

if(WIN32)
  add_custom_command(TARGET ame_authoring_tool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:SDL2::SDL2>
      $<TARGET_FILE_DIR:ame_authoring_tool>
    COMMENT "Copying SDL2.dll to output directory"
  )
endif()
