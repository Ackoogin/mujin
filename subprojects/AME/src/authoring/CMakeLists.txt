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

# JetBrains Mono — distinctive technical UI font that fits the HUD aesthetic.
# Vendored at build time and copied next to the executable so the runtime can
# load it via AddFontFromFileTTF.
FetchContent_Declare(
  jetbrains_mono
  GIT_REPOSITORY https://github.com/JetBrains/JetBrainsMono
  GIT_TAG v2.304
  GIT_SHALLOW TRUE
)

# tinyfiledialogs — single-file C library for native open/save dialogs.
# Cross-platform; on Windows pops up the standard comdlg32 dialogs.
FetchContent_Declare(
  tinyfiledialogs
  GIT_REPOSITORY https://git.code.sf.net/p/tinyfiledialogs/code
  GIT_TAG master
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(imgui imgui_node_editor sdl2)
FetchContent_GetProperties(stb)
if(NOT stb_POPULATED)
  FetchContent_Populate(stb)
endif()
FetchContent_GetProperties(jetbrains_mono)
if(NOT jetbrains_mono_POPULATED)
  FetchContent_Populate(jetbrains_mono)
endif()
FetchContent_GetProperties(tinyfiledialogs)
if(NOT tinyfiledialogs_POPULATED)
  FetchContent_Populate(tinyfiledialogs)
endif()

add_library(tinyfiledialogs STATIC
  ${tinyfiledialogs_SOURCE_DIR}/tinyfiledialogs.c
)
target_include_directories(tinyfiledialogs PUBLIC ${tinyfiledialogs_SOURCE_DIR})
if(WIN32)
  target_link_libraries(tinyfiledialogs PUBLIC comdlg32 ole32)
endif()

if(NOT TARGET nlohmann_json::nlohmann_json)
  add_library(nlohmann_json INTERFACE)
  add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
  target_include_directories(nlohmann_json
    INTERFACE
      "${PYRAMID_ROOT_DIR}/core/external"
  )
endif()

add_library(ame_authoring_project_model STATIC
    assurance_report.cpp
    contingency_analyser.cpp
    fact_action_matrix.cpp
    fact_chooser.cpp
    failure_explainer.cpp
    guided_editor_model.cpp
    import_merge.cpp
    lifecycle_model.cpp
    mission_commands.cpp
    model_edits.cpp
    neighbourhood_model.cpp
    project_model.cpp
    pddl_generator.cpp
    pddl_importer.cpp
    pddl_validator.cpp
    problem_list.cpp
    recent_projects.cpp
    relation_index.cpp
    review_pack.cpp
    run_record.cpp
    scenario_runner.cpp
    simulation_engine.cpp
    structural_validator.cpp
)
target_include_directories(ame_authoring_project_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ame_authoring_project_model
  PUBLIC
    nlohmann_json::nlohmann_json
    ame_core
)

# The headless commands live in their own executable, linking only the core and
# the project-model library. Keeping them out of the graphical tool means a
# build agent needs no display stack to run a mission, and the conventions match
# contingency_verifier so that both tools are scripted the same way.
add_executable(ame_mission_cli mission_cli_main.cpp)
target_link_libraries(ame_mission_cli
  PRIVATE
    ame_core
    ame_authoring_project_model
)
set_target_properties(ame_mission_cli PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/..
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
    # Enables the per-item hook Dear ImGui provides for its test framework. The
    # authoring tool uses it to find items that share an identity, which the
    # library itself only reports while the mouse is resting on one of them.
    # The hook functions are defined in imgui_id_audit.cpp. This must be public
    # so that every translation unit sees the same declarations.
    IMGUI_ENABLE_TEST_ENGINE
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
  imgui_id_audit.cpp
  plan_graph_panel.cpp
  type_hierarchy_panel.cpp
  app_theme.cpp
)

# A real saved project for the offscreen self-test to open. The models the
# self-test builds by hand are deliberately tiny, which meant the domain views
# were never exercised against a domain where one action appears under several
# headings at once. This one has that shape.
target_compile_definitions(ame_authoring_tool PRIVATE
  "AME_AUTHORING_SAMPLE_PROJECT=\"${CMAKE_SOURCE_DIR}/vehicle-autonomy.ameproj.json\"")

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
    tinyfiledialogs
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

# Copy JetBrains Mono Regular next to the executable so runtime can load it.
add_custom_command(TARGET ame_authoring_tool POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${jetbrains_mono_SOURCE_DIR}/fonts/ttf/JetBrainsMono-Regular.ttf
    $<TARGET_FILE_DIR:ame_authoring_tool>/JetBrainsMono-Regular.ttf
  COMMENT "Copying JetBrainsMono-Regular.ttf to output directory"
)
