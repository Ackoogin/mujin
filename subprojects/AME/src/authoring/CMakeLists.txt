cmake_minimum_required(VERSION 3.20)

find_package(OpenGL REQUIRED)

# Dear ImGui, the node editor, SDL2, stb, the interface font and
# tinyfiledialogs. All of them are taken from AME's checked-in copies under
# subprojects/AME/external when those are present, so that this tool builds
# with no network access; see cmake/AmeDependencies.cmake.
ame_add_authoring_dependencies()

add_library(tinyfiledialogs STATIC
  ${tinyfiledialogs_SOURCE_DIR}/tinyfiledialogs.c
)
target_include_directories(tinyfiledialogs PUBLIC ${tinyfiledialogs_SOURCE_DIR})
if(WIN32)
  target_link_libraries(tinyfiledialogs PUBLIC comdlg32 ole32)
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
    presentation_groups.cpp
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
  "AME_AUTHORING_SAMPLE_PROJECT=\"${AME_SAMPLE_PROJECT}\"")

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
