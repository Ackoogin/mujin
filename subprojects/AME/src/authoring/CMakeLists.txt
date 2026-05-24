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
  GIT_TAG v0.9.3
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR cmake/unused
)

FetchContent_Declare(
  sdl2
  GIT_REPOSITORY https://github.com/libsdl-org/SDL
  GIT_TAG release-2.30.10
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(imgui imgui_node_editor sdl2)

add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_compile_definitions(imgui_lib
  PUBLIC
    IMGUI_ENABLE_DOCKING
)

target_include_directories(imgui_lib
  PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui_lib
  PUBLIC
    SDL2::SDL2
    OpenGL::GL
)

if(WIN32)
  add_executable(ame_authoring_tool WIN32 main.cpp)
else()
  add_executable(ame_authoring_tool main.cpp)
endif()

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
