# AME's third-party dependencies.
#
# AME is deployed into air-gapped environments, so every dependency it needs is
# checked into the repository under subprojects/AME/external, the way PCL and
# PYRAMID are. That directory is written by scripts/vendor_dependencies.py,
# which also records what came from where in external/manifest.json.
#
# Each dependency below is taken from external/ when it is present, and fetched
# from its upstream repository when it is not. The fallback exists so that a
# developer who has deleted external/, or who is adding a dependency before
# vendoring it, still has a working build; it is never what a deployment uses.
# Set AME_REQUIRE_VENDORED_DEPENDENCIES=ON to turn a missing copy into an error
# instead, which is what an air-gapped build should do so that it fails on the
# spot rather than hanging on a network call.
#
# Every declaration here is guarded on the target or variable it defines, so
# that this file can be included from the workspace root and from AME's own
# standalone build without anything being declared twice. When AME is built
# inside the unmanned workspace, PYRAMID may already have declared some of the
# same dependencies, and those declarations win.

include_guard(GLOBAL)
include(FetchContent)

set(AME_EXTERNAL_DIR "${CMAKE_CURRENT_LIST_DIR}/../external"
    CACHE PATH "Where AME's checked-in third-party sources live")
get_filename_component(AME_EXTERNAL_DIR "${AME_EXTERNAL_DIR}" ABSOLUTE)

option(AME_REQUIRE_VENDORED_DEPENDENCIES
    "Fail rather than download when a dependency is missing from external/" OFF)

# ---------------------------------------------------------------------------
# ame_dependency_source(<name> <external subdirectory> <output variable>)
#
# Sets <output variable> to the checked-in copy when it exists. Otherwise
# leaves it empty, and the caller falls back to FetchContent.
# ---------------------------------------------------------------------------
function(ame_dependency_source name subdirectory out_var)
    set(candidate "${AME_EXTERNAL_DIR}/${subdirectory}")
    if(EXISTS "${candidate}")
        set(${out_var} "${candidate}" PARENT_SCOPE)
        return()
    endif()
    if(AME_REQUIRE_VENDORED_DEPENDENCIES)
        message(FATAL_ERROR
            "AME: ${name} is not checked in at ${candidate}, and "
            "AME_REQUIRE_VENDORED_DEPENDENCIES is on so it will not be "
            "downloaded. Run subprojects/AME/scripts/vendor_dependencies.py "
            "on a networked machine and commit the result.")
    endif()
    message(STATUS
        "AME: ${name} is not checked in at ${candidate}; falling back to "
        "downloading it. This build needs network access.")
    set(${out_var} "" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# BehaviorTree.CPP — runs the trees the plan compiler emits
# ---------------------------------------------------------------------------
function(ame_add_behaviortree_cpp)
    if(TARGET behaviortree_cpp)
        return()
    endif()
    set(BTCPP_EXAMPLES        OFF CACHE BOOL "" FORCE)
    set(BTCPP_UNIT_TESTS      OFF CACHE BOOL "" FORCE)
    set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
    set(BTCPP_SQLITE_LOGGING  OFF CACHE BOOL "" FORCE)
    set(BTCPP_SHARED_LIBS     OFF CACHE BOOL "" FORCE)
    # The command-line tools are not part of what AME builds, and their sources
    # are not among the files vendored under external/.
    set(BTCPP_BUILD_TOOLS     OFF CACHE BOOL "" FORCE)
    # Upstream installs headers and a config package that AME does not use, and
    # whose install script CMake would then expect to find.
    set(CMAKE_SKIP_INSTALL_RULES TRUE)

    ame_dependency_source("BehaviorTree.CPP" behaviortree_cpp vendored)
    if(vendored)
        set(build_dir "${CMAKE_BINARY_DIR}/_ame_external/behaviortree_cpp")
        add_subdirectory("${vendored}" "${build_dir}" EXCLUDE_FROM_ALL)
        # AME's install rules read these two, which FetchContent would have set.
        set(behaviortree_cpp_SOURCE_DIR "${vendored}"  CACHE INTERNAL "")
        set(behaviortree_cpp_BINARY_DIR "${build_dir}" CACHE INTERNAL "")
    else()
        FetchContent_Declare(behaviortree_cpp
            GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
            GIT_TAG        4.6.2
            GIT_SHALLOW    TRUE)
        FetchContent_MakeAvailable(behaviortree_cpp)
    endif()
    set(CMAKE_SKIP_INSTALL_RULES FALSE)
endfunction()

# ---------------------------------------------------------------------------
# LAPKT — the classical planner behind ame::Planner
#
# Upstream ships no usable CMake target for the subset AME needs, so the
# library is declared here from its sources, which is what the workspace root
# did before this file existed.
# ---------------------------------------------------------------------------
function(ame_add_lapkt)
    if(TARGET lapkt_core)
        return()
    endif()
    ame_dependency_source("LAPKT" lapkt vendored)
    if(vendored)
        set(lapkt_root "${vendored}")
    else()
        FetchContent_Declare(lapkt_src
            GIT_REPOSITORY https://github.com/LAPKT-dev/LAPKT-public.git
            GIT_TAG        Devel2.0
            GIT_SHALLOW    TRUE)
        FetchContent_GetProperties(lapkt_src)
        if(NOT lapkt_src_POPULATED)
            FetchContent_Populate(lapkt_src)
        endif()
        set(lapkt_root "${lapkt_src_SOURCE_DIR}")
    endif()

    set(LAPKT_SRC "${lapkt_root}/src")
    add_library(lapkt_core STATIC
        ${LAPKT_SRC}/ltl/bit_array.cxx
        ${LAPKT_SRC}/ltl/bit_set.cxx
        ${LAPKT_SRC}/ltl/memory.cxx
        ${LAPKT_SRC}/ltl/resources_control.cxx
        ${LAPKT_SRC}/model/action.cxx
        ${LAPKT_SRC}/model/cond_eff.cxx
        ${LAPKT_SRC}/model/conj_comp_prob.cxx
        ${LAPKT_SRC}/model/fl_conj.cxx
        ${LAPKT_SRC}/model/fluent.cxx
        ${LAPKT_SRC}/model/fwd_search_prob.cxx
        ${LAPKT_SRC}/model/mutex_set.cxx
        ${LAPKT_SRC}/model/strips_prob.cxx
        ${LAPKT_SRC}/model/strips_state.cxx
        ${LAPKT_SRC}/model/succ_gen.cxx
        ${LAPKT_SRC}/component/match_tree.cxx
        ${LAPKT_SRC}/component/reachability.cxx
        ${LAPKT_SRC}/component/watched_lit_succ_gen.cxx
        ${LAPKT_SRC}/node_eval/heuristic/landmark_graph.cxx
    )
    target_include_directories(lapkt_core PUBLIC
        ${LAPKT_SRC}
        ${LAPKT_SRC}/model
        ${LAPKT_SRC}/component
        ${LAPKT_SRC}/engine
        ${LAPKT_SRC}/node_eval/heuristic
        ${LAPKT_SRC}/node_eval/novelty
        ${LAPKT_SRC}/ltl
    )
    if(MSVC)
        # LAPKT is written for GCC; the shims supply what MSVC does not have.
        # CMAKE_CURRENT_FUNCTION_LIST_DIR is this file's directory. Inside a
        # function CMAKE_CURRENT_LIST_DIR would be the caller's directory
        # instead, which is not where the shims live.
        target_include_directories(lapkt_core BEFORE PUBLIC
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/compat")
        target_compile_definitions(lapkt_core PRIVATE WIN32)
        target_compile_options(lapkt_core PRIVATE /W0)
    else()
        target_compile_options(lapkt_core PRIVATE -w)
    endif()
    target_compile_features(lapkt_core PUBLIC cxx_std_17)
endfunction()

# ---------------------------------------------------------------------------
# GoogleTest — every AME test binary
# ---------------------------------------------------------------------------
function(ame_add_googletest)
    if(TARGET GTest::gtest_main)
        return()
    endif()
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    ame_dependency_source("GoogleTest" googletest vendored)
    if(vendored)
        add_subdirectory("${vendored}"
                         "${CMAKE_BINARY_DIR}/_ame_external/googletest"
                         EXCLUDE_FROM_ALL)
    else()
        FetchContent_Declare(googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        v1.14.0
            GIT_SHALLOW    TRUE)
        FetchContent_MakeAvailable(googletest)
    endif()
endfunction()

# ---------------------------------------------------------------------------
# nlohmann/json — the authoring project format, run records and reports
#
# A single header, so it is an interface target rather than a build.
# ---------------------------------------------------------------------------
function(ame_add_nlohmann_json)
    if(TARGET nlohmann_json::nlohmann_json)
        return()
    endif()
    ame_dependency_source("nlohmann/json" nlohmann_json/include vendored)
    if(NOT vendored)
        FetchContent_Declare(nlohmann_json
            GIT_REPOSITORY https://github.com/nlohmann/json.git
            GIT_TAG        v3.11.3
            GIT_SHALLOW    TRUE)
        FetchContent_GetProperties(nlohmann_json)
        if(NOT nlohmann_json_POPULATED)
            FetchContent_Populate(nlohmann_json)
        endif()
        set(vendored "${nlohmann_json_SOURCE_DIR}/single_include")
    endif()
    add_library(ame_nlohmann_json INTERFACE)
    add_library(nlohmann_json::nlohmann_json ALIAS ame_nlohmann_json)
    target_include_directories(ame_nlohmann_json INTERFACE "${vendored}")
endfunction()

# ---------------------------------------------------------------------------
# websocketpp and standalone Asio — the Foxglove bridge only
#
# Both are header-only. They are exposed as source directories rather than
# targets because that is how the Foxglove bridge already consumes them, and
# because PYRAMID's OWP client consumes the same two variables.
# ---------------------------------------------------------------------------
function(ame_add_websocket_headers)
    if(NOT websocketpp_SOURCE_DIR)
        ame_dependency_source("websocketpp" websocketpp vendored)
        if(NOT vendored)
            FetchContent_Declare(websocketpp
                GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
                GIT_TAG        0.8.2
                GIT_SHALLOW    TRUE)
            FetchContent_GetProperties(websocketpp)
            if(NOT websocketpp_POPULATED)
                FetchContent_Populate(websocketpp)
            endif()
            set(vendored "${websocketpp_SOURCE_DIR}")
        endif()
        set(websocketpp_SOURCE_DIR "${vendored}" CACHE INTERNAL "")
    endif()

    if(NOT asio_SOURCE_DIR)
        ame_dependency_source("Asio" asio vendored)
        if(NOT vendored)
            FetchContent_Declare(asio
                GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
                GIT_TAG        asio-1-28-0
                GIT_SHALLOW    TRUE)
            FetchContent_GetProperties(asio)
            if(NOT asio_POPULATED)
                FetchContent_Populate(asio)
            endif()
            set(vendored "${asio_SOURCE_DIR}")
        endif()
        set(asio_SOURCE_DIR "${vendored}" CACHE INTERNAL "")
    endif()
endfunction()

# ---------------------------------------------------------------------------
# The graphical authoring tool's dependencies
#
# These are exposed as source-directory variables under the names the authoring
# CMakeLists already uses, so that switching between checked-in and downloaded
# copies changes nothing downstream.
# ---------------------------------------------------------------------------
function(ame_add_authoring_dependencies)
    ame_dependency_source("Dear ImGui" imgui imgui_dir)
    if(NOT imgui_dir)
        FetchContent_Declare(imgui
            GIT_REPOSITORY https://github.com/ocornut/imgui
            GIT_TAG        v1.91.6-docking
            GIT_SHALLOW    TRUE)
        FetchContent_GetProperties(imgui)
        if(NOT imgui_POPULATED)
            FetchContent_Populate(imgui)
        endif()
        set(imgui_dir "${imgui_SOURCE_DIR}")
    endif()
    set(imgui_SOURCE_DIR "${imgui_dir}" CACHE INTERNAL "")

    ame_dependency_source("imgui-node-editor" imgui_node_editor node_editor_dir)
    if(NOT node_editor_dir)
        FetchContent_Declare(imgui_node_editor
            GIT_REPOSITORY https://github.com/thedmd/imgui-node-editor
            GIT_TAG        develop
            GIT_SHALLOW    TRUE
            SOURCE_SUBDIR  cmake/unused)
        FetchContent_GetProperties(imgui_node_editor)
        if(NOT imgui_node_editor_POPULATED)
            FetchContent_Populate(imgui_node_editor)
        endif()
        set(node_editor_dir "${imgui_node_editor_SOURCE_DIR}")
    endif()
    set(imgui_node_editor_SOURCE_DIR "${node_editor_dir}" CACHE INTERNAL "")

    ame_dependency_source("stb" stb stb_dir)
    if(NOT stb_dir)
        FetchContent_Declare(stb
            GIT_REPOSITORY https://github.com/nothings/stb
            GIT_TAG        5736b15f7ea0ffb08dd38af21067c314d6a3aae9
            GIT_SHALLOW    FALSE)
        FetchContent_GetProperties(stb)
        if(NOT stb_POPULATED)
            FetchContent_Populate(stb)
        endif()
        set(stb_dir "${stb_SOURCE_DIR}")
    endif()
    set(stb_SOURCE_DIR "${stb_dir}" CACHE INTERNAL "")

    ame_dependency_source("JetBrains Mono" jetbrains_mono font_dir)
    if(NOT font_dir)
        FetchContent_Declare(jetbrains_mono
            GIT_REPOSITORY https://github.com/JetBrains/JetBrainsMono
            GIT_TAG        v2.304
            GIT_SHALLOW    TRUE)
        FetchContent_GetProperties(jetbrains_mono)
        if(NOT jetbrains_mono_POPULATED)
            FetchContent_Populate(jetbrains_mono)
        endif()
        set(font_dir "${jetbrains_mono_SOURCE_DIR}")
    endif()
    set(jetbrains_mono_SOURCE_DIR "${font_dir}" CACHE INTERNAL "")

    ame_dependency_source("tinyfiledialogs" tinyfiledialogs tfd_dir)
    if(NOT tfd_dir)
        FetchContent_Declare(tinyfiledialogs
            GIT_REPOSITORY https://git.code.sf.net/p/tinyfiledialogs/code
            GIT_TAG        master
            GIT_SHALLOW    TRUE)
        FetchContent_GetProperties(tinyfiledialogs)
        if(NOT tinyfiledialogs_POPULATED)
            FetchContent_Populate(tinyfiledialogs)
        endif()
        set(tfd_dir "${tinyfiledialogs_SOURCE_DIR}")
    endif()
    set(tinyfiledialogs_SOURCE_DIR "${tfd_dir}" CACHE INTERNAL "")

    if(NOT TARGET SDL2::SDL2)
        set(SDL_SHARED ON  CACHE BOOL "" FORCE)
        set(SDL_STATIC OFF CACHE BOOL "" FORCE)
        set(SDL_TEST   OFF CACHE BOOL "" FORCE)
        set(SDL_TESTS  OFF CACHE BOOL "" FORCE)
        ame_dependency_source("SDL2" sdl2 sdl_dir)
        if(sdl_dir)
            add_subdirectory("${sdl_dir}"
                             "${CMAKE_BINARY_DIR}/_ame_external/sdl2"
                             EXCLUDE_FROM_ALL)
        else()
            FetchContent_Declare(sdl2
                GIT_REPOSITORY https://github.com/libsdl-org/SDL
                GIT_TAG        release-2.30.10
                GIT_SHALLOW    TRUE)
            FetchContent_MakeAvailable(sdl2)
        endif()
    endif()
endfunction()
