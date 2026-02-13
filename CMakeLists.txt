cmake_minimum_required(VERSION 3.21)
project(mujin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies via FetchContent
# ---------------------------------------------------------------------------
include(FetchContent)

# --- BehaviorTree.CPP v4 ---
FetchContent_Declare(
    behaviortree_cpp
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG        4.6.2
    GIT_SHALLOW    TRUE
)
# Disable BT.CPP extras we don't need
set(BTCPP_EXAMPLES        OFF CACHE BOOL "" FORCE)
set(BTCPP_UNIT_TESTS      OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING  OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(behaviortree_cpp)

# --- LAPKT (core only — we build from source, bypassing its own CMake) ---
FetchContent_Declare(
    lapkt_src
    GIT_REPOSITORY https://github.com/LAPKT-dev/LAPKT-public.git
    GIT_TAG        Devel2.0
    GIT_SHALLOW    TRUE
    # We do NOT want LAPKT's CMakeLists to run — we build core ourselves
)
FetchContent_GetProperties(lapkt_src)
if(NOT lapkt_src_POPULATED)
    FetchContent_Populate(lapkt_src)
endif()

# Build LAPKT core as a static library from individual source files.
# This avoids LAPKT's complex superbuild + pybind11 + Boost dependencies.
set(LAPKT_SRC ${lapkt_src_SOURCE_DIR}/src)

add_library(lapkt_core STATIC
    # ltl (utility library)
    ${LAPKT_SRC}/ltl/bit_array.cxx
    ${LAPKT_SRC}/ltl/bit_set.cxx
    ${LAPKT_SRC}/ltl/memory.cxx
    ${LAPKT_SRC}/ltl/resources_control.cxx
    # model (STRIPS planning model)
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
    # component (search data structures)
    ${LAPKT_SRC}/component/match_tree.cxx
    ${LAPKT_SRC}/component/reachability.cxx
    ${LAPKT_SRC}/component/watched_lit_succ_gen.cxx
    # node_eval/heuristic
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

# MSVC compatibility: provide shim headers for POSIX includes
# (sys/time.h, unistd.h, sys/resource.h) that LAPKT expects
if(MSVC)
    target_include_directories(lapkt_core BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/compat
    )
    target_compile_definitions(lapkt_core PRIVATE WIN32)
endif()

target_compile_features(lapkt_core PUBLIC cxx_std_17)

# Suppress warnings from third-party code
if(MSVC)
    target_compile_options(lapkt_core PRIVATE /W0)
else()
    target_compile_options(lapkt_core PRIVATE -w)
endif()

# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
add_subdirectory(src)
