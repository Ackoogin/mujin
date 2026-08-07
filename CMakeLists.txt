cmake_minimum_required(VERSION 3.21)
project(unmanned_workspace LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(UNMANNED_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(PCL_ROOT_DIR "${UNMANNED_WORKSPACE_ROOT}/subprojects/PCL")
set(PYRAMID_ROOT_DIR "${UNMANNED_WORKSPACE_ROOT}/subprojects/PYRAMID")
set(AME_ROOT_DIR "${UNMANNED_WORKSPACE_ROOT}/subprojects/AME")

option(UNMANNED_BUILD_PYRAMID "Build the PYRAMID subproject" ON)
option(UNMANNED_BUILD_AME "Build the AME subproject and AME-only dependencies" OFF)
option(PYRAMID_ENABLE_FLATBUFFERS "Enable FlatBuffers codec/transport for PYRAMID" ON)
option(PYRAMID_ENABLE_OWP "Enable the PYRAMID OMS WebSocket Protocol client" ON)
option(AME_BUILD_AGRA_MA_BRIDGE
    "Build the AME bridge against the generated A-GRA P3 C2 seam"
    OFF)
if(AME_BUILD_AGRA_MA_BRIDGE)
    if(NOT UNMANNED_BUILD_PYRAMID)
        message(FATAL_ERROR
            "AME_BUILD_AGRA_MA_BRIDGE requires UNMANNED_BUILD_PYRAMID=ON")
    endif()
    set(PYRAMID_ENABLE_AGRA_P3_BINDINGS ON CACHE BOOL
        "Generate A-GRA P3 C++ bindings without full proof targets" FORCE)
endif()

# ---------------------------------------------------------------------------
# Shared third-party dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

if(UNMANNED_BUILD_AME)
FetchContent_Declare(
    behaviortree_cpp
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG        4.6.2
    GIT_SHALLOW    TRUE
)
set(BTCPP_EXAMPLES        OFF CACHE BOOL "" FORCE)
set(BTCPP_UNIT_TESTS      OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING  OFF CACHE BOOL "" FORCE)
set(BTCPP_SHARED_LIBS     OFF CACHE BOOL "" FORCE)
set(CMAKE_SKIP_INSTALL_RULES TRUE)
FetchContent_MakeAvailable(behaviortree_cpp)
set(CMAKE_SKIP_INSTALL_RULES FALSE)

FetchContent_Declare(
    lapkt_src
    GIT_REPOSITORY https://github.com/LAPKT-dev/LAPKT-public.git
    GIT_TAG        Devel2.0
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(lapkt_src)
if(NOT lapkt_src_POPULATED)
    FetchContent_Populate(lapkt_src)
endif()

set(LAPKT_SRC ${lapkt_src_SOURCE_DIR}/src)

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
    target_include_directories(lapkt_core BEFORE PUBLIC
        ${UNMANNED_WORKSPACE_ROOT}/cmake/compat
    )
    target_compile_definitions(lapkt_core PRIVATE WIN32)
    target_compile_options(lapkt_core PRIVATE /W0)
else()
    target_compile_options(lapkt_core PRIVATE -w)
endif()

target_compile_features(lapkt_core PUBLIC cxx_std_17)

option(AME_FOXGLOVE "Enable Foxglove WebSocket bridge for live monitoring" ON)
option(AME_NEURO   "Neuro-symbolic core infrastructure (ame_neuro)"       OFF)
option(AME_LLM     "LLM backends via libcurl (implies AME_NEURO)"         OFF)
option(AME_NEURAL  "ONNX Runtime backends (implies AME_NEURO)"            OFF)
if(AME_LLM OR AME_NEURAL)
    set(AME_NEURO ON CACHE BOOL "" FORCE)
endif()
if(AME_LLM)
    find_package(CURL REQUIRED)
endif()
endif()

# Websocket++ and standalone Asio are header-only dependencies shared by the
# AME Foxglove bridge and the PYRAMID OWP client.  OWP may be enabled without
# building AME, so this cannot be nested under UNMANNED_BUILD_AME.
if((UNMANNED_BUILD_AME AND AME_FOXGLOVE)
   OR (UNMANNED_BUILD_PYRAMID AND PYRAMID_ENABLE_OWP))
    FetchContent_Declare(
        websocketpp
        GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
        GIT_TAG        0.8.2
        GIT_SHALLOW    TRUE
    )
    FetchContent_GetProperties(websocketpp)
    if(NOT websocketpp_POPULATED)
        FetchContent_Populate(websocketpp)
    endif()

    FetchContent_Declare(
        asio
        GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
        GIT_TAG        asio-1-28-0
        GIT_SHALLOW    TRUE
    )
    FetchContent_GetProperties(asio)
    if(NOT asio_POPULATED)
        FetchContent_Populate(asio)
    endif()
endif()

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

if(UNMANNED_BUILD_PYRAMID AND PYRAMID_ENABLE_FLATBUFFERS)
    FetchContent_Declare(
        flatbuffers
        GIT_REPOSITORY https://github.com/google/flatbuffers.git
        GIT_TAG        v24.3.25
        GIT_SHALLOW    TRUE
    )
    set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(FLATBUFFERS_BUILD_FLATLIB OFF CACHE BOOL "" FORCE)
    set(FLATBUFFERS_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(flatbuffers)
endif()

option(AME_BUILD_PYTHON "Build Python bindings for ame_core" OFF)
if(UNMANNED_BUILD_AME AND AME_BUILD_PYTHON)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH "" FORCE)

    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.11.1
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(pybind11)
endif()

enable_testing()

# ---------------------------------------------------------------------------
# Ada toolchain detection (shared by all subprojects that use gprbuild)
# ---------------------------------------------------------------------------
# Prefer the gprbuild that lives alongside g++ so the Ada compiler, C/C++
# compiler, linker, and runtime libraries all come from one installation.
find_program(GXX_FOR_ADA g++)
if(GXX_FOR_ADA)
    get_filename_component(_gxx_dir "${GXX_FOR_ADA}" DIRECTORY)
    find_program(GPRBUILD gprbuild HINTS "${_gxx_dir}" NO_DEFAULT_PATH)
    unset(_gxx_dir)
endif()
if(NOT GPRBUILD)
    find_program(GPRBUILD gprbuild)
endif()

# ---------------------------------------------------------------------------
# Contract tree selection
# ---------------------------------------------------------------------------
# PYRAMID generates its C++ bindings from a single contract tree named by
# PYRAMID_PROTO_DIR. PYRAMID's own CAL has no product contract, so when its
# proof layer is enabled it defaults that variable to the proof contracts.
#
# AME is a product consumer, not part of the proof layer, and it needs only a
# small closure of contracts: the options file, the base, common and autonomy
# data models, and the autonomy_backend and agra_ma_grounding component
# services. Those are checked in under subprojects/AME/contracts/proto so that
# AME can be built with PYRAMID's proof layer switched off. This is the
# "external contract tree" arrangement PYRAMID's own generated-bindings guide
# describes.
#
# When the proof layer is on, its contract tree is a superset of AME's and is
# used for both. The ame_contract_tree_matches_pyramid test asserts that the two
# copies of the shared files are identical, so which one a given build uses
# never changes the generated bindings.
set(AME_CONTRACT_PROTO_DIR "${AME_ROOT_DIR}/contracts/proto"
    CACHE PATH "Contract tree owned by AME, used when PYRAMID's proof contracts are not built")

# The choice below depends on whether the proof layer is being built, and
# PYRAMID declares that option itself. Its declaration runs during
# add_subdirectory(subprojects/PYRAMID), which is further down this file, so on
# a first configure the variable would still be empty here and every build would
# be treated as a proof-layer-off build. Declaring the option here settles the
# choice before it is read; PYRAMID's own option() call then finds it already
# cached and leaves it alone.
option(PYRAMID_BUILD_PROOFS
    "Build the non-product contracts, components, examples, and integration proofs"
    ON)

if(UNMANNED_BUILD_AME AND UNMANNED_BUILD_PYRAMID)
    if(NOT PYRAMID_BUILD_PROOFS)
        if(NOT DEFINED PYRAMID_PROTO_DIR OR PYRAMID_PROTO_DIR STREQUAL "")
            set(PYRAMID_PROTO_DIR "${AME_CONTRACT_PROTO_DIR}" CACHE PATH
                "PYRAMID proto IDL source directory" FORCE)
            message(STATUS
                "AME: PYRAMID proof contracts are not built, so binding generation "
                "uses AME's own contract tree at ${AME_CONTRACT_PROTO_DIR}")
        endif()
    elseif(DEFINED PYRAMID_PROTO_DIR AND
           PYRAMID_PROTO_DIR STREQUAL AME_CONTRACT_PROTO_DIR)
        # This build tree was configured earlier without the proof layer, which
        # pinned AME's contract tree in its cache. PYRAMID cannot correct that
        # itself, because it sets the default without FORCE and so leaves any
        # cached value alone. The proof targets would then be built against
        # bindings that were never generated, and the configure would fail on
        # targets with no sources. Clearing the entry lets PYRAMID's own default
        # apply.
        unset(PYRAMID_PROTO_DIR CACHE)
        message(STATUS
            "AME: the proof layer is being built, so binding generation returns "
            "to PYRAMID's proof contract tree")
    endif()
endif()

# ---------------------------------------------------------------------------
# Component subprojects
# ---------------------------------------------------------------------------
add_subdirectory(subprojects/PCL)
if(UNMANNED_BUILD_PYRAMID)
    add_subdirectory(subprojects/PYRAMID)
endif()
if(UNMANNED_BUILD_AME)
    add_subdirectory(subprojects/AME)
endif()
