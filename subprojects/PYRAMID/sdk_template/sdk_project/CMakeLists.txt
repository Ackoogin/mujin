cmake_minimum_required(VERSION 3.21)
project(pyramid_sdk_plugins LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Standalone "proto -> plugin" build for the PCL/PYRAMID offline SDK.
#
# This project is deliberately independent of the parent monorepo: no
# FetchContent, no network access, no third-party source fetches. It compiles
# the *generated* bindings (produced by ../scripts/generate_bindings before
# this project is configured) against prebuilt PCL static libs and vendored
# header-only dependencies that ship in this SDK.
#
# Layout this project expects (all relative to this file's parent dir, the
# SDK root):
#   include/pcl/*                        PCL public headers
#   include/external/{nlohmann,tl,flatbuffers}/*  vendored header-only deps
#   lib/msvc/{Debug,Release}/pcl_core.lib prebuilt PCL static lib
#   tools/flatc.exe                      prebuilt FlatBuffers schema compiler
#   sdk_project/generated/               bindings produced by generate_bindings
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The generated marshal/codec STATIC libs are linked into MODULE (.so/.dll)
# codec plugins, so their objects must be position-independent. A no-op on
# MSVC; required on ELF toolchains (otherwise ld rejects the shared link).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(SDK_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH
    "Root of the PCL/PYRAMID offline SDK deploy directory")
set(SDK_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated" CACHE PATH
    "Directory containing bindings produced by scripts/generate_bindings")

option(PYRAMID_SDK_ENABLE_FLATBUFFERS
    "Build the FlatBuffers codec plugin (needs generated/flatbuffers/cpp/*.fbs)" ON)

if(NOT EXISTS "${SDK_GENERATED_DIR}")
    message(FATAL_ERROR
        "No generated bindings found at ${SDK_GENERATED_DIR}. Run "
        "scripts/generate_bindings first.")
endif()

# ---------------------------------------------------------------------------
# Prebuilt PCL (imported, no sources compiled here)
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)

add_library(pcl_core STATIC IMPORTED GLOBAL)
if(WIN32)
    set_target_properties(pcl_core PROPERTIES
        IMPORTED_LOCATION                "${SDK_ROOT_DIR}/lib/msvc/Release/pcl_core.lib"
        IMPORTED_LOCATION_RELEASE        "${SDK_ROOT_DIR}/lib/msvc/Release/pcl_core.lib"
        IMPORTED_LOCATION_DEBUG          "${SDK_ROOT_DIR}/lib/msvc/Debug/pcl_core.lib"
        IMPORTED_LOCATION_RELWITHDEBINFO "${SDK_ROOT_DIR}/lib/msvc/Release/pcl_core.lib"
        IMPORTED_LOCATION_MINSIZEREL     "${SDK_ROOT_DIR}/lib/msvc/Release/pcl_core.lib"
    )
else()
    set_target_properties(pcl_core PROPERTIES
        IMPORTED_LOCATION "${SDK_ROOT_DIR}/lib/linux/libpcl_core.a")
endif()
set_target_properties(pcl_core PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES    "${SDK_ROOT_DIR}/include"
    INTERFACE_LINK_LIBRARIES         "Threads::Threads;${CMAKE_DL_LIBS}"
)

# ---------------------------------------------------------------------------
# Vendored header-only dependencies
# ---------------------------------------------------------------------------
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE "${SDK_ROOT_DIR}/include/external")

# ---------------------------------------------------------------------------
# Generated C-ABI marshalling (native <-> frozen pyramid_<Type>_c struct)
# ---------------------------------------------------------------------------
file(GLOB _sdk_marshal_srcs CONFIGURE_DEPENDS
    "${SDK_GENERATED_DIR}/pyramid_data_model_*_cabi_marshal.cpp"
    "${SDK_GENERATED_DIR}/pyramid_components_*_cabi_marshal.cpp")
if(NOT _sdk_marshal_srcs)
    message(FATAL_ERROR
        "No generated marshalling sources found under ${SDK_GENERATED_DIR}. "
        "Run scripts/generate_bindings first.")
endif()

add_library(pyramid_generated_marshal STATIC ${_sdk_marshal_srcs})
target_include_directories(pyramid_generated_marshal PUBLIC
    "${SDK_GENERATED_DIR}"
    "${SDK_ROOT_DIR}/include/external")
target_link_libraries(pyramid_generated_marshal PUBLIC pcl_core)

# ---------------------------------------------------------------------------
# FlatBuffers: flatc-driven header generation from the generated .fbs schemas
# ---------------------------------------------------------------------------
add_library(flatbuffers_runtime INTERFACE)

if(PYRAMID_SDK_ENABLE_FLATBUFFERS)
    add_executable(flatc IMPORTED GLOBAL)
    if(WIN32)
        set_target_properties(flatc PROPERTIES
            IMPORTED_LOCATION "${SDK_ROOT_DIR}/tools/flatc.exe")
    else()
        set_target_properties(flatc PROPERTIES
            IMPORTED_LOCATION "${SDK_ROOT_DIR}/tools/flatc")
    endif()

    target_include_directories(flatbuffers_runtime INTERFACE "${SDK_ROOT_DIR}/include/external")

    set(SDK_FLATBUFFERS_SCHEMA_DIR "${SDK_GENERATED_DIR}/flatbuffers/cpp")
    set(SDK_FLATBUFFERS_GEN_DIR "${CMAKE_BINARY_DIR}/generated/flatbuffers/cpp")
    file(GLOB SDK_FLATBUFFERS_SCHEMAS CONFIGURE_DEPENDS
        "${SDK_FLATBUFFERS_SCHEMA_DIR}/pyramid_*.fbs")

    if(SDK_FLATBUFFERS_SCHEMAS)
        set(SDK_FLATBUFFERS_OUTPUTS)
        foreach(_schema IN LISTS SDK_FLATBUFFERS_SCHEMAS)
            get_filename_component(_stem "${_schema}" NAME_WE)
            set(_out "${SDK_FLATBUFFERS_GEN_DIR}/${_stem}_generated.h")
            add_custom_command(
                OUTPUT "${_out}"
                COMMAND "${CMAKE_COMMAND}" -E make_directory "${SDK_FLATBUFFERS_GEN_DIR}"
                COMMAND $<TARGET_FILE:flatc> --cpp --gen-object-api -o "${SDK_FLATBUFFERS_GEN_DIR}" "${_schema}"
                DEPENDS flatc "${_schema}"
                COMMENT "Generating FlatBuffers header for ${_stem}.fbs"
                VERBATIM
            )
            list(APPEND SDK_FLATBUFFERS_OUTPUTS "${_out}")
        endforeach()
        add_custom_target(pyramid_sdk_flatbuffers_codegen DEPENDS ${SDK_FLATBUFFERS_OUTPUTS})

        target_include_directories(flatbuffers_runtime INTERFACE
            "${SDK_FLATBUFFERS_GEN_DIR}"
            "${SDK_FLATBUFFERS_SCHEMA_DIR}")
    else()
        message(STATUS
            "PYRAMID SDK: no generated FlatBuffers schemas found under "
            "${SDK_FLATBUFFERS_SCHEMA_DIR} -- FlatBuffers codec plugins will "
            "be skipped. Regenerate with --backends json,flatbuffers if needed.")
        set(PYRAMID_SDK_ENABLE_FLATBUFFERS OFF)
    endif()
endif()

# ---------------------------------------------------------------------------
# Wire codecs: JSON (always) + FlatBuffers (optional)
# ---------------------------------------------------------------------------
file(GLOB _sdk_codec_srcs CONFIGURE_DEPENDS
    "${SDK_GENERATED_DIR}/pyramid_data_model_*_codec.cpp"
    "${SDK_GENERATED_DIR}/pyramid_components_*_codec.cpp")
if(PYRAMID_SDK_ENABLE_FLATBUFFERS)
    file(GLOB _sdk_flatbuffers_codec_srcs CONFIGURE_DEPENDS
        "${SDK_FLATBUFFERS_SCHEMA_DIR}/pyramid_services_*_flatbuffers_codec.cpp")
    list(APPEND _sdk_codec_srcs ${_sdk_flatbuffers_codec_srcs})
endif()
if(NOT _sdk_codec_srcs)
    message(FATAL_ERROR "No generated codec sources found under ${SDK_GENERATED_DIR}.")
endif()

add_library(pyramid_generated_codecs STATIC ${_sdk_codec_srcs})
target_link_libraries(pyramid_generated_codecs PUBLIC pyramid_generated_marshal nlohmann_json)
target_include_directories(pyramid_generated_codecs PUBLIC "${SDK_GENERATED_DIR}")
if(PYRAMID_SDK_ENABLE_FLATBUFFERS)
    add_dependencies(pyramid_generated_codecs pyramid_sdk_flatbuffers_codegen)
    target_link_libraries(pyramid_generated_codecs PUBLIC flatbuffers_runtime)
endif()

# ---------------------------------------------------------------------------
# Codec plugins: one MODULE (.dll) per component, per enabled codec
# ---------------------------------------------------------------------------
set(PYRAMID_SDK_PLUGIN_TARGETS)

file(GLOB _sdk_json_plugin_srcs CONFIGURE_DEPENDS
    "${SDK_GENERATED_DIR}/*_json_codec_plugin.cpp")
foreach(_src IN LISTS _sdk_json_plugin_srcs)
    get_filename_component(_stem "${_src}" NAME_WE)
    string(REGEX REPLACE "^pyramid_services_(.+)_json_codec_plugin$"
        "pyramid_codec_json_\\1" _target "${_stem}")
    add_library(${_target} MODULE "${_src}")
    target_link_libraries(${_target} PRIVATE pcl_core pyramid_generated_codecs nlohmann_json)
    target_include_directories(${_target} PRIVATE "${SDK_GENERATED_DIR}")
    list(APPEND PYRAMID_SDK_PLUGIN_TARGETS ${_target})
endforeach()

if(PYRAMID_SDK_ENABLE_FLATBUFFERS)
    file(GLOB _sdk_flatbuffers_plugin_srcs CONFIGURE_DEPENDS
        "${SDK_GENERATED_DIR}/*_flatbuffers_codec_plugin.cpp")
    foreach(_src IN LISTS _sdk_flatbuffers_plugin_srcs)
        get_filename_component(_stem "${_src}" NAME_WE)
        string(REGEX REPLACE "^pyramid_services_(.+)_flatbuffers_codec_plugin$"
            "pyramid_codec_flatbuffers_\\1" _target "${_stem}")
        add_library(${_target} MODULE "${_src}")
        add_dependencies(${_target} pyramid_sdk_flatbuffers_codegen)
        target_link_libraries(${_target} PRIVATE
            pcl_core pyramid_generated_codecs flatbuffers_runtime nlohmann_json)
        target_include_directories(${_target} PRIVATE
            "${SDK_GENERATED_DIR}" "${SDK_FLATBUFFERS_SCHEMA_DIR}")
        list(APPEND PYRAMID_SDK_PLUGIN_TARGETS ${_target})
    endforeach()
endif()

if(NOT PYRAMID_SDK_PLUGIN_TARGETS)
    message(FATAL_ERROR
        "No codec plugin sources discovered under ${SDK_GENERATED_DIR}. "
        "Check that scripts/generate_bindings ran against a proto/ tree "
        "containing at least one service.")
endif()

add_custom_target(pyramid_sdk_plugins ALL DEPENDS ${PYRAMID_SDK_PLUGIN_TARGETS})

message(STATUS "PYRAMID SDK: codec plugin targets: ${PYRAMID_SDK_PLUGIN_TARGETS}")
