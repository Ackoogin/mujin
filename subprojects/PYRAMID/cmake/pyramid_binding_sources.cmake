# pyramid_binding_sources.cmake
#
# Source-selection wrappers that let CMake build targets from the generator's
# binding_manifest.json roles ("manifest" mode) or from legacy pyramid_* filename
# globs ("glob" mode, the default). Kept in its own module so the selection
# logic is unit-testable in isolation (cmake -P) without configuring the whole
# project.
#
# These functions read the following variables from the calling scope
# (CMake dynamic scoping):
#   PYRAMID_BINDING_SOURCE_MODE  "glob" | "manifest"
#   _PYRAMID_MANIFEST_JSON       manifest JSON string ("" when unavailable)
#   PYRAMID_CPP_BINDINGS_DIR     directory the generated artifacts live in

include("${CMAKE_CURRENT_LIST_DIR}/pyramid_manifest.cmake")

# Is manifest-driven selection both requested and available?
function(pyramid_binding_manifest_active out_var)
    if(PYRAMID_BINDING_SOURCE_MODE STREQUAL "manifest"
       AND NOT "${_PYRAMID_MANIFEST_JSON}" STREQUAL "")
        set(${out_var} TRUE PARENT_SCOPE)
    else()
        set(${out_var} FALSE PARENT_SCOPE)
    endif()
endfunction()

# Select generated sources by manifest ROLE (when manifest mode is active and
# the role yields entries) or by legacy filename GLOBS otherwise. The glob path
# reproduces the previous behavior exactly, so "glob" mode is byte-identical to
# the pre-manifest build.
#
#   pyramid_binding_sources(OUT
#       ROLE json_codecs SUFFIX .cpp
#       GLOBS "${dir}/pyramid_data_model_*_codec.cpp")
function(pyramid_binding_sources out_var)
    cmake_parse_arguments(_PBS "" "ROLE;SUFFIX" "GLOBS" ${ARGN})
    set(_srcs "")
    pyramid_binding_manifest_active(_active)
    if(_active AND _PBS_ROLE)
        if(_PBS_SUFFIX)
            pyramid_manifest_sources(_srcs "${_PYRAMID_MANIFEST_JSON}" "${_PBS_ROLE}"
                "${PYRAMID_CPP_BINDINGS_DIR}" SUFFIX "${_PBS_SUFFIX}")
        else()
            pyramid_manifest_sources(_srcs "${_PYRAMID_MANIFEST_JSON}" "${_PBS_ROLE}"
                "${PYRAMID_CPP_BINDINGS_DIR}")
        endif()
        if(_srcs)
            list(SORT _srcs)
            set(${out_var} "${_srcs}" PARENT_SCOPE)
            return()
        endif()
    endif()
    if(_PBS_GLOBS)
        # CONFIGURE_DEPENDS is only valid during project configuration, not in
        # cmake -P script mode (used by the unit tests).
        if(CMAKE_SCRIPT_MODE_FILE)
            file(GLOB_RECURSE _srcs ${_PBS_GLOBS})
        else()
            file(GLOB_RECURSE _srcs CONFIGURE_DEPENDS ${_PBS_GLOBS})
        endif()
        list(SORT _srcs)
    endif()
    set(${out_var} "${_srcs}" PARENT_SCOPE)
endfunction()
