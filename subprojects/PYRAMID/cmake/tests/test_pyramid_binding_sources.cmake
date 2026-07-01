# Standalone test for pyramid_binding_sources.cmake. Run with:
#   cmake -DMANIFEST=<binding_manifest.json> -DBINDINGS_DIR=<dir> \
#         -P cmake/tests/test_pyramid_binding_sources.cmake

include("${CMAKE_CURRENT_LIST_DIR}/../pyramid_binding_sources.cmake")

if(NOT DEFINED MANIFEST OR NOT DEFINED BINDINGS_DIR)
    message(FATAL_ERROR "MANIFEST and BINDINGS_DIR required")
endif()

set(PYRAMID_CPP_BINDINGS_DIR "${BINDINGS_DIR}")
pyramid_manifest_read(_PYRAMID_MANIFEST_JSON "${MANIFEST}")

# --- glob mode: manifest ignored, globs used --------------------------------
set(PYRAMID_BINDING_SOURCE_MODE "glob")
pyramid_binding_manifest_active(_active)
if(_active)
    message(FATAL_ERROR "ASSERT FAILED: manifest must be inactive in glob mode")
endif()
pyramid_binding_sources(_glob_codecs
    ROLE json_codecs SUFFIX .cpp
    GLOBS "${BINDINGS_DIR}/pyramid_data_model_*_codec.cpp")
list(LENGTH _glob_codecs _n_glob)
if(NOT _n_glob GREATER 0)
    message(FATAL_ERROR "ASSERT FAILED: glob mode found no codec sources")
endif()
foreach(_s IN LISTS _glob_codecs)
    if(NOT "${_s}" MATCHES "pyramid_data_model_.*_codec\\.cpp$")
        message(FATAL_ERROR "ASSERT FAILED: glob returned unexpected file ${_s}")
    endif()
endforeach()

# --- manifest mode: role list used ------------------------------------------
set(PYRAMID_BINDING_SOURCE_MODE "manifest")
pyramid_binding_manifest_active(_active)
if(NOT _active)
    message(FATAL_ERROR "ASSERT FAILED: manifest must be active in manifest mode")
endif()
pyramid_binding_sources(_mani_codecs
    ROLE json_codecs SUFFIX .cpp
    GLOBS "${BINDINGS_DIR}/pyramid_data_model_*_codec.cpp")
list(LENGTH _mani_codecs _n_mani)
if(NOT _n_mani GREATER 0)
    message(FATAL_ERROR "ASSERT FAILED: manifest mode found no codec sources")
endif()
foreach(_s IN LISTS _mani_codecs)
    if(NOT "${_s}" MATCHES "\\.cpp$")
        message(FATAL_ERROR "ASSERT FAILED: manifest returned non-.cpp ${_s}")
    endif()
    if("${_s}" MATCHES "\\.hpp$")
        message(FATAL_ERROR "ASSERT FAILED: .hpp leaked in manifest mode ${_s}")
    endif()
endforeach()

# --- manifest mode with a role absent from the manifest falls back to glob ---
pyramid_binding_sources(_fallback
    ROLE role_not_in_manifest
    GLOBS "${BINDINGS_DIR}/pyramid_data_model_*_codec.cpp")
list(LENGTH _fallback _n_fb)
if(NOT _n_fb EQUAL _n_glob)
    message(FATAL_ERROR "ASSERT FAILED: absent role should fall back to glob (${_n_fb} != ${_n_glob})")
endif()

message(STATUS "pyramid_binding_sources test OK: glob=${_n_glob} manifest=${_n_mani} fallback=${_n_fb}")
