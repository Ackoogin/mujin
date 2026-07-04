# Standalone test for pyramid_manifest.cmake. Run with:
#   cmake -DMANIFEST=<path/to/binding_manifest.json> \
#         -DBINDINGS_DIR=<dir> \
#         -P cmake/tests/test_pyramid_manifest.cmake
#
# Exits non-zero (FATAL_ERROR) on the first failed assertion.

include("${CMAKE_CURRENT_LIST_DIR}/../pyramid_manifest.cmake")

if(NOT DEFINED MANIFEST)
    message(FATAL_ERROR "MANIFEST not provided")
endif()
if(NOT DEFINED BINDINGS_DIR)
    set(BINDINGS_DIR "/bindings")
endif()

pyramid_manifest_read(_json "${MANIFEST}")
if("${_json}" STREQUAL "")
    message(FATAL_ERROR "ASSERT FAILED: manifest read produced empty json")
endif()

pyramid_manifest_field(_layout "${_json}" layout)
if("${_layout}" STREQUAL "")
    message(FATAL_ERROR "ASSERT FAILED: layout field missing")
endif()

# json codec .cpp sources: non-empty, every entry ends in .cpp and is anchored.
pyramid_manifest_sources(_codec_srcs "${_json}" json_codecs "${BINDINGS_DIR}" SUFFIX .cpp)
list(LENGTH _codec_srcs _n_codec)
if(NOT _n_codec GREATER 0)
    message(FATAL_ERROR "ASSERT FAILED: no json_codecs .cpp sources found")
endif()
foreach(_s IN LISTS _codec_srcs)
    string(FIND "${_s}" "${BINDINGS_DIR}/" _pos)
    if(NOT _pos EQUAL 0)
        message(FATAL_ERROR "ASSERT FAILED: codec source not anchored: ${_s}")
    endif()
    if(NOT "${_s}" MATCHES "\\.cpp$")
        message(FATAL_ERROR "ASSERT FAILED: codec source not .cpp: ${_s}")
    endif()
    if("${_s}" MATCHES "\\.hpp$")
        message(FATAL_ERROR "ASSERT FAILED: .hpp leaked into .cpp list: ${_s}")
    endif()
endforeach()

# cabi marshal sources. Generic layout does not (yet) emit C-ABI marshal, so
# only require them under the pyramid compatibility layout.
pyramid_manifest_sources(_marshal_srcs "${_json}" cabi "${BINDINGS_DIR}" SUFFIX _cabi_marshal.cpp)
list(LENGTH _marshal_srcs _n_marshal)
if(_layout STREQUAL "pyramid" AND NOT _n_marshal GREATER 0)
    message(FATAL_ERROR "ASSERT FAILED: no cabi marshal sources found under pyramid layout")
endif()

# Absent role returns empty, no error.
pyramid_manifest_sources(_none "${_json}" this_role_does_not_exist "${BINDINGS_DIR}")
list(LENGTH _none _n_none)
if(NOT _n_none EQUAL 0)
    message(FATAL_ERROR "ASSERT FAILED: absent role should yield empty list")
endif()

# codec plugins filtered by content type.
pyramid_manifest_plugins(_ptgts _psrcs _pctypes "${_json}" "${BINDINGS_DIR}" CONTENT_TYPE application/json)
list(LENGTH _ptgts _n_json_plugins)
foreach(_c IN LISTS _pctypes)
    if(NOT "${_c}" STREQUAL "application/json")
        message(FATAL_ERROR "ASSERT FAILED: content-type filter leaked: ${_c}")
    endif()
endforeach()

# marshal_modules: parallel module + source lists driving the per-module
# churn-isolation loop. One entry per data-model cabi marshal source; every
# source anchored under the bindings dir and ending _cabi_marshal.cpp.
pyramid_manifest_marshal_modules(_mods _msrcs "${_json}" "${BINDINGS_DIR}")
list(LENGTH _mods _n_mods)
list(LENGTH _msrcs _n_msrcs)
if(NOT _n_mods EQUAL _n_msrcs)
    message(FATAL_ERROR "ASSERT FAILED: marshal module/source lists not parallel (${_n_mods} vs ${_n_msrcs})")
endif()
if(_layout STREQUAL "pyramid")
    if(NOT _n_mods GREATER 0)
        message(FATAL_ERROR "ASSERT FAILED: no marshal_modules under pyramid layout")
    endif()
    if(NOT _n_mods EQUAL _n_marshal)
        message(FATAL_ERROR "ASSERT FAILED: marshal_modules count ${_n_mods} != cabi marshal source count ${_n_marshal}")
    endif()
endif()
foreach(_ms IN LISTS _msrcs)
    if(NOT "${_ms}" MATCHES "_cabi_marshal\\.cpp$")
        message(FATAL_ERROR "ASSERT FAILED: marshal source not _cabi_marshal.cpp: ${_ms}")
    endif()
    string(FIND "${_ms}" "${BINDINGS_DIR}/" _ms_pos)
    if(NOT _ms_pos EQUAL 0)
        message(FATAL_ERROR "ASSERT FAILED: marshal source not anchored: ${_ms}")
    endif()
endforeach()

message(STATUS "pyramid_manifest test OK: layout=${_layout} codecs=${_n_codec} marshal=${_n_marshal} marshal_modules=${_n_mods} json_plugins=${_n_json_plugins}")
