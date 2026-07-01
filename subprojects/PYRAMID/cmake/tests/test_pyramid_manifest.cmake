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

message(STATUS "pyramid_manifest test OK: layout=${_layout} codecs=${_n_codec} marshal=${_n_marshal} json_plugins=${_n_json_plugins}")
