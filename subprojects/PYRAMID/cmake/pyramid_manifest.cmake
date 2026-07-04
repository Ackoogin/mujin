# pyramid_manifest.cmake
#
# Reusable helpers for consuming the binding generator's `binding_manifest.json`
# instead of reverse-engineering artifact roles from `pyramid_*` filename globs.
#
# The generator (pim/generate_bindings.py) writes a manifest describing every
# generated artifact by ROLE:
#
#   {
#     "layout": "pyramid" | "generic",
#     "proto_import_roots": [...],
#     "proto_files": [...],                # relative to the import root
#     "types": ["..._types.hpp", ...],
#     "json_codecs": ["..._codec.cpp", "..._codec.hpp", ...],
#     "cabi": ["..._cabi.h", "..._cabi_marshal.cpp", ...],
#     "services": ["..._provided.cpp", ...],
#     "codec_plugins": [ {"target","source","content_type"}, ... ],
#     "flatbuffers_schemas": ["flatbuffers/cpp/....fbs", ...],
#     "protobuf_protos": [...],
#     "grpc_service_protos": [...]
#   }
#
# These helpers turn those roles into absolute source lists so build targets are
# created from declared roles rather than package-prefixed filename patterns.
#
# Requires CMake >= 3.19 for string(JSON ...).

# Read a manifest file into a JSON string variable. Sets the output to the empty
# string when the manifest is absent, so callers can fall back to globbing.
function(pyramid_manifest_read out_var manifest_path)
    if(EXISTS "${manifest_path}")
        file(READ "${manifest_path}" _json)
        set(${out_var} "${_json}" PARENT_SCOPE)
    else()
        set(${out_var} "" PARENT_SCOPE)
    endif()
endfunction()

# Return the string entries of a top-level array role, optionally filtered to a
# filename SUFFIX (e.g. ".cpp"), each anchored under base_dir.
#
#   pyramid_manifest_sources(SRCS "${json}" json_codecs "${bindings_dir}" SUFFIX .cpp)
function(pyramid_manifest_sources out_var json role base_dir)
    cmake_parse_arguments(_PMS "" "SUFFIX" "" ${ARGN})
    set(_result "")
    if(NOT json)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    string(JSON _len ERROR_VARIABLE _err LENGTH "${json}" "${role}")
    if(_err OR _len STREQUAL "" OR _len EQUAL 0)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    math(EXPR _last "${_len} - 1")
    foreach(_i RANGE ${_last})
        string(JSON _entry GET "${json}" "${role}" ${_i})
        if(_PMS_SUFFIX)
            string(LENGTH "${_entry}" _elen)
            string(LENGTH "${_PMS_SUFFIX}" _slen)
            if(_elen LESS _slen)
                continue()
            endif()
            math(EXPR _off "${_elen} - ${_slen}")
            string(SUBSTRING "${_entry}" ${_off} ${_slen} _tail)
            if(NOT _tail STREQUAL "${_PMS_SUFFIX}")
                continue()
            endif()
        endif()
        list(APPEND _result "${base_dir}/${_entry}")
    endforeach()
    set(${out_var} "${_result}" PARENT_SCOPE)
endfunction()

# Return three parallel lists describing codec plugins: targets, sources
# (anchored under base_dir), and content types. Optionally filter by
# CONTENT_TYPE (e.g. "application/json").
#
#   pyramid_manifest_plugins(TGTS SRCS CTYPES "${json}" "${dir}" CONTENT_TYPE application/json)
function(pyramid_manifest_plugins out_targets out_sources out_content_types json base_dir)
    cmake_parse_arguments(_PMP "" "CONTENT_TYPE" "" ${ARGN})
    set(_targets "")
    set(_sources "")
    set(_ctypes "")
    if(NOT json)
        set(${out_targets} "" PARENT_SCOPE)
        set(${out_sources} "" PARENT_SCOPE)
        set(${out_content_types} "" PARENT_SCOPE)
        return()
    endif()

    string(JSON _len ERROR_VARIABLE _err LENGTH "${json}" "codec_plugins")
    if(_err OR _len STREQUAL "" OR _len EQUAL 0)
        set(${out_targets} "" PARENT_SCOPE)
        set(${out_sources} "" PARENT_SCOPE)
        set(${out_content_types} "" PARENT_SCOPE)
        return()
    endif()

    math(EXPR _last "${_len} - 1")
    foreach(_i RANGE ${_last})
        string(JSON _obj GET "${json}" "codec_plugins" ${_i})
        string(JSON _target GET "${_obj}" "target")
        string(JSON _source GET "${_obj}" "source")
        string(JSON _ctype GET "${_obj}" "content_type")
        if(_PMP_CONTENT_TYPE AND NOT _ctype STREQUAL "${_PMP_CONTENT_TYPE}")
            continue()
        endif()
        list(APPEND _targets "${_target}")
        list(APPEND _sources "${base_dir}/${_source}")
        list(APPEND _ctypes "${_ctype}")
    endforeach()
    set(${out_targets} "${_targets}" PARENT_SCOPE)
    set(${out_sources} "${_sources}" PARENT_SCOPE)
    set(${out_content_types} "${_ctypes}" PARENT_SCOPE)
endfunction()

# Return two parallel lists for the per-module C-ABI marshal sources: the module
# names and the sources (anchored under base_dir), from the `marshal_modules`
# role. Lets the churn-isolation loop read module identity from the manifest
# instead of re-parsing it out of `pyramid_data_model_<module>_cabi_marshal.cpp`
# filenames.
function(pyramid_manifest_marshal_modules out_modules out_sources json base_dir)
    set(_modules "")
    set(_sources "")
    if(NOT json)
        set(${out_modules} "" PARENT_SCOPE)
        set(${out_sources} "" PARENT_SCOPE)
        return()
    endif()

    string(JSON _len ERROR_VARIABLE _err LENGTH "${json}" "marshal_modules")
    if(_err OR _len STREQUAL "" OR _len EQUAL 0)
        set(${out_modules} "" PARENT_SCOPE)
        set(${out_sources} "" PARENT_SCOPE)
        return()
    endif()

    math(EXPR _last "${_len} - 1")
    foreach(_i RANGE ${_last})
        string(JSON _obj GET "${json}" "marshal_modules" ${_i})
        string(JSON _module GET "${_obj}" "module")
        string(JSON _source GET "${_obj}" "source")
        list(APPEND _modules "${_module}")
        list(APPEND _sources "${base_dir}/${_source}")
    endforeach()
    set(${out_modules} "${_modules}" PARENT_SCOPE)
    set(${out_sources} "${_sources}" PARENT_SCOPE)
endfunction()

# Return a scalar top-level string field (e.g. "layout"), or empty if absent.
function(pyramid_manifest_field out_var json field)
    if(NOT json)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()
    string(JSON _v ERROR_VARIABLE _err GET "${json}" "${field}")
    if(_err)
        set(${out_var} "" PARENT_SCOPE)
    else()
        set(${out_var} "${_v}" PARENT_SCOPE)
    endif()
endfunction()
