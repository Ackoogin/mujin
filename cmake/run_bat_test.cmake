# run_bat_test.cmake -- CTest wrapper for .bat test scripts on Windows.
#
# CTest's add_test() quotes each argument individually, which breaks
# cmd.exe /c parsing.  This wrapper uses execute_process() which
# constructs the command line correctly for Windows.
#
# Required: -DSCRIPT=path/to/script.bat
# Optional: -DSERVER_BIN=...  -DCLIENT_BIN=...  -DBRIDGE_BIN=...
#           -DAPP_BIN=...  -DPORT=...  -DTIMEOUT_VAL=...
#           -DBUILD_DIR=...  -DBUILD_CONFIG=...

if(NOT DEFINED SCRIPT)
  message(FATAL_ERROR "SCRIPT not defined")
endif()

string(REPLACE "/" "\\" NATIVE_SCRIPT "${SCRIPT}")
get_filename_component(SCRIPT_DIR "${SCRIPT}" DIRECTORY)
get_filename_component(PYRAMID_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
get_filename_component(WORKSPACE_ROOT "${PYRAMID_ROOT}/../.." ABSOLUTE)

set(extra_args)
if(DEFINED SERVER_BIN)
  list(APPEND extra_args --server-bin "${SERVER_BIN}")
endif()
if(DEFINED BRIDGE_BIN)
  list(APPEND extra_args --bridge-bin "${BRIDGE_BIN}")
endif()
if(DEFINED CLIENT_BIN)
  list(APPEND extra_args --client-bin "${CLIENT_BIN}")
endif()
if(DEFINED APP_BIN)
  list(APPEND extra_args --app-bin "${APP_BIN}")
endif()
if(DEFINED STUB_BIN)
  list(APPEND extra_args --stub-bin "${STUB_BIN}")
endif()
if(DEFINED EVIDENCE_BIN)
  list(APPEND extra_args --evidence-bin "${EVIDENCE_BIN}")
endif()
if(DEFINED PORT)
  list(APPEND extra_args --port "${PORT}")
endif()
if(DEFINED TIMEOUT_VAL)
  list(APPEND extra_args --timeout "${TIMEOUT_VAL}")
endif()
if(DEFINED SCRIPT_ARGS)
  string(REPLACE "|" ";" parsed_script_args "${SCRIPT_ARGS}")
  list(APPEND extra_args ${parsed_script_args})
endif()

if(NOT DEFINED BUILD_DIR)
  foreach(candidate SERVER_BIN BRIDGE_BIN CLIENT_BIN APP_BIN STUB_BIN EVIDENCE_BIN)
    if(DEFINED ${candidate})
      file(TO_CMAKE_PATH "${${candidate}}" candidate_path)
      if(candidate_path MATCHES "^(.*)/subprojects/PYRAMID/.*$")
        set(BUILD_DIR "${CMAKE_MATCH_1}")
        break()
      endif()
    endif()
  endforeach()
endif()

if(NOT DEFINED BUILD_CONFIG)
  foreach(candidate SERVER_BIN BRIDGE_BIN CLIENT_BIN APP_BIN STUB_BIN EVIDENCE_BIN)
    if(DEFINED ${candidate})
      file(TO_CMAKE_PATH "${${candidate}}" candidate_path)
      if(candidate_path MATCHES "/(Debug|Release|RelWithDebInfo|MinSizeRel)/[^/]+(\\.exe)?$")
        set(BUILD_CONFIG "${CMAKE_MATCH_1}")
        break()
      endif()
    endif()
  endforeach()
endif()

if(NOT DEFINED BUILD_DIR)
  set(BUILD_DIR "${WORKSPACE_ROOT}/build")
endif()
if(NOT DEFINED BUILD_CONFIG)
  set(BUILD_CONFIG "Release")
endif()

file(TO_CMAKE_PATH "${BUILD_DIR}" BUILD_DIR_CMAKE)
string(REPLACE "/" "\\" BUILD_DIR_NATIVE "${BUILD_DIR_CMAKE}")

execute_process(
  COMMAND ${CMAKE_COMMAND} -E env
          PYRAMID_ROOT=${PYRAMID_ROOT}
          WORKSPACE_ROOT=${WORKSPACE_ROOT}
          PYRAMID_BUILD_DIR=${BUILD_DIR_NATIVE}
          PYRAMID_BUILD_CONFIG=${BUILD_CONFIG}
          cmd /c call "${NATIVE_SCRIPT}" ${extra_args}
  RESULT_VARIABLE result
)

if(NOT result EQUAL 0)
  message(FATAL_ERROR "Test failed with exit code ${result}")
endif()
