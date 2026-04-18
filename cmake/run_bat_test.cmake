# run_bat_test.cmake — CTest wrapper for .bat test scripts on Windows.
#
# CTest's add_test() quotes each argument individually, which breaks
# cmd.exe /c parsing.  This wrapper uses execute_process() which
# constructs the command line correctly for Windows.
#
# Required: -DSCRIPT=path/to/script.bat
# Optional: -DSERVER_BIN=...  -DCLIENT_BIN=...  -DBRIDGE_BIN=...
#           -DAPP_BIN=...  -DPORT=...  -DTIMEOUT_VAL=...

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

execute_process(
  COMMAND ${CMAKE_COMMAND} -E env
          PYRAMID_ROOT=${PYRAMID_ROOT}
          WORKSPACE_ROOT=${WORKSPACE_ROOT}
          cmd /c call "${NATIVE_SCRIPT}" ${extra_args}
  RESULT_VARIABLE result
)

if(NOT result EQUAL 0)
  message(FATAL_ERROR "Test failed with exit code ${result}")
endif()
