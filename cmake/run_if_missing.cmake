if(NOT DEFINED OUTPUT)
  message(FATAL_ERROR "run_if_missing.cmake requires -DOUTPUT=<path>")
endif()

if(NOT DEFINED RUN_COMMAND)
  message(FATAL_ERROR "run_if_missing.cmake requires -DRUN_COMMAND=<command list>")
endif()

if(EXISTS "${OUTPUT}")
  message(STATUS "Refreshing: ${OUTPUT}")
  file(REMOVE "${OUTPUT}")
endif()

set(_working_directory_args)
if(DEFINED WORKING_DIRECTORY AND NOT "${WORKING_DIRECTORY}" STREQUAL "")
  set(_working_directory_args WORKING_DIRECTORY "${WORKING_DIRECTORY}")
endif()

execute_process(
  COMMAND ${RUN_COMMAND}
  ${_working_directory_args}
  RESULT_VARIABLE _result)
if(NOT _result EQUAL 0)
  message(FATAL_ERROR "Command failed with exit code ${_result}: ${RUN_COMMAND}")
endif()

if(NOT EXISTS "${OUTPUT}")
  message(FATAL_ERROR "Command completed but did not produce expected output: ${OUTPUT}")
endif()
