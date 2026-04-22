set(PYRAMID_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")

set(PYRAMID_PROTO_DIR "${PYRAMID_ROOT_DIR}/proto")
if(EXISTS "${PYRAMID_PROTO_DIR}")
    option(PYRAMID_ENABLE_PROTOBUF "Enable protobuf codecs for PYRAMID" ON)
else()
    option(PYRAMID_ENABLE_PROTOBUF "Enable protobuf codecs for PYRAMID" OFF)
    message(STATUS "PYRAMID: proto directory not found, disabling protobuf support")
endif()
option(PYRAMID_ENABLE_FLATBUFFERS "Enable FlatBuffers codec/transport for PYRAMID" ON)
option(PYRAMID_ENABLE_GRPC "Enable gRPC transport projection for PYRAMID" OFF)
option(PYRAMID_ENABLE_ROS2 "Enable ROS2 semantic transport projection for PYRAMID" ON)
set(PYRAMID_GRPC_TAG "v1.72.0" CACHE STRING "gRPC git tag to fetch when PYRAMID_ENABLE_GRPC is ON")

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_library(nlohmann_json INTERFACE)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
target_include_directories(nlohmann_json
    INTERFACE
        "${PYRAMID_ROOT_DIR}/core/external"
)

function(pyramid_fetch_grpc_dependency)
    if(TARGET gRPC::grpc++ OR TARGET grpc++)
        return()
    endif()

    FetchContent_Declare(
        grpc
        GIT_REPOSITORY https://github.com/grpc/grpc.git
        GIT_TAG        ${PYRAMID_GRPC_TAG}
        GIT_SHALLOW    TRUE
    )
    set(gRPC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(gRPC_BUILD_CODEGEN ON CACHE BOOL "" FORCE)
    set(gRPC_INSTALL OFF CACHE BOOL "" FORCE)
    set(gRPC_MSVC_STATIC_RUNTIME OFF CACHE BOOL "" FORCE)
    set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(protobuf_MSVC_STATIC_RUNTIME OFF CACHE BOOL "" FORCE)
    set(gRPC_DOWNLOAD_ARCHIVES ON CACHE BOOL "" FORCE)
    set(gRPC_ABSL_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_CARES_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_PROTOBUF_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_RE2_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_SSL_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_ZLIB_PROVIDER module CACHE STRING "" FORCE)
    FetchContent_MakeAvailable(grpc)

    if(TARGET grpc++ AND NOT TARGET gRPC::grpc++)
        add_library(gRPC::grpc++ ALIAS grpc++)
    endif()
endfunction()

set(PYRAMID_CPP_BINDINGS_DIR
    "${CMAKE_BINARY_DIR}/generated/pyramid_cpp_bindings"
    CACHE PATH "Directory containing or receiving generated PYRAMID C++ bindings")
set(PYRAMID_GENERATE_CPP_BINDINGS_DEFAULT ON)
if(NOT EXISTS "${PYRAMID_PROTO_DIR}")
    set(PYRAMID_GENERATE_CPP_BINDINGS_DEFAULT OFF)
endif()
option(PYRAMID_GENERATE_CPP_BINDINGS
    "Generate PYRAMID C++ bindings from proto contracts during configure/build"
    ${PYRAMID_GENERATE_CPP_BINDINGS_DEFAULT})
set(PYRAMID_CPP_GENERATOR_BACKENDS json)
if(PYRAMID_ENABLE_FLATBUFFERS)
    list(APPEND PYRAMID_CPP_GENERATOR_BACKENDS flatbuffers)
endif()
if(PYRAMID_ENABLE_PROTOBUF)
    list(APPEND PYRAMID_CPP_GENERATOR_BACKENDS protobuf)
endif()
if(PYRAMID_ENABLE_GRPC)
    list(APPEND PYRAMID_CPP_GENERATOR_BACKENDS grpc)
endif()
if(PYRAMID_ENABLE_ROS2)
    list(APPEND PYRAMID_CPP_GENERATOR_BACKENDS ros2)
endif()
list(JOIN PYRAMID_CPP_GENERATOR_BACKENDS "," PYRAMID_CPP_GENERATOR_BACKENDS_ARG)

file(GLOB_RECURSE PYRAMID_PROTO_INPUTS CONFIGURE_DEPENDS
    "${PYRAMID_PROTO_DIR}/*.proto")

file(GLOB_RECURSE PYRAMID_BINDING_GENERATOR_INPUTS CONFIGURE_DEPENDS
    "${PYRAMID_ROOT_DIR}/pim/*.py"
    "${PYRAMID_ROOT_DIR}/pim/backends/*.py")

function(pyramid_glob_generated out_var)
    file(GLOB_RECURSE _pyramid_globbed CONFIGURE_DEPENDS ${ARGN})
    list(SORT _pyramid_globbed)
    set(${out_var} ${_pyramid_globbed} PARENT_SCOPE)
endfunction()

function(pyramid_require_generated var_name description)
    if(NOT DEFINED ${var_name} OR "${${var_name}}" STREQUAL "")
        message(FATAL_ERROR
            "No ${description} found. Run the PYRAMID binding generator, "
            "point the relevant binding directory at delivered bindings, "
            "or enable PYRAMID_GENERATE_CPP_BINDINGS.")
    endif()
endfunction()

if(PYRAMID_GENERATE_CPP_BINDINGS)
    if(NOT PYRAMID_PROTO_INPUTS)
        message(FATAL_ERROR
            "PYRAMID_GENERATE_CPP_BINDINGS is ON, but no proto files were "
            "found under ${PYRAMID_PROTO_DIR}.")
    endif()
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E rm -rf "${PYRAMID_CPP_BINDINGS_DIR}"
        COMMAND_ERROR_IS_FATAL ANY)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}"
                "${PYRAMID_ROOT_DIR}/pim/generate_bindings.py"
                "${PYRAMID_PROTO_DIR}"
                "${PYRAMID_CPP_BINDINGS_DIR}"
                --languages cpp
                --backends "${PYRAMID_CPP_GENERATOR_BACKENDS_ARG}"
        COMMAND_ERROR_IS_FATAL ANY)
endif()

pyramid_glob_generated(PYRAMID_CPP_BINDING_OUTPUTS
    "${PYRAMID_CPP_BINDINGS_DIR}/*.cpp"
    "${PYRAMID_CPP_BINDINGS_DIR}/*.fbs"
    "${PYRAMID_CPP_BINDINGS_DIR}/*.h"
    "${PYRAMID_CPP_BINDINGS_DIR}/*.hpp")
pyramid_require_generated(PYRAMID_CPP_BINDING_OUTPUTS
    "PYRAMID C++ binding artifacts")
set_source_files_properties(${PYRAMID_CPP_BINDING_OUTPUTS}
    PROPERTIES GENERATED TRUE)

set(PYRAMID_CPP_BINDINGS_STAMP
    "${PYRAMID_CPP_BINDINGS_DIR}/.pyramid_cpp_bindings.stamp")
if(PYRAMID_GENERATE_CPP_BINDINGS)
    add_custom_command(
        OUTPUT "${PYRAMID_CPP_BINDINGS_STAMP}"
        COMMAND "${CMAKE_COMMAND}" -E rm -rf "${PYRAMID_CPP_BINDINGS_DIR}"
        COMMAND "${Python3_EXECUTABLE}"
                "${PYRAMID_ROOT_DIR}/pim/generate_bindings.py"
                "${PYRAMID_PROTO_DIR}"
                "${PYRAMID_CPP_BINDINGS_DIR}"
                --languages cpp
                --backends "${PYRAMID_CPP_GENERATOR_BACKENDS_ARG}"
        COMMAND "${CMAKE_COMMAND}" -E touch "${PYRAMID_CPP_BINDINGS_STAMP}"
        DEPENDS
            ${PYRAMID_BINDING_GENERATOR_INPUTS}
            ${PYRAMID_PROTO_INPUTS}
        COMMENT "Generating PYRAMID C++ bindings (${PYRAMID_CPP_GENERATOR_BACKENDS_ARG})"
        VERBATIM
    )
    add_custom_target(pyramid_cpp_bindings_codegen
        DEPENDS "${PYRAMID_CPP_BINDINGS_STAMP}")
else()
    add_custom_target(pyramid_cpp_bindings_codegen)
endif()

add_library(pyramid_flatbuffers_support INTERFACE)

if(PYRAMID_ENABLE_FLATBUFFERS)
    set(PYRAMID_FLATBUFFERS_SCHEMA_DIR "${PYRAMID_CPP_BINDINGS_DIR}/flatbuffers/cpp")
    set(PYRAMID_FLATBUFFERS_GEN_DIR "${CMAKE_BINARY_DIR}/generated/flatbuffers/cpp")
    pyramid_glob_generated(PYRAMID_FLATBUFFERS_SCHEMAS
        "${PYRAMID_FLATBUFFERS_SCHEMA_DIR}/pyramid_services_*.fbs")
    pyramid_require_generated(PYRAMID_FLATBUFFERS_SCHEMAS
        "generated FlatBuffers service schemas")
    set(PYRAMID_FLATBUFFERS_OUTPUTS)
    foreach(PYRAMID_FLATBUFFERS_SCHEMA IN LISTS PYRAMID_FLATBUFFERS_SCHEMAS)
        get_filename_component(PYRAMID_FLATBUFFERS_STEM "${PYRAMID_FLATBUFFERS_SCHEMA}" NAME_WE)
        set(PYRAMID_FLATBUFFERS_OUTPUT "${PYRAMID_FLATBUFFERS_GEN_DIR}/${PYRAMID_FLATBUFFERS_STEM}_generated.h")
        add_custom_command(
            OUTPUT "${PYRAMID_FLATBUFFERS_OUTPUT}"
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${PYRAMID_FLATBUFFERS_GEN_DIR}"
            COMMAND $<TARGET_FILE:flatc> --cpp --gen-object-api -o "${PYRAMID_FLATBUFFERS_GEN_DIR}" "${PYRAMID_FLATBUFFERS_SCHEMA}"
            DEPENDS flatc pyramid_cpp_bindings_codegen "${PYRAMID_FLATBUFFERS_SCHEMA}"
            COMMENT "Generating FlatBuffers header for ${PYRAMID_FLATBUFFERS_STEM}.fbs"
            VERBATIM
        )
        list(APPEND PYRAMID_FLATBUFFERS_OUTPUTS "${PYRAMID_FLATBUFFERS_OUTPUT}")
    endforeach()

    add_custom_target(pyramid_flatbuffers_codegen DEPENDS ${PYRAMID_FLATBUFFERS_OUTPUTS})
    add_dependencies(pyramid_flatbuffers_codegen pyramid_cpp_bindings_codegen)

    add_dependencies(pyramid_flatbuffers_support pyramid_flatbuffers_codegen)
    target_include_directories(pyramid_flatbuffers_support
        INTERFACE
            "${PYRAMID_FLATBUFFERS_GEN_DIR}"
            "${flatbuffers_SOURCE_DIR}/include"
            "${PYRAMID_FLATBUFFERS_SCHEMA_DIR}"
    )
    target_compile_definitions(pyramid_flatbuffers_support
        INTERFACE PYRAMID_ENABLE_FLATBUFFERS=1)
endif()

pyramid_glob_generated(_pyramid_generated_codecs_srcs
    "${PYRAMID_CPP_BINDINGS_DIR}/pyramid_data_model_*_codec.cpp")
if(PYRAMID_ENABLE_FLATBUFFERS)
    pyramid_glob_generated(_pyramid_generated_flatbuffers_codec_srcs
        "${PYRAMID_CPP_BINDINGS_DIR}/flatbuffers/cpp/pyramid_services_*_flatbuffers_codec.cpp")
    list(APPEND _pyramid_generated_codecs_srcs
        ${_pyramid_generated_flatbuffers_codec_srcs})
    unset(_pyramid_generated_flatbuffers_codec_srcs)
endif()
pyramid_require_generated(_pyramid_generated_codecs_srcs
    "generated PYRAMID codec sources")

add_library(pyramid_generated_codecs STATIC ${_pyramid_generated_codecs_srcs})
unset(_pyramid_generated_codecs_srcs)
add_dependencies(pyramid_generated_codecs pyramid_cpp_bindings_codegen)

if(PYRAMID_ENABLE_FLATBUFFERS)
    add_dependencies(pyramid_generated_codecs pyramid_flatbuffers_codegen)
    target_link_libraries(pyramid_generated_codecs PUBLIC pyramid_flatbuffers_support)
endif()
target_link_libraries(pyramid_generated_codecs PUBLIC nlohmann_json::nlohmann_json)
target_include_directories(pyramid_generated_codecs
    PUBLIC
        "${PYRAMID_CPP_BINDINGS_DIR}"
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated"
)

if(PYRAMID_ENABLE_PROTOBUF)
    if(NOT TARGET protobuf::libprotobuf)
        find_package(Protobuf CONFIG QUIET)
    endif()

    if(NOT Protobuf_FOUND AND NOT TARGET protobuf::libprotobuf AND PYRAMID_ENABLE_GRPC)
        pyramid_fetch_grpc_dependency()
    endif()

    if(NOT Protobuf_FOUND AND NOT TARGET protobuf::libprotobuf)
        FetchContent_Declare(
            protobuf
            GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
            GIT_TAG        v3.21.12
            GIT_SHALLOW    TRUE
            SOURCE_SUBDIR  cmake
        )
        set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
        set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
        set(protobuf_MSVC_STATIC_RUNTIME OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(protobuf)
    endif()

    if(TARGET libprotobuf AND NOT TARGET protobuf::libprotobuf)
        add_library(protobuf::libprotobuf ALIAS libprotobuf)
    endif()

    if(TARGET protobuf::protoc)
        set(PYRAMID_PROTOC "$<TARGET_FILE:protobuf::protoc>")
    else()
        find_program(PYRAMID_PROTOC NAMES protoc REQUIRED)
    endif()

    set(PYRAMID_PROTOBUF_GEN_DIR "${CMAKE_BINARY_DIR}/generated/protobuf")
    set(PYRAMID_PROTO_IMPORT_DIRS "${PYRAMID_PROTO_DIR}")
    if(DEFINED protobuf_SOURCE_DIR AND EXISTS "${protobuf_SOURCE_DIR}/src/google/protobuf/timestamp.proto")
        list(APPEND PYRAMID_PROTO_IMPORT_DIRS "${protobuf_SOURCE_DIR}/src")
    endif()
    if(DEFINED grpc_SOURCE_DIR AND EXISTS "${grpc_SOURCE_DIR}/third_party/protobuf/src/google/protobuf/timestamp.proto")
        list(APPEND PYRAMID_PROTO_IMPORT_DIRS "${grpc_SOURCE_DIR}/third_party/protobuf/src")
    endif()
    set(PYRAMID_PROTOC_INCLUDE_ARGS)
    foreach(PYRAMID_PROTO_IMPORT_DIR IN LISTS PYRAMID_PROTO_IMPORT_DIRS)
        list(APPEND PYRAMID_PROTOC_INCLUDE_ARGS -I "${PYRAMID_PROTO_IMPORT_DIR}")
    endforeach()
    file(GLOB_RECURSE PYRAMID_PROTOBUF_PROTOS CONFIGURE_DEPENDS
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/*.proto")
    if(NOT PYRAMID_PROTOBUF_PROTOS)
        message(FATAL_ERROR
            "PYRAMID_ENABLE_PROTOBUF is ON, but no data-model proto files "
            "were found under ${PYRAMID_PROTO_DIR}/pyramid/data_model.")
    endif()

    set(PYRAMID_PROTOBUF_OUTPUTS)
    set(PYRAMID_PROTOBUF_SRCS)
    foreach(PYRAMID_PROTO_FILE IN LISTS PYRAMID_PROTOBUF_PROTOS)
        file(RELATIVE_PATH PYRAMID_PROTO_REL "${PYRAMID_PROTO_DIR}" "${PYRAMID_PROTO_FILE}")
        string(REPLACE ".proto" ".pb.cc" PYRAMID_PROTO_CC "${PYRAMID_PROTO_REL}")
        string(REPLACE ".proto" ".pb.h" PYRAMID_PROTO_H "${PYRAMID_PROTO_REL}")
        get_filename_component(PYRAMID_PROTO_OUT_DIR
            "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_PROTO_REL}" DIRECTORY)
        add_custom_command(
            OUTPUT
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_PROTO_CC}"
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_PROTO_H}"
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${PYRAMID_PROTO_OUT_DIR}"
            COMMAND "${PYRAMID_PROTOC}"
                    --cpp_out "${PYRAMID_PROTOBUF_GEN_DIR}"
                    ${PYRAMID_PROTOC_INCLUDE_ARGS}
                    "${PYRAMID_PROTO_FILE}"
            DEPENDS "${PYRAMID_PROTO_FILE}"
            COMMENT "Generating protobuf C++ sources for ${PYRAMID_PROTO_REL}"
            VERBATIM
        )
        list(APPEND PYRAMID_PROTOBUF_OUTPUTS
            "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_PROTO_CC}"
            "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_PROTO_H}"
        )
        list(APPEND PYRAMID_PROTOBUF_SRCS
            "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_PROTO_CC}"
        )
    endforeach()

    add_custom_target(pyramid_protobuf_codegen DEPENDS ${PYRAMID_PROTOBUF_OUTPUTS})

    pyramid_glob_generated(PYRAMID_PROTOBUF_SUPPORT_SRCS
        "${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp/*.cpp")
    pyramid_require_generated(PYRAMID_PROTOBUF_SUPPORT_SRCS
        "PYRAMID protobuf support sources")
    add_library(pyramid_protobuf_support STATIC
        ${PYRAMID_PROTOBUF_SUPPORT_SRCS}
        ${PYRAMID_PROTOBUF_SRCS}
    )
    add_dependencies(pyramid_protobuf_support pyramid_protobuf_codegen)
    target_link_libraries(pyramid_protobuf_support
        PUBLIC
            nlohmann_json::nlohmann_json
            protobuf::libprotobuf
    )
    target_include_directories(pyramid_protobuf_support
        PUBLIC
            "${PYRAMID_CPP_BINDINGS_DIR}"
            "${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp"
            "${PYRAMID_PROTOBUF_GEN_DIR}"
    )
    target_compile_definitions(pyramid_protobuf_support
        PUBLIC PYRAMID_ENABLE_PROTOBUF=1)

    target_link_libraries(pyramid_generated_codecs
        PUBLIC
            pyramid_protobuf_support
    )
endif()

if(PYRAMID_ENABLE_GRPC)
    if(NOT PYRAMID_ENABLE_PROTOBUF)
        message(FATAL_ERROR "PYRAMID_ENABLE_GRPC requires PYRAMID_ENABLE_PROTOBUF")
    endif()

    if(NOT TARGET gRPC::grpc++ AND NOT TARGET grpc++)
        find_package(gRPC CONFIG QUIET)
    endif()

    if(NOT gRPC_FOUND AND NOT TARGET gRPC::grpc++ AND NOT TARGET grpc++)
        pyramid_fetch_grpc_dependency()
    endif()

    if(TARGET grpc++ AND NOT TARGET gRPC::grpc++)
        add_library(gRPC::grpc++ ALIAS grpc++)
    endif()

    if(TARGET gRPC::grpc_cpp_plugin)
        set(PYRAMID_GRPC_CPP_PLUGIN_ARG
            "--plugin=protoc-gen-grpc=$<SHELL_PATH:$<TARGET_FILE:gRPC::grpc_cpp_plugin>>")
    elseif(TARGET grpc_cpp_plugin)
        set(PYRAMID_GRPC_CPP_PLUGIN_ARG
            "--plugin=protoc-gen-grpc=$<SHELL_PATH:$<TARGET_FILE:grpc_cpp_plugin>>")
    else()
        find_program(PYRAMID_GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin)
        if(PYRAMID_GRPC_CPP_PLUGIN)
            set(PYRAMID_GRPC_CPP_PLUGIN_ARG
                "--plugin=protoc-gen-grpc=${PYRAMID_GRPC_CPP_PLUGIN}")
        endif()
    endif()

    if((NOT TARGET gRPC::grpc++ AND NOT TARGET grpc++) OR NOT PYRAMID_GRPC_CPP_PLUGIN_ARG)
        message(WARNING
            "PYRAMID_ENABLE_GRPC was requested, but gRPC or grpc_cpp_plugin "
            "was not found. Skipping Tactical Objects gRPC transport targets.")
    else()
        file(GLOB_RECURSE PYRAMID_GRPC_SERVICE_PROTOS CONFIGURE_DEPENDS
            "${PYRAMID_PROTO_DIR}/pyramid/components/*/services/*.proto")
        if(NOT PYRAMID_GRPC_SERVICE_PROTOS)
            message(FATAL_ERROR
                "PYRAMID_ENABLE_GRPC is ON, but no service proto files "
                "were found under ${PYRAMID_PROTO_DIR}/pyramid/components.")
        endif()

        set(PYRAMID_GRPC_OUTPUTS)
        set(PYRAMID_GRPC_SRCS)
        foreach(PYRAMID_GRPC_PROTO IN LISTS PYRAMID_GRPC_SERVICE_PROTOS)
            file(RELATIVE_PATH PYRAMID_GRPC_PROTO_REL "${PYRAMID_PROTO_DIR}" "${PYRAMID_GRPC_PROTO}")
            string(REPLACE ".proto" ".pb.cc" PYRAMID_GRPC_PB_CC "${PYRAMID_GRPC_PROTO_REL}")
            string(REPLACE ".proto" ".pb.h" PYRAMID_GRPC_PB_H "${PYRAMID_GRPC_PROTO_REL}")
            string(REPLACE ".proto" ".grpc.pb.cc" PYRAMID_GRPC_CC "${PYRAMID_GRPC_PROTO_REL}")
            string(REPLACE ".proto" ".grpc.pb.h" PYRAMID_GRPC_H "${PYRAMID_GRPC_PROTO_REL}")
            get_filename_component(
                PYRAMID_GRPC_OUT_DIR
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_PROTO_REL}"
                DIRECTORY
            )
            add_custom_command(
                OUTPUT
                    "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_PB_CC}"
                    "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_PB_H}"
                    "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_CC}"
                    "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_H}"
                COMMAND "${CMAKE_COMMAND}" -E make_directory "${PYRAMID_GRPC_OUT_DIR}"
                COMMAND "${PYRAMID_PROTOC}"
                        --cpp_out "${PYRAMID_PROTOBUF_GEN_DIR}"
                        --grpc_out "${PYRAMID_PROTOBUF_GEN_DIR}"
                        ${PYRAMID_GRPC_CPP_PLUGIN_ARG}
                        ${PYRAMID_PROTOC_INCLUDE_ARGS}
                        "${PYRAMID_GRPC_PROTO}"
                DEPENDS "${PYRAMID_GRPC_PROTO}" pyramid_protobuf_codegen
                COMMENT "Generating gRPC C++ sources for ${PYRAMID_GRPC_PROTO_REL}"
                VERBATIM
            )
            list(APPEND PYRAMID_GRPC_OUTPUTS
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_PB_CC}"
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_PB_H}"
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_CC}"
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_H}"
            )
            list(APPEND PYRAMID_GRPC_SRCS
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_PB_CC}"
                "${PYRAMID_PROTOBUF_GEN_DIR}/${PYRAMID_GRPC_CC}"
            )
        endforeach()

        add_custom_target(pyramid_grpc_codegen DEPENDS ${PYRAMID_GRPC_OUTPUTS})

        pyramid_glob_generated(PYRAMID_GRPC_TRANSPORT_SRCS
            "${PYRAMID_CPP_BINDINGS_DIR}/grpc/cpp/*_grpc_transport.cpp")
        pyramid_require_generated(PYRAMID_GRPC_TRANSPORT_SRCS
            "generated PYRAMID gRPC transport sources")
        add_library(pyramid_grpc_transport STATIC
            ${PYRAMID_GRPC_TRANSPORT_SRCS}
            ${PYRAMID_GRPC_SRCS}
        )
        add_dependencies(pyramid_grpc_transport
            pyramid_cpp_bindings_codegen
            pyramid_grpc_codegen
            pyramid_protobuf_codegen)
        target_link_libraries(pyramid_grpc_transport
            PUBLIC
                gRPC::grpc++
                protobuf::libprotobuf
                pcl_core
                pyramid_generated_codecs
        )
        target_include_directories(pyramid_grpc_transport
            PUBLIC
                "${PCL_ROOT_DIR}/include"
                "${PYRAMID_CPP_BINDINGS_DIR}"
                "${PYRAMID_CPP_BINDINGS_DIR}/grpc/cpp"
                "${PYRAMID_PROTOBUF_GEN_DIR}"
        )

        set(PYRAMID_GRPC_C_SHIM_DIR
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp")
        pyramid_glob_generated(PYRAMID_GRPC_C_SHIM_SRCS
            "${PYRAMID_GRPC_C_SHIM_DIR}/*_grpc_c_api*.cpp")
        pyramid_require_generated(PYRAMID_GRPC_C_SHIM_SRCS
            "generated PYRAMID gRPC C shim sources")
        add_library(pyramid_grpc_c_shim SHARED
            ${PYRAMID_GRPC_C_SHIM_SRCS}
        )
        add_dependencies(pyramid_grpc_c_shim
            pyramid_cpp_bindings_codegen
            pyramid_grpc_codegen
            pyramid_protobuf_codegen)
        target_link_libraries(pyramid_grpc_c_shim
            PUBLIC
                gRPC::grpc++
                protobuf::libprotobuf
                pyramid_grpc_transport
                pyramid_protobuf_support
        )
        target_include_directories(pyramid_grpc_c_shim
            PUBLIC
                "${PYRAMID_GRPC_C_SHIM_DIR}"
                "${PYRAMID_CPP_BINDINGS_DIR}/grpc/cpp"
                "${PYRAMID_PROTOBUF_GEN_DIR}"
        )
    endif()
endif()

if(PYRAMID_ENABLE_ROS2)
    set(PYRAMID_ROS2_SUPPORT_DIR
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/ros2/cpp")
    pyramid_glob_generated(PYRAMID_ROS2_TRANSPORT_SRCS
        "${PYRAMID_ROS2_SUPPORT_DIR}/*.cpp")
    pyramid_require_generated(PYRAMID_ROS2_TRANSPORT_SRCS
        "generated PYRAMID ROS2 transport sources")
    add_library(pyramid_ros2_transport STATIC ${PYRAMID_ROS2_TRANSPORT_SRCS})
    add_dependencies(pyramid_ros2_transport pyramid_cpp_bindings_codegen)
    target_link_libraries(pyramid_ros2_transport
        PUBLIC
            pcl_core
    )
    target_include_directories(pyramid_ros2_transport
        PUBLIC
            "${PCL_ROOT_DIR}/include"
            "${PYRAMID_ROS2_SUPPORT_DIR}"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated"
    )
endif()

add_subdirectory(core)
add_subdirectory(tactical_objects)
add_subdirectory(pyramid_bridge)
add_subdirectory(tests)
