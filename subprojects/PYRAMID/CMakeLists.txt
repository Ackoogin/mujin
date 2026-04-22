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

add_library(pyramid_flatbuffers_support INTERFACE)

if(PYRAMID_ENABLE_FLATBUFFERS)
    set(PYRAMID_FLATBUFFERS_SCHEMA_DIR "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/flatbuffers/cpp")
    set(PYRAMID_FLATBUFFERS_GEN_DIR "${CMAKE_BINARY_DIR}/generated/flatbuffers/cpp")
    set(PYRAMID_FLATBUFFERS_SCHEMAS
        "${PYRAMID_FLATBUFFERS_SCHEMA_DIR}/pyramid_services_autonomy_backend.fbs"
        "${PYRAMID_FLATBUFFERS_SCHEMA_DIR}/pyramid_services_tactical_objects.fbs")
    set(PYRAMID_FLATBUFFERS_OUTPUTS)
    foreach(PYRAMID_FLATBUFFERS_SCHEMA IN LISTS PYRAMID_FLATBUFFERS_SCHEMAS)
        get_filename_component(PYRAMID_FLATBUFFERS_STEM "${PYRAMID_FLATBUFFERS_SCHEMA}" NAME_WE)
        set(PYRAMID_FLATBUFFERS_OUTPUT "${PYRAMID_FLATBUFFERS_GEN_DIR}/${PYRAMID_FLATBUFFERS_STEM}_generated.h")
        add_custom_command(
            OUTPUT "${PYRAMID_FLATBUFFERS_OUTPUT}"
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${PYRAMID_FLATBUFFERS_GEN_DIR}"
            COMMAND $<TARGET_FILE:flatc> --cpp --gen-object-api -o "${PYRAMID_FLATBUFFERS_GEN_DIR}" "${PYRAMID_FLATBUFFERS_SCHEMA}"
            DEPENDS flatc "${PYRAMID_FLATBUFFERS_SCHEMA}"
            COMMENT "Generating FlatBuffers header for ${PYRAMID_FLATBUFFERS_STEM}.fbs"
            VERBATIM
        )
        list(APPEND PYRAMID_FLATBUFFERS_OUTPUTS "${PYRAMID_FLATBUFFERS_OUTPUT}")
    endforeach()

    add_custom_target(pyramid_flatbuffers_codegen DEPENDS ${PYRAMID_FLATBUFFERS_OUTPUTS})

    add_dependencies(pyramid_flatbuffers_support pyramid_flatbuffers_codegen)
    target_include_directories(pyramid_flatbuffers_support
        INTERFACE
            "${PYRAMID_FLATBUFFERS_GEN_DIR}"
            "${flatbuffers_SOURCE_DIR}/include"
            "${PYRAMID_FLATBUFFERS_SCHEMA_DIR}"
    )
endif()

set(_pyramid_generated_codecs_srcs
    "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/pyramid_data_model_autonomy_codec.cpp"
    "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/pyramid_data_model_base_codec.cpp"
    "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/pyramid_data_model_common_codec.cpp"
    "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/pyramid_data_model_tactical_codec.cpp"
)
if(PYRAMID_ENABLE_FLATBUFFERS)
    list(APPEND _pyramid_generated_codecs_srcs
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/flatbuffers/cpp/pyramid_services_autonomy_backend_flatbuffers_codec.cpp"
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.cpp"
    )
endif()

add_library(pyramid_generated_codecs STATIC ${_pyramid_generated_codecs_srcs})
unset(_pyramid_generated_codecs_srcs)

if(PYRAMID_ENABLE_FLATBUFFERS)
    add_dependencies(pyramid_generated_codecs pyramid_flatbuffers_codegen)
    target_link_libraries(pyramid_generated_codecs PUBLIC pyramid_flatbuffers_support)
endif()
target_link_libraries(pyramid_generated_codecs PUBLIC nlohmann_json::nlohmann_json)
target_include_directories(pyramid_generated_codecs
    PUBLIC
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
    set(PYRAMID_PROTOBUF_PROTOS
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/base.proto"
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/common.proto"
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/tactical.proto"
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/autonomy.proto"
    )

    set(PYRAMID_PROTOBUF_OUTPUTS)
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
    endforeach()

    add_custom_target(pyramid_protobuf_codegen DEPENDS ${PYRAMID_PROTOBUF_OUTPUTS})

    add_library(pyramid_protobuf_support STATIC
        "${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp/pyramid_services_tactical_objects_protobuf_codec.cpp"
        "${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp/pyramid_services_tactical_objects_protobuf_shim.cpp"
        "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/data_model/base.pb.cc"
        "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/data_model/common.pb.cc"
        "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/data_model/tactical.pb.cc"
        "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/data_model/autonomy.pb.cc"
    )
    add_dependencies(pyramid_protobuf_support pyramid_protobuf_codegen)
    target_link_libraries(pyramid_protobuf_support
        PUBLIC
            nlohmann_json::nlohmann_json
            protobuf::libprotobuf
    )
    target_include_directories(pyramid_protobuf_support
        PUBLIC
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated"
            "${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp"
            "${PYRAMID_PROTOBUF_GEN_DIR}"
    )

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
        set(PYRAMID_GRPC_SERVICE_PROTOS
            "${PYRAMID_PROTO_DIR}/pyramid/components/tactical_objects/services/provided.proto"
            "${PYRAMID_PROTO_DIR}/pyramid/components/tactical_objects/services/consumed.proto"
        )

        set(PYRAMID_GRPC_OUTPUTS)
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
        endforeach()

        add_custom_target(pyramid_grpc_codegen DEPENDS ${PYRAMID_GRPC_OUTPUTS})

        add_library(pyramid_grpc_transport STATIC
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp/pyramid_components_tactical_objects_services_provided_grpc_transport.cpp"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp/pyramid_components_tactical_objects_services_consumed_grpc_transport.cpp"
            "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/components/tactical_objects/services/provided.pb.cc"
            "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/components/tactical_objects/services/provided.grpc.pb.cc"
            "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/components/tactical_objects/services/consumed.pb.cc"
            "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/components/tactical_objects/services/consumed.grpc.pb.cc"
        )
        add_dependencies(pyramid_grpc_transport pyramid_grpc_codegen pyramid_protobuf_codegen)
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
                "${PYRAMID_ROOT_DIR}/bindings/cpp/generated"
                "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp"
                "${PYRAMID_PROTOBUF_GEN_DIR}"
        )

        add_library(pyramid_grpc_c_shim SHARED
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp/pyramid_services_tactical_objects_grpc_c_api_support.cpp"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp/pyramid_components_tactical_objects_services_provided_grpc_c_api.cpp"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp/pyramid_components_tactical_objects_services_consumed_grpc_c_api.cpp"
        )
        add_dependencies(pyramid_grpc_c_shim pyramid_grpc_codegen pyramid_protobuf_codegen)
        target_link_libraries(pyramid_grpc_c_shim
            PUBLIC
                gRPC::grpc++
                protobuf::libprotobuf
                pyramid_grpc_transport
                pyramid_protobuf_support
        )
        target_include_directories(pyramid_grpc_c_shim
            PUBLIC
                "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp"
                "${PYRAMID_PROTOBUF_GEN_DIR}"
        )
    endif()
endif()

if(PYRAMID_ENABLE_ROS2)
    add_library(pyramid_ros2_transport STATIC
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/ros2/cpp/pyramid_ros2_transport_support.cpp"
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/ros2/cpp/pyramid_components_tactical_objects_ros2_transport.cpp"
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/ros2/cpp/pyramid_components_tactical_objects_services_consumed_ros2_transport.cpp"
        "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/ros2/cpp/pyramid_components_tactical_objects_services_provided_ros2_transport.cpp"
    )
    target_link_libraries(pyramid_ros2_transport
        PUBLIC
            pcl_core
    )
    target_include_directories(pyramid_ros2_transport
        PUBLIC
            "${PCL_ROOT_DIR}/include"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/ros2/cpp"
    )
endif()

add_subdirectory(core)
add_subdirectory(tactical_objects)
add_subdirectory(pyramid_bridge)
add_subdirectory(tests)
