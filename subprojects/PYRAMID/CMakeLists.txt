set(PYRAMID_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")

option(PYRAMID_ENABLE_PROTOBUF "Enable protobuf codecs for PYRAMID" ON)
option(PYRAMID_ENABLE_GRPC "Enable gRPC transport projection for PYRAMID" OFF)

add_library(nlohmann_json INTERFACE)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
target_include_directories(nlohmann_json
    INTERFACE
        "${PYRAMID_ROOT_DIR}/core/external"
)

set(PYRAMID_FLATBUFFERS_SCHEMA_DIR "${PYRAMID_ROOT_DIR}/examples/cpp/generated/flatbuffers/cpp")
set(PYRAMID_FLATBUFFERS_GEN_DIR "${CMAKE_BINARY_DIR}/generated/flatbuffers/cpp")
set(PYRAMID_FLATBUFFERS_SCHEMAS
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

add_library(pyramid_flatbuffers_support INTERFACE)
add_dependencies(pyramid_flatbuffers_support pyramid_flatbuffers_codegen)
target_include_directories(pyramid_flatbuffers_support
    INTERFACE
        "${PYRAMID_FLATBUFFERS_GEN_DIR}"
        "${flatbuffers_SOURCE_DIR}/include"
        "${PYRAMID_FLATBUFFERS_SCHEMA_DIR}"
)

add_library(pyramid_generated_codecs STATIC
    "${PYRAMID_ROOT_DIR}/examples/cpp/generated/flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.cpp"
    "${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_data_model_common_codec.cpp"
    "${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_data_model_tactical_codec.cpp"
)
add_dependencies(pyramid_generated_codecs pyramid_flatbuffers_codegen)
target_link_libraries(pyramid_generated_codecs
    PUBLIC
        nlohmann_json::nlohmann_json
        pyramid_flatbuffers_support
)
target_include_directories(pyramid_generated_codecs
    PUBLIC
        "${PYRAMID_ROOT_DIR}/examples/cpp/generated"
)

if(PYRAMID_ENABLE_PROTOBUF)
    find_package(Protobuf CONFIG QUIET)
    if(NOT Protobuf_FOUND)
        FetchContent_Declare(
            protobuf
            GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
            GIT_TAG        v32.1
            GIT_SHALLOW    TRUE
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

    find_program(PYRAMID_PROTOC NAMES protoc REQUIRED)

    set(PYRAMID_PROTO_DIR "${PYRAMID_ROOT_DIR}/proto")
    set(PYRAMID_PROTOBUF_GEN_DIR "${CMAKE_BINARY_DIR}/generated/protobuf")
    set(PYRAMID_PROTOBUF_PROTOS
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/base.proto"
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/common.proto"
        "${PYRAMID_PROTO_DIR}/pyramid/data_model/tactical.proto"
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
                    -I "${PYRAMID_PROTO_DIR}"
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
        "${PYRAMID_ROOT_DIR}/examples/protobuf/cpp/pyramid_services_tactical_objects_protobuf_codec.cpp"
        "${PYRAMID_ROOT_DIR}/examples/protobuf/cpp/pyramid_services_tactical_objects_protobuf_shim.cpp"
        "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/data_model/base.pb.cc"
        "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/data_model/common.pb.cc"
        "${PYRAMID_PROTOBUF_GEN_DIR}/pyramid/data_model/tactical.pb.cc"
    )
    add_dependencies(pyramid_protobuf_support pyramid_protobuf_codegen)
    target_link_libraries(pyramid_protobuf_support
        PUBLIC
            nlohmann_json::nlohmann_json
            protobuf::libprotobuf
    )
    target_include_directories(pyramid_protobuf_support
        PUBLIC
            "${PYRAMID_ROOT_DIR}/examples/cpp/generated"
            "${PYRAMID_ROOT_DIR}/examples/protobuf/cpp"
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

    find_package(gRPC CONFIG QUIET)
    if(TARGET gRPC::grpc_cpp_plugin)
        set(PYRAMID_GRPC_CPP_PLUGIN "$<TARGET_FILE:gRPC::grpc_cpp_plugin>")
    else()
        find_program(PYRAMID_GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin)
    endif()

    if(NOT gRPC_FOUND OR NOT PYRAMID_GRPC_CPP_PLUGIN)
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
                        --plugin=protoc-gen-grpc="${PYRAMID_GRPC_CPP_PLUGIN}"
                        -I "${PYRAMID_PROTO_DIR}"
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
            "${PYRAMID_ROOT_DIR}/examples/grpc/cpp/pyramid_components_tactical_objects_services_provided_grpc_transport.cpp"
            "${PYRAMID_ROOT_DIR}/examples/grpc/cpp/pyramid_components_tactical_objects_services_consumed_grpc_transport.cpp"
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
                pyramid_generated_codecs
        )
        target_include_directories(pyramid_grpc_transport
            PUBLIC
                "${PYRAMID_ROOT_DIR}/examples/cpp/generated"
                "${PYRAMID_ROOT_DIR}/examples/grpc/cpp"
                "${PYRAMID_PROTOBUF_GEN_DIR}"
        )
    endif()
endif()

add_subdirectory(core)
add_subdirectory(tactical_objects)
add_subdirectory(tests)
