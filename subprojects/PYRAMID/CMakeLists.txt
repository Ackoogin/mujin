set(PYRAMID_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")

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
    "${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_services_tactical_objects_json_codec.cpp"
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

add_subdirectory(core)
add_subdirectory(tactical_objects)
add_subdirectory(tests)
