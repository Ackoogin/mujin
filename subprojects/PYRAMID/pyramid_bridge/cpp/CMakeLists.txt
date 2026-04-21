set(AME_STUB_BINDINGS_DIR ${PYRAMID_ROOT_DIR}/bindings/cpp/generated)

add_executable(ame_backend_stub
    ame_backend_stub.cpp
    ${AME_STUB_BINDINGS_DIR}/pyramid_data_model_autonomy_codec.cpp
    ${AME_STUB_BINDINGS_DIR}/pyramid_services_autonomy_backend_provided.cpp
)

target_link_libraries(ame_backend_stub PRIVATE
    pcl_core
    pcl_transport_shared_memory
    pyramid_generated_codecs
    pyramid_flatbuffers_support
    nlohmann_json::nlohmann_json
)

target_include_directories(ame_backend_stub PRIVATE
    ${AME_STUB_BINDINGS_DIR}
    ${PCL_ROOT_DIR}/include
)

if(NOT WIN32)
    target_link_libraries(ame_backend_stub PRIVATE pthread rt)
endif()
