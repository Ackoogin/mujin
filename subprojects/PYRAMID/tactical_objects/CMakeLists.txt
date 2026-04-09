add_library(tactical_objects)
add_library(pyramid::tactical_objects ALIAS tactical_objects)

target_sources(tactical_objects
    PRIVATE
        src/store/ObjectStore.cpp
        src/spatial/SpatialIndex.cpp
        src/milclass/MilClassEngine.cpp
        src/correlation/CorrelationEngine.cpp
        src/zone/ZoneEngine.cpp
        src/query/QueryEngine.cpp
        src/relationship/RelationshipIndex.cpp
        src/interest/InterestManager.cpp
        src/history/TacticalHistory.cpp
        src/TacticalObjectsCodec.cpp
        src/TacticalObjectsRuntime.cpp
        src/StreamingCodec.cpp
)

target_include_directories(tactical_objects
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tactical_objects
    PUBLIC
        pyramid::core
        nlohmann_json::nlohmann_json
        pcl_core
)

target_compile_features(tactical_objects PUBLIC cxx_std_14)

# ---------------------------------------------------------------------------
# TacticalObjectsComponent — PCL integration
# ---------------------------------------------------------------------------
add_library(tactical_objects_component)
add_library(pyramid::tactical_objects_component ALIAS tactical_objects_component)

target_sources(tactical_objects_component PRIVATE
    src/TacticalObjectsComponent.cpp
    src/StandardBridge.cpp
)
target_include_directories(tactical_objects_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tactical_objects_component PUBLIC
  tactical_objects
  pcl_core
)
target_compile_features(tactical_objects_component PUBLIC cxx_std_14)

# ---------------------------------------------------------------------------
# tactical_objects_app - standalone runtime demo
# ---------------------------------------------------------------------------
add_executable(tactical_objects_app
    src/apps/tactical_objects_main.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_services_tactical_objects_consumed.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_services_tactical_objects_json_codec.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_data_model_common_codec.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_data_model_tactical_codec.cpp
)

add_executable(tactical_objects_test_client
    src/apps/tactical_objects_test_client.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_services_tactical_objects_json_codec.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_services_tactical_objects_provided.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_data_model_common_codec.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/generated/pyramid_data_model_tactical_codec.cpp
)

target_link_libraries(tactical_objects_app
    PRIVATE
        tactical_objects
        tactical_objects_component
        pcl_transport_socket
)

target_include_directories(tactical_objects_app
    PRIVATE
        ${PYRAMID_ROOT_DIR}/examples/cpp/generated
)

target_compile_features(tactical_objects_app PRIVATE cxx_std_14)

target_link_libraries(tactical_objects_test_client
    PRIVATE
        pcl_core
        pcl_transport_socket
        nlohmann_json::nlohmann_json
)

target_include_directories(tactical_objects_test_client
    PRIVATE
        ${PYRAMID_ROOT_DIR}/examples/cpp/generated
)

target_compile_features(tactical_objects_test_client PRIVATE cxx_std_14)
