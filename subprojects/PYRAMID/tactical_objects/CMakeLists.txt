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
# TacticalObjectsComponent -- PCL integration
# ---------------------------------------------------------------------------
add_library(tactical_objects_component)
add_library(pyramid::tactical_objects_component ALIAS tactical_objects_component)

target_sources(tactical_objects_component PRIVATE
    src/TacticalObjectsComponent.cpp
    src/StandardBridge.cpp
    ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
    ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
)
add_dependencies(tactical_objects_component pyramid_cpp_bindings_codegen)
target_include_directories(tactical_objects_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(tactical_objects_component PRIVATE
  ${PYRAMID_CPP_BINDINGS_DIR}
  ${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp
)
target_link_libraries(tactical_objects_component PUBLIC
  tactical_objects
  pcl_core
  pyramid_generated_codecs
  pyramid_flatbuffers_support
)
if(TARGET pyramid_protobuf_support)
  target_link_libraries(tactical_objects_component PUBLIC pyramid_protobuf_support)
endif()
target_compile_features(tactical_objects_component PUBLIC cxx_std_14)

# ---------------------------------------------------------------------------
# tactical_objects_app - standalone runtime demo
# ---------------------------------------------------------------------------
add_executable(tactical_objects_app
    src/apps/tactical_objects_main.cpp
)

add_executable(tactical_objects_test_client
    src/apps/tactical_objects_test_client.cpp
    ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
)
add_dependencies(tactical_objects_test_client pyramid_cpp_bindings_codegen)

target_link_libraries(tactical_objects_app
    PRIVATE
        tactical_objects
        tactical_objects_component
        pcl_transport_shared_memory
        pcl_transport_socket
        pyramid_generated_codecs
        pyramid_flatbuffers_support
)
if(TARGET pyramid_protobuf_support)
  target_link_libraries(tactical_objects_app PRIVATE pyramid_protobuf_support)
endif()

target_include_directories(tactical_objects_app
    PRIVATE
        ${PYRAMID_CPP_BINDINGS_DIR}
        ${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp
)

target_compile_features(tactical_objects_app PRIVATE cxx_std_14)

target_link_libraries(tactical_objects_test_client
    PRIVATE
        pcl_core
        pcl_transport_socket
        nlohmann_json::nlohmann_json
        pyramid_generated_codecs
        pyramid_flatbuffers_support
)
if(TARGET pyramid_protobuf_support)
  target_link_libraries(tactical_objects_test_client PRIVATE pyramid_protobuf_support)
endif()

target_include_directories(tactical_objects_test_client
    PRIVATE
        ${PYRAMID_CPP_BINDINGS_DIR}
        ${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp
)

target_compile_features(tactical_objects_test_client PRIVATE cxx_std_14)
