include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

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
