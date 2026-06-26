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
set(TACTICAL_OBJECTS_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${TACTICAL_OBJECTS_GENERATED_DIR})
set(_tactical_codec_paths_header "#pragma once\n\n#include <array>\n\n")
set(_tactical_codec_plugin_targets)
foreach(_tactical_codec_target IN ITEMS
        pyramid_codec_json_tactical_objects
        pyramid_codec_flatbuffers_tactical_objects)
    if(TARGET ${_tactical_codec_target})
        list(APPEND _tactical_codec_plugin_targets ${_tactical_codec_target})
    endif()
endforeach()
list(LENGTH _tactical_codec_plugin_targets _tactical_codec_plugin_count)
string(APPEND _tactical_codec_paths_header
    "inline constexpr std::array<const char*, ${_tactical_codec_plugin_count}> kTacticalObjectsDefaultCodecPlugins = {{\n")
foreach(_tactical_codec_target IN LISTS _tactical_codec_plugin_targets)
    string(APPEND _tactical_codec_paths_header
        "    \"$<TARGET_FILE:${_tactical_codec_target}>\",\n")
endforeach()
string(APPEND _tactical_codec_paths_header "}};\n\n")
# Default transport plugin path so the client runs standalone; overridable with
# --transport-plugin. nullptr when the plugin target is unavailable.
if(TARGET pcl_transport_socket_plugin)
    string(APPEND _tactical_codec_paths_header
        "inline constexpr const char* kTacticalObjectsSocketTransportPlugin = \"$<TARGET_FILE:pcl_transport_socket_plugin>\";\n")
else()
    string(APPEND _tactical_codec_paths_header
        "inline constexpr const char* kTacticalObjectsSocketTransportPlugin = nullptr;\n")
endif()
if(TARGET pcl_transport_shared_memory_plugin)
    string(APPEND _tactical_codec_paths_header
        "inline constexpr const char* kTacticalObjectsShmTransportPlugin = \"$<TARGET_FILE:pcl_transport_shared_memory_plugin>\";\n")
else()
    string(APPEND _tactical_codec_paths_header
        "inline constexpr const char* kTacticalObjectsShmTransportPlugin = nullptr;\n")
endif()
file(GENERATE
    OUTPUT ${TACTICAL_OBJECTS_GENERATED_DIR}/tactical_objects_codec_plugin_paths.hpp
    CONTENT "${_tactical_codec_paths_header}")

add_executable(tactical_objects_app
    src/apps/tactical_objects_main.cpp
)

add_executable(tactical_objects_test_client
    src/apps/tactical_objects_test_client.cpp
    ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
    ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
)
add_dependencies(tactical_objects_test_client pyramid_cpp_bindings_codegen)
if(_tactical_codec_plugin_targets)
  add_dependencies(tactical_objects_app ${_tactical_codec_plugin_targets})
  add_dependencies(tactical_objects_test_client ${_tactical_codec_plugin_targets})
endif()

# The app loads its transport (socket/shm) from .so plugins at run time, so it
# links no transport library -- only the component, framework and codecs.
target_link_libraries(tactical_objects_app
    PRIVATE
        tactical_objects
        tactical_objects_component
        pyramid_generated_codecs
        pyramid_flatbuffers_support
)
if(TARGET pyramid_protobuf_support)
  target_link_libraries(tactical_objects_app PRIVATE pyramid_protobuf_support)
endif()
# Ensure the transport plugins the app loads at run time are built first.
foreach(_tactical_transport_plugin IN ITEMS
        pcl_transport_socket_plugin pcl_transport_shared_memory_plugin)
  if(TARGET ${_tactical_transport_plugin})
    add_dependencies(tactical_objects_app ${_tactical_transport_plugin})
  endif()
endforeach()

target_include_directories(tactical_objects_app
    PRIVATE
        ${PYRAMID_CPP_BINDINGS_DIR}
        ${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp
        ${TACTICAL_OBJECTS_GENERATED_DIR}
)

target_compile_features(tactical_objects_app PRIVATE cxx_std_14)

# Plugin-only client: links the framework + the native<->C-struct marshalling
# ONLY. The codec (JSON/FlatBuffers) AND the transport (socket) are both loaded
# from .so plugins at runtime, so the client links neither a wire codec, nor
# FlatBuffers, nor any transport library -- only pcl_core + marshalling.
target_link_libraries(tactical_objects_test_client
    PRIVATE
        pcl_core
        pyramid_generated_marshal
)
# The client loads the socket transport plugin at runtime; ensure it is built.
if(TARGET pcl_transport_socket_plugin)
  add_dependencies(tactical_objects_test_client pcl_transport_socket_plugin)
endif()

target_include_directories(tactical_objects_test_client
    PRIVATE
        ${PYRAMID_CPP_BINDINGS_DIR}
        ${PYRAMID_ROOT_DIR}/bindings/protobuf/cpp
        ${TACTICAL_OBJECTS_GENERATED_DIR}
)

target_compile_features(tactical_objects_test_client PRIVATE cxx_std_14)
