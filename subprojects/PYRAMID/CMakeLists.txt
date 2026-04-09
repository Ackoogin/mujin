set(PYRAMID_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")

add_library(nlohmann_json INTERFACE)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
target_include_directories(nlohmann_json
    INTERFACE
        "${PYRAMID_ROOT_DIR}/core/external"
)

add_subdirectory(core)
add_subdirectory(tactical_objects)
add_subdirectory(tests)
