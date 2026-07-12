find_package(Threads REQUIRED)

add_library(pyramid_owp STATIC owp_frame.cpp owp_client.cpp)
# Built PIC so it can be linked into the LA-CAL transport plugin (a shared MODULE).
set_target_properties(pyramid_owp PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pyramid_owp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(pyramid_owp PRIVATE ${websocketpp_SOURCE_DIR} ${asio_SOURCE_DIR}/asio/include)
target_compile_definitions(pyramid_owp PRIVATE ASIO_STANDALONE)
# MSVC does not set __cplusplus to a C++11-or-later value unless /Zc:__cplusplus
# is passed, which websocketpp's cpp11.hpp detection relies on; without this
# define it silently falls back to the boost-based random device and fails to
# find <boost/version.hpp>. Same workaround as ame_foxglove (see
# subprojects/AME/src/CMakeLists.txt / foxglove_bridge.cpp).
target_compile_definitions(pyramid_owp PUBLIC _WEBSOCKETPP_CPP11_STL_)
target_link_libraries(pyramid_owp PUBLIC Threads::Threads)
