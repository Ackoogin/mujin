add_executable(mujin_test_app main.cpp)

target_link_libraries(mujin_test_app
    PRIVATE
        behaviortree_cpp
        lapkt_core
)

# Copy BT.CPP DLL next to the executable on Windows
if(WIN32)
    add_custom_command(TARGET mujin_test_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:behaviortree_cpp>
            $<TARGET_FILE_DIR:mujin_test_app>
        COMMENT "Copying behaviortree_cpp DLL to output directory"
    )
endif()
