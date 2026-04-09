@echo off
call D:\Dev\ros2-windows\setup.bat
pixi run --manifest-path D:\Dev\ros2-windows\pixi.toml colcon build --packages-select ame_ros2 --base-paths D:\Dev\repo\mujin\subprojects\AME\ros2 --cmake-force-configure --cmake-args "-DCMAKE_PREFIX_PATH=D:/Dev/ros2-windows;D:/Dev/repo/mujin/build/install"
