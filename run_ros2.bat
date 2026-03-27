@echo off
REM Run ame_ros2 nodes (in-process mode)
REM Usage:
REM   run_ros2.bat [domain.pddl] [problem.pddl]
REM Or via pixi: pixi run --manifest-path D:\Dev\ros2-windows\pixi.toml cmd /c D:\Dev\repo\mujin\run_ros2.bat

set DOMAIN=%1
set PROBLEM=%2
if "%DOMAIN%"=="" set DOMAIN=D:\Dev\repo\mujin\domains\uav_search\domain.pddl
if "%PROBLEM%"=="" set PROBLEM=D:\Dev\repo\mujin\domains\uav_search\problem.pddl

REM Pin CycloneDDS to one interface (avoids multi-NIC reply confusion with VMware adapters)
set CYCLONEDDS_URI=file://D:/Dev/repo/mujin/cyclonedds_localhost.xml

REM 1. Add pixi conda Library/bin to PATH (yaml.dll, vcruntime, openssl, etc.)
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\Library\bin;%PATH%
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\bin;%PATH%

REM 2. Source ROS2 base environment
call D:\Dev\ros2-windows\setup.bat

REM 3. Source local ame_ros2 install
call D:\Dev\repo\mujin\install\setup.bat

echo.
echo Starting ame_combined (in-process mode)
echo   domain:  %DOMAIN%
echo   problem: %PROBLEM%
echo.

ros2 run ame_ros2 ame_combined ^
  --ros-args ^
  -p domain.pddl_file:=%DOMAIN% ^
  -p domain.problem_file:=%PROBLEM%
