@echo off
REM Run mujin ROS2 nodes
REM Usage: From a pixi shell at D:\Dev\ros2-windows, run:
REM   call D:\Dev\repo\mujin\run_ros2.bat [domain.pddl] [problem.pddl]
REM Or directly: pixi run --manifest-path D:\Dev\ros2-windows\pixi.toml cmd /c D:\Dev\repo\mujin\run_ros2.bat

set DOMAIN=%1
set PROBLEM=%2
if "%DOMAIN%"=="" set DOMAIN=D:\Dev\repo\mujin\domains\uav_search\domain.pddl
if "%PROBLEM%"=="" set PROBLEM=D:\Dev\repo\mujin\domains\uav_search\problem.pddl

REM 1. Add pixi conda Library/bin to PATH (yaml.dll, vcruntime, openssl, etc.)
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\Library\bin;%PATH%
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\bin;%PATH%

REM 2. Source ROS2 base environment
call D:\Dev\ros2-windows\setup.bat

REM 3. Source local mujin_ros2 install
call D:\Dev\repo\mujin\install\setup.bat

echo.
echo Starting mujin_combined (in-process mode)
echo   domain:  %DOMAIN%
echo   problem: %PROBLEM%
echo.

ros2 run mujin_ros2 mujin_combined ^
  --ros-args ^
  -p domain.pddl_file:=%DOMAIN% ^
  -p domain.problem_file:=%PROBLEM%
