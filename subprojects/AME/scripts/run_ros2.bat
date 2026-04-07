@echo off
REM Run ame_ros2 nodes (in-process mode)
REM Usage:
REM   run_ros2.bat [domain.pddl] [problem.pddl]
REM Or via pixi: pixi run --manifest-path D:\Dev\ros2-windows\pixi.toml cmd /c D:\Dev\repo\mujin\subprojects\AME\scripts\run_ros2.bat

set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%\..\..\..") do set REPO_ROOT=%%~fI

set DOMAIN=%~1
set PROBLEM=%~2
if "%DOMAIN%"=="" set DOMAIN=%REPO_ROOT%\subprojects\AME\domains\uav_search\domain.pddl
if "%PROBLEM%"=="" set PROBLEM=%REPO_ROOT%\subprojects\AME\domains\uav_search\problem.pddl

REM Pin CycloneDDS to one interface (avoids multi-NIC reply confusion with VMware adapters)
set CYCLONEDDS_URI=file://%REPO_ROOT:\=/%/cyclonedds_localhost.xml

REM 1. Add pixi conda Library/bin to PATH (yaml.dll, vcruntime, openssl, etc.)
set PATH=D:\Dev\ros2-windows\.pixi\envs\default;%PATH%
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\Scripts;%PATH%
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\Library\bin;%PATH%
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\bin;%PATH%

REM 2. Source ROS2 base environment
call D:\Dev\ros2-windows\setup.bat

REM 3. Source local ame_ros2 install
call %REPO_ROOT%\install\setup.bat

echo.
echo Starting ame_combined (in-process mode)
echo   domain:  %DOMAIN%
echo   problem: %PROBLEM%
echo.

ros2 run ame_ros2 ame_combined ^
  --ros-args ^
  -p domain.pddl_file:=%DOMAIN% ^
  -p domain.problem_file:=%PROBLEM%
