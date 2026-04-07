@echo off
setlocal

set SCRIPT_DIR=%~dp0
if "%SCRIPT_DIR:~-1%"=="\" set SCRIPT_DIR=%SCRIPT_DIR:~0,-1%
set PORT=%3
if "%PORT%"=="" set PORT=8765

echo.
echo Starting AME ROS2 stack in one window and foxglove_bridge in another
echo   Foxglove URL: ws://localhost:%PORT%
echo.

if "%~1"=="" goto no_args
if "%~2"=="" goto one_arg

start "AME ROS2" cmd /k call "%SCRIPT_DIR%\run_ros2.bat" "%~1" "%~2"
goto start_bridge

:one_arg
start "AME ROS2" cmd /k call "%SCRIPT_DIR%\run_ros2.bat" "%~1"
goto start_bridge

:no_args
start "AME ROS2" cmd /k call "%SCRIPT_DIR%\run_ros2.bat"

:start_bridge
start "Foxglove Bridge" cmd /k call "%SCRIPT_DIR%\run_foxglove_bridge.bat" %PORT%
