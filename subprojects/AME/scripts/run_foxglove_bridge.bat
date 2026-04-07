@echo off
setlocal

set SCRIPT_DIR=%~dp0
if "%SCRIPT_DIR:~-1%"=="\" set SCRIPT_DIR=%SCRIPT_DIR:~0,-1%
for %%I in ("%SCRIPT_DIR%\..\..\..") do set REPO_ROOT=%%~fI
set ROS2_ROOT=D:\Dev\ros2-windows
set ROS2_PIXI=%ROS2_ROOT%\.pixi\envs\default
set PORT=%1
if "%PORT%"=="" set PORT=8765

set CYCLONEDDS_URI=file://%REPO_ROOT:\=/%/cyclonedds_localhost.xml
set PATH=%ROS2_PIXI%;%ROS2_PIXI%\Scripts;%ROS2_PIXI%\Library\bin;%ROS2_PIXI%\bin;%PATH%

echo.
echo Starting ame_ros2 foxglove_bridge on ws://localhost:%PORT%
echo.

call %ROS2_ROOT%\setup.bat
if errorlevel 1 exit /b 1

call %REPO_ROOT%\install\setup.bat
if errorlevel 1 exit /b 1

ros2 run ame_ros2 foxglove_bridge --ros-args -p port:=%PORT%
