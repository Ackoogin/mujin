@echo off
rem AME Dev Environment - launch script
rem Usage: start_devenv.bat [--foxglove-url URL] [--no-ros2] [--width W] [--height H]
rem Run setup_venv.bat once first if the .venv directory does not exist.

setlocal

set SCRIPT_DIR=%~dp0
REM tools\devenv\ up 4 levels to repo root (devenv -> tools -> AME -> subprojects -> unmanned)
for %%I in ("%SCRIPT_DIR%\..\..\..\..") do set REPO_ROOT=%%~fI
set PYTHON=%SCRIPT_DIR%.venv\Scripts\python.exe
set ROS2_ROOT=D:\Dev\ros2-windows
set ROS2_PIXI=%ROS2_ROOT%\.pixi\envs\default

if not exist "%PYTHON%" (
    echo Virtual environment not found.  Running setup_venv.bat...
    call "%SCRIPT_DIR%setup_venv.bat"
    if errorlevel 1 exit /b 1
)

REM Pin CycloneDDS to one interface (avoids multi-NIC reply confusion with VMware adapters)
set CYCLONEDDS_URI=file://D:/Dev/repo/unmanned/cyclonedds_localhost.xml

REM Ensure the venv can import ROS2 Python packages and load their native DLLs.
set PATH=%ROS2_PIXI%;%ROS2_PIXI%\Scripts;%ROS2_PIXI%\Library\bin;%ROS2_ROOT%\Scripts;%ROS2_ROOT%\bin;%REPO_ROOT%\install\ame_ros2\bin;%PATH%

cd /d "%REPO_ROOT%"
call "%ROS2_ROOT%\setup.bat"
call "%REPO_ROOT%\install\setup.bat"
set PYTHONPATH=%REPO_ROOT%\build\python;%ROS2_PIXI%\Lib\site-packages;%PYTHONPATH%
"%PYTHON%" -m subprojects.AME.tools.devenv %*
