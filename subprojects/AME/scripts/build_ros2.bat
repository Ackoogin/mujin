@echo off
REM Build ame_core + ame_ros2 colcon package.
REM Run from a pixi shell at D:\Dev\ros2-windows, or run directly.
REM
REM Usage:
REM   build_ros2.bat              -- build everything
REM   build_ros2.bat core-only    -- only install ame_core, skip colcon
REM   build_ros2.bat ros2-only    -- only colcon build (ame_core must already be installed)

set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%\..\..\..") do set AME_ROOT=%%~fI

REM 1. Add ROS2 bin + pixi conda Library/bin to PATH *before* vcvars to avoid 8191-char CMD PATH limit.
REM    Python is NOT added here (would overflow); cmake receives it via -DPython3_EXECUTABLE instead.
set PATH=D:\Dev\ros2-windows\bin;D:\Dev\ros2-windows\.pixi\envs\default\Library\bin;%PATH%

REM 2. VS 2022 build tools environment
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

if "%1"=="ros2-only" goto ros2_build

echo.
echo === Step 1: Build and install ame_core ===
pushd %AME_ROOT%
cmake --preset default ^
  -DCMAKE_INSTALL_PREFIX=%AME_ROOT%\build\install ^
  -DPython3_EXECUTABLE=D:\Dev\ros2-windows\.pixi\envs\default\python.exe ^
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON ^
  -DUNMANNED_BUILD_PYRAMID=OFF
if errorlevel 1 ( popd & goto error )
cmake --build %AME_ROOT%\build --config Release -j%NUMBER_OF_PROCESSORS%
if errorlevel 1 ( popd & goto error )
cmake --install %AME_ROOT%\build --config Release
if errorlevel 1 ( popd & goto error )
popd

if "%1"=="core-only" goto done

:ros2_build
echo.
echo === Step 2: Build ame_ros2 with colcon ===
call D:\Dev\ros2-windows\setup.bat

pixi run --manifest-path D:\Dev\ros2-windows\pixi.toml colcon build ^
  --packages-select ame_ros2 ^
  --base-paths %AME_ROOT%\subprojects\AME\ros2 ^
  --cmake-force-configure ^
  --cmake-args ^
    "-DCMAKE_PREFIX_PATH=D:/Dev/ros2-windows;D:/Dev/repo/unmanned/build/install" ^
    "-DPython3_EXECUTABLE=D:/Dev/ros2-windows/.pixi/envs/default/python.exe"
if errorlevel 1 goto error

:done
echo.
echo === Build complete. Run nodes with: run_ros2.bat ===
exit /b 0

:error
echo.
echo === BUILD FAILED ===
exit /b 1
