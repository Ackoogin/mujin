@echo off
REM Build mujin_core + mujin_ros2 colcon package.
REM Run from a pixi shell at D:\Dev\ros2-windows, or run directly.
REM
REM Usage:
REM   build_ros2.bat              -- build everything
REM   build_ros2.bat core-only    -- only install mujin_core, skip colcon
REM   build_ros2.bat ros2-only    -- only colcon build (mujin_core must already be installed)

set MUJIN_ROOT=D:\Dev\repo\mujin

REM 1. VS 2022 build tools environment
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

REM 2. Add pixi conda Library/bin to PATH (yaml, spdlog, openssl, etc.)
set PATH=D:\Dev\ros2-windows\.pixi\envs\default\Library\bin;%PATH%

if "%1"=="ros2-only" goto ros2_build

echo.
echo === Step 1: Build and install mujin_core ===
cmake --preset default -DCMAKE_INSTALL_PREFIX=%MUJIN_ROOT%\build\install
if errorlevel 1 goto error
cmake --build %MUJIN_ROOT%\build --config Release -j%NUMBER_OF_PROCESSORS%
if errorlevel 1 goto error
cmake --install %MUJIN_ROOT%\build --config Release
if errorlevel 1 goto error

if "%1"=="core-only" goto done

:ros2_build
echo.
echo === Step 2: Build mujin_ros2 with colcon ===
call D:\Dev\ros2-windows\setup.bat

pixi run --manifest-path D:\Dev\ros2-windows\pixi.toml colcon build ^
  --packages-select mujin_ros2 ^
  --base-paths %MUJIN_ROOT%\ros2 ^
  --cmake-args "-DCMAKE_PREFIX_PATH=D:/Dev/ros2-windows;D:/Dev/repo/mujin/build/install"
if errorlevel 1 goto error

:done
echo.
echo === Build complete. Run nodes with: run_ros2.bat ===
exit /b 0

:error
echo.
echo === BUILD FAILED ===
exit /b 1
