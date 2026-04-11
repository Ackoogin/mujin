@echo off
REM Build the PYRAMID ROS2 runtime transport package with colcon.
REM
REM Usage:
REM   build_ros2_transport.bat              -- build ROS2 package using existing pcl_core
REM   build_ros2_transport.bat refresh-core -- rebuild pcl_core + pyramid_ros2_transport first

set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%\..\..\..") do set MUJIN_ROOT=%%~fI
set PCL_CORE_LIB=%MUJIN_ROOT%\build\subprojects\PCL\src\Release\pcl_core.lib
set MUJIN_ROOT_CMAKE=%MUJIN_ROOT:\=/%
set PCL_CORE_LIB_CMAKE=%PCL_CORE_LIB:\=/%

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

if "%1"=="refresh-core" (
  echo.
  echo === Step 1: Build core workspace artifacts needed by the ROS2 adapter ===
  pushd %MUJIN_ROOT%
  cmake --preset default ^
    -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
  if errorlevel 1 ( popd & goto error )
  cmake --build %MUJIN_ROOT%\build --config Release -j%NUMBER_OF_PROCESSORS% --target pcl_core pyramid_ros2_transport
  if errorlevel 1 ( popd & goto error )
  popd
)

if not exist "%PCL_CORE_LIB%" (
  echo.
  echo === Missing prerequisite: %PCL_CORE_LIB% ===
  echo Build the main workspace first, or rerun with: build_ros2_transport.bat refresh-core
  exit /b 1
)

echo.
echo === Step 2: Build pyramid_ros2_transport with colcon ===
set PATH=D:\Dev\ros2-windows\bin;D:\Dev\ros2-windows\.pixi\envs\default\Library\bin;%PATH%
call D:\Dev\ros2-windows\setup.bat

pixi run --manifest-path D:\Dev\ros2-windows\pixi.toml colcon build ^
  --packages-select pyramid_ros2_transport ^
  --base-paths %MUJIN_ROOT%\subprojects\PYRAMID\ros2 ^
  --cmake-force-configure ^
  --cmake-args ^
    "-DMUJIN_ROOT=%MUJIN_ROOT_CMAKE%" ^
    "-DPCL_CORE_LIB=%PCL_CORE_LIB_CMAKE%" ^
    "-DPCL_CORE_INCLUDE_DIR=%MUJIN_ROOT_CMAKE%/subprojects/PCL/include"
if errorlevel 1 goto error

echo.
echo === Build complete ===
exit /b 0

:error
echo.
echo === BUILD FAILED ===
exit /b 1
