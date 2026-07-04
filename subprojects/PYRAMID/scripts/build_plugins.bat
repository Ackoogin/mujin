@echo off
REM build_plugins.bat -- end-to-end ".proto -> runtime plugin .dll" build.
REM
REM Windows counterpart to build_plugins.sh. Regenerates the C++ bindings from
REM the .proto data model and builds every runtime codec/transport plugin via
REM the CMake `pyramid_plugins` target. Suitable as a CI/CD stage or a manual
REM engineer step.
REM
REM Pipeline:
REM
REM   proto\**.proto
REM     |                      (CMake configure, PYRAMID_GENERATE_CPP_BINDINGS=ON)
REM     v
REM   generated C++ bindings  -- pim\generate_bindings.py
REM     |                      (C++ compile -> shared modules)
REM     v
REM   plugins (.dll):
REM     pyramid_codec_json_<component>.dll
REM     pyramid_codec_flatbuffers_<component>.dll
REM     pcl_transport_socket_plugin.dll
REM     pcl_transport_shared_memory_plugin.dll
REM     [pyramid_grpc_coupled_plugin.dll]   (with --grpc)
REM
REM   optionally:  stage per-component deployment dirs (stage_plugin_deploy.bat)
REM
REM The codec plugin is the single cross-language .dll (it consumes the frozen
REM pyramid_<Type>_c C struct) and is loaded from both C++ and Ada clients.
REM
REM Usage:
REM   build_plugins.bat [--proto-dir DIR] [--build-dir DIR] [--grpc] [--jobs N]
REM                     [--clean] [--stage] [--stage-out DIR]
REM
REM Options:
REM   --proto-dir DIR  proto IDL source tree to build  (default: PYRAMID\proto);
REM                    e.g. subprojects\PYRAMID\pim\test for the new PIM proto set.
REM                    Redirecting the proto set usually warrants a fresh build dir
REM                    (use a distinct --build-dir or --clean).
REM   --build-dir DIR  CMake build directory          (default: <repo>\build-plugins)
REM   --grpc           also build protobuf + the coupled gRPC target plugin
REM                    (fetches gRPC from source on first configure -- needs network)
REM   --jobs N         parallel build jobs            (default: %NUMBER_OF_PROCESSORS%)
REM   --clean          delete the build dir before configuring
REM   --stage          after building, stage per-component deployment dirs
REM   --stage-out DIR  staging output dir             (default: <repo>\dist\plugin_deploy)
REM
REM CI example:
REM   subprojects\PYRAMID\scripts\build_plugins.bat --grpc --stage
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..\..") do set "REPO_ROOT=%%~fI"

set "BUILD_DIR=%REPO_ROOT%\build-plugins"
set "STAGE_OUT=%REPO_ROOT%\dist\plugin_deploy"
set "PROTO_DIR="
set "ENABLE_GRPC=0"
set "DO_STAGE=0"
set "CLEAN=0"
set "JOBS=%NUMBER_OF_PROCESSORS%"
if not defined JOBS set "JOBS=4"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--proto-dir" (set "PROTO_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--grpc"      (set "ENABLE_GRPC=1" & shift & goto parse_args)
if /i "%~1"=="--jobs"      (set "JOBS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--clean"     (set "CLEAN=1" & shift & goto parse_args)
if /i "%~1"=="--stage"     (set "DO_STAGE=1" & shift & goto parse_args)
if /i "%~1"=="--stage-out" (set "STAGE_OUT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-h"     goto usage
if /i "%~1"=="--help" goto usage
echo [build_plugins] unknown arg: %~1>&2
exit /b 2

:usage
for /f "tokens=1* delims=:" %%A in ('findstr /b /n "REM" "%~f0"') do (
  set "line=%%B"
  setlocal enabledelayedexpansion
  if "!line!"=="REM" (echo.) else (echo !line:~4!)
  endlocal
)
exit /b 0

:done_args
set "GRPC=off"
if "%ENABLE_GRPC%"=="1" set "GRPC=on"

REM Resolve --proto-dir to an absolute path so CMake (run from BUILD_DIR) finds it.
if defined PROTO_DIR (
  if not exist "%PROTO_DIR%" (
    echo [build_plugins] proto dir not found: %PROTO_DIR%>&2
    exit /b 2
  )
  for %%I in ("%PROTO_DIR%") do set "PROTO_DIR=%%~fI"
)

set "PROTO_DISP=%PROTO_DIR%"
if not defined PROTO_DISP set "PROTO_DISP=(default: PYRAMID\proto)"

echo [build_plugins] repo      : %REPO_ROOT%
echo [build_plugins] build-dir : %BUILD_DIR%
echo [build_plugins] proto-dir : %PROTO_DISP%
echo [build_plugins] grpc      : %GRPC%
echo [build_plugins] jobs      : %JOBS%

if "%CLEAN%"=="1" (
  echo [build_plugins] cleaning %BUILD_DIR%
  if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
)

REM Lean, plugin-focused configuration:
REM   - PYRAMID only (no AME), no Foxglove, no ROS2
REM   - FlatBuffers + JSON codecs always; PYRAMID_GENERATE_CPP_BINDINGS regenerates
REM     the C++ bindings from .proto so this really is "proto -> plugins"
REM   - --grpc adds protobuf + the coupled gRPC target plugin
if "%ENABLE_GRPC%"=="1" (
  set "GRPC_ARGS=-DPYRAMID_ENABLE_PROTOBUF=ON -DPYRAMID_ENABLE_GRPC=ON"
) else (
  set "GRPC_ARGS=-DPYRAMID_ENABLE_PROTOBUF=OFF -DPYRAMID_ENABLE_GRPC=OFF"
)

REM Redirect the proto set (e.g. pim\test) when --proto-dir was given.
set "PROTO_ARGS="
if defined PROTO_DIR set "PROTO_ARGS=-DPYRAMID_PROTO_DIR=%PROTO_DIR%"

echo [build_plugins] configure ...
cmake -S "%REPO_ROOT%" -B "%BUILD_DIR%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DUNMANNED_BUILD_PYRAMID=ON ^
  -DUNMANNED_BUILD_AME=OFF ^
  -DAME_FOXGLOVE=OFF ^
  -DPYRAMID_ENABLE_ROS2=OFF ^
  -DPYRAMID_ENABLE_FLATBUFFERS=ON ^
  -DPYRAMID_GENERATE_CPP_BINDINGS=ON ^
  -DPYRAMID_BUILD_TESTS=OFF ^
  %GRPC_ARGS% %PROTO_ARGS%
if errorlevel 1 exit /b 1

echo [build_plugins] build pyramid_plugins ...
cmake --build "%BUILD_DIR%" --target pyramid_plugins --config Release --parallel %JOBS%
if errorlevel 1 exit /b 1

echo.
echo [build_plugins] produced plugins:
REM Codec plugins are named pyramid_codec_<codec>_<component>.dll (no "plugin"
REM in the name); transport/coupled plugins carry "_plugin".
for %%P in (pyramid_codec_*.dll pcl_transport_socket_plugin.dll pcl_transport_shared_memory_plugin.dll pyramid_grpc_coupled_plugin.dll) do (
  for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\subprojects\%%P" 2^>nul') do (
    set "found=%%F"
    setlocal enabledelayedexpansion
    echo   !found:%BUILD_DIR%\=!
    endlocal
  )
)

if "%DO_STAGE%"=="1" (
  echo.
  echo [build_plugins] staging deployment -^> %STAGE_OUT%
  call "%SCRIPT_DIR%stage_plugin_deploy.bat" --build-dir "%BUILD_DIR%" --out "%STAGE_OUT%" --clean
  if errorlevel 1 exit /b 1
)

echo.
echo [build_plugins] PASS
exit /b 0
