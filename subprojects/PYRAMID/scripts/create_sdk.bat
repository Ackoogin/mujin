@echo off
REM create_sdk.bat -- one command to build the monorepo and package the offline
REM                   PCL/PYRAMID SDK. Windows counterpart to create_sdk.sh.
REM
REM Maintainer-run, from a VS developer command prompt inside this repo.
REM Collapses the "Maintainer: create a distribution" flow from
REM sdk_template\README.md into a single command:
REM
REM   1. cmake --preset flatbuffers-only                 (configure)
REM   2. cmake --build --preset flatbuffers-only-<cfg>   (build repo + C++ bindings)
REM   3. scripts\build_plugins.bat --build-dir build-flatbuffers-only
REM   4. [Ada] subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat <dir>\ada_gnat_pcl
REM            (skipped with --no-ada or when no GNAT toolchain is on PATH)
REM   5. scripts\package_sdk.bat --build-dir build-flatbuffers-only --clean --out <out>
REM   6. [--verify] run the downstream verify flow inside the packaged SDK
REM
REM Usage:
REM   create_sdk.bat [--out DIR] [--config Release^|Debug] [--no-ada]
REM                  [--skip-build] [--build-dir DIR] [--proto-dir DIR]
REM                  [--gra] [--verify] [--jobs N]
REM
REM Defaults: --out dist\pcl_pyramid_sdk, --config Release, Ada libs built when a
REM           GNAT toolchain is on PATH, full configure+build, build dir
REM           build-flatbuffers-only. --build-dir overrides the build/package dir
REM           and is meant to be paired with --skip-build to reuse an existing,
REM           already-configured build (FlatBuffers + C++ bindings must be on).
REM           --gra requires an explicit --proto-dir and packages the OMS JSON
REM           codec + LA-CAL WebSocket transport.
setlocal enabledelayedexpansion

for %%I in ("%~f0") do set "SCRIPT_DIR=%%~dpI"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%I in ("%SCRIPT_DIR%\..\..\..") do set "REPO_ROOT=%%~fI"
set "PCL_ROOT=%REPO_ROOT%\subprojects\PCL"
set "BUILD_DIR=%REPO_ROOT%\build-flatbuffers-only"

set "OUT_DIR=%REPO_ROOT%\dist\pcl_pyramid_sdk"
set "CONFIG=Release"
set "WITH_ADA=1"
set "SKIP_BUILD=0"
set "VERIFY=0"
set "PROTO_DIR="
set "GRA=0"
set "JOBS=%NUMBER_OF_PROCESSORS%"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--out" (set "OUT_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--proto-dir" (set "PROTO_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--gra" (set "GRA=1" & shift & goto parse_args)
if /i "%~1"=="--config" (set "CONFIG=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--no-ada" (set "WITH_ADA=0" & shift & goto parse_args)
if /i "%~1"=="--skip-build" (set "SKIP_BUILD=1" & shift & goto parse_args)
if /i "%~1"=="--verify" (set "VERIFY=1" & shift & goto parse_args)
if /i "%~1"=="--jobs" (set "JOBS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-h" goto usage
if /i "%~1"=="--help" goto usage
echo [create_sdk] unknown arg: %~1 1>&2
exit /b 2
:done_args

if "%GRA%"=="1" if not defined PROTO_DIR (
  echo [create_sdk] ERROR: --gra requires --proto-dir; select the intended OMS/A-GRA contract explicitly. 1>&2
  exit /b 2
)

if /i "%CONFIG%"=="Release" (set "BUILD_PRESET=flatbuffers-only-release") else if /i "%CONFIG%"=="Debug" (set "BUILD_PRESET=flatbuffers-only-debug") else (
  echo [create_sdk] ERROR: --config must be Release or Debug ^(got '%CONFIG%'^) 1>&2
  exit /b 2
)

if "%SKIP_BUILD%"=="1" (
  echo.
  echo ==== [create_sdk] step 1-2/6: skipping configure+build ^(--skip-build^); reusing %BUILD_DIR% ====
  if not exist "%BUILD_DIR%" (
    echo [create_sdk] ERROR: --skip-build but %BUILD_DIR% does not exist. 1>&2
    exit /b 1
  )
) else (
  echo.
  echo ==== [create_sdk] step 1/6: configure ^(cmake --preset flatbuffers-only^) ====
  cmake --preset flatbuffers-only || exit /b 1
  echo.
  echo ==== [create_sdk] step 2/6: build repo + C++ bindings ^(cmake --build --preset %BUILD_PRESET%^) ====
  cmake --build --preset %BUILD_PRESET% --parallel %JOBS% || exit /b 1
)

echo.
echo ==== [create_sdk] step 3/6: build codec/transport plugins ^(build_plugins.bat^) ====
if "%GRA%"=="1" (
  call "%SCRIPT_DIR%\build_plugins.bat" --build-dir "%BUILD_DIR%" --jobs %JOBS% --gra --proto-dir "%PROTO_DIR%" || exit /b 1
) else if defined PROTO_DIR (
  call "%SCRIPT_DIR%\build_plugins.bat" --build-dir "%BUILD_DIR%" --jobs %JOBS% --proto-dir "%PROTO_DIR%" || exit /b 1
) else (
  call "%SCRIPT_DIR%\build_plugins.bat" --build-dir "%BUILD_DIR%" --jobs %JOBS% || exit /b 1
)

if "%WITH_ADA%"=="1" (
  where gnatmake >nul 2>&1 || where gcc >nul 2>&1
  if errorlevel 1 (
    echo [create_sdk] step 4/6: no GNAT toolchain on PATH -- skipping Ada libs ^(use --no-ada to silence^).
  ) else (
    echo.
    echo ==== [create_sdk] step 4/6: build GNAT PCL static libs ^(Ada consumers^) ====
    call "%PCL_ROOT%\scripts\build_gnat_pcl_static_libs.bat" "%BUILD_DIR%\ada_gnat_pcl" || echo [create_sdk] WARN: GNAT PCL lib build failed; package_sdk will retry / Ada may be unavailable.
  )
) else (
  echo.
  echo ==== [create_sdk] step 4/6: Ada libs skipped ^(--no-ada^) ====
)

echo.
echo ==== [create_sdk] step 5/6: package the SDK ^(package_sdk.bat --clean --out %OUT_DIR%^) ====
if "%GRA%"=="1" (
  call "%SCRIPT_DIR%\package_sdk.bat" --build-dir "%BUILD_DIR%" --out "%OUT_DIR%" --clean --gra --proto-dir "%PROTO_DIR%" || exit /b 1
) else if defined PROTO_DIR (
  call "%SCRIPT_DIR%\package_sdk.bat" --build-dir "%BUILD_DIR%" --out "%OUT_DIR%" --clean --proto-dir "%PROTO_DIR%" || exit /b 1
) else (
  call "%SCRIPT_DIR%\package_sdk.bat" --build-dir "%BUILD_DIR%" --out "%OUT_DIR%" --clean || exit /b 1
)

if "%VERIFY%"=="1" (
  echo.
  echo ==== [create_sdk] step 6/6: verify the packaged SDK end-to-end ^(build_sdk.bat^) ====
  call "%OUT_DIR%\scripts\build_sdk.bat" --jobs %JOBS% || exit /b 1
) else (
  echo [create_sdk] step 6/6: verification skipped ^(pass --verify to build+smoke-test the package^).
)

echo.
echo [create_sdk] Done. Offline SDK: %OUT_DIR%
echo [create_sdk] Ship that directory; downstream builds it with scripts\build_sdk.bat.
exit /b 0

:usage
for /f "tokens=1* delims=:" %%A in ('findstr /n "^REM" "%~f0"') do (
  set "line=%%B"
  echo(!line:~4!
)
exit /b 0
