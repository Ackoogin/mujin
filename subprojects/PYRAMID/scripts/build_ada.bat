@echo off
REM build_ada.bat -- end-to-end ".proto -> Ada client/bridge binaries" build.
REM
REM Windows counterpart to build_ada.sh. Ada does NOT produce plugins: the
REM codec/transport plugins are language-neutral C-ABI DLLs built by the C++
REM chain (build_plugins.bat / the `pyramid_plugins` target). Ada clients and
REM the Ada bridge consume those same DLLs at run time (codec via
REM PYRAMID_CODEC_PLUGINS, transport via PCL_TRANSPORT_PLUGIN). This script
REM builds the Ada consumers.
REM
REM Pipeline:
REM
REM   proto\**.proto
REM     |                          (--regen: generate_bindings.bat --ada -> build-local Ada bindings)
REM     v
REM   <build>\generated\pyramid_ada_bindings\**.{ads,adb} (compiled from source by GNAT)
REM     |                          (CMake + gprbuild; GNAT FlatBuffers archive built automatically)
REM     v
REM   Ada binaries: ada_tobj_client, ada_active_find_e2e, pyramid_bridge (Ada), ...
REM     + the C++ codec/transport plugins they load at run time (pyramid_plugins)
REM
REM Usage:
REM   build_ada.bat [--build-dir DIR] [--regen] [--backends LIST]
REM                 [--jobs N] [--clean] [--test]
REM
REM Options:
REM   --build-dir DIR  CMake build directory       (default: <repo>\build-ada)
REM   --regen          regenerate build-local Ada bindings from proto before building
REM   --backends LIST  backends for --regen         (default: json,flatbuffers)
REM   --jobs N         parallel build jobs          (default: %NUMBER_OF_PROCESSORS%)
REM   --clean          delete the build dir before configuring
REM   --test           run the Ada test suite (ctest -L ada) after building
REM
REM CI example:
REM   subprojects\PYRAMID\scripts\build_ada.bat --regen --test
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..\..") do set "REPO_ROOT=%%~fI"

set "BUILD_DIR=%REPO_ROOT%\build-ada"
set "BACKENDS=json,flatbuffers"
set "DO_REGEN=0"
set "DO_TEST=0"
set "CLEAN=0"
set "JOBS=%NUMBER_OF_PROCESSORS%"
if not defined JOBS set "JOBS=4"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--regen"     (set "DO_REGEN=1" & shift & goto parse_args)
if /i "%~1"=="--backends"  (set "BACKENDS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--jobs"      (set "JOBS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--clean"     (set "CLEAN=1" & shift & goto parse_args)
if /i "%~1"=="--test"      (set "DO_TEST=1" & shift & goto parse_args)
if /i "%~1"=="-h"     goto usage
if /i "%~1"=="--help" goto usage
echo [build_ada] unknown arg: %~1>&2
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
where gprbuild >nul 2>&1
if errorlevel 1 (
  echo [build_ada] FAIL: gprbuild not found on PATH ^(Ada builds need a GNAT toolchain^).>&2
  exit /b 1
)

set "REGEN=off"
if "%DO_REGEN%"=="1" set "REGEN=on (%BACKENDS%)"

echo [build_ada] repo      : %REPO_ROOT%
echo [build_ada] build-dir : %BUILD_DIR%
echo [build_ada] regen     : %REGEN%
echo [build_ada] jobs      : %JOBS%

if "%CLEAN%"=="1" (
  echo [build_ada] cleaning %BUILD_DIR%
  if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
)

if "%DO_REGEN%"=="1" (
  echo [build_ada] regenerating build-local Ada bindings from proto ...
  call "%SCRIPT_DIR%generate_bindings.bat" ^
    --ada ^
    --backends "%BACKENDS%" ^
    --ada-out "%BUILD_DIR%\generated\pyramid_ada_bindings"
  if errorlevel 1 exit /b 1
)

REM Lean configuration matching build_plugins.bat (PYRAMID only,
REM FlatBuffers+JSON, no AME/Foxglove/ROS2). The Ada targets are picked up
REM when gprbuild is found.
echo [build_ada] configure ...
cmake -S "%REPO_ROOT%" -B "%BUILD_DIR%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DUNMANNED_BUILD_PYRAMID=ON ^
  -DUNMANNED_BUILD_AME=OFF ^
  -DAME_FOXGLOVE=OFF ^
  -DPYRAMID_ENABLE_ROS2=OFF ^
  -DPYRAMID_ENABLE_FLATBUFFERS=ON ^
  -DPYRAMID_GENERATE_CPP_BINDINGS=ON ^
  -DPYRAMID_GENERATE_ADA_BINDINGS=ON ^
  -DPYRAMID_BUILD_TESTS=ON
if errorlevel 1 exit /b 1

REM pyramid_plugins  -> the codec/transport DLLs the Ada binaries load at run time
REM pyramid_ada_all  -> the Ada binaries (auto-builds the GNAT FlatBuffers archive
REM                     and GNAT-compatible PCL archives)
echo [build_ada] build pyramid_plugins + pyramid_ada_all ...
cmake --build "%BUILD_DIR%" --target pyramid_plugins pyramid_ada_all --config Release --parallel %JOBS%
if errorlevel 1 exit /b 1

echo.
echo [build_ada] produced Ada binaries:
for %%D in ("%REPO_ROOT%\subprojects\PYRAMID\examples\ada\bin" "%REPO_ROOT%\subprojects\PYRAMID\tests\ada\bin" "%REPO_ROOT%\subprojects\PYRAMID\pyramid_bridge\ada\bin") do (
  if exist "%%~fD" (
    for /f "delims=" %%F in ('dir /b /a-d "%%~fD" 2^>nul') do (
      echo   %%~fD\%%F
    )
  )
)

if "%DO_TEST%"=="1" (
  echo.
  echo [build_ada] running Ada test suite ^(ctest -L ada^) ...
  ctest --test-dir "%BUILD_DIR%" -C Release -L ada --output-on-failure
  if errorlevel 1 exit /b 1
)

echo.
echo [build_ada] PASS
exit /b 0
