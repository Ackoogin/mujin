@echo off
REM build_plugins.bat -- build codec plugin DLLs from generated bindings.
REM
REM Standalone: configures/builds sdk_project (no FetchContent, no network,
REM no parent monorepo). Run scripts\generate_bindings.bat first.
REM
REM Usage:
REM   build_plugins.bat [--build-dir DIR] [--config Release|Debug] [--clean] [--jobs N] [--smoke-tests]
setlocal enabledelayedexpansion

for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"
for %%I in ("%SCRIPT_BASE%..") do set "SDK_ROOT=%%~fI"

set "BUILD_DIR=%SDK_ROOT%\build"
set "CONFIG=Release"
set "CLEAN=0"
set "RUN_SMOKE_TESTS=0"
set "JOBS=%NUMBER_OF_PROCESSORS%"
if not defined JOBS set "JOBS=4"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--config"    (set "CONFIG=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--clean"     (set "CLEAN=1" & shift & goto parse_args)
if /i "%~1"=="--jobs"      (set "JOBS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--smoke-tests" (set "RUN_SMOKE_TESTS=1" & shift & goto parse_args)
echo [build_plugins] unknown arg: %~1>&2
exit /b 2
:done_args

if not exist "%SDK_ROOT%\sdk_project\generated" (
  echo [build_plugins] FAIL: no generated bindings at %SDK_ROOT%\sdk_project\generated -- run scripts\generate_bindings.bat first.>&2
  exit /b 1
)

if "%CLEAN%"=="1" (
  echo [build_plugins] cleaning %BUILD_DIR%
  if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
)

echo [build_plugins] configure ...
cmake -S "%SDK_ROOT%\sdk_project" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%CONFIG%
if errorlevel 1 exit /b 1

echo [build_plugins] build ...
cmake --build "%BUILD_DIR%" --target pyramid_sdk_plugins --config %CONFIG% --parallel %JOBS%
if errorlevel 1 exit /b 1

if "%RUN_SMOKE_TESTS%"=="1" (
  echo [build_plugins] build smoke tests ...
  cmake --build "%BUILD_DIR%" --target pyramid_sdk_smoke_tests --config %CONFIG% --parallel %JOBS%
  if errorlevel 1 exit /b 1

  echo [build_plugins] run smoke tests ...
  ctest --test-dir "%BUILD_DIR%" --output-on-failure -C %CONFIG% -R "^sdk_"
  if errorlevel 1 exit /b 1
)

echo.
echo [build_plugins] produced plugins:
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\pyramid_codec_*.dll" 2^>nul') do (
  set "found=%%F"
  setlocal enabledelayedexpansion
  echo   !found:%BUILD_DIR%\=!
  endlocal
)

echo.
echo [build_plugins] PASS
exit /b 0
