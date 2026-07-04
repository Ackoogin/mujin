@echo off
REM build_sdk.bat -- one command to turn proto\ into runnable codec plugins.
REM                  Windows counterpart to build_sdk.sh.
REM
REM The downstream end-to-end for this offline SDK. Chains the individual steps
REM from README.md ("Downstream: verify the distribution" / "build your
REM contracts") into a single command:
REM
REM   1. scripts\generate_bindings.bat          (proto\ -> generated bindings)
REM   2. scripts\build_plugins.bat [--smoke-tests]
REM                                            (build codec plugin DLLs; run
REM                                             the SDK smoke tests by default)
REM   3. [--ada] scripts\build_ada.bat          (build the C-ABI marshal archive
REM                                             for Ada clients)
REM
REM Run from a VS developer command prompt. No network access and no parent
REM monorepo required. Edit proto\ first (or leave the starter contracts to try
REM the toolchain), then run this.
REM
REM Usage:
REM   build_sdk.bat [--no-smoke] [--ada] [--clean] [--config Release^|Debug]
REM                 [--jobs N] [--cpp-only^|--ada-only]
REM
REM Defaults: generate both C++ and Ada bindings, build C++ codec plugins, run
REM           the smoke tests, skip the Ada archive build (add --ada).
setlocal enabledelayedexpansion

for %%I in ("%~f0") do set "SCRIPT_DIR=%%~dpI"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%I in ("%SCRIPT_DIR%\..") do set "SDK_ROOT=%%~fI"

set "RUN_SMOKE=1"
set "BUILD_ADA=0"
set "CLEAN=0"
set "CONFIG=Release"
set "JOBS=%NUMBER_OF_PROCESSORS%"
set "GEN_ARGS="
set "CPP_SKIP=0"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--no-smoke" (set "RUN_SMOKE=0" & shift & goto parse_args)
if /i "%~1"=="--ada" (set "BUILD_ADA=1" & shift & goto parse_args)
if /i "%~1"=="--clean" (set "CLEAN=1" & shift & goto parse_args)
if /i "%~1"=="--config" (set "CONFIG=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--jobs" (set "JOBS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--cpp-only" (set "GEN_ARGS=--cpp" & shift & goto parse_args)
if /i "%~1"=="--ada-only" (set "GEN_ARGS=--ada" & set "BUILD_ADA=1" & set "CPP_SKIP=1" & shift & goto parse_args)
if /i "%~1"=="-h" goto usage
if /i "%~1"=="--help" goto usage
echo [build_sdk] unknown arg: %~1 1>&2
exit /b 2
:done_args

echo.
echo ==== [build_sdk] step 1/3: generate bindings from proto\ ====
call "%SCRIPT_DIR%\generate_bindings.bat" %GEN_ARGS% || exit /b 1

if "%CPP_SKIP%"=="1" (
  echo [build_sdk] step 2/3: C++ plugin build skipped ^(--ada-only^).
) else (
  set "PLUGIN_ARGS=--build-dir "%SDK_ROOT%\build" --config %CONFIG% --jobs %JOBS%"
  if "%CLEAN%"=="1" set "PLUGIN_ARGS=!PLUGIN_ARGS! --clean"
  if "%RUN_SMOKE%"=="1" set "PLUGIN_ARGS=!PLUGIN_ARGS! --smoke-tests"
  echo.
  echo ==== [build_sdk] step 2/3: build C++ codec plugins ====
  call "%SCRIPT_DIR%\build_plugins.bat" !PLUGIN_ARGS! || exit /b 1
)

if "%BUILD_ADA%"=="1" (
  echo.
  echo ==== [build_sdk] step 3/3: build Ada C-ABI marshal archive ====
  call "%SCRIPT_DIR%\build_ada.bat" || exit /b 1
) else (
  echo [build_sdk] step 3/3: Ada archive skipped ^(add --ada to build it^).
)

echo.
echo [build_sdk] Done. Codec plugins are under %SDK_ROOT%\build.
echo [build_sdk] Load them alongside plugins\ per README.md 'Running against the plugins'.
exit /b 0

:usage
for /f "tokens=1* delims=:" %%A in ('findstr /n "^REM" "%~f0"') do (
  set "line=%%B"
  echo(!line:~4!
)
exit /b 0
