@echo off
REM test_generated_codec_dispatch.bat -- Regenerate bindings, rebuild codec
REM dispatch test, and prove a non-JSON runtime codec selection works.
setlocal enabledelayedexpansion

set "FILTER=CodecDispatchE2E.FlatBuffersPubSubRoundTrip"
set "BUILD_DIR="
set "CONFIG=Release"
set "TEST_BIN="
for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"

if not defined PYRAMID_ROOT (
    for %%I in ("%SCRIPT_BASE%..") do set "PYRAMID_ROOT=%%~fI"
)
if not defined WORKSPACE_ROOT (
    for %%I in ("%PYRAMID_ROOT%\..\..") do set "WORKSPACE_ROOT=%%~fI"
)

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--filter" (set "FILTER=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--config" (set "CONFIG=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--test-bin" (set "TEST_BIN=%~2" & shift & shift & goto parse_args)
echo [driver] Unknown argument: %~1
exit /b 1

:done_args
if "%BUILD_DIR%"=="" set "BUILD_DIR=%WORKSPACE_ROOT%\build"
if "%TEST_BIN%"=="" set "TEST_BIN=%BUILD_DIR%\subprojects\PYRAMID\tests\%CONFIG%\test_codec_dispatch_e2e.exe"

echo === Generated Codec Dispatch Test ===
call "%PYRAMID_ROOT%\scripts\generate_bindings.bat" --cpp --backends "json,flatbuffers"
if errorlevel 1 (
    echo [driver] FAIL: binding generation failed
    exit /b 1
)

echo [driver] Rebuilding test_codec_dispatch_e2e...
cmake --build "%BUILD_DIR%" --config "%CONFIG%" --target test_codec_dispatch_e2e -j4
if errorlevel 1 (
    echo [driver] FAIL: rebuild failed
    exit /b 1
)

if not exist "%TEST_BIN%" (
    echo [driver] FAIL: test binary not found at %TEST_BIN%
    exit /b 1
)

echo [driver] Running %FILTER%...
"%TEST_BIN%" --gtest_filter=%FILTER%
if errorlevel 1 (
    echo [driver] FAIL: runtime codec selection test failed
    exit /b 1
)

echo [driver] PASS: generated bindings rebuilt and non-JSON runtime dispatch succeeded
exit /b 0
