@echo off
REM test_ada_socket_e2e.bat -- Cross-process E2E test driver (Windows).
REM
REM Orchestrates:
REM   1. Start C++ server (tobj_socket_server)
REM   2. Wait for server to be ready (port file appears)
REM   3. Start Ada client (ada_tobj_client)
REM   4. Wait for Ada client to exit
REM   5. Terminate server
REM   6. Report pass/fail
REM
REM Ada binaries must be pre-built via:
REM   cmake --build --target pyramid_ada_all
REM
REM Usage: scripts\test_ada_socket_e2e.bat [--server-bin PATH] [--client-bin PATH]
REM                                        [--port PORT] [--timeout SECONDS]
setlocal enabledelayedexpansion

set "SERVER_BIN="
set "CLIENT_BIN="
set "PORT=19234"
set "TIMEOUT_SEC=15"
set "BUILD_DIR=%PYRAMID_BUILD_DIR%"
set "BUILD_CONFIG=%PYRAMID_BUILD_CONFIG%"
for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"

if not defined PYRAMID_ROOT (
    for %%I in ("%SCRIPT_BASE%..") do set "PYRAMID_ROOT=%%~fI"
)
if not defined WORKSPACE_ROOT (
    for %%I in ("%PYRAMID_ROOT%\..\..") do set "WORKSPACE_ROOT=%%~fI"
)

REM Parse arguments
:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--server-bin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--client-bin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--port" (set "PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--config" (set "BUILD_CONFIG=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ServerBin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ClientBin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Port" (set "PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-BuildDir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Config" (set "BUILD_CONFIG=%~2" & shift & shift & goto parse_args)
shift
goto parse_args
:done_args

if not defined BUILD_DIR set "BUILD_DIR=%WORKSPACE_ROOT%\build"
if not defined BUILD_CONFIG set "BUILD_CONFIG=Release"
if "%SERVER_BIN%"=="" set "SERVER_BIN=%BUILD_DIR%\subprojects\PYRAMID\tests\%BUILD_CONFIG%\tobj_socket_server.exe"
if "%CLIENT_BIN%"=="" (
    set "CLIENT_BIN=%PYRAMID_ROOT%\examples\ada\bin\ada_tobj_client.exe"
    if not exist "!CLIENT_BIN!" set "CLIENT_BIN=%PYRAMID_ROOT%\examples\ada\bin\ada_tobj_client"
)

set "PORT_FILE=%TEMP%\ame_socket_e2e_port_%RANDOM%.tmp"
set "SERVER_PID="

echo === Ada Socket E2E Test ===

REM Resolve client path
if not exist "%CLIENT_BIN%" (
    if exist "%CLIENT_BIN%.exe" set "CLIENT_BIN=%CLIENT_BIN%.exe"
)
if not exist "%CLIENT_BIN%" (
    echo [driver] SKIP: Ada client binary not found at %CLIENT_BIN%
    echo [driver]   Run: cmake --build --target pyramid_ada_all
    exit /b 0
)
if not exist "%SERVER_BIN%" (
    echo [driver] FAIL: Server binary not found at %SERVER_BIN%
    exit /b 1
)

REM Step 1: Start server
echo [driver] Starting server on port %PORT%...
start /b "" "%SERVER_BIN%" --port %PORT% --port-file "%PORT_FILE%" --timeout %TIMEOUT_SEC% >nul 2>&1

REM Step 2: Wait for port file (up to 5 seconds)
echo [driver] Waiting for server to write port file...
set "ATTEMPTS=0"
:wait_port
if %ATTEMPTS% geq 50 goto port_timeout
timeout /t 1 /nobreak >nul 2>&1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
if exist "%PORT_FILE%" (
    for %%S in ("%PORT_FILE%") do if %%~zS gtr 0 goto port_ready
)
set /a ATTEMPTS+=1
goto wait_port

:port_timeout
echo [driver] FAIL: Server did not write port file within 5 seconds
goto cleanup_fail

:port_ready
set /p ACTUAL_PORT=<"%PORT_FILE%"
echo [driver] Server ready on port %ACTUAL_PORT%

REM Step 3: Brief delay so server enters accept()
timeout /t 1 /nobreak >nul 2>&1

REM Step 4: Start Ada client
echo [driver] Starting Ada client...
"%CLIENT_BIN%" --host 127.0.0.1 --port %ACTUAL_PORT%
set "CLIENT_EXIT=%errorlevel%"

REM Step 5: Cleanup
call :cleanup

REM Step 6: Report
if %CLIENT_EXIT% equ 0 (
    echo [driver] PASS: Ada client received entity updates over socket transport
    exit /b 0
) else (
    echo [driver] FAIL: Ada client exited with code %CLIENT_EXIT%
    exit /b 1
)

:cleanup_fail
call :cleanup
exit /b 1

:cleanup
REM Kill any remaining server process
taskkill /f /im tobj_socket_server.exe >nul 2>&1
if exist "%PORT_FILE%" del /f "%PORT_FILE%" >nul 2>&1
goto :eof
