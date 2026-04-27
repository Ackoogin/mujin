@echo off
REM test_ada_active_find_e2e.bat -- Cross-process ActiveFind E2E test driver (Windows).
REM
REM 3-process architecture:
REM   1. TacticalObjectsComponent server (--no-bridge --no-entity)
REM   2. Standalone StandardBridge (dual TCP: client->server, server->Ada)
REM   3. Ada active-find client (connects to bridge)
REM
REM Ada binaries must be pre-built via:
REM   cmake --build --target pyramid_ada_all
REM
REM Usage: scripts\test_ada_active_find_e2e.bat [--server-bin PATH]
REM            [--bridge-bin PATH] [--client-bin PATH]
setlocal enabledelayedexpansion

set "SERVER_BIN="
set "BRIDGE_BIN="
set "CLIENT_BIN="
set "BACKEND_PORT=19235"
set "FRONTEND_PORT=19236"
set "CONTENT_TYPE=application/json"
set "TIMEOUT_SEC=25"
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
if /i "%~1"=="--bridge-bin" (set "BRIDGE_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--client-bin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--backend-port" (set "BACKEND_PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--frontend-port" (set "FRONTEND_PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--content-type" (set "CONTENT_TYPE=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--config" (set "BUILD_CONFIG=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ServerBin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-BridgeBin" (set "BRIDGE_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ClientBin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-BackendPort" (set "BACKEND_PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-FrontendPort" (set "FRONTEND_PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ContentType" (set "CONTENT_TYPE=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-BuildDir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Config" (set "BUILD_CONFIG=%~2" & shift & shift & goto parse_args)
shift
goto parse_args
:done_args

if not defined BUILD_DIR set "BUILD_DIR=%WORKSPACE_ROOT%\build"
if not defined BUILD_CONFIG set "BUILD_CONFIG=Release"
if "%SERVER_BIN%"=="" set "SERVER_BIN=%BUILD_DIR%\subprojects\PYRAMID\tests\%BUILD_CONFIG%\tobj_socket_server.exe"
if "%BRIDGE_BIN%"=="" set "BRIDGE_BIN=%BUILD_DIR%\subprojects\PYRAMID\tests\%BUILD_CONFIG%\standalone_bridge.exe"
if "%CLIENT_BIN%"=="" (
    set "CLIENT_BIN=%PYRAMID_ROOT%\tests\ada\bin\ada_active_find_e2e.exe"
    if not exist "!CLIENT_BIN!" set "CLIENT_BIN=%PYRAMID_ROOT%\tests\ada\bin\ada_active_find_e2e"
)

set "PORT_FILE=%TEMP%\ame_af_e2e_port_%RANDOM%.tmp"
set "BRIDGE_PORT_FILE=%TEMP%\ame_af_e2e_bport_%RANDOM%.tmp"

echo === Ada ActiveFind E2E Test (3-process: server -^> bridge -^> client) ===

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
if not exist "%BRIDGE_BIN%" (
    echo [driver] FAIL: Bridge binary not found at %BRIDGE_BIN%
    exit /b 1
)

REM Step 1: Start backend server with --no-bridge --no-entity
echo [driver] Starting backend server on port %BACKEND_PORT% (--no-bridge --no-entity)...
start /b "" "%SERVER_BIN%" --port %BACKEND_PORT% --port-file "%PORT_FILE%" --timeout %TIMEOUT_SEC% --no-entity --no-bridge >nul 2>&1

REM Step 2: Wait for server port file (up to 5 seconds)
echo [driver] Waiting for server to write port file...
set "ATTEMPTS=0"
:wait_server_port
if %ATTEMPTS% geq 50 goto server_port_timeout
timeout /t 1 /nobreak >nul 2>&1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
if exist "%PORT_FILE%" (
    for %%S in ("%PORT_FILE%") do if %%~zS gtr 0 goto server_port_ready
)
set /a ATTEMPTS+=1
goto wait_server_port

:server_port_timeout
echo [driver] FAIL: Server did not write port file within 5 seconds
goto cleanup_fail

:server_port_ready
set /p ACTUAL_BACKEND_PORT=<"%PORT_FILE%"
echo [driver] Backend server ready on port %ACTUAL_BACKEND_PORT%

REM Step 3: Brief delay so server enters accept()
timeout /t 1 /nobreak >nul 2>&1

REM Step 4: Start standalone bridge
echo [driver] Starting standalone bridge (backend=%ACTUAL_BACKEND_PORT%, frontend=%FRONTEND_PORT%)...
start /b "" "%BRIDGE_BIN%" --backend-host 127.0.0.1 --backend-port %ACTUAL_BACKEND_PORT% --frontend-port %FRONTEND_PORT% --port-file "%BRIDGE_PORT_FILE%" --frontend-content-type "%CONTENT_TYPE%" --timeout %TIMEOUT_SEC% >nul 2>&1

REM Step 5: Wait for bridge port file (up to 5 seconds)
echo [driver] Waiting for bridge to write port file...
set "ATTEMPTS=0"
:wait_bridge_port
if %ATTEMPTS% geq 50 goto bridge_port_timeout
timeout /t 1 /nobreak >nul 2>&1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
if exist "%BRIDGE_PORT_FILE%" (
    for %%S in ("%BRIDGE_PORT_FILE%") do if %%~zS gtr 0 goto bridge_port_ready
)
set /a ATTEMPTS+=1
goto wait_bridge_port

:bridge_port_timeout
echo [driver] FAIL: Bridge did not write port file within 5 seconds
goto cleanup_fail

:bridge_port_ready
set /p ACTUAL_FRONTEND_PORT=<"%BRIDGE_PORT_FILE%"
echo [driver] Bridge ready, frontend on port %ACTUAL_FRONTEND_PORT%

REM Step 6: Brief delay so bridge enters accept()
timeout /t 1 /nobreak >nul 2>&1

REM Step 7: Start Ada active-find client (connects to bridge)
echo [driver] Starting Ada active-find client (-^> bridge port %ACTUAL_FRONTEND_PORT%)...
"%CLIENT_BIN%" --host 127.0.0.1 --port %ACTUAL_FRONTEND_PORT% --content-type "%CONTENT_TYPE%"
set "CLIENT_EXIT=%errorlevel%"

REM Step 8: Cleanup
call :cleanup

REM Step 9: Report
if %CLIENT_EXIT% equ 0 (
    echo [driver] PASS: Ada ActiveFind E2E -- evidence + correlation via standalone bridge succeeded
    exit /b 0
) else (
    echo [driver] FAIL: Ada active-find client exited with code %CLIENT_EXIT%
    exit /b 1
)

:cleanup_fail
call :cleanup
exit /b 1

:cleanup
taskkill /f /im standalone_bridge.exe >nul 2>&1
taskkill /f /im tobj_socket_server.exe >nul 2>&1
if exist "%PORT_FILE%" del /f "%PORT_FILE%" >nul 2>&1
if exist "%BRIDGE_PORT_FILE%" del /f "%BRIDGE_PORT_FILE%" >nul 2>&1
goto :eof
