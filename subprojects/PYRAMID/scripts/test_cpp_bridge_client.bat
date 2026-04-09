@echo off
REM test_cpp_bridge_client.bat -- C++ client -> standalone bridge integration test.
setlocal enabledelayedexpansion

set "SERVER_BIN="
set "BRIDGE_BIN="
set "CLIENT_BIN="
set "BACKEND_PORT=19285"
set "FRONTEND_PORT=19286"
set "CONTENT_TYPE=application/json"
set "TIMEOUT_SEC=20"
for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"

if not defined PYRAMID_ROOT (
    for %%I in ("%SCRIPT_BASE%..") do set "PYRAMID_ROOT=%%~fI"
)
if not defined WORKSPACE_ROOT (
    for %%I in ("%PYRAMID_ROOT%\..\..") do set "WORKSPACE_ROOT=%%~fI"
)

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--server-bin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--bridge-bin" (set "BRIDGE_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--client-bin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--backend-port" (set "BACKEND_PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--frontend-port" (set "FRONTEND_PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--content-type" (set "CONTENT_TYPE=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
shift
goto parse_args

:done_args
if "%SERVER_BIN%"=="" set "SERVER_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\tests\Release\tobj_socket_server.exe"
if "%BRIDGE_BIN%"=="" set "BRIDGE_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\tests\Release\standalone_bridge.exe"
if "%CLIENT_BIN%"=="" set "CLIENT_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\tactical_objects\Release\tactical_objects_test_client.exe"

set "PORT_FILE=%TEMP%\cpp_bridge_server_%RANDOM%.tmp"
set "BRIDGE_PORT_FILE=%TEMP%\cpp_bridge_frontend_%RANDOM%.tmp"

echo === C++ Bridge Client Test (%CONTENT_TYPE%) ===

start /b "" "%SERVER_BIN%" --port %BACKEND_PORT% --port-file "%PORT_FILE%" --timeout %TIMEOUT_SEC% --no-entity --no-bridge >nul 2>&1
set "ATTEMPTS=0"
:wait_server
if %ATTEMPTS% geq 50 goto fail_server
if exist "%PORT_FILE%" (
    for %%S in ("%PORT_FILE%") do if %%~zS gtr 0 goto server_ready
)
set /a ATTEMPTS+=1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
goto wait_server

:fail_server
echo [driver] FAIL: backend server did not start
goto cleanup_fail

:server_ready
set /p ACTUAL_BACKEND_PORT=<"%PORT_FILE%"

start /b "" "%BRIDGE_BIN%" --backend-host 127.0.0.1 --backend-port %ACTUAL_BACKEND_PORT% --frontend-port %FRONTEND_PORT% --port-file "%BRIDGE_PORT_FILE%" --frontend-content-type "%CONTENT_TYPE%" --timeout %TIMEOUT_SEC% >nul 2>&1
set "ATTEMPTS=0"
:wait_bridge
if %ATTEMPTS% geq 50 goto fail_bridge
if exist "%BRIDGE_PORT_FILE%" (
    for %%S in ("%BRIDGE_PORT_FILE%") do if %%~zS gtr 0 goto bridge_ready
)
set /a ATTEMPTS+=1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
goto wait_bridge

:fail_bridge
echo [driver] FAIL: frontend bridge did not start
goto cleanup_fail

:bridge_ready
set /p ACTUAL_FRONTEND_PORT=<"%BRIDGE_PORT_FILE%"
"%CLIENT_BIN%" --host 127.0.0.1 --port %ACTUAL_FRONTEND_PORT% --content-type "%CONTENT_TYPE%"
set "CLIENT_EXIT=%errorlevel%"

call :cleanup
exit /b %CLIENT_EXIT%

:cleanup_fail
call :cleanup
exit /b 1

:cleanup
taskkill /f /im standalone_bridge.exe >nul 2>&1
taskkill /f /im tobj_socket_server.exe >nul 2>&1
if exist "%PORT_FILE%" del /f "%PORT_FILE%" >nul 2>&1
if exist "%BRIDGE_PORT_FILE%" del /f "%BRIDGE_PORT_FILE%" >nul 2>&1
goto :eof
