@echo off
REM test_cpp_app_client.bat -- C++ client -> tactical_objects_app integration test.
setlocal enabledelayedexpansion

set "APP_BIN="
set "CLIENT_BIN="
set "PORT=19295"
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
if /i "%~1"=="--app-bin" (set "APP_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--client-bin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--port" (set "PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--content-type" (set "CONTENT_TYPE=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
shift
goto parse_args

:done_args
if "%APP_BIN%"=="" set "APP_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\tactical_objects\Release\tactical_objects_app.exe"
if "%CLIENT_BIN%"=="" set "CLIENT_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\tactical_objects\Release\tactical_objects_test_client.exe"

set "PORT_FILE=%TEMP%\cpp_app_frontend_%RANDOM%.tmp"

echo === C++ App Client Test (%CONTENT_TYPE%) ===

start /b "" "%APP_BIN%" --port %PORT% --port-file "%PORT_FILE%" --timeout %TIMEOUT_SEC% --content-type "%CONTENT_TYPE%" >nul 2>&1
set "ATTEMPTS=0"
:wait_app
if %ATTEMPTS% geq 50 goto fail_app
if exist "%PORT_FILE%" (
    for %%S in ("%PORT_FILE%") do if %%~zS gtr 0 goto app_ready
)
set /a ATTEMPTS+=1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
goto wait_app

:fail_app
echo [driver] FAIL: tactical_objects_app did not start
goto cleanup_fail

:app_ready
set /p ACTUAL_PORT=<"%PORT_FILE%"
"%CLIENT_BIN%" --host 127.0.0.1 --port %ACTUAL_PORT% --content-type "%CONTENT_TYPE%"
set "CLIENT_EXIT=%errorlevel%"

call :cleanup
exit /b %CLIENT_EXIT%

:cleanup_fail
call :cleanup
exit /b 1

:cleanup
taskkill /f /im tactical_objects_app.exe >nul 2>&1
if exist "%PORT_FILE%" del /f "%PORT_FILE%" >nul 2>&1
goto :eof
