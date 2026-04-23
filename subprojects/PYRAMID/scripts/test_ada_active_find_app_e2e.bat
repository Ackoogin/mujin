@echo off
REM test_ada_active_find_app_e2e.bat -- Ada ActiveFind against tactical_objects_app.
setlocal enabledelayedexpansion

set "APP_BIN="
set "CLIENT_BIN="
set "PORT=19305"
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

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--app-bin" (set "APP_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--client-bin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--port" (set "PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--content-type" (set "CONTENT_TYPE=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--config" (set "BUILD_CONFIG=%~2" & shift & shift & goto parse_args)
shift
goto parse_args

:done_args
if not defined BUILD_DIR set "BUILD_DIR=%WORKSPACE_ROOT%\build"
if not defined BUILD_CONFIG set "BUILD_CONFIG=Release"
if "%APP_BIN%"=="" set "APP_BIN=%BUILD_DIR%\subprojects\PYRAMID\tactical_objects\%BUILD_CONFIG%\tactical_objects_app.exe"
if "%CLIENT_BIN%"=="" (
    set "CLIENT_BIN=%PYRAMID_ROOT%\tests\ada\bin\ada_active_find_e2e.exe"
    if not exist "!CLIENT_BIN!" set "CLIENT_BIN=%PYRAMID_ROOT%\tests\ada\bin\ada_active_find_e2e"
)

echo === Ada ActiveFind Real-App E2E (%CONTENT_TYPE%) ===

where python >nul 2>&1
if %errorlevel% equ 0 (
    echo [driver] Generating Ada bindings from proto...
    call "%PYRAMID_ROOT%\scripts\generate_bindings.bat" --ada
    if !errorlevel! neq 0 (
        echo [driver] FAIL: Ada service generation failed
        exit /b 1
    )
) else (
    echo [driver] python not found -- skipping Ada stub generation
)

where gprbuild >nul 2>&1
if %errorlevel% equ 0 (
    echo [driver] Building Ada active-find client...
    set "ADA_DIR=%PYRAMID_ROOT%\tests\ada"
    set "ADA_PCL_LIB_DIR=%BUILD_DIR%\ada_gnat_pcl"
    set "ADA_PYRAMID_LIB_DIR=%BUILD_DIR%\ada_gnat_pyramid"
    set "ADA_PCL_BUILD_SCRIPT=%WORKSPACE_ROOT%\subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat"
    set "ADA_PYRAMID_BUILD_SCRIPT=%PYRAMID_ROOT%\scripts\build_gnat_generated_flatbuffers_libs.bat"
    set "CAN_BUILD_ADA=1"
    call "!ADA_PCL_BUILD_SCRIPT!" "!ADA_PCL_LIB_DIR!" --force
    if !errorlevel! neq 0 set "CAN_BUILD_ADA=0"
    if "!CAN_BUILD_ADA!"=="1" (
        call "!ADA_PYRAMID_BUILD_SCRIPT!" "!ADA_PYRAMID_LIB_DIR!" --build-dir "!BUILD_DIR!" --config "!BUILD_CONFIG!"
        if !errorlevel! neq 0 set "CAN_BUILD_ADA=0"
    )
    if "!CAN_BUILD_ADA!"=="1" (
        set "UNMANNED_ROOT=%WORKSPACE_ROOT%"
        pushd "!ADA_DIR!"
        gprbuild -P ada_active_find_e2e.gpr -q ^
          -XUNMANNED_ROOT=!WORKSPACE_ROOT! ^
          -XPCL_INCLUDE_DIR=!WORKSPACE_ROOT!\subprojects\PCL\include ^
          -XPCL_LIB_DIR=!ADA_PCL_LIB_DIR! ^
          -XPCL_LIB_NAME=pcl_core ^
          -XPCL_SOCKET_LIB_NAME=pcl_transport_socket ^
          -XPYRAMID_GEN_LIB_DIR=!ADA_PYRAMID_LIB_DIR! ^
          -XPYRAMID_GEN_LIB_NAME=pyramid_generated_flatbuffers_codec >nul 2>&1
        if !errorlevel! neq 0 echo [driver] gprbuild failed -- falling back to pre-built binary
        popd
    ) else (
        echo [driver] WARNING: unable to refresh GNAT support archives -- falling back to pre-built binary
    )
) else (
    echo [driver] gprbuild not found -- checking for pre-built client...
)

if not exist "%CLIENT_BIN%" (
    if exist "%CLIENT_BIN%.exe" set "CLIENT_BIN=%CLIENT_BIN%.exe"
)
if not exist "%CLIENT_BIN%" (
    echo [driver] SKIP: Ada client binary not found at %CLIENT_BIN%
    exit /b 0
)
if not exist "%APP_BIN%" (
    echo [driver] FAIL: tactical_objects_app not found at %APP_BIN%
    exit /b 1
)

set "PORT_FILE=%TEMP%\tobj_app_af_%RANDOM%.tmp"

echo [driver] Starting tactical_objects_app on port %PORT%...
start /b "" "%APP_BIN%" --port %PORT% --port-file "%PORT_FILE%" --timeout %TIMEOUT_SEC% --content-type "%CONTENT_TYPE%" >nul 2>&1

set "ATTEMPTS=0"
:wait_app
if %ATTEMPTS% geq 50 goto app_timeout
if exist "%PORT_FILE%" (
    for %%S in ("%PORT_FILE%") do if %%~zS gtr 0 goto app_ready
)
set /a ATTEMPTS+=1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
goto wait_app

:app_timeout
echo [driver] FAIL: tactical_objects_app did not start
goto cleanup_fail

:app_ready
set /p ACTUAL_PORT=<"%PORT_FILE%"
echo [driver] App ready on port %ACTUAL_PORT%
echo [driver] Starting Ada active-find client...
"%CLIENT_BIN%" --host 127.0.0.1 --port %ACTUAL_PORT% --content-type "%CONTENT_TYPE%"
set "CLIENT_EXIT=%errorlevel%"

call :cleanup
if %CLIENT_EXIT% equ 0 (
    echo [driver] PASS: Ada ActiveFind real-app E2E succeeded
    exit /b 0
) else (
    echo [driver] FAIL: Ada active-find client exited with code %CLIENT_EXIT%
    exit /b 1
)

:cleanup_fail
call :cleanup
exit /b 1

:cleanup
taskkill /f /im tactical_objects_app.exe >nul 2>&1
if exist "%PORT_FILE%" del /f "%PORT_FILE%" >nul 2>&1
goto :eof
