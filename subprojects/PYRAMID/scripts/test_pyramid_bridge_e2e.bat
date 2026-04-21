@echo off
REM test_pyramid_bridge_e2e.bat -- Pyramid Bridge E2E test driver (Windows).
REM
REM Orchestrates:
REM   1. Start ame_backend_stub on shared-memory bus (background)
REM   2. Start tactical_objects_app with demo evidence injection (background)
REM   3. Wait for tactical_objects_app to write its port file
REM   4. Start Ada pyramid_bridge_main (background)
REM   5. Wait for ame_backend_stub to exit (it exits 0 when it receives >=1 fact)
REM   6. Report pass/fail
REM
REM Usage: scripts\test_pyramid_bridge_e2e.bat
REM          [--app-bin    PATH]   tactical_objects_app binary
REM          [--stub-bin   PATH]   ame_backend_stub binary
REM          [--evidence-bin PATH] pyramid_bridge_evidence_client binary
REM          [--bridge-bin PATH]   pyramid_bridge_main Ada binary
REM          [--port       NUM]    tactical_objects_app TCP port (default 19400)
REM          [--bus        NAME]   shared-memory bus name (default pyramid_bridge)
REM          [--tobj-bus   NAME]   optional Tactical Objects bus override
REM          [--timeout    SECS]   per-process wall-clock timeout (default 25)
setlocal enabledelayedexpansion

set "APP_BIN="
set "STUB_BIN="
set "EVIDENCE_BIN="
set "BRIDGE_BIN="
set "PORT=19400"
set "BUS_NAME=pyramid_bridge"
set "TOBJ_BUS_NAME="
set "TIMEOUT_SEC=25"

for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"
if not defined PYRAMID_ROOT (
    for %%I in ("%SCRIPT_BASE%..") do set "PYRAMID_ROOT=%%~fI"
)
if not defined WORKSPACE_ROOT (
    for %%I in ("%PYRAMID_ROOT%\..\..") do set "WORKSPACE_ROOT=%%~fI"
)

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--app-bin"    (set "APP_BIN=%~2"    & shift & shift & goto parse_args)
if /i "%~1"=="--stub-bin"   (set "STUB_BIN=%~2"   & shift & shift & goto parse_args)
if /i "%~1"=="--evidence-bin" (set "EVIDENCE_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--bridge-bin" (set "BRIDGE_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--port"       (set "PORT=%~2"        & shift & shift & goto parse_args)
if /i "%~1"=="--bus"        (set "BUS_NAME=%~2"    & shift & shift & goto parse_args)
if /i "%~1"=="--tobj-bus"   (set "TOBJ_BUS_NAME=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout"    (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
shift
goto parse_args
:done_args

if "%TOBJ_BUS_NAME%"=="" set "TOBJ_BUS_NAME=%BUS_NAME%"

if "%APP_BIN%"==""    set "APP_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\tactical_objects\Release\tactical_objects_app.exe"
if "%STUB_BIN%"==""   set "STUB_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\pyramid_bridge\cpp\Release\ame_backend_stub.exe"
if "%EVIDENCE_BIN%"=="" set "EVIDENCE_BIN=%WORKSPACE_ROOT%\build\subprojects\PYRAMID\pyramid_bridge\cpp\Release\pyramid_bridge_evidence_client.exe"
if "%BRIDGE_BIN%"=="" (
    set "BRIDGE_BIN=%PYRAMID_ROOT%\pyramid_bridge\ada\bin\pyramid_bridge_main.exe"
    if not exist "!BRIDGE_BIN!" set "BRIDGE_BIN=%PYRAMID_ROOT%\pyramid_bridge\ada\bin\pyramid_bridge_main"
)
if not exist "%BRIDGE_BIN%" if exist "%BRIDGE_BIN%.exe" set "BRIDGE_BIN=%BRIDGE_BIN%.exe"

echo === Pyramid Bridge E2E ===
echo [driver] ame_bus=%BUS_NAME% tobj_bus=%TOBJ_BUS_NAME% timeout=%TIMEOUT_SEC%s

REM Optional: build Ada bridge if gprbuild is available
where gprbuild >nul 2>&1
if %errorlevel% equ 0 (
    echo [driver] Building Ada Pyramid Bridge...
    set "ADA_PCL_LIB_DIR=%WORKSPACE_ROOT%\build\ada_gnat_pcl"
    set "ADA_PYRAMID_LIB_DIR=%WORKSPACE_ROOT%\build\ada_gnat_pyramid"
    set "CAN_BUILD_ADA=1"
    call "%WORKSPACE_ROOT%\subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat" "!ADA_PCL_LIB_DIR!" --force
    if !errorlevel! neq 0 (
        echo [driver] FAIL: unable to build GNAT-compatible PCL archives
        exit /b 1
    )
    call "%PYRAMID_ROOT%\scripts\build_gnat_generated_flatbuffers_libs.bat" "!ADA_PYRAMID_LIB_DIR!"
    if !errorlevel! neq 0 (
        echo [driver] FAIL: unable to build GNAT-compatible generated FlatBuffers archive
        exit /b 1
    )
    pushd "%PYRAMID_ROOT%\pyramid_bridge\ada"
    gprbuild -P pyramid_bridge.gpr -q ^
      -XMUJIN_ROOT=!WORKSPACE_ROOT! ^
      -XPCL_INCLUDE_DIR=!WORKSPACE_ROOT!\subprojects\PCL\include ^
      -XPCL_LIB_DIR=!ADA_PCL_LIB_DIR! ^
      -XPCL_LIB_NAME=pcl_core ^
      -XPCL_SOCKET_LIB_NAME=pcl_transport_socket ^
      -XPCL_SHMEM_LIB_NAME=pcl_transport_shared_memory ^
      -XPYRAMID_GEN_LIB_DIR=!ADA_PYRAMID_LIB_DIR! ^
      -XPYRAMID_GEN_LIB_NAME=pyramid_generated_flatbuffers_codec
    if !errorlevel! neq 0 (
        echo [driver] FAIL: gprbuild failed
        popd
        exit /b 1
    )
    popd
) else (
    echo [driver] gprbuild not found -- checking for pre-built bridge binary...
)

REM Pre-flight checks
if not exist "%APP_BIN%" (
    echo [driver] FAIL: tactical_objects_app not found at %APP_BIN%
    exit /b 1
)
if not exist "%STUB_BIN%" (
    echo [driver] FAIL: ame_backend_stub not found at %STUB_BIN%
    exit /b 1
)
if not exist "%EVIDENCE_BIN%" (
    echo [driver] FAIL: pyramid_bridge_evidence_client not found at %EVIDENCE_BIN%
    exit /b 1
)
if not exist "%BRIDGE_BIN%" (
    echo [driver] FAIL: pyramid_bridge_main not found at %BRIDGE_BIN%
    exit /b 1
)

set "PORT_FILE=%TEMP%\tobj_bridge_e2e_%RANDOM%.tmp"
set "RC_FILE=%TEMP%\stub_rc_%RANDOM%.txt"
set "STUB_WRAPPER=%TEMP%\run_stub_%RANDOM%.bat"
set "STUB_LOG=%TEMP%\pyramid_bridge_stub_%RANDOM%.log"
set "APP_LOG=%TEMP%\pyramid_bridge_app_%RANDOM%.log"
set "BRIDGE_LOG=%TEMP%\pyramid_bridge_main_%RANDOM%.log"
set "EVIDENCE_LOG=%TEMP%\pyramid_bridge_evidence_%RANDOM%.log"

REM Write a wrapper script that runs stub and captures its exit code
(
    echo @echo off
    echo "%STUB_BIN%" --bus "%BUS_NAME%" --timeout %TIMEOUT_SEC% ^> "%STUB_LOG%" 2^>^&1
    echo ^> "%RC_FILE%" echo %%ERRORLEVEL%%
) > "%STUB_WRAPPER%"

REM Start stub via wrapper (separate cmd window, captures exit code to file)
echo [driver] Starting ame_backend_stub (bus=%BUS_NAME%)...
start "stub" /min cmd /c "%STUB_WRAPPER%"
ping -n 2 -w 100 127.0.0.1 >nul 2>&1

REM Start tactical_objects_app in background
echo [driver] Starting tactical_objects_app on shared-memory bus %TOBJ_BUS_NAME%...
start /b "" "%APP_BIN%" --shmem-bus "%TOBJ_BUS_NAME%" --port-file "%PORT_FILE%" --timeout %TIMEOUT_SEC% > "%APP_LOG%" 2>&1

REM Wait for port file (up to 10 seconds)
set "ATTEMPTS=0"
:wait_port
if %ATTEMPTS% geq 100 goto port_timeout
if exist "%PORT_FILE%" (
    for %%S in ("%PORT_FILE%") do if %%~zS gtr 0 goto port_ready
)
set /a ATTEMPTS+=1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
goto wait_port

:port_timeout
echo [driver] FAIL: tactical_objects_app did not write port file
goto cleanup_fail

:port_ready
set /p ACTUAL_PORT=<"%PORT_FILE%"
echo [driver] tactical_objects_app ready on shared-memory bus %ACTUAL_PORT%

REM Start Ada Pyramid Bridge in background
echo [driver] Starting pyramid_bridge_main...
start /b "" "%BRIDGE_BIN%" --tobj-bus "%TOBJ_BUS_NAME%" --ame-bus "%BUS_NAME%" --timeout %TIMEOUT_SEC% > "%BRIDGE_LOG%" 2>&1

echo [driver] Starting pyramid_bridge_evidence_client...
start /b "" "%EVIDENCE_BIN%" --bus "%TOBJ_BUS_NAME%" --timeout %TIMEOUT_SEC% > "%EVIDENCE_LOG%" 2>&1

REM Wait for stub to finish (poll for RC file)
echo [driver] Waiting for ame_backend_stub to finish...
set /a MAX_WAIT=%TIMEOUT_SEC%+10
set "WAITED=0"
:wait_rc
if exist "%RC_FILE%" goto check_rc
if %WAITED% geq %MAX_WAIT% goto stub_timeout
ping -n 2 -w 100 127.0.0.1 >nul 2>&1
set /a WAITED+=1
goto wait_rc

:stub_timeout
echo [driver] FAIL: ame_backend_stub timed out waiting for world facts
goto cleanup_fail

:check_rc
set /p STUB_RC=<"%RC_FILE%"
if "!STUB_RC!"=="0" (
    call :cleanup
    echo [driver] PASS: ame_backend_stub received world facts from Pyramid Bridge
    exit /b 0
) else (
    call :dump_logs
    call :cleanup
    echo [driver] FAIL: ame_backend_stub did not receive world facts (exit=!STUB_RC!)
    exit /b 1
)

:cleanup_fail
call :dump_logs
call :cleanup
exit /b 1

:dump_logs
echo [driver] --- ame_backend_stub log ---
if exist "%STUB_LOG%" (type "%STUB_LOG%") else echo [driver] missing stub log
echo [driver] --- tactical_objects_app log ---
if exist "%APP_LOG%" (type "%APP_LOG%") else echo [driver] missing app log
echo [driver] --- pyramid_bridge_main log ---
if exist "%BRIDGE_LOG%" (type "%BRIDGE_LOG%") else echo [driver] missing bridge log
echo [driver] --- pyramid_bridge_evidence_client log ---
if exist "%EVIDENCE_LOG%" (type "%EVIDENCE_LOG%") else echo [driver] missing evidence client log
goto :eof

:cleanup
taskkill /f /im pyramid_bridge_main.exe >nul 2>&1
taskkill /f /im pyramid_bridge_evidence_client.exe >nul 2>&1
taskkill /f /im ame_backend_stub.exe >nul 2>&1
taskkill /f /im tactical_objects_app.exe >nul 2>&1
if exist "%PORT_FILE%"    del /f /q "%PORT_FILE%"    >nul 2>&1
if exist "%RC_FILE%"      del /f /q "%RC_FILE%"      >nul 2>&1
if exist "%STUB_WRAPPER%" del /f /q "%STUB_WRAPPER%" >nul 2>&1
if exist "%STUB_LOG%"     del /f /q "%STUB_LOG%"     >nul 2>&1
if exist "%APP_LOG%"      del /f /q "%APP_LOG%"      >nul 2>&1
if exist "%BRIDGE_LOG%"   del /f /q "%BRIDGE_LOG%"   >nul 2>&1
if exist "%EVIDENCE_LOG%" del /f /q "%EVIDENCE_LOG%" >nul 2>&1
goto :eof
