@echo off
REM test_ada_socket_e2e.bat -- Cross-process E2E test driver (Windows).
REM
REM Orchestrates:
REM   1. Build Ada client (if gprbuild available)
REM   2. Start C++ server (tobj_socket_server)
REM   3. Wait for server to be ready (port file appears)
REM   4. Start Ada client (ada_tobj_client)
REM   5. Wait for Ada client to exit
REM   6. Terminate server
REM   7. Report pass/fail
REM
REM Usage: scripts\test_ada_socket_e2e.bat [--server-bin PATH] [--client-bin PATH]
REM                                        [--port PORT] [--timeout SECONDS]
setlocal enabledelayedexpansion

set "SERVER_BIN="
set "CLIENT_BIN="
set "PORT=19234"
set "TIMEOUT_SEC=15"

REM Parse arguments
:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--server-bin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--client-bin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--port" (set "PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ServerBin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ClientBin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Port" (set "PORT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
shift
goto parse_args
:done_args

set "SCRIPT_BASE=%~dp0"
for %%I in ("%SCRIPT_BASE%") do set "SCRIPT_BASE=%%~fI"
set "ROOT_DIR=%SCRIPT_BASE%"
if not exist "%ROOT_DIR%\pim\ada_service_generator.py" (
  for %%I in ("%SCRIPT_BASE%\..") do set "ROOT_DIR=%%~fI"
)

if "%SERVER_BIN%"=="" set "SERVER_BIN=%ROOT_DIR%\build\tests\Release\tobj_socket_server.exe"
if "%CLIENT_BIN%"=="" (
    set "CLIENT_BIN=%ROOT_DIR%\examples\ada\bin\ada_tobj_client.exe"
    if not exist "!CLIENT_BIN!" set "CLIENT_BIN=%ROOT_DIR%\examples\ada\bin\ada_tobj_client"
)

set "PORT_FILE=%TEMP%\ame_socket_e2e_port_%RANDOM%.tmp"
set "SERVER_PID="

echo === Ada Socket E2E Test ===

REM Step 0: Generate Ada service stubs from proto (provided + consumed)
where python >nul 2>&1
if %errorlevel% equ 0 (
    set "ADA_GEN_OUT=%ROOT_DIR%\examples\ada\generated"
    set "GEN_SCRIPT=%ROOT_DIR%\pim\ada_service_generator.py"
    set "PROTO_DIR=%ROOT_DIR%\proto\pyramid\components\tactical_objects\services"
    set "DATA_MODEL_DIR=%ROOT_DIR%\proto\pyramid\data_model"
    echo [driver] Generating Ada types, codecs and service stubs from proto...
    set "GEN_OK=1"
    python "!GEN_SCRIPT!" --types "!DATA_MODEL_DIR!" "!ADA_GEN_OUT!"
    if !errorlevel! neq 0 set "GEN_OK=0"
    python "!GEN_SCRIPT!" --codec "!DATA_MODEL_DIR!" "!ADA_GEN_OUT!"
    if !errorlevel! neq 0 set "GEN_OK=0"
    python "!GEN_SCRIPT!" "!PROTO_DIR!\provided.proto" "!ADA_GEN_OUT!"
    if !errorlevel! neq 0 set "GEN_OK=0"
    python "!GEN_SCRIPT!" "!PROTO_DIR!\consumed.proto" "!ADA_GEN_OUT!"
    if !errorlevel! neq 0 set "GEN_OK=0"
    if "!GEN_OK!"=="0" (
        echo [driver] FAIL: Ada service generation failed
        exit /b 1
    )
    echo [driver] Generated stubs in !ADA_GEN_OUT!
) else (
    echo [driver] python not found -- skipping Ada stub generation
)

REM Step 1: Build Ada client if gprbuild is available
where gprbuild >nul 2>&1
if %errorlevel% equ 0 (
    echo [driver] Building Ada client...
    set "ADA_DIR=%ROOT_DIR%\examples\ada"
    set "ADA_PCL_LIB_DIR=%ROOT_DIR%\build\ada_gnat_pcl"
    set "ADA_PCL_BUILD_SCRIPT=%ROOT_DIR%\scripts\build_gnat_pcl_static_libs.bat"
    set "CAN_BUILD_ADA=1"
    if not exist "!ADA_PCL_LIB_DIR!\libpcl_core.a" (
        echo [driver] Preparing GNAT-compatible PCL static archives...
        call "!ADA_PCL_BUILD_SCRIPT!" "!ADA_PCL_LIB_DIR!"
        if !errorlevel! neq 0 (
            echo [driver] WARNING: failed to build GNAT PCL static archives -- falling back to pre-built binary
            set "CAN_BUILD_ADA=0"
        )
    )
    if "!CAN_BUILD_ADA!"=="1" if not exist "!ADA_PCL_LIB_DIR!\libpcl_transport_socket.a" (
        echo [driver] Preparing GNAT-compatible transport static archive...
        call "!ADA_PCL_BUILD_SCRIPT!" "!ADA_PCL_LIB_DIR!"
        if !errorlevel! neq 0 (
            echo [driver] WARNING: failed to build GNAT transport static archive -- falling back to pre-built binary
            set "CAN_BUILD_ADA=0"
        )
    )
    if "!CAN_BUILD_ADA!"=="1" (
        set "MUJIN_ROOT=%ROOT_DIR%"
        pushd "!ADA_DIR!"
        gprbuild -P ada_tobj_client.gpr -q ^
          -XMUJIN_ROOT=!ROOT_DIR! ^
          -XPCL_INCLUDE_DIR=!ROOT_DIR!\include ^
          -XPCL_LIB_DIR=!ADA_PCL_LIB_DIR! ^
          -XPCL_LIB_NAME=pcl_core ^
          -XPCL_SOCKET_LIB_NAME=pcl_transport_socket >nul 2>&1
        if !errorlevel! neq 0 (
            echo [driver] gprbuild failed -- falling back to pre-built binary
        )
        popd
    )
) else (
    echo [driver] gprbuild not found -- checking for pre-built client...
)

REM Resolve client path
if not exist "%CLIENT_BIN%" (
    if exist "%CLIENT_BIN%.exe" set "CLIENT_BIN=%CLIENT_BIN%.exe"
)
if not exist "%CLIENT_BIN%" (
    echo [driver] SKIP: Ada client binary not found at %CLIENT_BIN%
    exit /b 0
)
if not exist "%SERVER_BIN%" (
    echo [driver] FAIL: Server binary not found at %SERVER_BIN%
    exit /b 1
)

REM Step 2: Start server
echo [driver] Starting server on port %PORT%...
start /b "" "%SERVER_BIN%" --port %PORT% --port-file "%PORT_FILE%" --timeout %TIMEOUT_SEC% >nul 2>&1

REM Step 3: Wait for port file (up to 5 seconds)
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

REM Step 4: Brief delay so server enters accept()
timeout /t 1 /nobreak >nul 2>&1

REM Step 5: Start Ada client
echo [driver] Starting Ada client...
"%CLIENT_BIN%" --host 127.0.0.1 --port %ACTUAL_PORT%
set "CLIENT_EXIT=%errorlevel%"

REM Step 6: Cleanup
call :cleanup

REM Step 7: Report
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
