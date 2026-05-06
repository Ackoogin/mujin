@echo off
REM test_ada_grpc_cpp_interop_e2e.bat -- Cross-process Ada/C++ gRPC interop.
REM
REM Orchestrates:
REM   1. Build Ada gRPC client if gprbuild is available
REM   2. Start standalone C++ gRPC server
REM   3. Wait for ready file
REM   4. Run Ada client through the standard generated service facade
REM   5. Terminate the C++ server and report pass/fail
setlocal enabledelayedexpansion

set "SERVER_BIN="
set "CLIENT_BIN="
set "DLL_BIN="
set "ADDRESS=auto"
set "TIMEOUT_SEC=20"
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
if /i "%~1"=="--server-bin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--client-bin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--dll-bin" (set "DLL_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--address" (set "ADDRESS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--config" (set "BUILD_CONFIG=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ServerBin" (set "SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-ClientBin" (set "CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-DllBin" (set "DLL_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Address" (set "ADDRESS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Timeout" (set "TIMEOUT_SEC=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-BuildDir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="-Config" (set "BUILD_CONFIG=%~2" & shift & shift & goto parse_args)
shift
goto parse_args
:done_args

if not defined BUILD_DIR set "BUILD_DIR=%WORKSPACE_ROOT%\build"
if not defined BUILD_CONFIG set "BUILD_CONFIG=Release"
if "%SERVER_BIN%"=="" set "SERVER_BIN=%BUILD_DIR%\subprojects\PYRAMID\tests\%BUILD_CONFIG%\tobj_grpc_server.exe"
if "%CLIENT_BIN%"=="" (
    set "CLIENT_BIN=%PYRAMID_ROOT%\tests\ada\bin\ada_grpc_cpp_interop_e2e.exe"
    if not exist "!CLIENT_BIN!" set "CLIENT_BIN=%PYRAMID_ROOT%\tests\ada\bin\ada_grpc_cpp_interop_e2e"
)
if "%DLL_BIN%"=="" set "DLL_BIN=%BUILD_DIR%\subprojects\PYRAMID\tests\%BUILD_CONFIG%\pyramid_grpc_ada_interop_shim.dll"

set "READY_FILE=%TEMP%\pyramid_grpc_ready_%RANDOM%.tmp"

echo === Ada gRPC C++ Interop E2E Test ===

where gprbuild >nul 2>&1
if %errorlevel% equ 0 (
    echo [driver] Building Ada gRPC client...
    set "ADA_DIR=%PYRAMID_ROOT%\tests\ada"
    pushd "!ADA_DIR!"
    gprbuild -P ada_grpc_cpp_interop_e2e.gpr -q -XUNMANNED_ROOT="%WORKSPACE_ROOT%" -XPCL_LIB_DIR="%BUILD_DIR%\ada_gnat_pcl" -XPYRAMID_GEN_LIB_DIR="%BUILD_DIR%\ada_gnat_pyramid" >nul 2>&1
    if !errorlevel! neq 0 (
        echo [driver] gprbuild failed -- falling back to pre-built binary
    )
    popd
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
if not exist "%SERVER_BIN%" (
    echo [driver] FAIL: gRPC server binary not found at %SERVER_BIN%
    exit /b 1
)
if not exist "%DLL_BIN%" (
    echo [driver] FAIL: gRPC shim DLL not found at %DLL_BIN%
    exit /b 1
)

if /i "%ADDRESS%"=="auto" set "ADDRESS="
if "%ADDRESS%"=="" (
    call :pick_loopback_address
    if errorlevel 1 (
        echo [driver] FAIL: could not allocate a loopback gRPC port
        exit /b 1
    )
)

echo [driver] Starting gRPC server on %ADDRESS%...
start /b "" "%SERVER_BIN%" --address "%ADDRESS%" --ready-file "%READY_FILE%" --timeout %TIMEOUT_SEC% >nul 2>&1

echo [driver] Waiting for gRPC server ready file...
set "ATTEMPTS=0"
:wait_ready
if %ATTEMPTS% geq 50 goto ready_timeout
timeout /t 1 /nobreak >nul 2>&1
ping -n 1 -w 100 127.0.0.1 >nul 2>&1
if exist "%READY_FILE%" (
    for %%S in ("%READY_FILE%") do if %%~zS gtr 0 goto ready_ok
)
set /a ATTEMPTS+=1
goto wait_ready

:ready_timeout
echo [driver] FAIL: gRPC server did not become ready within 5 seconds
goto cleanup_fail

:ready_ok
echo [driver] gRPC server ready on %ADDRESS%
timeout /t 1 /nobreak >nul 2>&1

echo [driver] Starting Ada gRPC client via generated service facade...
"%CLIENT_BIN%" --dll "%DLL_BIN%" --address "%ADDRESS%"
set "CLIENT_EXIT=%errorlevel%"

call :cleanup

if %CLIENT_EXIT% equ 0 (
    echo [driver] PASS: Ada standard facade exchanged a mid-complexity gRPC/protobuf message
    exit /b 0
) else (
    echo [driver] FAIL: Ada gRPC client exited with code %CLIENT_EXIT%
    exit /b 1
)

:cleanup_fail
call :cleanup
exit /b 1

:cleanup
taskkill /f /im tobj_grpc_server.exe >nul 2>&1
if exist "%READY_FILE%" del /f "%READY_FILE%" >nul 2>&1
goto :eof

:pick_loopback_address
set "GRPC_PORT="
set "PORT_FILE=%TEMP%\pyramid_grpc_port_%RANDOM%.tmp"
python "%SCRIPT_BASE%pick_loopback_port.py" > "%PORT_FILE%" 2>nul
if exist "%PORT_FILE%" (
    set /p GRPC_PORT=<"%PORT_FILE%"
    del /f "%PORT_FILE%" >nul 2>&1
)
if not defined GRPC_PORT exit /b 1
set "ADDRESS=127.0.0.1:%GRPC_PORT%"
goto :eof
