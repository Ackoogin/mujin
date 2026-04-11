@echo off
REM test_tobj_master_conformance.bat -- Master cross-language conformance driver.
REM
REM Executes the current Tactical Objects matrix:
REM   1. socket + JSON
REM   2. socket + FlatBuffers
REM   3. gRPC + Protobuf
setlocal enabledelayedexpansion

set "SOCKET_SERVER_BIN="
set "BRIDGE_BIN="
set "SOCKET_CLIENT_BIN="
set "GRPC_SERVER_BIN="
set "GRPC_CLIENT_BIN="
set "GRPC_DLL_BIN="
for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"

if not defined PYRAMID_ROOT (
    for %%I in ("%SCRIPT_BASE%..") do set "PYRAMID_ROOT=%%~fI"
)

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--socket-server-bin" (set "SOCKET_SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--bridge-bin" (set "BRIDGE_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--socket-client-bin" (set "SOCKET_CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--grpc-server-bin" (set "GRPC_SERVER_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--grpc-client-bin" (set "GRPC_CLIENT_BIN=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--grpc-dll-bin" (set "GRPC_DLL_BIN=%~2" & shift & shift & goto parse_args)
shift
goto parse_args
:done_args

if "%SOCKET_SERVER_BIN%"=="" set "SOCKET_SERVER_BIN=%PYRAMID_ROOT%\..\..\build\subprojects\PYRAMID\tests\Release\tobj_socket_server.exe"
if "%BRIDGE_BIN%"=="" set "BRIDGE_BIN=%PYRAMID_ROOT%\..\..\build\subprojects\PYRAMID\tests\Release\standalone_bridge.exe"
if "%SOCKET_CLIENT_BIN%"=="" set "SOCKET_CLIENT_BIN=%PYRAMID_ROOT%\examples\ada\bin\ada_active_find_e2e.exe"
if "%GRPC_SERVER_BIN%"=="" set "GRPC_SERVER_BIN=%PYRAMID_ROOT%\..\..\build\subprojects\PYRAMID\tests\Release\tobj_grpc_server.exe"
if "%GRPC_CLIENT_BIN%"=="" set "GRPC_CLIENT_BIN=%PYRAMID_ROOT%\examples\ada\bin\ada_grpc_cpp_interop_e2e.exe"
if "%GRPC_DLL_BIN%"=="" set "GRPC_DLL_BIN=%PYRAMID_ROOT%\..\..\build\subprojects\PYRAMID\tests\Release\pyramid_grpc_ada_interop_shim.dll"

echo === Tactical Objects Master Conformance ===

call "%SCRIPT_BASE%test_ada_active_find_e2e.bat" --server-bin "%SOCKET_SERVER_BIN%" --bridge-bin "%BRIDGE_BIN%" --client-bin "%SOCKET_CLIENT_BIN%" --content-type application/json
if errorlevel 1 (
    echo [master] FAIL: socket/json leg failed
    exit /b 1
)

call "%SCRIPT_BASE%test_ada_active_find_e2e.bat" --server-bin "%SOCKET_SERVER_BIN%" --bridge-bin "%BRIDGE_BIN%" --client-bin "%SOCKET_CLIENT_BIN%" --content-type application/flatbuffers
if errorlevel 1 (
    echo [master] FAIL: socket/flatbuffers leg failed
    exit /b 1
)

call "%SCRIPT_BASE%test_ada_grpc_cpp_interop_e2e.bat" --server-bin "%GRPC_SERVER_BIN%" --client-bin "%GRPC_CLIENT_BIN%" --dll-bin "%GRPC_DLL_BIN%"
if errorlevel 1 (
    echo [master] FAIL: grpc/protobuf leg failed
    exit /b 1
)

echo [master] PASS: all currently available Tactical Objects transport/codec paths conformed
exit /b 0
