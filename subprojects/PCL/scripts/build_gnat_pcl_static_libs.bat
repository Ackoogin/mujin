@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "ROOT_DIR=%%~fI"

set "OUT_DIR=%~1"
if "%OUT_DIR%"=="" set "OUT_DIR=%ROOT_DIR%\build\ada_gnat_pcl"
set "OBJ_DIR=%OUT_DIR%\obj"
set "FORCE_REBUILD=0"

if /i "%~2"=="--force" set "FORCE_REBUILD=1"

set "CORE_LIB=%OUT_DIR%\libpcl_core.a"
set "SOCKET_LIB=%OUT_DIR%\libpcl_transport_socket.a"

if "%FORCE_REBUILD%"=="0" if exist "%CORE_LIB%" if exist "%SOCKET_LIB%" (
  echo [ada-pcl] GNAT static archives already present in "%OUT_DIR%"
  exit /b 0
)

where gcc >nul 2>&1
if errorlevel 1 (
  echo [ada-pcl] ERROR: gcc not found in PATH
  exit /b 1
)

where ar >nul 2>&1
if errorlevel 1 (
  echo [ada-pcl] ERROR: ar not found in PATH
  exit /b 1
)

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

if "%FORCE_REBUILD%"=="1" (
  if exist "%CORE_LIB%" del /f /q "%CORE_LIB%" >nul 2>&1
  if exist "%SOCKET_LIB%" del /f /q "%SOCKET_LIB%" >nul 2>&1
  del /f /q "%OBJ_DIR%\*.o" >nul 2>&1
)

echo [ada-pcl] Building GNAT-compatible PCL archives in "%OUT_DIR%"

set "CFLAGS=-std=c11 -O2 -I%ROOT_DIR%\include -I%ROOT_DIR%\src"

gcc %CFLAGS% -c "%ROOT_DIR%\src\pcl_container.c" -o "%OBJ_DIR%\pcl_container.o" || exit /b 1
gcc %CFLAGS% -c "%ROOT_DIR%\src\pcl_executor.c" -o "%OBJ_DIR%\pcl_executor.o" || exit /b 1
gcc %CFLAGS% -c "%ROOT_DIR%\src\pcl_log.c" -o "%OBJ_DIR%\pcl_log.o" || exit /b 1
gcc %CFLAGS% -c "%ROOT_DIR%\src\pcl_bridge.c" -o "%OBJ_DIR%\pcl_bridge.o" || exit /b 1
gcc %CFLAGS% -c "%ROOT_DIR%\src\pcl_transport_socket.c" -o "%OBJ_DIR%\pcl_transport_socket.o" || exit /b 1

ar rcs "%CORE_LIB%" "%OBJ_DIR%\pcl_container.o" "%OBJ_DIR%\pcl_executor.o" "%OBJ_DIR%\pcl_log.o" "%OBJ_DIR%\pcl_bridge.o" || exit /b 1
ar rcs "%SOCKET_LIB%" "%OBJ_DIR%\pcl_transport_socket.o" || exit /b 1

echo [ada-pcl] Built:
echo [ada-pcl]   %CORE_LIB%
echo [ada-pcl]   %SOCKET_LIB%

exit /b 0
