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
set "UDP_LIB=%OUT_DIR%\libpcl_transport_udp.a"
set "SHMEM_LIB=%OUT_DIR%\libpcl_transport_shared_memory.a"
set "RAND_SUFFIX=%RANDOM%%RANDOM%"
set "CORE_TMP=%OUT_DIR%\libpcl_core_%RAND_SUFFIX%.a"
set "SOCKET_TMP=%OUT_DIR%\libpcl_transport_socket_%RAND_SUFFIX%.a"
set "UDP_TMP=%OUT_DIR%\libpcl_transport_udp_%RAND_SUFFIX%.a"
set "SHMEM_TMP=%OUT_DIR%\libpcl_transport_shared_memory_%RAND_SUFFIX%.a"

if "%FORCE_REBUILD%"=="0" if exist "%CORE_LIB%" if exist "%SOCKET_LIB%" if exist "%UDP_LIB%" if exist "%SHMEM_LIB%" (
  echo [ada-pcl] GNAT static archives already present in "%OUT_DIR%"
  exit /b 0
)

:: Use gcc and all companion tools (ar) from the same bin directory as g++ so
:: that the compiler and archiver are always from the same toolchain and produce
:: mutually-compatible object/archive formats.
set "GXX_DIR="
for /f "tokens=*" %%G in ('where g++ 2^>nul') do (
  if not defined GXX_DIR set "GXX_DIR=%%~dpG"
)
if not defined GXX_DIR (
  echo [ada-pcl] ERROR: g++ not found in PATH
  exit /b 1
)

set "CC=!GXX_DIR!gcc.exe"
set "AR=!GXX_DIR!ar.exe"

echo [ada-pcl] Compiler : !CC!
echo [ada-pcl] Archiver : !AR!

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

if "%FORCE_REBUILD%"=="1" (
  if exist "%CORE_LIB%" del /f /q "%CORE_LIB%" >nul 2>&1
  if exist "%SOCKET_LIB%" del /f /q "%SOCKET_LIB%" >nul 2>&1
  if exist "%UDP_LIB%" del /f /q "%UDP_LIB%" >nul 2>&1
  if exist "%SHMEM_LIB%" del /f /q "%SHMEM_LIB%" >nul 2>&1
  del /f /q "%OUT_DIR%\libpcl_core_*.a" >nul 2>&1
  del /f /q "%OUT_DIR%\libpcl_transport_socket_*.a" >nul 2>&1
  del /f /q "%OUT_DIR%\libpcl_transport_udp_*.a" >nul 2>&1
  del /f /q "%OUT_DIR%\libpcl_transport_shared_memory_*.a" >nul 2>&1
  del /f /q "%OBJ_DIR%\*.o" >nul 2>&1
)

echo [ada-pcl] Building GNAT-compatible PCL archives in "%OUT_DIR%"

set "CFLAGS=-std=c11 -O2 -I%ROOT_DIR%\include -I%ROOT_DIR%\src"

"!CC!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_container.c"               -o "%OBJ_DIR%\pcl_container.o"               || exit /b 1
"!CC!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_executor.c"                -o "%OBJ_DIR%\pcl_executor.o"                || exit /b 1
"!CC!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_log.c"                     -o "%OBJ_DIR%\pcl_log.o"                     || exit /b 1
"!CC!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_bridge.c"                  -o "%OBJ_DIR%\pcl_bridge.o"                  || exit /b 1
"!CC!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_transport_socket.c"        -o "%OBJ_DIR%\pcl_transport_socket.o"        || exit /b 1
"!CC!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_transport_udp.c"           -o "%OBJ_DIR%\pcl_transport_udp.o"           || exit /b 1
"!CC!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_transport_shared_memory.c" -o "%OBJ_DIR%\pcl_transport_shared_memory.o" || exit /b 1

if exist "%CORE_TMP%"   del /f /q "%CORE_TMP%"   >nul 2>&1
if exist "%SOCKET_TMP%" del /f /q "%SOCKET_TMP%" >nul 2>&1
if exist "%UDP_TMP%"    del /f /q "%UDP_TMP%"    >nul 2>&1
if exist "%SHMEM_TMP%"  del /f /q "%SHMEM_TMP%"  >nul 2>&1

"!AR!" rcs "%CORE_TMP%"   "%OBJ_DIR%\pcl_container.o" "%OBJ_DIR%\pcl_executor.o" "%OBJ_DIR%\pcl_log.o" "%OBJ_DIR%\pcl_bridge.o" || exit /b 1
"!AR!" rcs "%SOCKET_TMP%" "%OBJ_DIR%\pcl_transport_socket.o"        || exit /b 1
"!AR!" rcs "%UDP_TMP%"    "%OBJ_DIR%\pcl_transport_udp.o"           || exit /b 1
"!AR!" rcs "%SHMEM_TMP%"  "%OBJ_DIR%\pcl_transport_shared_memory.o" || exit /b 1

if exist "%CORE_LIB%"   del /f /q "%CORE_LIB%"   >nul 2>&1
if exist "%SOCKET_LIB%" del /f /q "%SOCKET_LIB%" >nul 2>&1
if exist "%UDP_LIB%"    del /f /q "%UDP_LIB%"    >nul 2>&1
if exist "%SHMEM_LIB%"  del /f /q "%SHMEM_LIB%"  >nul 2>&1
move /y "%CORE_TMP%"   "%CORE_LIB%"   >nul || exit /b 1
move /y "%SOCKET_TMP%" "%SOCKET_LIB%" >nul || exit /b 1
move /y "%UDP_TMP%"    "%UDP_LIB%"    >nul || exit /b 1
move /y "%SHMEM_TMP%"  "%SHMEM_LIB%"  >nul || exit /b 1

echo [ada-pcl] Built:
echo [ada-pcl]   %CORE_LIB%
echo [ada-pcl]   %SOCKET_LIB%
echo [ada-pcl]   %UDP_LIB%
echo [ada-pcl]   %SHMEM_LIB%

exit /b 0
