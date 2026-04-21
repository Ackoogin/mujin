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
set "SHMEM_LIB=%OUT_DIR%\libpcl_transport_shared_memory.a"
set "RAND_SUFFIX=%RANDOM%%RANDOM%"
set "CORE_TMP=%OUT_DIR%\libpcl_core_%RAND_SUFFIX%.a"
set "SOCKET_TMP=%OUT_DIR%\libpcl_transport_socket_%RAND_SUFFIX%.a"
set "SHMEM_TMP=%OUT_DIR%\libpcl_transport_shared_memory_%RAND_SUFFIX%.a"

if "%FORCE_REBUILD%"=="0" if exist "%CORE_LIB%" if exist "%SOCKET_LIB%" if exist "%SHMEM_LIB%" (
  echo [ada-pcl] GNAT static archives already present in "%OUT_DIR%"
  exit /b 0
)

:: Prefer GNAT's own gcc and ar so objects and archive index are in the
:: i686-pc-mingw32 PE/COFF format that GNAT's linker expects.  GNAT gcc
:: supports C (cc1) even though it lacks C++ (cc1plus), so it can compile
:: all PCL sources directly.  Fall back to system gcc/ar with a warning.
set "GNAT_BIN="
for /f "tokens=*" %%G in ('where gprbuild 2^>nul') do (
  if not defined GNAT_BIN set "GNAT_BIN=%%~dpG"
)

set "CC_CMD="
set "AR_CMD="
set "ARCH_FLAG="
if defined GNAT_BIN (
  if exist "!GNAT_BIN!gcc.exe"    set "CC_CMD=!GNAT_BIN!gcc.exe"
  if exist "!GNAT_BIN!gcc-ar.exe" ( set "AR_CMD=!GNAT_BIN!gcc-ar.exe"
  ) else if exist "!GNAT_BIN!ar.exe" ( set "AR_CMD=!GNAT_BIN!ar.exe" )

  if exist "!GNAT_BIN!gcc.exe" (
    for /f "tokens=*" %%T in ('"!GNAT_BIN!gcc.exe" -dumpmachine 2^>nul') do set "GNAT_TARGET=%%T"
    echo [ada-pcl] GNAT target : !GNAT_TARGET!
    echo !GNAT_TARGET! | findstr /i "i686 i386 i486 i586" >nul 2>&1
    if not errorlevel 1 set "ARCH_FLAG=-m32"
    echo !GNAT_TARGET! | findstr /i "x86_64 amd64" >nul 2>&1
    if not errorlevel 1 set "ARCH_FLAG=-m64"
  )
)

if not defined CC_CMD (
  where gcc >nul 2>&1
  if errorlevel 1 (
    echo [ada-pcl] ERROR: GNAT gcc not found and gcc not found in PATH
    exit /b 1
  )
  set "CC_CMD=gcc"
  echo [ada-pcl] WARNING: GNAT gcc not found; using system gcc - link may fail if it targets a different architecture
)

if not defined AR_CMD (
  where ar >nul 2>&1
  if errorlevel 1 (
    echo [ada-pcl] ERROR: no ar found (checked GNAT bin and PATH^)
    exit /b 1
  )
  set "AR_CMD=ar"
)

echo [ada-pcl] Compiler : !CC_CMD!
echo [ada-pcl] Archiver : !AR_CMD!

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

if "%FORCE_REBUILD%"=="1" (
  if exist "%CORE_LIB%" del /f /q "%CORE_LIB%" >nul 2>&1
  if exist "%SOCKET_LIB%" del /f /q "%SOCKET_LIB%" >nul 2>&1
  if exist "%SHMEM_LIB%" del /f /q "%SHMEM_LIB%" >nul 2>&1
  del /f /q "%OUT_DIR%\libpcl_core_*.a" >nul 2>&1
  del /f /q "%OUT_DIR%\libpcl_transport_socket_*.a" >nul 2>&1
  del /f /q "%OUT_DIR%\libpcl_transport_shared_memory_*.a" >nul 2>&1
  del /f /q "%OBJ_DIR%\*.o" >nul 2>&1
)

echo [ada-pcl] Building GNAT-compatible PCL archives in "%OUT_DIR%"

set "CFLAGS=!ARCH_FLAG! -std=c11 -O2 -I%ROOT_DIR%\include -I%ROOT_DIR%\src"

"!CC_CMD!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_container.c"               -o "%OBJ_DIR%\pcl_container.o"               || exit /b 1
"!CC_CMD!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_executor.c"                -o "%OBJ_DIR%\pcl_executor.o"                || exit /b 1
"!CC_CMD!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_log.c"                     -o "%OBJ_DIR%\pcl_log.o"                     || exit /b 1
"!CC_CMD!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_bridge.c"                  -o "%OBJ_DIR%\pcl_bridge.o"                  || exit /b 1
"!CC_CMD!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_transport_socket.c"        -o "%OBJ_DIR%\pcl_transport_socket.o"        || exit /b 1
"!CC_CMD!" %CFLAGS% -c "%ROOT_DIR%\src\pcl_transport_shared_memory.c" -o "%OBJ_DIR%\pcl_transport_shared_memory.o" || exit /b 1

if exist "%CORE_TMP%"   del /f /q "%CORE_TMP%"   >nul 2>&1
if exist "%SOCKET_TMP%" del /f /q "%SOCKET_TMP%" >nul 2>&1
if exist "%SHMEM_TMP%"  del /f /q "%SHMEM_TMP%"  >nul 2>&1

"!AR_CMD!" rcs "%CORE_TMP%"   "%OBJ_DIR%\pcl_container.o" "%OBJ_DIR%\pcl_executor.o" "%OBJ_DIR%\pcl_log.o" "%OBJ_DIR%\pcl_bridge.o" || exit /b 1
"!AR_CMD!" rcs "%SOCKET_TMP%" "%OBJ_DIR%\pcl_transport_socket.o"        || exit /b 1
"!AR_CMD!" rcs "%SHMEM_TMP%"  "%OBJ_DIR%\pcl_transport_shared_memory.o" || exit /b 1

if exist "%CORE_LIB%"   del /f /q "%CORE_LIB%"   >nul 2>&1
if exist "%SOCKET_LIB%" del /f /q "%SOCKET_LIB%" >nul 2>&1
if exist "%SHMEM_LIB%"  del /f /q "%SHMEM_LIB%"  >nul 2>&1
move /y "%CORE_TMP%"   "%CORE_LIB%"   >nul || exit /b 1
move /y "%SOCKET_TMP%" "%SOCKET_LIB%" >nul || exit /b 1
move /y "%SHMEM_TMP%"  "%SHMEM_LIB%"  >nul || exit /b 1

echo [ada-pcl] Built:
echo [ada-pcl]   %CORE_LIB%
echo [ada-pcl]   %SOCKET_LIB%
echo [ada-pcl]   %SHMEM_LIB%

exit /b 0
