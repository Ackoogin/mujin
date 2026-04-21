@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "PYRAMID_ROOT=%%~fI"
for %%I in ("%PYRAMID_ROOT%\..\..") do set "WORKSPACE_ROOT=%%~fI"

set "OUT_DIR=%~1"
if "%OUT_DIR%"=="" set "OUT_DIR=%WORKSPACE_ROOT%\build\ada_gnat_pyramid"
set "OBJ_DIR=%OUT_DIR%\obj"
set "FORCE_REBUILD=0"

if /i "%~2"=="--force" set "FORCE_REBUILD=1"

set "LIB_NAME=pyramid_generated_flatbuffers_codec"
set "LIB_FILE=%OUT_DIR%\lib%LIB_NAME%.a"
set "RAND_SUFFIX=%RANDOM%%RANDOM%"
set "TMP_LIB_FILE=%OUT_DIR%\lib%LIB_NAME%_%RAND_SUFFIX%.a"

if "%FORCE_REBUILD%"=="0" if exist "%LIB_FILE%" (
  echo [ada-pyramid] GNAT FlatBuffers archive already present in "%OUT_DIR%"
  exit /b 0
)

:: Locate GNAT toolchain (next to gprbuild).  GNAT gcc is Ada/C only (no cc1plus),
:: so system g++ is used for C++ compilation.  We query GNAT gcc for its target
:: triple to derive the right -m32/-m64 flag, ensuring the objects match the
:: architecture GNAT links for.  GNAT's ar is used for a compatible archive index.
:: libstdc++.a from the g++ installation is copied into OUT_DIR so that GNAT's
:: linker (which only searches GNAT's own lib paths) can satisfy -lstdc++.
set "GNAT_BIN="
for /f "tokens=*" %%G in ('where gprbuild 2^>nul') do (
  if not defined GNAT_BIN set "GNAT_BIN=%%~dpG"
)

set "AR_CMD="
set "ARCH_FLAG="
if defined GNAT_BIN (
  if exist "!GNAT_BIN!gcc-ar.exe" ( set "AR_CMD=!GNAT_BIN!gcc-ar.exe"
  ) else if exist "!GNAT_BIN!ar.exe" ( set "AR_CMD=!GNAT_BIN!ar.exe" )

  if exist "!GNAT_BIN!gcc.exe" (
    for /f "tokens=*" %%T in ('"!GNAT_BIN!gcc.exe" -dumpmachine 2^>nul') do set "GNAT_TARGET=%%T"
    echo [ada-pyramid] GNAT target : !GNAT_TARGET!
    echo !GNAT_TARGET! | findstr /i "i686 i386 i486 i586" >nul 2>&1
    if not errorlevel 1 set "ARCH_FLAG=-m32"
    echo !GNAT_TARGET! | findstr /i "x86_64 amd64" >nul 2>&1
    if not errorlevel 1 set "ARCH_FLAG=-m64"
  )
)

if not defined AR_CMD (
  where ar >nul 2>&1
  if errorlevel 1 (
    echo [ada-pyramid] ERROR: no ar found (checked GNAT bin and PATH^)
    exit /b 1
  )
  set "AR_CMD=ar"
)

where g++ >nul 2>&1
if errorlevel 1 (
  echo [ada-pyramid] ERROR: g++ not found in PATH
  exit /b 1
)

echo [ada-pyramid] Compiler : g++ !ARCH_FLAG!
echo [ada-pyramid] Archiver : !AR_CMD!

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

if "%FORCE_REBUILD%"=="1" (
  if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
  del /f /q "%OUT_DIR%\lib%LIB_NAME%_*.a" >nul 2>&1
  del /f /q "%OBJ_DIR%\*.o" >nul 2>&1
)

set "GEN_DIR=%PYRAMID_ROOT%\bindings\cpp\generated"
set "GEN_FB_DIR=%GEN_DIR%\flatbuffers\cpp"
set "BUILD_FB_DIR=%WORKSPACE_ROOT%\build\generated\flatbuffers\cpp"
set "FLATBUFFERS_INCLUDE=%WORKSPACE_ROOT%\build\_deps\flatbuffers-src\include"
set "NLOHMANN_INCLUDE=%WORKSPACE_ROOT%\build\_deps\nlohmann_json-src\include"

if not exist "%BUILD_FB_DIR%\pyramid_services_tactical_objects_generated.h" (
  where cmake >nul 2>&1
  if errorlevel 1 (
    echo [ada-pyramid] ERROR: missing flatc-generated header and cmake not available
    exit /b 1
  )
  echo [ada-pyramid] Refreshing flatc-generated headers...
  cmake --build "%WORKSPACE_ROOT%\build" --config Release --target pyramid_flatbuffers_codegen -j4 || exit /b 1
)

echo [ada-pyramid] Building GNAT-compatible generated FlatBuffers archive in "%OUT_DIR%"

set "CXXFLAGS=!ARCH_FLAG! -std=c++17 -O2 -I%GEN_DIR% -I%GEN_FB_DIR% -I%BUILD_FB_DIR% -I%FLATBUFFERS_INCLUDE% -I%NLOHMANN_INCLUDE%"

g++ %CXXFLAGS% -c "%GEN_DIR%\pyramid_data_model_base_codec.cpp"    -o "%OBJ_DIR%\pyramid_data_model_base_codec.o"    || exit /b 1
g++ %CXXFLAGS% -c "%GEN_DIR%\pyramid_data_model_common_codec.cpp"  -o "%OBJ_DIR%\pyramid_data_model_common_codec.o"  || exit /b 1
g++ %CXXFLAGS% -c "%GEN_DIR%\pyramid_data_model_tactical_codec.cpp" -o "%OBJ_DIR%\pyramid_data_model_tactical_codec.o" || exit /b 1
g++ %CXXFLAGS% -c "%GEN_FB_DIR%\pyramid_services_tactical_objects_flatbuffers_codec.cpp" -o "%OBJ_DIR%\pyramid_services_tactical_objects_flatbuffers_codec.o" || exit /b 1

if exist "%TMP_LIB_FILE%" del /f /q "%TMP_LIB_FILE%" >nul 2>&1

"!AR_CMD!" rcs "%TMP_LIB_FILE%" ^
  "%OBJ_DIR%\pyramid_data_model_base_codec.o" ^
  "%OBJ_DIR%\pyramid_data_model_common_codec.o" ^
  "%OBJ_DIR%\pyramid_data_model_tactical_codec.o" ^
  "%OBJ_DIR%\pyramid_services_tactical_objects_flatbuffers_codec.o" || exit /b 1

if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
move /y "%TMP_LIB_FILE%" "%LIB_FILE%" >nul || exit /b 1

:: Copy libstdc++ from g++'s own installation into OUT_DIR so GNAT's linker
:: (which only searches GNAT's own lib paths) can satisfy -lstdc++ at link time.
set "GXX_LIBSTDCXX="
for /f "tokens=*" %%L in ('g++ !ARCH_FLAG! -print-file-name=libstdc++.a 2^>nul') do set "GXX_LIBSTDCXX=%%L"
if defined GXX_LIBSTDCXX if not "!GXX_LIBSTDCXX!"=="libstdc++.a" (
  for %%D in ("!GXX_LIBSTDCXX!") do set "GXX_LIB_DIR=%%~dpD"
  echo [ada-pyramid] Copying libstdc++ from !GXX_LIB_DIR! to "%OUT_DIR%"
  if exist "!GXX_LIB_DIR!libstdc++.a"     copy /y "!GXX_LIB_DIR!libstdc++.a"     "%OUT_DIR%\" >nul
  if exist "!GXX_LIB_DIR!libstdc++.dll.a" copy /y "!GXX_LIB_DIR!libstdc++.dll.a" "%OUT_DIR%\" >nul
) else (
  echo [ada-pyramid] WARNING: 32-bit libstdc++ not found via 'g++ !ARCH_FLAG! -print-file-name'; link may fail with cannot find -lstdc++
)

echo [ada-pyramid] Built:
echo [ada-pyramid]   %LIB_FILE%
exit /b 0
