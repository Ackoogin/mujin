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
set "MANIFEST_FILE=%OUT_DIR%\lib%LIB_NAME%.manifest"
set "RAND_SUFFIX=%RANDOM%%RANDOM%"
set "TMP_LIB_FILE=%OUT_DIR%\lib%LIB_NAME%_%RAND_SUFFIX%.a"
set "TMP_MANIFEST_FILE=%OUT_DIR%\lib%LIB_NAME%_%RAND_SUFFIX%.manifest"

:: Use g++ and all companion tools (ar, ranlib) from the same bin directory so
:: that the compiler, archiver, and symbol-table indexer are always from the
:: same toolchain and produce mutually-compatible object/archive formats.
set "GXX_DIR="
for /f "tokens=*" %%G in ('where g++ 2^>nul') do (
  if not defined GXX_DIR set "GXX_DIR=%%~dpG"
)
if not defined GXX_DIR (
  echo [ada-pyramid] ERROR: g++ not found in PATH
  exit /b 1
)

set "CXX=!GXX_DIR!g++.exe"
set "AR=!GXX_DIR!ar.exe"

echo [ada-pyramid] Compiler : !CXX!
echo [ada-pyramid] Archiver : !AR!

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

if "%FORCE_REBUILD%"=="1" (
  if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
  if exist "%MANIFEST_FILE%" del /f /q "%MANIFEST_FILE%" >nul 2>&1
  del /f /q "%OUT_DIR%\lib%LIB_NAME%_*.a" >nul 2>&1
  del /f /q "%OUT_DIR%\lib%LIB_NAME%_*.manifest" >nul 2>&1
  del /f /q "%OBJ_DIR%\*.o" >nul 2>&1
)

set "GEN_DIR=%PYRAMID_ROOT%\bindings\cpp\generated"
set "GEN_FB_DIR=%GEN_DIR%\flatbuffers\cpp"
set "BUILD_FB_DIR=%WORKSPACE_ROOT%\build\generated\flatbuffers\cpp"
set "FLATBUFFERS_INCLUDE=%WORKSPACE_ROOT%\build\_deps\flatbuffers-src\include"
set "NLOHMANN_INCLUDE=%WORKSPACE_ROOT%\build\_deps\nlohmann_json-src\include"

if not exist "%BUILD_FB_DIR%" (
  where cmake >nul 2>&1
  if errorlevel 1 (
    echo [ada-pyramid] ERROR: missing flatc-generated headers and cmake not available
    exit /b 1
  )
  echo [ada-pyramid] Refreshing flatc-generated headers...
  cmake --build "%WORKSPACE_ROOT%\build" --config Release --target pyramid_flatbuffers_codegen -j4 || exit /b 1
)

set "CXXFLAGS=-std=c++17 -O2 -I%GEN_DIR% -I%GEN_FB_DIR% -I%BUILD_FB_DIR% -I%FLATBUFFERS_INCLUDE% -I%NLOHMANN_INCLUDE%"

if exist "%TMP_MANIFEST_FILE%" del /f /q "%TMP_MANIFEST_FILE%" >nul 2>&1
>"%TMP_MANIFEST_FILE%" echo CXX=!CXX!
>>"%TMP_MANIFEST_FILE%" echo AR=!AR!
>>"%TMP_MANIFEST_FILE%" echo CXXFLAGS=%CXXFLAGS%

set "SOURCE_FILES="
for %%F in ("%GEN_DIR%\pyramid_data_model_*_codec.cpp" "%GEN_FB_DIR%\*_flatbuffers_codec.cpp" "%GEN_DIR%\pyramid_data_model_*_codec.hpp" "%GEN_FB_DIR%\*_flatbuffers_codec.hpp" "%BUILD_FB_DIR%\*_generated.h") do (
  if exist "%%~fF" (
    >>"%TMP_MANIFEST_FILE%" echo %%~fF^|%%~zF^|%%~tF
    if /i "%%~xF"==".cpp" set SOURCE_FILES=!SOURCE_FILES! "%%~fF"
  )
)

if not defined SOURCE_FILES (
  echo [ada-pyramid] ERROR: no generated codec sources found
  if exist "%TMP_MANIFEST_FILE%" del /f /q "%TMP_MANIFEST_FILE%" >nul 2>&1
  exit /b 1
)

set "NEED_REBUILD=%FORCE_REBUILD%"
if not exist "%LIB_FILE%" set "NEED_REBUILD=1"
if not exist "%MANIFEST_FILE%" set "NEED_REBUILD=1"
if "%NEED_REBUILD%"=="0" (
  fc /b "%MANIFEST_FILE%" "%TMP_MANIFEST_FILE%" >nul 2>&1
  if errorlevel 1 set "NEED_REBUILD=1"
)

if "%NEED_REBUILD%"=="0" (
  echo [ada-pyramid] GNAT FlatBuffers archive is up to date in "%OUT_DIR%"
  if exist "%TMP_MANIFEST_FILE%" del /f /q "%TMP_MANIFEST_FILE%" >nul 2>&1
  goto copy_libstdcxx
)

echo [ada-pyramid] Building GNAT-compatible generated FlatBuffers archive in "%OUT_DIR%"

set "OBJECT_FILES="
for %%F in (!SOURCE_FILES!) do (
  echo [ada-pyramid]   compiling %%~nxF
  set "OBJ_FILE=%OBJ_DIR%\%%~nF.o"
  "!CXX!" %CXXFLAGS% -c "%%~fF" -o "!OBJ_FILE!" || exit /b 1
  set OBJECT_FILES=!OBJECT_FILES! "!OBJ_FILE!"
)

if not defined OBJECT_FILES (
  echo [ada-pyramid] ERROR: no generated codec sources found
  exit /b 1
)

if exist "%TMP_LIB_FILE%" del /f /q "%TMP_LIB_FILE%" >nul 2>&1

"!AR!" rcs "%TMP_LIB_FILE%" !OBJECT_FILES! || exit /b 1

if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
move /y "%TMP_LIB_FILE%" "%LIB_FILE%" >nul || exit /b 1
move /y "%TMP_MANIFEST_FILE%" "%MANIFEST_FILE%" >nul || exit /b 1

:: OUT_DIR is on GNAT's linker -L path but GNAT's ld doesn't know about
:: the g++ installation's lib directory.  Copy libstdc++.a (and the DLL
:: import stub) from g++'s own lib into OUT_DIR so -lstdc++ resolves there.
:copy_libstdcxx
set "LIBSTDCXX="
for /f "tokens=*" %%L in ('"!CXX!" -print-file-name=libstdc++.a 2^>nul') do set "LIBSTDCXX=%%L"
if defined LIBSTDCXX if not "!LIBSTDCXX!"=="libstdc++.a" (
  echo [ada-pyramid] Copying libstdc++ from !LIBSTDCXX! to "%OUT_DIR%"
  copy /y "!LIBSTDCXX!" "%OUT_DIR%\" >nul
  for %%D in ("!LIBSTDCXX!") do (
    if exist "%%~dpDlibstdc++.dll.a" copy /y "%%~dpDlibstdc++.dll.a" "%OUT_DIR%\" >nul
  )
) else (
  echo [ada-pyramid] WARNING: libstdc++.a not found via '!CXX! -print-file-name'; link may fail with cannot find -lstdc++
)

echo [ada-pyramid] Ready:
echo [ada-pyramid]   %LIB_FILE%
exit /b 0
