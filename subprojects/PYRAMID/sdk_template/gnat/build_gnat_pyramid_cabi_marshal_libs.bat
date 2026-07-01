@echo off
REM build_gnat_pyramid_cabi_marshal_libs.bat -- SDK counterpart of the
REM monorepo's subprojects/PYRAMID/scripts/build_gnat_pyramid_cabi_marshal_libs.bat.
REM
REM Compiles the GENERATED (proto-dependent) C-ABI marshalling sources -- the
REM only pieces of the generated C++ bindings an Ada client needs, so it can
REM build/tear down the pyramid_<Type>_c structs that cross a codec plugin's
REM vtable -- into a GNAT-compatible static archive. This cannot be prebuilt
REM by the SDK maintainer because its content depends on the user's own
REM proto/ tree; run scripts\generate_bindings.bat first.
REM
REM Usage: gnat\build_gnat_pyramid_cabi_marshal_libs.bat [OUT_DIR] [--force]
REM Default OUT_DIR: <sdk_root>\gnat\build
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "SDK_ROOT=%%~fI"

set "OUT_DIR="
set "FORCE_REBUILD=0"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--force" (set "FORCE_REBUILD=1" & shift & goto parse_args)
if not defined OUT_DIR set "OUT_DIR=%~1"
shift
goto parse_args
:done_args

if "%OUT_DIR%"=="" set "OUT_DIR=%SDK_ROOT%\gnat\build"
set "OBJ_DIR=%OUT_DIR%\obj"

set "LIB_NAME=pyramid_generated_cabi_marshal"
set "LIB_FILE=%OUT_DIR%\lib%LIB_NAME%.a"
set "MANIFEST_FILE=%OUT_DIR%\lib%LIB_NAME%.manifest"
set "RAND_SUFFIX=%RANDOM%%RANDOM%"
set "TMP_LIB_FILE=%OUT_DIR%\lib%LIB_NAME%_%RAND_SUFFIX%.a"
set "TMP_MANIFEST_FILE=%OUT_DIR%\lib%LIB_NAME%_%RAND_SUFFIX%.manifest"

:: Use g++ and all companion tools (ar) from the same bin directory so the
:: compiler and archiver are always from the same toolchain.
set "GXX_DIR="
for /f "tokens=*" %%G in ('where g++ 2^>nul') do (
  if not defined GXX_DIR set "GXX_DIR=%%~dpG"
)
if not defined GXX_DIR (
  echo [sdk-ada] ERROR: g++ not found in PATH
  exit /b 1
)

set "CXX=!GXX_DIR!g++.exe"
set "AR=!GXX_DIR!ar.exe"

echo [sdk-ada] Compiler : !CXX!
echo [sdk-ada] Archiver : !AR!

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

if "%FORCE_REBUILD%"=="1" (
  if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
  if exist "%MANIFEST_FILE%" del /f /q "%MANIFEST_FILE%" >nul 2>&1
  del /f /q "%OUT_DIR%\lib%LIB_NAME%_*.a" >nul 2>&1
  del /f /q "%OUT_DIR%\lib%LIB_NAME%_*.manifest" >nul 2>&1
  del /f /q "%OBJ_DIR%\*.o" >nul 2>&1
)

REM NOTE: this archive covers to_c/from_c/_c_free only. Ada never calls the
REM wire-codec (toJson/fromJson/toBinary/fromBinary) functions directly -- it
REM has its own pure-Ada JSON codec, and wire encode/decode goes through a
REM loaded codec plugin DLL -- so wire-codec sources are not compiled here.
set "GEN_DIR=%SDK_ROOT%\sdk_project\generated"
set "CORE_EXTERNAL_INCLUDE=%SDK_ROOT%\include\external"
set "PCL_INCLUDE=%SDK_ROOT%\include"

if not exist "%GEN_DIR%" (
  echo [sdk-ada] ERROR: no generated bindings at %GEN_DIR% -- run scripts\generate_bindings first.>&2
  exit /b 1
)

set "CXXFLAGS=-std=c++17 -O2 -I%GEN_DIR% -I%CORE_EXTERNAL_INCLUDE% -I%PCL_INCLUDE%"

if exist "%TMP_MANIFEST_FILE%" del /f /q "%TMP_MANIFEST_FILE%" >nul 2>&1
>"%TMP_MANIFEST_FILE%" echo CXX=!CXX!
>>"%TMP_MANIFEST_FILE%" echo AR=!AR!
>>"%TMP_MANIFEST_FILE%" echo CXXFLAGS=%CXXFLAGS%

set "SOURCE_FILES="
for %%F in ("%GEN_DIR%\pyramid_data_model_*_cabi_marshal.cpp" "%GEN_DIR%\pyramid_components_*_cabi_marshal.cpp" "%GEN_DIR%\pyramid_data_model_*_cabi_marshal.hpp" "%GEN_DIR%\pyramid_components_*_cabi_marshal.hpp") do (
  if exist "%%~fF" (
    >>"%TMP_MANIFEST_FILE%" echo %%~fF^|%%~zF^|%%~tF
    if /i "%%~xF"==".cpp" set SOURCE_FILES=!SOURCE_FILES! "%%~fF"
  )
)

if not defined SOURCE_FILES (
  echo [sdk-ada] ERROR: no generated C-ABI marshal sources found under %GEN_DIR%
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
  echo [sdk-ada] GNAT C-ABI marshal archive is up to date in "%OUT_DIR%"
  if exist "%TMP_MANIFEST_FILE%" del /f /q "%TMP_MANIFEST_FILE%" >nul 2>&1
  goto copy_libstdcxx
)

echo [sdk-ada] Building GNAT-compatible C-ABI marshal archive in "%OUT_DIR%"

set "OBJECT_FILES="
for %%F in (!SOURCE_FILES!) do (
  echo [sdk-ada]   compiling %%~nxF
  set "OBJ_FILE=%OBJ_DIR%\%%~nF.o"
  "!CXX!" %CXXFLAGS% -c "%%~fF" -o "!OBJ_FILE!" || exit /b 1
  set OBJECT_FILES=!OBJECT_FILES! "!OBJ_FILE!"
)

if exist "%TMP_LIB_FILE%" del /f /q "%TMP_LIB_FILE%" >nul 2>&1
"!AR!" rcs "%TMP_LIB_FILE%" !OBJECT_FILES! || exit /b 1

if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
move /y "%TMP_LIB_FILE%" "%LIB_FILE%" >nul || exit /b 1
move /y "%TMP_MANIFEST_FILE%" "%MANIFEST_FILE%" >nul || exit /b 1

:copy_libstdcxx
set "LIBSTDCXX="
for /f "tokens=*" %%L in ('"!CXX!" -print-file-name=libstdc++.a 2^>nul') do set "LIBSTDCXX=%%L"
if defined LIBSTDCXX if not "!LIBSTDCXX!"=="libstdc++.a" (
  echo [sdk-ada] Copying libstdc++ from !LIBSTDCXX! to "%OUT_DIR%"
  copy /y "!LIBSTDCXX!" "%OUT_DIR%\" >nul
  for %%D in ("!LIBSTDCXX!") do (
    if exist "%%~dpDlibstdc++.dll.a" copy /y "%%~dpDlibstdc++.dll.a" "%OUT_DIR%\" >nul
  )
) else (
  echo [sdk-ada] WARNING: libstdc++.a not found via '!CXX! -print-file-name'; link may fail with cannot find -lstdc++
)

echo [sdk-ada] Ready:
echo [sdk-ada]   %LIB_FILE%
exit /b 0
