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

where g++ >nul 2>&1
if errorlevel 1 (
  echo [ada-pyramid] ERROR: g++ not found in PATH
  exit /b 1
)

where ar >nul 2>&1
if errorlevel 1 (
  echo [ada-pyramid] ERROR: ar not found in PATH
  exit /b 1
)

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

if "%FORCE_REBUILD%"=="1" (
  if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
  del /f /q "%OUT_DIR%\lib%LIB_NAME%_*.a" >nul 2>&1
  del /f /q "%OBJ_DIR%\*.o" >nul 2>&1
)

set "GEN_DIR=%PYRAMID_ROOT%\examples\cpp\generated"
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

set "CXXFLAGS=-std=c++17 -O2 -I%GEN_DIR% -I%GEN_FB_DIR% -I%BUILD_FB_DIR% -I%FLATBUFFERS_INCLUDE% -I%NLOHMANN_INCLUDE%"

g++ %CXXFLAGS% -c "%GEN_DIR%\pyramid_data_model_base_codec.cpp" -o "%OBJ_DIR%\pyramid_data_model_base_codec.o" || exit /b 1
g++ %CXXFLAGS% -c "%GEN_DIR%\pyramid_data_model_common_codec.cpp" -o "%OBJ_DIR%\pyramid_data_model_common_codec.o" || exit /b 1
g++ %CXXFLAGS% -c "%GEN_DIR%\pyramid_data_model_tactical_codec.cpp" -o "%OBJ_DIR%\pyramid_data_model_tactical_codec.o" || exit /b 1
g++ %CXXFLAGS% -c "%GEN_DIR%\pyramid_services_tactical_objects_json_codec.cpp" -o "%OBJ_DIR%\pyramid_services_tactical_objects_json_codec.o" || exit /b 1
g++ %CXXFLAGS% -c "%GEN_FB_DIR%\pyramid_services_tactical_objects_flatbuffers_codec.cpp" -o "%OBJ_DIR%\pyramid_services_tactical_objects_flatbuffers_codec.o" || exit /b 1

if exist "%TMP_LIB_FILE%" del /f /q "%TMP_LIB_FILE%" >nul 2>&1

ar rcs "%TMP_LIB_FILE%" ^
  "%OBJ_DIR%\pyramid_data_model_base_codec.o" ^
  "%OBJ_DIR%\pyramid_data_model_common_codec.o" ^
  "%OBJ_DIR%\pyramid_data_model_tactical_codec.o" ^
  "%OBJ_DIR%\pyramid_services_tactical_objects_json_codec.o" ^
  "%OBJ_DIR%\pyramid_services_tactical_objects_flatbuffers_codec.o" || exit /b 1

if exist "%LIB_FILE%" del /f /q "%LIB_FILE%" >nul 2>&1
move /y "%TMP_LIB_FILE%" "%LIB_FILE%" >nul || exit /b 1

echo [ada-pyramid] Built:
echo [ada-pyramid]   %LIB_FILE%
exit /b 0
