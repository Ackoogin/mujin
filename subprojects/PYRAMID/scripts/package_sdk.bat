@echo off
REM package_sdk.bat -- produce the offline PCL/PYRAMID SDK deploy directory.
REM
REM Maintainer-run, from inside this repo. Packages prebuilt PCL static libs +
REM headers + a prebuilt flatc.exe alongside the pure-Python generator, starter
REM .proto contracts, and a standalone CMake/GNAT project template, so a
REM downstream developer can build their OWN proto -> plugin on a firewalled
REM machine with zero network access and no dependency on this monorepo.
REM
REM Preconditions:
REM   --build-dir must already be configured+built (Debug and/or Release) with
REM   PYRAMID_ENABLE_FLATBUFFERS=ON, PYRAMID_GENERATE_CPP_BINDINGS=ON, e.g.:
REM     subprojects\PYRAMID\scripts\build_plugins.bat
REM   For the GNAT/Ada libs, run first (or let this script attempt it if
REM   gcc/ar are on PATH):
REM     subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat
REM
REM Usage:
REM   package_sdk.bat [--build-dir DIR] [--gnat-pcl-dir DIR] [--proto-dir DIR]
REM                   [--gra] [--out DIR] [--clean]
REM
REM Defaults: --build-dir build-flatbuffers-only
REM           --gnat-pcl-dir <build-dir>\ada_gnat_pcl
REM           --proto-dir subprojects\PYRAMID\proto
REM           --out dist\pcl_pyramid_sdk
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..\..") do set "REPO_ROOT=%%~fI"

set "BUILD_DIR=%REPO_ROOT%\build-flatbuffers-only"
set "GNAT_PCL_DIR="
set "PROTO_DIR="
set "OUT_DIR=%REPO_ROOT%\dist\pcl_pyramid_sdk"
set "CLEAN=0"
set "GRA=0"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--build-dir"    (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--gnat-pcl-dir" (set "GNAT_PCL_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--proto-dir"    (set "PROTO_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--gra"          (set "GRA=1" & shift & goto parse_args)
if /i "%~1"=="--out"          (set "OUT_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--clean"        (set "CLEAN=1" & shift & goto parse_args)
if /i "%~1"=="-h"     goto usage
if /i "%~1"=="--help" goto usage
echo [package_sdk] unknown arg: %~1>&2
exit /b 2

:usage
for /f "tokens=1* delims=:" %%A in ('findstr /b /n "REM" "%~f0"') do (
  set "line=%%B"
  setlocal enabledelayedexpansion
  if "!line!"=="REM" (echo.) else (echo !line:~4!)
  endlocal
)
exit /b 0

:done_args
for %%I in ("%BUILD_DIR%") do set "BUILD_DIR=%%~fI"
if not defined GNAT_PCL_DIR set "GNAT_PCL_DIR=%BUILD_DIR%\ada_gnat_pcl"
for %%I in ("%GNAT_PCL_DIR%") do set "GNAT_PCL_DIR=%%~fI"
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
for %%I in ("%OUT_DIR%") do set "OUT_DIR=%%~fI"

set "PYRAMID_ROOT=%REPO_ROOT%\subprojects\PYRAMID"
set "PCL_ROOT=%REPO_ROOT%\subprojects\PCL"
set "SDK_TEMPLATE=%PYRAMID_ROOT%\sdk_template"
if not defined PROTO_DIR set "PROTO_DIR=%PYRAMID_ROOT%\proto"
for %%I in ("%PROTO_DIR%") do set "PROTO_DIR=%%~fI"

if not exist "%PROTO_DIR%" (
  echo [package_sdk] ERROR: proto dir not found: %PROTO_DIR%>&2
  exit /b 1
)
dir /b /s "%PROTO_DIR%\*.proto" >nul 2>&1
if errorlevel 1 (
  echo [package_sdk] ERROR: no .proto contracts found under %PROTO_DIR%>&2
  exit /b 1
)

echo [package_sdk] repo         : %REPO_ROOT%
echo [package_sdk] build-dir    : %BUILD_DIR%
echo [package_sdk] gnat-pcl-dir : %GNAT_PCL_DIR%
echo [package_sdk] proto-dir    : %PROTO_DIR%
echo [package_sdk] gra profile  : %GRA%
echo [package_sdk] out          : %OUT_DIR%

if not exist "%BUILD_DIR%" (
  echo [package_sdk] ERROR: build dir not found: %BUILD_DIR% -- run subprojects\PYRAMID\scripts\build_plugins.bat first.>&2
  exit /b 1
)

if "%CLEAN%"=="1" (
  echo [package_sdk] cleaning %OUT_DIR%
  for /d %%D in ("%OUT_DIR%\*") do rmdir /s /q "%%D"
  del /q "%OUT_DIR%\*" >nul 2>&1
)

REM --- Locate prebuilt artifacts -----------------------------------------
set "PCL_LIB_DEBUG="
set "PCL_LIB_RELEASE="
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\pcl_core.lib" 2^>nul') do (
  echo %%F | findstr /i "\\Debug\\" >nul && set "PCL_LIB_DEBUG=%%F"
  echo %%F | findstr /i /l /c:"\Release\" >nul && set "PCL_LIB_RELEASE=%%F"
)
if exist "%BUILD_DIR%\subprojects\PCL\src\Debug\pcl_core.lib" set "PCL_LIB_DEBUG=%BUILD_DIR%\subprojects\PCL\src\Debug\pcl_core.lib"
if exist "%BUILD_DIR%\subprojects\PCL\src\Release\pcl_core.lib" set "PCL_LIB_RELEASE=%BUILD_DIR%\subprojects\PCL\src\Release\pcl_core.lib"
if not defined PCL_LIB_RELEASE if not defined PCL_LIB_DEBUG (
  echo [package_sdk] ERROR: no pcl_core.lib found under %BUILD_DIR%>&2
  exit /b 1
)

set "FLATC_EXE="
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\flatc.exe" 2^>nul') do (
  if not defined FLATC_EXE set "FLATC_EXE=%%F"
  echo %%F | findstr /i /l /c:"\Release\" >nul && set "FLATC_EXE=%%F"
)
if exist "%BUILD_DIR%\_deps\flatbuffers-build\Release\flatc.exe" set "FLATC_EXE=%BUILD_DIR%\_deps\flatbuffers-build\Release\flatc.exe"
if not defined FLATC_EXE (
  echo [package_sdk] ERROR: flatc.exe not found under %BUILD_DIR% -- build with PYRAMID_ENABLE_FLATBUFFERS=ON.>&2
  exit /b 1
)

set "FLATBUFFERS_INCLUDE=%BUILD_DIR%\_deps\flatbuffers-src\include\flatbuffers"
if not exist "%FLATBUFFERS_INCLUDE%" (
  echo [package_sdk] ERROR: flatbuffers runtime headers not found at %FLATBUFFERS_INCLUDE%>&2
  exit /b 1
)

set "TRANSPORT_SOCKET_DLL="
set "TRANSPORT_SHM_DLL="
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\pcl_transport_socket_plugin.dll" 2^>nul') do (
  if not defined TRANSPORT_SOCKET_DLL set "TRANSPORT_SOCKET_DLL=%%F"
  echo %%F | findstr /i /l /c:"\Release\" >nul && set "TRANSPORT_SOCKET_DLL=%%F"
)
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\pcl_transport_shared_memory_plugin.dll" 2^>nul') do (
  if not defined TRANSPORT_SHM_DLL set "TRANSPORT_SHM_DLL=%%F"
  echo %%F | findstr /i /l /c:"\Release\" >nul && set "TRANSPORT_SHM_DLL=%%F"
)
if exist "%BUILD_DIR%\subprojects\PCL\src\Release\pcl_transport_socket_plugin.dll" set "TRANSPORT_SOCKET_DLL=%BUILD_DIR%\subprojects\PCL\src\Release\pcl_transport_socket_plugin.dll"
if exist "%BUILD_DIR%\subprojects\PCL\src\Release\pcl_transport_shared_memory_plugin.dll" set "TRANSPORT_SHM_DLL=%BUILD_DIR%\subprojects\PCL\src\Release\pcl_transport_shared_memory_plugin.dll"
if not defined TRANSPORT_SOCKET_DLL (
  echo [package_sdk] WARN: pcl_transport_socket_plugin.dll not found under %BUILD_DIR%
)
if not defined TRANSPORT_SHM_DLL (
  echo [package_sdk] WARN: pcl_transport_shared_memory_plugin.dll not found under %BUILD_DIR%
)

set "OMS_CODEC_DLL="
set "LACAL_TRANSPORT_DLL="
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\pyramid_codec_oms_json_uci.dll" 2^>nul') do (
  if not defined OMS_CODEC_DLL set "OMS_CODEC_DLL=%%F"
  echo %%F | findstr /i /l /c:"\Release\" >nul && set "OMS_CODEC_DLL=%%F"
)
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\pyramid_lacal_transport_plugin.dll" 2^>nul') do (
  if not defined LACAL_TRANSPORT_DLL set "LACAL_TRANSPORT_DLL=%%F"
  echo %%F | findstr /i /l /c:"\Release\" >nul && set "LACAL_TRANSPORT_DLL=%%F"
)
if exist "%BUILD_DIR%\subprojects\PYRAMID\Release\pyramid_codec_oms_json_uci.dll" set "OMS_CODEC_DLL=%BUILD_DIR%\subprojects\PYRAMID\Release\pyramid_codec_oms_json_uci.dll"
if exist "%BUILD_DIR%\subprojects\PYRAMID\Release\pyramid_lacal_transport_plugin.dll" set "LACAL_TRANSPORT_DLL=%BUILD_DIR%\subprojects\PYRAMID\Release\pyramid_lacal_transport_plugin.dll"
if "%GRA%"=="1" if not defined OMS_CODEC_DLL (
  echo [package_sdk] ERROR: --gra requires pyramid_codec_oms_json_uci.dll; rebuild with build_plugins.bat --gra.>&2
  exit /b 1
)
if "%GRA%"=="1" if not defined LACAL_TRANSPORT_DLL (
  echo [package_sdk] ERROR: --gra requires pyramid_lacal_transport_plugin.dll; rebuild with build_plugins.bat --gra.>&2
  exit /b 1
)

if not exist "%GNAT_PCL_DIR%\libpcl_core.a" (
  where gcc >nul 2>&1
  if errorlevel 1 (
    echo [package_sdk] WARN: no GNAT PCL archives at %GNAT_PCL_DIR% and gcc not on PATH -- skipping GNAT/Ada libs. Run subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat manually if Ada support is needed.
  ) else (
    echo [package_sdk] building GNAT PCL static libs ...
    call "%PCL_ROOT%\scripts\build_gnat_pcl_static_libs.bat" "%GNAT_PCL_DIR%"
    if errorlevel 1 (
      echo [package_sdk] WARN: GNAT PCL static lib build failed -- skipping GNAT/Ada libs.
    )
  )
)

REM --- Lay out the deploy directory --------------------------------------
mkdir "%OUT_DIR%\include\pcl"          2>nul
mkdir "%OUT_DIR%\include\external"     2>nul
mkdir "%OUT_DIR%\lib\msvc\Debug"       2>nul
mkdir "%OUT_DIR%\lib\msvc\Release"     2>nul
mkdir "%OUT_DIR%\lib\gnat"             2>nul
mkdir "%OUT_DIR%\plugins"              2>nul
mkdir "%OUT_DIR%\tools"                2>nul
mkdir "%OUT_DIR%\generator\backends"   2>nul
mkdir "%OUT_DIR%\proto"                2>nul
mkdir "%OUT_DIR%\gnat\pcl_bindings"    2>nul

echo [package_sdk] copying PCL headers ...
xcopy /y /q "%PCL_ROOT%\include\pcl\*" "%OUT_DIR%\include\pcl\" >nul

echo [package_sdk] copying vendored header-only deps ...
xcopy /y /q /i "%PYRAMID_ROOT%\core\external\nlohmann\*" "%OUT_DIR%\include\external\nlohmann\" >nul
xcopy /y /q /i "%PYRAMID_ROOT%\core\external\tl\*"       "%OUT_DIR%\include\external\tl\"       >nul
xcopy /y /q /i "%FLATBUFFERS_INCLUDE%\*"                 "%OUT_DIR%\include\external\flatbuffers\" >nul

echo [package_sdk] copying prebuilt MSVC libs ...
if defined PCL_LIB_DEBUG   copy /y "%PCL_LIB_DEBUG%"   "%OUT_DIR%\lib\msvc\Debug\pcl_core.lib"   >nul
if defined PCL_LIB_RELEASE copy /y "%PCL_LIB_RELEASE%" "%OUT_DIR%\lib\msvc\Release\pcl_core.lib" >nul

echo [package_sdk] copying prebuilt GNAT libs ...
if exist "%GNAT_PCL_DIR%\libpcl_core.a" (
  copy /y "%GNAT_PCL_DIR%\lib*.a" "%OUT_DIR%\lib\gnat\" >nul
) else (
  echo [package_sdk]   none -- Ada support unavailable in this deploy
)

echo [package_sdk] copying prebuilt transport plugins ...
if defined TRANSPORT_SOCKET_DLL copy /y "%TRANSPORT_SOCKET_DLL%" "%OUT_DIR%\plugins\" >nul
if defined TRANSPORT_SHM_DLL    copy /y "%TRANSPORT_SHM_DLL%"    "%OUT_DIR%\plugins\" >nul
if "%GRA%"=="1" copy /y "%OMS_CODEC_DLL%" "%OUT_DIR%\plugins\" >nul
if "%GRA%"=="1" copy /y "%LACAL_TRANSPORT_DLL%" "%OUT_DIR%\plugins\" >nul

echo [package_sdk] copying flatc.exe ...
copy /y "%FLATC_EXE%" "%OUT_DIR%\tools\flatc.exe" >nul

echo [package_sdk] copying generator (pim\*.py) ...
xcopy /y /q "%PYRAMID_ROOT%\pim\*.py" "%OUT_DIR%\generator\" >nul
xcopy /y /q "%PYRAMID_ROOT%\pim\backends\*.py" "%OUT_DIR%\generator\backends\" >nul
xcopy /y /q "%PYRAMID_ROOT%\pim\cpp\*.py" "%OUT_DIR%\generator\cpp\" >nul
xcopy /y /q "%PYRAMID_ROOT%\pim\ada\*.py" "%OUT_DIR%\generator\ada\" >nul
mkdir "%OUT_DIR%\generator\topic_metadata" 2>nul
xcopy /y /q "%PYRAMID_ROOT%\pim\topic_metadata\*.json" "%OUT_DIR%\generator\topic_metadata\" >nul

echo [package_sdk] copying starter proto contracts ...
xcopy /y /q /s /i "%PROTO_DIR%\*" "%OUT_DIR%\proto\" >nul

echo [package_sdk] copying Ada PCL bindings (source only) ...
xcopy /y /q "%PCL_ROOT%\bindings\ada\*.ads" "%OUT_DIR%\gnat\pcl_bindings\" >nul
xcopy /y /q "%PCL_ROOT%\bindings\ada\*.adb" "%OUT_DIR%\gnat\pcl_bindings\" >nul

echo [package_sdk] copying GNATCOLL JSON (Ada JSON codec dependency) ...
xcopy /y /q /s /i "%PYRAMID_ROOT%\core\external\gnatcoll-core" "%OUT_DIR%\gnat\external\gnatcoll-core" >nul

echo [package_sdk] copying SDK project template (CMake + GNAT + scripts + README) ...
xcopy /y /q /i "%SDK_TEMPLATE%\README.md" "%OUT_DIR%\" >nul
if exist "%OUT_DIR%\sdk_project" rmdir /s /q "%OUT_DIR%\sdk_project"
xcopy /y /q /s /i "%SDK_TEMPLATE%\sdk_project" "%OUT_DIR%\sdk_project" >nul
xcopy /y /q /i "%SDK_TEMPLATE%\gnat\pyramid_sdk_ada.gpr" "%OUT_DIR%\gnat\" >nul
xcopy /y /q /i "%SDK_TEMPLATE%\gnat\build_gnat_pyramid_cabi_marshal_libs.bat" "%OUT_DIR%\gnat\" >nul
xcopy /y /q /i "%SDK_TEMPLATE%\gnat\build_gnat_pyramid_cabi_marshal_libs.sh"  "%OUT_DIR%\gnat\" >nul
xcopy /y /q /s /i "%SDK_TEMPLATE%\scripts" "%OUT_DIR%\scripts" >nul

REM --- Manifest ------------------------------------------------------------
> "%OUT_DIR%\MANIFEST.txt" echo # PCL/PYRAMID offline SDK -- packaged from %BUILD_DIR%
>>"%OUT_DIR%\MANIFEST.txt" echo # gnat-pcl-dir: %GNAT_PCL_DIR%
>>"%OUT_DIR%\MANIFEST.txt" echo # proto-dir: %PROTO_DIR%
>>"%OUT_DIR%\MANIFEST.txt" echo # gra-profile: %GRA%
for /f "delims=" %%F in ('dir /b /s /a:-d "%OUT_DIR%" 2^>nul') do (
  set "p=%%F"
  setlocal enabledelayedexpansion
  >>"%OUT_DIR%\MANIFEST.txt" echo !p:%OUT_DIR%\=!
  endlocal
)

echo.
echo [package_sdk] Done. Deploy directory: %OUT_DIR%
echo [package_sdk] See %OUT_DIR%\README.md for the downstream quick start.
exit /b 0
