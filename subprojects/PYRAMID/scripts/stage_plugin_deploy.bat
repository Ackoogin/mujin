@echo off
REM stage_plugin_deploy.bat
REM
REM Windows counterpart to stage_plugin_deploy.sh. Stage the runtime plugin
REM binding artifacts into per-component deployment directories so the output can
REM be reviewed and a client can be built against them. For each PYRAMID
REM component it collects:
REM
REM   <out>\<component>\
REM     plugins\   codec plugin .dll(s) for the component + the transport plugin .dll
REM     include\   client-facing headers   (pcl\*  and  pyramid\*  generated facade)
REM     src\       the generated contract facade the client compiles against
REM     lib\       static libraries a client links (.lib)
REM     MANIFEST.txt   plugin .dll paths (feed to --codec-plugin / PCL_CODEC_PLUGIN_PATH)
REM     README.md      how to build a client and run it against the plugin
REM
REM The codec plugin is the single cross-language .dll (it consumes the frozen
REM pyramid_<Type>_c C struct); the same file is loaded from C++ and Ada.
REM
REM Usage:
REM   stage_plugin_deploy.bat [--build-dir DIR] [--out DIR] [--clean]
REM
REM Defaults: --build-dir build-flatbuffers-only   --out dist\plugin_deploy
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..\..") do set "REPO_ROOT=%%~fI"

set "BUILD_DIR=%REPO_ROOT%\build-flatbuffers-only"
set "OUT_DIR=%REPO_ROOT%\dist\plugin_deploy"
set "CLEAN=0"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--build-dir" (set "BUILD_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--out"       (set "OUT_DIR=%~2"   & shift & shift & goto parse_args)
if /i "%~1"=="--clean"     (set "CLEAN=1"       & shift & goto parse_args)
if /i "%~1"=="-h"     goto usage
if /i "%~1"=="--help" goto usage
echo unknown arg: %~1>&2
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
REM Resolve to absolute paths.
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
for %%I in ("%BUILD_DIR%") do set "BUILD_DIR=%%~fI"
for %%I in ("%OUT_DIR%")   do set "OUT_DIR=%%~fI"

set "PCL_INC=%REPO_ROOT%\subprojects\PCL\include\pcl"
set "GEN_DIR=%BUILD_DIR%\generated\pyramid_cpp_bindings"

REM Discover the multi-config output directories that actually hold the plugins.
REM (VS generators drop .dll/.lib under a per-config subdir, e.g. Release\.)
set "PYRAMID_LIBDIR="
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\subprojects\PYRAMID\pyramid_codec_*.dll" 2^>nul') do (
  if not defined PYRAMID_LIBDIR set "PYRAMID_LIBDIR=%%~dpF"
)
set "PCL_SRCDIR="
for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\subprojects\PCL\src\pcl_transport_socket_plugin.dll" 2^>nul') do (
  if not defined PCL_SRCDIR set "PCL_SRCDIR=%%~dpF"
)
if not defined PCL_SRCDIR (
  for /f "delims=" %%F in ('dir /b /s "%BUILD_DIR%\subprojects\PCL\src\pcl_core.lib" 2^>nul') do (
    if not defined PCL_SRCDIR set "PCL_SRCDIR=%%~dpF"
  )
)

if not exist "%GEN_DIR%" (
  echo ERROR: generated bindings not found at %GEN_DIR% -- build first.>&2
  exit /b 1
)
if not exist "%PCL_INC%" (
  echo ERROR: PCL include dir not found at %PCL_INC%>&2
  exit /b 1
)
if not defined PYRAMID_LIBDIR (
  echo ERROR: no codec plugin .dll found under %BUILD_DIR%\subprojects\PYRAMID>&2
  exit /b 1
)

echo build-dir : %BUILD_DIR%
echo out       : %OUT_DIR%
if "%CLEAN%"=="1" (
  echo cleaning %OUT_DIR%
  for /d %%D in ("%OUT_DIR%\*") do rmdir /s /q "%%D"
  del /q "%OUT_DIR%\*" >nul 2>&1
)

REM All known data-model modules (used as a fallback closure when per-component
REM detection finds nothing).
set "ALL_MARSHAL_MODULES=common base tactical autonomy sensors sensorproducts radar"

REM Discover components from the built codec plugin .dll names:
REM   pyramid_codec_<codec>_<component>.dll
set "COMPONENTS="
for /f "delims=" %%F in ('dir /b "%PYRAMID_LIBDIR%pyramid_codec_*.dll" 2^>nul') do (
  set "stem=%%~nF"
  set "stem=!stem:pyramid_codec_json_=!"
  set "stem=!stem:pyramid_codec_flatbuffers_=!"
  set "stem=!stem:pyramid_codec_protobuf_=!"
  if not defined seen_!stem! (
    set "seen_!stem!=1"
    set "COMPONENTS=!COMPONENTS! !stem!"
  )
)
if "%COMPONENTS%"=="" (
  echo ERROR: no codec plugin .dll found in %PYRAMID_LIBDIR%>&2
  exit /b 1
)

echo components:%COMPONENTS%
echo.

for %%C in (%COMPONENTS%) do call :stage_comp "%%C"

echo.
echo Done. Layout:
for /f "delims=" %%F in ('dir /b /s /a:d "%OUT_DIR%" 2^>nul') do (
  set "p=%%F"
  setlocal enabledelayedexpansion
  echo !p:%OUT_DIR%=^<out^>!
  endlocal
)
exit /b 0


REM ===========================================================================
REM :stage_comp <component>
REM ===========================================================================
:stage_comp
setlocal enabledelayedexpansion
set "comp=%~1"
set "dest=%OUT_DIR%\%comp%"
echo ==^> %comp%  -^>  %dest%
mkdir "%dest%\plugins"         2>nul
mkdir "%dest%\include\pcl"     2>nul
mkdir "%dest%\include\pyramid" 2>nul
mkdir "%dest%\include\tl"      2>nul
mkdir "%dest%\src"             2>nul
mkdir "%dest%\lib"             2>nul

REM --- plugins: the per-component codec .dll(s) + the shared transport plugin ---
set "found_codec=0"
for %%K in (json flatbuffers protobuf) do (
  if exist "%PYRAMID_LIBDIR%pyramid_codec_%%K_%comp%.dll" (
    copy /y "%PYRAMID_LIBDIR%pyramid_codec_%%K_%comp%.dll" "%dest%\plugins\" >nul
    set "found_codec=1"
  )
)
if "!found_codec!"=="0" echo    WARN: no codec .dll for %comp%
set "found_transport=0"
if defined PCL_SRCDIR (
  for %%T in (pcl_transport_socket_plugin pcl_transport_shared_memory_plugin) do (
    if exist "%PCL_SRCDIR%%%T.dll" (
      copy /y "%PCL_SRCDIR%%%T.dll" "%dest%\plugins\" >nul
      set "found_transport=1"
    )
  )
)
if "!found_transport!"=="0" echo    WARN: no transport plugin found (build pcl_transport_socket_plugin / pcl_transport_shared_memory_plugin)

REM --- include\pcl: the public PCL headers a client uses ---
copy /y "%PCL_INC%\*.h"   "%dest%\include\pcl\" >nul 2>&1
copy /y "%PCL_INC%\*.hpp" "%dest%\include\pcl\" >nul 2>&1

REM --- include\pyramid: generated facade + data-model + C-ABI headers ---
REM (the data-model header set is shared; copy it whole so any closure resolves)
copy /y "%GEN_DIR%\pyramid_data_model_*.hpp"      "%dest%\include\pyramid\" >nul 2>&1
copy /y "%GEN_DIR%\pyramid_data_model_*.h"        "%dest%\include\pyramid\" >nul 2>&1
copy /y "%GEN_DIR%\pyramid_datamodel_cabi.h"      "%dest%\include\pyramid\" >nul 2>&1
copy /y "%GEN_DIR%\pyramid_services_%comp%_*.hpp" "%dest%\include\pyramid\" >nul 2>&1
REM header-only dep the generated data-model headers include (tl::optional)
copy /y "%REPO_ROOT%\subprojects\PYRAMID\core\external\tl\optional.hpp" "%dest%\include\tl\" >nul 2>&1

REM --- src: the contract facade the client compiles against ---
REM (exclude *_codec_plugin.cpp -- that is the plugin's own source, already
REM  built into the .dll; a client must not recompile it)
for %%F in ("%GEN_DIR%\pyramid_services_%comp%_*.cpp") do (
  echo %%~nxF | findstr /i /c:"_codec_plugin.cpp" >nul
  if errorlevel 1 copy /y "%%~fF" "%dest%\src\" >nul
)

REM --- lib: framework + per-module marshalling closure -----------------------
if exist "%PCL_SRCDIR%pcl_core.lib" copy /y "%PCL_SRCDIR%pcl_core.lib" "%dest%\lib\" >nul

REM Module closure: the data-model modules this component's codec actually
REM marshals (from the generated codec plugin's *_cabi_marshal includes).
REM Staging only these means a tactical_objects deployment changes when
REM tactical/common/base change -- not when autonomy/sensors/... change.
set "comp_modules="
for %%M in (%ALL_MARSHAL_MODULES%) do (
  findstr /m /c:"pyramid_data_model_%%M_cabi_marshal" "%GEN_DIR%\pyramid_services_%comp%_*_codec_plugin.cpp" >nul 2>&1
  if not errorlevel 1 set "comp_modules=!comp_modules! %%M"
)
if not defined comp_modules set "comp_modules=%ALL_MARSHAL_MODULES%"
set "staged_modules="
for %%M in (!comp_modules!) do (
  if exist "%PYRAMID_LIBDIR%pyramid_marshal_%%M.lib" (
    copy /y "%PYRAMID_LIBDIR%pyramid_marshal_%%M.lib" "%dest%\lib\" >nul
    set "staged_modules=!staged_modules! %%M"
  )
)
if not defined staged_modules set "staged_modules= <none>"
echo    marshalling modules:!staged_modules!

REM --- MANIFEST: absolute plugin paths (for --codec-plugin / PCL_CODEC_PLUGIN_PATH) ---
type nul > "%dest%\MANIFEST.txt"
for %%F in ("%dest%\plugins\*.dll") do echo %%~fF>>"%dest%\MANIFEST.txt"

REM --- codec_manifest.txt: codec plugin paths the runtime auto-loads ----------
REM Point PCL_CODEC_MANIFEST (or --codec-manifest) at this file so a component
REM runs without naming each codec plugin. Transport plugins are listed
REM separately because loading a transport needs runtime config (the executor).
> "%dest%\codec_manifest.txt" echo # Codec plugins for %comp% -- auto-loaded via PCL_CODEC_MANIFEST.
for %%F in ("%dest%\plugins\pyramid_codec_*.dll") do echo %%~fF>>"%dest%\codec_manifest.txt"

REM --- transport_manifest.txt: transport plugin path(s) for this component ----
> "%dest%\transport_manifest.txt" echo # Transport plugins for %comp% (pass via --transport-plugin).
for %%F in ("%dest%\plugins\pcl_transport_*_plugin.dll") do echo %%~fF>>"%dest%\transport_manifest.txt"

REM --- README ---
call :write_readme "%dest%" "%comp%"

endlocal
goto :eof


REM ===========================================================================
REM :write_readme <dest> <component>
REM ===========================================================================
:write_readme
setlocal enabledelayedexpansion
set "dest=%~1"
set "comp=%~2"
for %%I in ("%BUILD_DIR%") do set "build_name=%%~nxI"
set "rm=%dest%\README.md"
> "%rm%" echo # Deployment: %comp%
>>"%rm%" echo.
>>"%rm%" echo Generated by `stage_plugin_deploy.bat` from `%build_name%`.
>>"%rm%" echo.
>>"%rm%" echo ```
>>"%rm%" echo plugins\   codec plugin .dll(s) for %comp% + the socket transport plugin .dll
>>"%rm%" echo include\   pcl\*  (PCL public API)   pyramid\*  (generated typed contract + C-ABI)
>>"%rm%" echo src\       generated contract facade (.cpp) to compile into the client
>>"%rm%" echo lib\       static libraries to link (codec is loaded from plugins\ at run time)
>>"%rm%" echo MANIFEST.txt  absolute plugin .dll paths
>>"%rm%" echo ```
>>"%rm%" echo.
>>"%rm%" echo ## Plugins (the runtime binding)
for %%F in ("%dest%\plugins\*.dll") do >>"%rm%" echo - `%%~nxF`
>>"%rm%" echo.
>>"%rm%" echo The codec plugin is the **single cross-language** `.dll` -- it consumes the frozen
>>"%rm%" echo `pyramid_^<Type^>_c` C struct and is loaded from both C++ and Ada clients.
>>"%rm%" echo.
>>"%rm%" echo ## Build a client (plugin-only: no wire codec, no transport linked)
>>"%rm%" echo Compile your client together with the generated facade in `src\`, adding
>>"%rm%" echo `include\` and `include\pyramid\` to the include path, and link `lib\pcl_core.lib`
>>"%rm%" echo plus the per-module `lib\pyramid_marshal_*.lib`. With MSVC:
>>"%rm%" echo.
>>"%rm%" echo ```bat
>>"%rm%" echo cl /std:c++17 /EHsc my_client.cpp src\*.cpp ^^
>>"%rm%" echo    /Iinclude /Iinclude\pyramid ^^
>>"%rm%" echo    lib\pcl_core.lib lib\pyramid_marshal_*.lib
>>"%rm%" echo ```
>>"%rm%" echo The client links only the framework + the native^<-^>C-struct marshalling for
>>"%rm%" echo this component's data-model module closure (`lib\pyramid_marshal_^<module^>.lib`)
>>"%rm%" echo -- no JSON, no FlatBuffers, no codec, and **no transport**. Both the codec and
>>"%rm%" echo the socket transport arrive as `.dll` plugins composed at run time. Because only
>>"%rm%" echo the closure modules are shipped, an edit to an unrelated data-model module leaves
>>"%rm%" echo this deployment unchanged.
>>"%rm%" echo.
>>"%rm%" echo ## Run against the plugins
>>"%rm%" echo Name each plugin explicitly via `--codec-plugin` / `--transport-plugin`, or set
>>"%rm%" echo `PCL_CODEC_MANIFEST` to `codec_manifest.txt` so the runtime auto-loads every
>>"%rm%" echo codec and the client only needs the transport plugin path. The client calls the
>>"%rm%" echo generated service with native types; the loaded codec `.dll` supplies the wire
>>"%rm%" echo codec and the transport `.dll` supplies the socket (or shared-memory) transport.
>>"%rm%" echo With no codec plugin loaded the C++ facade fails closed.
endlocal
goto :eof
