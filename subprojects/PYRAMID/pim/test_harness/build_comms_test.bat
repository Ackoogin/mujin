@echo off
REM build_comms_test.bat -- Windows counterpart to build_comms_test.sh.
REM
REM Build + run the plugin-system comms test for the new PIM proto. Generates
REM C++/JSON bindings, compiles the generated sources + the osprey
REM sensor_products JSON codec plugin *into* the test against pcl_core.lib, and
REM runs it. (This is the statically-linked variant; build_plugin_load_test.bat
REM demonstrates the same round-trip loading the codec from a plugin DLL with no
REM relink.)
REM
REM pcl_core.lib is taken from an existing CMake build tree (build*/). Build the
REM plugins/PCL once first, e.g. subprojects\PYRAMID\scripts\build_plugins.bat.
REM Override discovery with the PCL_CORE_LIB environment variable if needed.
setlocal enabledelayedexpansion

set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "PYRAMID=%%~fI"
for %%I in ("%PYRAMID%\..\..") do set "ROOT=%%~fI"
set "GEN=%HERE%generated"

call "%HERE%_msvc_env.bat"
if errorlevel 1 exit /b 3

echo == generating bindings ==
if exist "%GEN%" rmdir /s /q "%GEN%"
mkdir "%GEN%"
python "%PYRAMID%\pim\generate_bindings.py" "%PYRAMID%\pim\test" "%GEN%" --languages cpp --backends json >nul
if errorlevel 1 (
  echo generation failed>&2
  exit /b 2
)

call :find_pcl_core
if not defined LIBPCL (
  echo pcl_core.lib not found under %ROOT%\build*\ -- build PCL/plugins first>&2
  echo   ^(e.g. subprojects\PYRAMID\scripts\build_plugins.bat^) or set PCL_CORE_LIB.>&2
  exit /b 2
)
echo == using pcl_core: %LIBPCL% ==

set INC=/I"%GEN%" /I"%PYRAMID%\..\PCL\include" /I"%PYRAMID%\core\external"

REM Generated sources in the osprey sensor_products dependency closure.
set PKGS=pyramid_data_model_base pyramid_data_model_common pyramid_data_model_generic_pim_generic pyramid_data_model_common_pim_components_authorisation pyramid_data_model_common_pim_components_sensor_products pyramid_data_model_pim_osprey_sensor_products pyramid_components_pim_osprey_sensor_products_services_provided

set SRCS="%HERE%components_comms_test.cpp" "%GEN%\pyramid_services_pim_osprey_sensor_products_provided.cpp" "%GEN%\pyramid_services_pim_osprey_sensor_products_json_codec_plugin.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp"        set SRCS=!SRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set SRCS=!SRCS! "%GEN%\%%P_cabi_marshal.cpp"
)

echo == compiling ==
cl /nologo /std:c++17 /EHsc /MD %INC% !SRCS! "%LIBPCL%" /Fe:"%HERE%comms_test.exe" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == running ==
"%HERE%comms_test.exe"
exit /b %errorlevel%

:find_pcl_core
set "LIBPCL="
if defined PCL_CORE_LIB if exist "%PCL_CORE_LIB%" ( set "LIBPCL=%PCL_CORE_LIB%" & goto :eof )
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined LIBPCL for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_core.lib" 2^>nul') do set "LIBPCL=%%F"
)
goto :eof
