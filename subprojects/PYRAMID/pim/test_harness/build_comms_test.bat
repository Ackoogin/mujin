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
set "FLAT_PLUGIN_DLL=%HERE%osprey_sensor_products_flatbuffers_codec.dll"

call "%HERE%_msvc_env.bat"
if errorlevel 1 exit /b 3

echo == generating bindings ==
if exist "%GEN%" rmdir /s /q "%GEN%"
mkdir "%GEN%"
python "%PYRAMID%\pim\generate_bindings.py" "%PYRAMID%\pim\test" "%GEN%" --languages cpp --backends json,flatbuffers >nul
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
call :find_flatc
if not defined FLATC (
  echo flatc.exe not found under %ROOT%\build*\_deps\flatbuffers-build\ -- build with FlatBuffers enabled>&2
  exit /b 2
)
call :find_flatbuffers_include
if not defined FLAT_INC (
  echo FlatBuffers headers not found under %ROOT%\build*\_deps\flatbuffers-src\include>&2
  exit /b 2
)

set INC=/I"%GEN%" /I"%PYRAMID%\..\PCL\include" /I"%PYRAMID%\core\external" /I"%FLAT_INC%"

echo == generating FlatBuffers headers ==
"%FLATC%" --cpp --gen-object-api -o "%GEN%\flatbuffers\cpp" "%GEN%\flatbuffers\cpp\pyramid_services_pim_osprey_sensor_products.fbs"
if errorlevel 1 exit /b 1

REM Generated sources in the osprey sensor_products dependency closure.
set PKGS=pyramid_data_model_base pyramid_data_model_common pyramid_data_model_generic_pim_generic pyramid_data_model_common_pim_components_authorisation pyramid_data_model_common_pim_components_sensor_products pyramid_data_model_pim_osprey_sensor_products pyramid_components_pim_osprey_sensor_products_services_consumed pyramid_components_pim_osprey_sensor_products_services_provided

set SRCS="%HERE%components_comms_test.cpp" "%GEN%\pyramid_services_pim_osprey_sensor_products_provided.cpp" "%GEN%\pyramid_services_pim_osprey_sensor_products_json_codec_plugin.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp"        set SRCS=!SRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set SRCS=!SRCS! "%GEN%\%%P_cabi_marshal.cpp"
)

set PLUGSRCS="%GEN%\pyramid_services_pim_osprey_sensor_products_flatbuffers_codec_plugin.cpp" "%GEN%\flatbuffers\cpp\pyramid_services_pim_osprey_sensor_products_flatbuffers_codec.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp"        set PLUGSRCS=!PLUGSRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set PLUGSRCS=!PLUGSRCS! "%GEN%\%%P_cabi_marshal.cpp"
)
set PLUGSRCS=!PLUGSRCS! "%PYRAMID%\..\PCL\src\pcl_alloc.c"
echo == building FlatBuffers codec plugin DLL ==
cl /nologo /std:c++17 /EHsc /MD /LD %INC% !PLUGSRCS! /Fe:"%FLAT_PLUGIN_DLL%" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == compiling ==
cl /nologo /std:c++17 /EHsc /MD %INC% !SRCS! "%LIBPCL%" /Fe:"%HERE%comms_test.exe" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == running ==
"%HERE%comms_test.exe" "%FLAT_PLUGIN_DLL%"
exit /b %errorlevel%

:find_pcl_core
set "LIBPCL="
if defined PCL_CORE_LIB if exist "%PCL_CORE_LIB%" ( set "LIBPCL=%PCL_CORE_LIB%" & goto :eof )
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined LIBPCL for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_core.lib" 2^>nul') do set "LIBPCL=%%F"
)
goto :eof

:find_flatc
set "FLATC="
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined FLATC for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\flatc.exe" 2^>nul') do set "FLATC=%%F"
)
goto :eof

:find_flatbuffers_include
set "FLAT_INC="
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined FLAT_INC for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\flatbuffers.h" 2^>nul') do (
    for %%I in ("%%~dpF..") do set "FLAT_INC=%%~fI"
  )
)
goto :eof
