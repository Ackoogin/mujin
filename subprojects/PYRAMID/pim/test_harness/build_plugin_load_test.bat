@echo off
REM build_plugin_load_test.bat -- no-relink plugin demonstration (Windows).
REM
REM Proves a component built from the new PIM proto can use a *prebuilt* codec
REM plugin with no relink, and that configuration is threaded through the loader
REM to the plugin:
REM
REM   1. Generate C++/JSON bindings from pim/test.
REM   2. Build the osprey sensor_products JSON codec plugin as a STANDALONE DLL
REM      (cl /LD) -- the plugin, complete with its wire codec.
REM   3. Build plugin_load_test.exe from the component-side sources ONLY (native
REM      types, C-ABI marshal, service facade, pcl_core.lib). No codec/plugin
REM      source is compiled into the test binary.
REM   4. Run the test: it loads the DLL at runtime via pcl_plugin_load_codec(),
REM      passing a config_json string, and drives a full provider/client
REM      round-trip through the loaded codec. A non-NULL codec_ctx confirms the
REM      config reached the plugin.
REM
REM pcl_core.lib is taken from an existing CMake build tree (build*/); build the
REM plugins/PCL once first (subprojects\PYRAMID\scripts\build_plugins.bat) or set
REM PCL_CORE_LIB. Optional first argument overrides the config_json passed to the
REM test (quoting complex JSON on the command line is fiddly; omit it to use the
REM test's built-in sample config).
setlocal enabledelayedexpansion

set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "PYRAMID=%%~fI"
for %%I in ("%PYRAMID%\..\..") do set "ROOT=%%~fI"
set "GEN=%HERE%generated"
set "CFG=%~1"
set "PLUGIN_DLL=%HERE%osprey_sensor_products_json_codec.dll"

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

set PKGS=pyramid_data_model_base pyramid_data_model_common pyramid_data_model_generic_pim_generic pyramid_data_model_common_pim_components_authorisation pyramid_data_model_common_pim_components_sensor_products pyramid_data_model_pim_osprey_sensor_products pyramid_components_pim_osprey_sensor_products_services_consumed pyramid_components_pim_osprey_sensor_products_services_provided

REM --- 1) the codec plugin as a standalone DLL (wire codec + marshal) ---
set PLUGSRCS="%GEN%\pyramid_services_pim_osprey_sensor_products_json_codec_plugin.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp"        set PLUGSRCS=!PLUGSRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set PLUGSRCS=!PLUGSRCS! "%GEN%\%%P_cabi_marshal.cpp"
)
set PLUGSRCS=!PLUGSRCS! "%PYRAMID%\..\PCL\src\pcl_alloc.c"
echo == building codec plugin DLL ==
cl /nologo /std:c++17 /EHsc /MD /LD %INC% !PLUGSRCS! /Fe:"%PLUGIN_DLL%" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

REM --- 2) the client/provider test with NO codec compiled in ---
REM        (component-side only: facade + C-ABI marshal + pcl_core)
set TESTSRCS="%HERE%plugin_load_test.cpp" "%GEN%\pyramid_services_pim_osprey_sensor_products_provided.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_cabi_marshal.cpp" set TESTSRCS=!TESTSRCS! "%GEN%\%%P_cabi_marshal.cpp"
)
echo == building plugin_load_test (no codec compiled in) ==
cl /nologo /std:c++17 /EHsc /MD %INC% !TESTSRCS! "%LIBPCL%" /Fe:"%HERE%plugin_load_test.exe" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == running (loads the DLL at runtime, no relink) ==
"%HERE%plugin_load_test.exe" "%PLUGIN_DLL%" %CFG%
exit /b %errorlevel%

:find_pcl_core
set "LIBPCL="
if defined PCL_CORE_LIB if exist "%PCL_CORE_LIB%" ( set "LIBPCL=%PCL_CORE_LIB%" & goto :eof )
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined LIBPCL for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_core.lib" 2^>nul') do set "LIBPCL=%%F"
)
goto :eof
