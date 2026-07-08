@echo off
REM build_routed_egress_test.bat -- Windows counterpart to
REM build_routed_egress_test.sh: Phase A of
REM doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md, the routed-egress
REM SHM proof. Authored for parity; not exercised in this Linux environment
REM (carried note, same as the other test_harness .bat scripts).
setlocal enabledelayedexpansion

set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "PYRAMID=%%~fI"
for %%I in ("%PYRAMID%\..\..") do set "ROOT=%%~fI"
set "GEN=%HERE%generated_routed_egress"
set "SCRATCH=%HERE%routed_egress_scratch"

call "%HERE%_msvc_env.bat"
if errorlevel 1 exit /b 3

echo == generating bindings ==
if exist "%GEN%" rmdir /s /q "%GEN%"
mkdir "%GEN%"
if exist "%SCRATCH%" rmdir /s /q "%SCRATCH%"
mkdir "%SCRATCH%"
python "%PYRAMID%\pim\generate_bindings.py" "%PYRAMID%\pim\test" "%GEN%" --languages cpp --backends json >nul
if errorlevel 1 exit /b 2

call :find_pcl_core
if not defined LIBPCL (
  echo pcl_core.lib not found under %ROOT%\build*\ -- build PCL/plugins first>&2
  exit /b 2
)

call :find_shm_plugin
if not defined SHM_PLUGIN (
  echo pcl_transport_shared_memory_plugin.dll not found under %ROOT%\build*\>&2
  exit /b 2
)

set INC=/I"%GEN%" /I"%PYRAMID%\..\PCL\include" /I"%PYRAMID%\core\external"

REM Generated sources in the osprey sensor_products dependency closure (same
REM closure as build_comms_test.bat).
set PKGS=pyramid_data_model_base pyramid_data_model_common pyramid_data_model_generic_pim_generic pyramid_data_model_common_pim_components_authorisation pyramid_data_model_common_pim_components_sensor_products pyramid_data_model_pim_osprey_sensor_products pyramid_components_pim_osprey_sensor_products_services_consumed pyramid_components_pim_osprey_sensor_products_services_provided

set SRCS="%HERE%routed_egress_shm_test.cpp" "%GEN%\pyramid_services_pim_osprey_sensor_products_provided.cpp" "%GEN%\pyramid_services_pim_osprey_sensor_products_json_codec_plugin.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp" set SRCS=!SRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set SRCS=!SRCS! "%GEN%\%%P_cabi_marshal.cpp"
)

echo == compiling ==
cl /nologo /std:c++17 /EHsc /MD %INC% %SRCS% "%LIBPCL%" /Fe:"%HERE%routed_egress_shm_test.exe" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == running ==
"%HERE%routed_egress_shm_test.exe" "%SHM_PLUGIN%" "%SCRATCH%"
exit /b %errorlevel%

:find_pcl_core
set "LIBPCL="
if defined PCL_CORE_LIB if exist "%PCL_CORE_LIB%" ( set "LIBPCL=%PCL_CORE_LIB%" & goto :eof )
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined LIBPCL for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_core.lib" 2^>nul') do set "LIBPCL=%%F"
)
goto :eof

:find_shm_plugin
set "SHM_PLUGIN="
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined SHM_PLUGIN for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_transport_shared_memory_plugin.dll" 2^>nul') do set "SHM_PLUGIN=%%F"
)
goto :eof
