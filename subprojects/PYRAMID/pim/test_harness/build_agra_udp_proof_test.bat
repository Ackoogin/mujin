@echo off
REM build_agra_udp_proof_test.bat -- Windows counterpart to
REM build_agra_udp_proof_test.sh: Phase D of
REM doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md, the UDP proof
REM (negative gate + BEST_EFFORT information topic over real UDP
REM datagrams). Authored for parity; not exercised in this Linux
REM environment (carried note, same as the other test_harness .bat scripts).
setlocal enabledelayedexpansion

set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "PYRAMID=%%~fI"
for %%I in ("%PYRAMID%\..\..") do set "ROOT=%%~fI"
set "GEN=%HERE%generated_agra_udp_proof"
set "SCRATCH=%HERE%agra_udp_proof_scratch"

call "%HERE%_msvc_env.bat"
if errorlevel 1 exit /b 3

echo == generating bindings ==
if exist "%GEN%" rmdir /s /q "%GEN%"
mkdir "%GEN%"
if exist "%SCRATCH%" rmdir /s /q "%SCRATCH%"
mkdir "%SCRATCH%"
python "%PYRAMID%\pim\generate_bindings.py" "%PYRAMID%\pim\agra_example" "%GEN%" --languages cpp --backends json >nul
if errorlevel 1 exit /b 2

call :find_pcl_core
if not defined LIBPCL (
  echo pcl_core.lib not found under %ROOT%\build*\ -- build PCL/plugins first>&2
  exit /b 2
)

call :find_udp_plugin
if not defined UDP_PLUGIN (
  echo pcl_transport_udp_plugin.dll not found under %ROOT%\build*\>&2
  exit /b 2
)

set INC=/I"%GEN%" /I"%PYRAMID%\..\PCL\include" /I"%PYRAMID%\core\external"

cl /nologo /std:c11 /MD /c /I"%PYRAMID%\..\PCL\include" "%PYRAMID%\..\PCL\src\pcl_alloc.c" /Fo"%GEN%\pcl_alloc.obj"
if errorlevel 1 exit /b 1

set COMMON_PKGS=pyramid_data_model_base pyramid_data_model_common pyramid_data_model_agra

call :build_json_codec ma pyramid_components_agra_mission_autonomy_services_provided pyramid_services_agra_mission_autonomy
if errorlevel 1 exit /b 1
call :build_json_codec c2 pyramid_components_agra_c2_station_services_consumed pyramid_services_agra_c2_station
if errorlevel 1 exit /b 1

set MAIN_PKGS=%COMMON_PKGS% pyramid_components_agra_mission_autonomy_services_provided pyramid_components_agra_c2_station_services_consumed
set SRCS="%HERE%agra_udp_proof_test.cpp" "%GEN%\pyramid_services_agra_mission_autonomy_provided.cpp" "%GEN%\pyramid_services_agra_c2_station_consumed.cpp"
for %%P in (%MAIN_PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp" set SRCS=!SRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set SRCS=!SRCS! "%GEN%\%%P_cabi_marshal.cpp"
)

echo == compiling ==
cl /nologo /std:c++17 /EHsc /MD %INC% %SRCS% "%LIBPCL%" /Fe:"%HERE%agra_udp_proof_test.exe" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == running ==
"%HERE%agra_udp_proof_test.exe" "%UDP_PLUGIN%" ^
    "%HERE%ma_agra_json_codec.dll" "%HERE%c2_agra_json_codec.dll" ^
    "%SCRATCH%"
exit /b %errorlevel%

:build_json_codec
setlocal
set "ROLE=%~1"
set "COMPONENTS_PKG=%~2"
set "PLUGIN_PKG=%~3"
set PKGS=%COMMON_PKGS% %COMPONENTS_PKG%
set JSON_SRCS="%GEN%\%PLUGIN_PKG%_json_codec_plugin.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp" set JSON_SRCS=!JSON_SRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set JSON_SRCS=!JSON_SRCS! "%GEN%\%%P_cabi_marshal.cpp"
)
echo == building %ROLE% JSON codec plugin DLL ==
cl /nologo /std:c++17 /EHsc /MD /LD %INC% !JSON_SRCS! "%GEN%\pcl_alloc.obj" /Fe:"%HERE%%ROLE%_agra_json_codec.dll" /Fo"%GEN%\\"
if errorlevel 1 (endlocal & exit /b 1)
endlocal
goto :eof

:find_pcl_core
set "LIBPCL="
if defined PCL_CORE_LIB if exist "%PCL_CORE_LIB%" ( set "LIBPCL=%PCL_CORE_LIB%" & goto :eof )
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined LIBPCL for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_core.lib" 2^>nul') do set "LIBPCL=%%F"
)
goto :eof

:find_udp_plugin
set "UDP_PLUGIN="
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined UDP_PLUGIN for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_transport_udp_plugin.dll" 2^>nul') do set "UDP_PLUGIN=%%F"
)
goto :eof
