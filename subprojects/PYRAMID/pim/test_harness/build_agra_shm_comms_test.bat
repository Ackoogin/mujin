@echo off
REM build_agra_shm_comms_test.bat -- Windows counterpart to
REM build_agra_shm_comms_test.sh: Phase C of
REM doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md, the A-GRA example
REM contract's correlated request/entity pair, cross-process over a
REM real SHM bus. Authored for parity; not exercised in this Linux
REM environment (carried note, same as the other test_harness .bat scripts).
setlocal enabledelayedexpansion

set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "PYRAMID=%%~fI"
for %%I in ("%PYRAMID%\..\..") do set "ROOT=%%~fI"
set "GEN=%HERE%generated_agra_shm_comms"
set "SCRATCH=%HERE%agra_shm_comms_scratch"

call "%HERE%_msvc_env.bat"
if errorlevel 1 exit /b 3

echo == generating bindings ==
if exist "%GEN%" rmdir /s /q "%GEN%"
mkdir "%GEN%"
if exist "%SCRATCH%" rmdir /s /q "%SCRATCH%"
mkdir "%SCRATCH%"
python "%PYRAMID%\pim\generate_bindings.py" "%PYRAMID%\pim\agra_example" "%GEN%" --languages cpp --backends json,flatbuffers >nul
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

call :find_flatc
if not defined FLATC (
  echo flatc.exe not found under %ROOT%\build*\_deps\flatbuffers-build\>&2
  exit /b 2
)
call :find_flatbuffers_include
if not defined FLAT_INC_DIR (
  echo FlatBuffers headers not found under %ROOT%\build*\_deps\flatbuffers-src\include\>&2
  exit /b 2
)

set INC=/I"%GEN%" /I"%PYRAMID%\..\PCL\include" /I"%PYRAMID%\core\external" /I"%FLAT_INC_DIR%"

echo == generating FlatBuffers headers ==
"%FLATC%" --cpp --gen-object-api -o "%GEN%\flatbuffers\cpp" "%GEN%\flatbuffers\cpp\pyramid_services_agra_mission_autonomy.fbs"
if errorlevel 1 exit /b 1
"%FLATC%" --cpp --gen-object-api -o "%GEN%\flatbuffers\cpp" "%GEN%\flatbuffers\cpp\pyramid_services_agra_c2_station.fbs"
if errorlevel 1 exit /b 1

cl /nologo /std:c11 /MD /c /I"%PYRAMID%\..\PCL\include" "%PYRAMID%\..\PCL\src\pcl_alloc.c" /Fo"%GEN%\pcl_alloc.obj"
if errorlevel 1 exit /b 1

REM Shared data-model dependency closure both mission_autonomy and
REM c2_station reference.
set COMMON_PKGS=pyramid_data_model_base pyramid_data_model_common pyramid_data_model_agra

call :build_role ma pyramid_components_agra_mission_autonomy_services_provided pyramid_services_agra_mission_autonomy
if errorlevel 1 exit /b 1
call :build_role c2 pyramid_components_agra_c2_station_services_consumed pyramid_services_agra_c2_station
if errorlevel 1 exit /b 1

REM Main test binary: the harness + both packages' typed wrapper/cabi-
REM marshal sources (the codec plugins themselves stay DLLs, never linked
REM here).
set MAIN_PKGS=%COMMON_PKGS% pyramid_components_agra_mission_autonomy_services_provided pyramid_components_agra_c2_station_services_consumed
set SRCS="%HERE%agra_shm_comms_test.cpp" "%GEN%\pyramid_services_agra_mission_autonomy_provided.cpp" "%GEN%\pyramid_services_agra_c2_station_consumed.cpp"
for %%P in (%MAIN_PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp" set SRCS=!SRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set SRCS=!SRCS! "%GEN%\%%P_cabi_marshal.cpp"
)

echo == compiling ==
cl /nologo /std:c++17 /EHsc /MD %INC% %SRCS% "%LIBPCL%" /Fe:"%HERE%agra_shm_comms_test.exe" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == running ==
"%HERE%agra_shm_comms_test.exe" "%SHM_PLUGIN%" ^
    "%HERE%ma_agra_json_codec.dll" "%HERE%c2_agra_json_codec.dll" ^
    "%HERE%ma_agra_flatbuffers_codec.dll" "%HERE%c2_agra_flatbuffers_codec.dll" ^
    "%SCRATCH%"
exit /b %errorlevel%

:build_role
REM %1=role_label %2=components_pkg %3=plugin_pkg
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

set FLAT_SRCS="%GEN%\%PLUGIN_PKG%_flatbuffers_codec_plugin.cpp" "%GEN%\flatbuffers\cpp\%PLUGIN_PKG%_flatbuffers_codec.cpp"
for %%P in (%PKGS%) do (
  if exist "%GEN%\%%P_codec.cpp" set FLAT_SRCS=!FLAT_SRCS! "%GEN%\%%P_codec.cpp"
  if exist "%GEN%\%%P_cabi_marshal.cpp" set FLAT_SRCS=!FLAT_SRCS! "%GEN%\%%P_cabi_marshal.cpp"
)
echo == building %ROLE% FlatBuffers codec plugin DLL ==
cl /nologo /std:c++17 /EHsc /MD /LD %INC% !FLAT_SRCS! "%GEN%\pcl_alloc.obj" /Fe:"%HERE%%ROLE%_agra_flatbuffers_codec.dll" /Fo"%GEN%\\"
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

:find_shm_plugin
set "SHM_PLUGIN="
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined SHM_PLUGIN for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_transport_shared_memory_plugin.dll" 2^>nul') do set "SHM_PLUGIN=%%F"
)
goto :eof

:find_flatc
set "FLATC="
for /f "delims=" %%F in ('dir /b /s "%ROOT%\build*\_deps\flatbuffers-build\flatc.exe" 2^>nul') do if not defined FLATC set "FLATC=%%F"
goto :eof

:find_flatbuffers_include
set "FLAT_INC_DIR="
for /f "delims=" %%F in ('dir /b /s "%ROOT%\build*\_deps\flatbuffers-src\include\flatbuffers\flatbuffers.h" 2^>nul') do if not defined FLAT_INC_DIR (
  for %%J in ("%%~dpF..") do set "FLAT_INC_DIR=%%~fJ"
)
goto :eof
