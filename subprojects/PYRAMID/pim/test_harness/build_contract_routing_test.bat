@echo off
REM build_contract_routing_test.bat -- Windows counterpart to the Linux
REM contract-derived endpoint routing validation.
setlocal enabledelayedexpansion

set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "PYRAMID=%%~fI"
for %%I in ("%PYRAMID%\..\..") do set "ROOT=%%~fI"
set "GEN=%HERE%generated_routing"
set "PLUGIN_DLL=%HERE%contract_transport_plugin.dll"
set "ROUTE_MANIFEST=%GEN%\contract_routes.pcl"

call "%HERE%_msvc_env.bat"
if errorlevel 1 exit /b 3

echo == generating bindings ==
if exist "%GEN%" rmdir /s /q "%GEN%"
mkdir "%GEN%"
python "%PYRAMID%\pim\generate_bindings.py" "%PYRAMID%\pim\test" "%GEN%" --languages cpp --backends json >nul
if errorlevel 1 exit /b 2

call :find_pcl_core
if not defined LIBPCL (
  echo pcl_core.lib not found under %ROOT%\build*\ -- build PCL/plugins first>&2
  exit /b 2
)

set INC=/I"%PYRAMID%\..\PCL\include"

echo == building contract transport plugin DLL ==
cl /nologo /std:c11 /MD /LD %INC% "%HERE%contract_transport_plugin.c" /Fe:"%PLUGIN_DLL%" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == deriving routing manifest from binding manifest ==
python "%HERE%contract_routing_manifest.py" "%GEN%\binding_manifest.json" "%PLUGIN_DLL%" "%ROUTE_MANIFEST%"
if errorlevel 1 exit /b 1

echo == compiling validation harness ==
cl /nologo /std:c++17 /EHsc /MD %INC% "%HERE%contract_routing_validation.cpp" "%LIBPCL%" /Fe:"%HERE%contract_routing_validation.exe" /Fo"%GEN%\\"
if errorlevel 1 exit /b 1

echo == running ==
"%HERE%contract_routing_validation.exe" "%ROUTE_MANIFEST%"
exit /b %errorlevel%

:find_pcl_core
set "LIBPCL="
if defined PCL_CORE_LIB if exist "%PCL_CORE_LIB%" ( set "LIBPCL=%PCL_CORE_LIB%" & goto :eof )
for %%D in (build build-plugins-win-newproto build-plugins build-all-off build-all-enabled build-all-off-debug) do (
  if not defined LIBPCL for /f "delims=" %%F in ('dir /b /s "%ROOT%\%%D\pcl_core.lib" 2^>nul') do set "LIBPCL=%%F"
)
goto :eof
