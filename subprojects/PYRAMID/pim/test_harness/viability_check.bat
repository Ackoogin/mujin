@echo off
REM viability_check.bat -- Windows counterpart to viability_check.sh.
REM
REM 1. Generates C++ bindings from the new PIM proto tree (pim/test).
REM 2. Syntax-checks a representative component facade against the PCL headers
REM    with MSVC (cl /Zs), so the new bindings are proven compilable on Windows.
REM
REM Exits non-zero (with the compiler diagnostics) if the generator output does
REM not compile. See FINDINGS.md.
setlocal enabledelayedexpansion

set "HERE=%~dp0"
for %%I in ("%HERE%..\..") do set "PYRAMID=%%~fI"
for %%I in ("%PYRAMID%\..\..") do set "ROOT=%%~fI"
set "PROTO=%PYRAMID%\pim\test"
set "OUT=%~1"
if not defined OUT set "OUT=%HERE%generated"

call "%HERE%_msvc_env.bat"
if errorlevel 1 exit /b 3

echo == generating bindings from %PROTO% ==
if exist "%OUT%" rmdir /s /q "%OUT%"
mkdir "%OUT%"
python "%PYRAMID%\pim\generate_bindings.py" "%PROTO%" "%OUT%" --languages cpp --backends json | findstr /R "Found Done"
if errorlevel 1 (
  echo generation failed>&2
  exit /b 2
)

set INC=/I"%OUT%" /I"%PYRAMID%\..\PCL\include" /I"%PYRAMID%\core\external"

echo.
echo == syntax-checking osprey.sensor_products provided facade ==
> "%OUT%\_probe.cpp" echo #include "pyramid_services_pim_osprey_sensor_products_provided.hpp"
>> "%OUT%\_probe.cpp" echo int main(){return 0;}

cl /nologo /std:c++17 /EHsc /MD %INC% /Zs "%OUT%\_probe.cpp"
if errorlevel 1 (
  echo.
  echo FAIL: new proto is not yet viable for the C++ plugin bindings.
  echo        See FINDINGS.md.
  exit /b 1
)
echo PASS: bindings compile -- run build_comms_test.bat / build_plugin_load_test.bat next
exit /b 0
