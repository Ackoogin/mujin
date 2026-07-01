@echo off
REM build_ada.bat -- build the generated Ada C-ABI marshal archive, offline SDK edition.
REM
REM Runs gnat\build_gnat_pyramid_cabi_marshal_libs.bat (compiles the
REM proto-dependent C-ABI marshalling sources into a GNAT-compatible archive;
REM Ada depends on PCL + this C-ABI layer only, never codec-specific content)
REM then leaves gprbuild to the caller's own client project, which should
REM `with` gnat\pyramid_sdk_ada.gpr.
REM
REM Prerequisite: scripts\generate_bindings.bat --ada (or no-arg, which does both).
REM
REM Usage: build_ada.bat [--force]
setlocal enabledelayedexpansion

for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"
for %%I in ("%SCRIPT_BASE%..") do set "SDK_ROOT=%%~fI"

where gprbuild >nul 2>&1
if errorlevel 1 (
  echo [build_ada] FAIL: gprbuild not found on PATH ^(Ada builds need a GNAT toolchain^).>&2
  exit /b 1
)

if not exist "%SDK_ROOT%\gnat\generated_ada" (
  echo [build_ada] FAIL: no generated Ada bindings at %SDK_ROOT%\gnat\generated_ada -- run scripts\generate_bindings.bat first.>&2
  exit /b 1
)

echo [build_ada] building GNAT-compatible C-ABI marshal archive ...
call "%SDK_ROOT%\gnat\build_gnat_pyramid_cabi_marshal_libs.bat" %*
if errorlevel 1 exit /b 1

echo.
echo [build_ada] Ready. Build your own client with:
echo [build_ada]   gprbuild -P your_client.gpr
echo [build_ada] where your_client.gpr contains:
echo [build_ada]   with "%SDK_ROOT%\gnat\pyramid_sdk_ada.gpr";
echo.
echo [build_ada] PASS
exit /b 0
