@echo off
REM _msvc_env.bat -- ensure an MSVC x64 developer environment is active.
REM
REM `call`-ed by the test_harness build scripts. If a VS dev environment is
REM already active (VSCMD_VER set), this is a no-op; otherwise it locates the
REM latest VS with the C++ x64 toolset via vswhere and enters vcvars64. Because
REM the callers use `call`, the environment changes persist into their session.
if defined VSCMD_VER exit /b 0
REM Already have the compiler on PATH (e.g. launched from a Developer prompt)?
where cl >nul 2>nul && exit /b 0

set "PF86=%ProgramFiles(x86)%"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo [msvc_env] vswhere.exe not found -- is Visual Studio installed?>&2
  exit /b 3
)

set "VSINSTALL="
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
if not defined VSINSTALL (
  echo [msvc_env] no Visual Studio with the C++ x64 toolset found>&2
  exit /b 3
)

REM vcvars64.bat probes `vswhere` bare and prints a benign "not recognized"
REM line to stderr; suppress it. Success is validated via VSCMD_VER below.
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
if not defined VSCMD_VER (
  echo [msvc_env] failed to enter vcvars64 dev environment>&2
  exit /b 3
)
exit /b 0
