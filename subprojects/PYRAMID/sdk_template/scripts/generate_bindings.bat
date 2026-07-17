@echo off
REM generate_bindings.bat -- proto -> generated bindings, offline SDK edition.
REM
REM Wraps generator\generate_bindings.py against the SDK's own proto\ tree.
REM No network access, no parent monorepo required.
REM
REM Usage:
REM   scripts\generate_bindings.bat [--cpp] [--ada] [--backends LIST]
REM                                 [--proto-dir DIR] [--cpp-out DIR] [--ada-out DIR]
REM                                 [--skeletons] [--scaffold-dir DIR]
REM
REM If neither --cpp nor --ada is supplied, both languages are generated.
setlocal enabledelayedexpansion

set "DO_CPP=0"
set "DO_ADA=0"
set "BACKENDS=json,flatbuffers"
set "PROTO_DIR="
set "CPP_OUT="
set "ADA_OUT="
set "SKELETONS=0"
set "SCAFFOLD_DIR="
for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"
for %%I in ("%SCRIPT_BASE%..") do set "SDK_ROOT=%%~fI"

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--cpp" (set "DO_CPP=1" & shift & goto parse_args)
if /i "%~1"=="--ada" (set "DO_ADA=1" & shift & goto parse_args)
if /i "%~1"=="--backends" (set "BACKENDS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--proto-dir" (set "PROTO_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--cpp-out" (set "CPP_OUT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--ada-out" (set "ADA_OUT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--skeletons" (set "SKELETONS=1" & shift & goto parse_args)
if /i "%~1"=="--scaffold-dir" (set "SCAFFOLD_DIR=%~2" & shift & shift & goto parse_args)
echo [generate] Unknown argument: %~1
exit /b 1

:done_args
if "%DO_CPP%"=="0" if "%DO_ADA%"=="0" (
    set "DO_CPP=1"
    set "DO_ADA=1"
)

if "%PROTO_DIR%"=="" set "PROTO_DIR=%SDK_ROOT%\proto"
if "%CPP_OUT%"=="" set "CPP_OUT=%SDK_ROOT%\sdk_project\generated"
if "%ADA_OUT%"=="" set "ADA_OUT=%SDK_ROOT%\gnat\generated_ada"

set "PY_CMD="
where python >nul 2>&1
if %errorlevel% equ 0 set "PY_CMD=python"
if not defined PY_CMD (
    where python3 >nul 2>&1
    if %errorlevel% equ 0 set "PY_CMD=python3"
)
if not defined PY_CMD (
    where py >nul 2>&1
    if %errorlevel% equ 0 set "PY_CMD=py -3"
)
if not defined PY_CMD (
    echo [generate] FAIL: python launcher not found -- Python 3 is required
    exit /b 1
)

set "GEN_SCRIPT=%SDK_ROOT%\generator\generate_bindings.py"
if not exist "%GEN_SCRIPT%" (
    echo [generate] FAIL: generator not found at %GEN_SCRIPT%
    exit /b 1
)
if not exist "%PROTO_DIR%" (
    echo [generate] FAIL: proto dir not found at %PROTO_DIR%
    exit /b 1
)

if "%DO_CPP%"=="1" (
    echo [generate] C++ bindings -^> %CPP_OUT%
    set "EXTRA_ARGS="
    if "!SKELETONS!"=="1" set "EXTRA_ARGS=!EXTRA_ARGS! --component-skeletons"
    if not "!SCAFFOLD_DIR!"=="" set EXTRA_ARGS=!EXTRA_ARGS! --scaffold-dir "!SCAFFOLD_DIR!"
    %PY_CMD% "%GEN_SCRIPT%" "%PROTO_DIR%" "%CPP_OUT%" --languages cpp --backends "%BACKENDS%" !EXTRA_ARGS!
    if errorlevel 1 exit /b 1
)

if "%DO_ADA%"=="1" (
    echo [generate] Ada bindings -^> %ADA_OUT%
    set "EXTRA_ARGS="
    if "!SKELETONS!"=="1" set "EXTRA_ARGS=!EXTRA_ARGS! --component-skeletons"
    if not "!SCAFFOLD_DIR!"=="" set EXTRA_ARGS=!EXTRA_ARGS! --scaffold-dir "!SCAFFOLD_DIR!"
    %PY_CMD% "%GEN_SCRIPT%" "%PROTO_DIR%" "%ADA_OUT%" --languages ada --backends "%BACKENDS%" !EXTRA_ARGS!
    if errorlevel 1 exit /b 1
)

echo [generate] PASS
goto :eof
