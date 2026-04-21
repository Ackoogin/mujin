@echo off
REM generate_bindings.bat -- Wrapper around pim/generate_bindings.py.
REM
REM Usage:
REM   scripts\generate_bindings.bat [--cpp] [--ada]
REM                                 [--backends LIST]
REM                                 [--proto-dir DIR]
REM                                 [--cpp-out DIR]
REM                                 [--ada-out DIR]
REM
REM If neither --cpp nor --ada is supplied, both languages are generated.
setlocal enabledelayedexpansion

set "DO_CPP=0"
set "DO_ADA=0"
set "BACKENDS=json,flatbuffers"
set "PROTO_DIR="
set "CPP_OUT="
set "ADA_OUT="
for %%I in ("%~f0") do set "SCRIPT_BASE=%%~dpI"

if not defined PYRAMID_ROOT (
    for %%I in ("%SCRIPT_BASE%..") do set "PYRAMID_ROOT=%%~fI"
)

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="--cpp" (set "DO_CPP=1" & shift & goto parse_args)
if /i "%~1"=="--ada" (set "DO_ADA=1" & shift & goto parse_args)
if /i "%~1"=="--backends" (set "BACKENDS=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--proto-dir" (set "PROTO_DIR=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--cpp-out" (set "CPP_OUT=%~2" & shift & shift & goto parse_args)
if /i "%~1"=="--ada-out" (set "ADA_OUT=%~2" & shift & shift & goto parse_args)
echo [generate] Unknown argument: %~1
exit /b 1

:done_args
if "%DO_CPP%"=="0" if "%DO_ADA%"=="0" (
    set "DO_CPP=1"
    set "DO_ADA=1"
)

if "%PROTO_DIR%"=="" set "PROTO_DIR=%PYRAMID_ROOT%\proto"
if "%CPP_OUT%"=="" set "CPP_OUT=%PYRAMID_ROOT%\bindings\cpp\generated"
if "%ADA_OUT%"=="" set "ADA_OUT=%PYRAMID_ROOT%\bindings\ada\generated"

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
    echo [generate] FAIL: python launcher not found
    exit /b 1
)

set "GEN_SCRIPT=%PYRAMID_ROOT%\pim\generate_bindings.py"
if not exist "%GEN_SCRIPT%" (
    echo [generate] FAIL: generator not found at %GEN_SCRIPT%
    exit /b 1
)

if "%DO_CPP%"=="1" (
    echo [generate] C++ bindings -> %CPP_OUT%
    %PY_CMD% "%GEN_SCRIPT%" "%PROTO_DIR%" "%CPP_OUT%" --languages cpp --backends "%BACKENDS%"
    if errorlevel 1 exit /b 1
)

if "%DO_ADA%"=="1" (
    echo [generate] Ada bindings -> %ADA_OUT%
    %PY_CMD% "%GEN_SCRIPT%" "%PROTO_DIR%" "%ADA_OUT%" --languages ada --backends "%BACKENDS%"
    if errorlevel 1 exit /b 1
)

echo [generate] PASS
goto :eof
