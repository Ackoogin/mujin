@echo off
rem AME Dev Environment - launch script
rem Usage: start_devenv.bat [--foxglove-url URL] [--no-ros2] [--width W] [--height H]
rem Run setup_venv.bat once first if the .venv directory does not exist.

setlocal

set SCRIPT_DIR=%~dp0
set REPO_ROOT=%SCRIPT_DIR%..\..
set PYTHON=%SCRIPT_DIR%.venv\Scripts\python.exe

if not exist "%PYTHON%" (
    echo Virtual environment not found.  Running setup_venv.bat...
    call "%SCRIPT_DIR%setup_venv.bat"
    if errorlevel 1 exit /b 1
)

REM Pin CycloneDDS to one interface (avoids multi-NIC reply confusion with VMware adapters)
set CYCLONEDDS_URI=file://D:/Dev/repo/mujin/cyclonedds_localhost.xml

cd /d "%REPO_ROOT%"
"%PYTHON%" -m tools.devenv %*
