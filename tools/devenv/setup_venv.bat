@echo off
rem AME Dev Environment - one-time virtual environment setup
:: Run from the repository root or from tools\devenv\

setlocal

set SCRIPT_DIR=%~dp0
set REPO_ROOT=%SCRIPT_DIR%..\..

:: Create venv at tools\devenv\.venv
if not exist "%SCRIPT_DIR%.venv" (
    echo Creating virtual environment...
    python -m venv "%SCRIPT_DIR%.venv"
    if errorlevel 1 (
        echo ERROR: Failed to create virtual environment.
        exit /b 1
    )
) else (
    echo Virtual environment already exists.
)

:: Install / upgrade dependencies
echo Installing dependencies...
"%SCRIPT_DIR%.venv\Scripts\python.exe" -m pip install --upgrade pip --quiet
"%SCRIPT_DIR%.venv\Scripts\pip.exe" install -r "%SCRIPT_DIR%requirements.txt" --quiet
if errorlevel 1 (
    echo ERROR: pip install failed.
    exit /b 1
)

echo.
echo Setup complete.  Run tools\devenv\start_devenv.bat to launch.
endlocal
