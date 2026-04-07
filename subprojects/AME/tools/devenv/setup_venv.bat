@echo off
rem AME Dev Environment - one-time virtual environment setup
:: Run from the repository root or from subprojects\AME\tools\devenv\

setlocal

set SCRIPT_DIR=%~dp0
set REPO_ROOT=%SCRIPT_DIR%..\..
set ROS2_PYTHON=D:\Dev\ros2-windows\.pixi\envs\default\python.exe

if exist "%ROS2_PYTHON%" (
    set BOOTSTRAP_PYTHON=%ROS2_PYTHON%
) else (
    set BOOTSTRAP_PYTHON=python
)

set VENV_DIR=%SCRIPT_DIR%.venv
set VENV_PYTHON=%VENV_DIR%\Scripts\python.exe

:: Recreate the venv if it was built against a different Python minor version.
if exist "%VENV_DIR%\pyvenv.cfg" (
    if exist "%ROS2_PYTHON%" (
        findstr /b /c:"version = 3.12." "%VENV_DIR%\pyvenv.cfg" >nul
        if errorlevel 1 (
            echo Rebuilding virtual environment for ROS2 Python 3.12...
            rmdir /s /q "%VENV_DIR%"
        )
    )
)

:: Create venv at subprojects\AME\tools\devenv\.venv
if not exist "%VENV_DIR%" (
    echo Creating virtual environment...
    "%BOOTSTRAP_PYTHON%" -m venv "%VENV_DIR%"
    if errorlevel 1 (
        echo ERROR: Failed to create virtual environment.
        exit /b 1
    )
) else (
    echo Virtual environment already exists.
)

:: Install / upgrade dependencies
echo Installing dependencies...
"%VENV_PYTHON%" -m pip install --upgrade pip --quiet
"%VENV_DIR%\Scripts\pip.exe" install -r "%SCRIPT_DIR%requirements.txt" --quiet
if errorlevel 1 (
    echo ERROR: pip install failed.
    exit /b 1
)

echo.
echo Setup complete.  Run subprojects\AME\tools\devenv\start_devenv.bat to launch.
endlocal
