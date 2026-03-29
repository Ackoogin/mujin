@echo off
REM ============================================================================
REM Start AME DevEnv with PCL backend (no ROS2 required)
REM ============================================================================

setlocal EnableDelayedExpansion

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%..\.."

REM Add the Python bindings to PYTHONPATH
set "PYTHONPATH=%REPO_ROOT%\build\python;%PYTHONPATH%"

REM Default PDDL paths (can be overridden via arguments)
set "DOMAIN_PATH=%REPO_ROOT%\domains\uav_search\domain.pddl"
set "PROBLEM_PATH=%REPO_ROOT%\domains\uav_search\problem.pddl"

REM Parse arguments
:parse_args
if "%~1"=="" goto :done_args
if /i "%~1"=="--domain" (
    set "DOMAIN_PATH=%~2"
    shift
    shift
    goto :parse_args
)
if /i "%~1"=="--problem" (
    set "PROBLEM_PATH=%~2"
    shift
    shift
    goto :parse_args
)
shift
goto :parse_args

:done_args

REM Activate venv if it exists
if exist "%SCRIPT_DIR%.venv\Scripts\activate.bat" (
    call "%SCRIPT_DIR%.venv\Scripts\activate.bat"
)

REM Check if Python bindings exist
if not exist "%REPO_ROOT%\build\python\_ame_py*.pyd" (
    echo.
    echo ERROR: Python bindings not found!
    echo.
    echo Please build with Python bindings enabled:
    echo   cmake -B build -DAME_BUILD_PYTHON=ON
    echo   cmake --build build --config Release --target _ame_py
    echo.
    pause
    exit /b 1
)

echo.
echo Starting AME DevEnv with PCL backend...
echo   Domain:  %DOMAIN_PATH%
echo   Problem: %PROBLEM_PATH%
echo.

REM Run devenv with PCL backend
cd /d "%REPO_ROOT%"
python -m tools.devenv --backend pcl --domain "%DOMAIN_PATH%" --problem "%PROBLEM_PATH%"

endlocal
