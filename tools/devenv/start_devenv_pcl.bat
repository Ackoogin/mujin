@echo off
REM ============================================================================
REM Start AME DevEnv with PCL backend (no ROS2 required)
REM ============================================================================

setlocal EnableDelayedExpansion

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%..\.."
set "PYTHON=%SCRIPT_DIR%.venv\Scripts\python.exe"

if not exist "%PYTHON%" (
    echo Virtual environment not found.  Running setup_venv.bat...
    call "%SCRIPT_DIR%setup_venv.bat"
    if errorlevel 1 exit /b 1
)

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

REM Verify the .pyd ABI matches the venv Python
for /f "tokens=2 delims= " %%V in ('"%PYTHON%" --version 2^>nul') do set "PY_VER=%%V"
for /f "tokens=1,2 delims=." %%A in ("%PY_VER%") do set "PY_ABI=cp%%A%%B"
for %%F in ("%REPO_ROOT%\build\python\_ame_py.cp*.pyd") do set "PYD_NAME=%%~nF"
if defined PYD_NAME (
    echo !PYD_NAME! | findstr /c:"!PY_ABI!" >nul
    if errorlevel 1 (
        echo.
        echo ERROR: Python version mismatch!
        echo   venv Python: %PY_VER% ^(!PY_ABI!^)
        echo   _ame_py built for: !PYD_NAME!
        echo.
        echo Rebuild _ame_py using the venv Python:
        echo   cmake -B build -DAME_BUILD_PYTHON=ON -DPython3_EXECUTABLE="%PYTHON%"
        echo   cmake --build build --config Release --target _ame_py
        echo.
        pause
        exit /b 1
    )
)

echo.
echo Starting AME DevEnv with PCL backend...
echo   Python:  %PYTHON% (%PY_VER%)
echo   Domain:  %DOMAIN_PATH%
echo   Problem: %PROBLEM_PATH%
echo.

REM Run devenv with PCL backend
cd /d "%REPO_ROOT%"
"%PYTHON%" -m tools.devenv --backend pcl --domain "%DOMAIN_PATH%" --problem "%PROBLEM_PATH%"

endlocal
