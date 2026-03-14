@echo off
REM Generate source statement coverage report for PCL (PYRAMID Container Library).
REM Requires: GCC (MinGW/msys2, etc.), gcov, gcovr (pip install gcovr)
REM
REM Output: coverage_pcl/index.html, coverage_pcl/summary.txt

setlocal enabledelayedexpansion
set SCRIPT_DIR=%~dp0
set ROOT=%SCRIPT_DIR%..
cd /d "%ROOT%"

set BUILD_DIR=build-coverage-pcl
set COV_OUT=coverage_pcl

where gcovr >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: gcovr not found in PATH. Install with: pip install gcovr
    exit /b 1
)

where gcc >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: gcc not found in PATH. Install MinGW or msys2 with GCC.
    exit /b 1
)

echo === Configuring (coverage build, GCC) ===
cmake -S . -B %BUILD_DIR% -G "Unix Makefiles" ^
  -DCMAKE_BUILD_TYPE=Debug ^
  -DAME_FOXGLOVE=OFF ^
  -DCMAKE_C_COMPILER=gcc ^
  -DCMAKE_CXX_COMPILER=g++ ^
  -DCMAKE_C_FLAGS="--coverage -O0" ^
  -DCMAKE_CXX_FLAGS="--coverage -O0" ^
  -DCMAKE_EXE_LINKER_FLAGS="--coverage"
if %ERRORLEVEL% neq 0 exit /b 1

echo.
echo === Building PCL tests ===
cmake --build %BUILD_DIR% -j%NUMBER_OF_PROCESSORS% ^
  --target test_pcl_lifecycle test_pcl_executor test_pcl_log test_pcl_robustness test_pcl_dining
if %ERRORLEVEL% neq 0 exit /b 1

echo.
echo === Running tests ===
for %%T in (test_pcl_lifecycle test_pcl_executor test_pcl_log test_pcl_robustness test_pcl_dining) do (
    set "EXE=%BUILD_DIR%\tests\%%T.exe"
    if exist "!EXE!" (
        echo Running %%T...
        "!EXE!"
    )
)

echo.
echo === Generating coverage report ===
if not exist "%COV_OUT%" mkdir "%COV_OUT%"
gcovr --root . ^
  --filter "src/pcl/.*" ^
  --object-directory %BUILD_DIR% ^
  --gcov-ignore-errors source_not_found ^
  --gcov-ignore-errors no_working_dir_found ^
  --gcov-ignore-parse-errors negative_hits.warn ^
  --html-details=%COV_OUT%/index.html ^
  --txt=%COV_OUT%/summary.txt

echo.
echo Coverage report: %COV_OUT%\index.html
echo Summary: %COV_OUT%\summary.txt
endlocal
