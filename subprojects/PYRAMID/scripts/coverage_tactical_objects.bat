@echo off
REM Generate source statement coverage report for tactical_objects module.
REM Requires: GCC (MinGW/msys2, etc.), gcov, gcovr (pip install gcovr)
REM
REM Output: coverage_tactical_objects/index.html, coverage_tactical_objects/summary.txt

setlocal enabledelayedexpansion
set SCRIPT_DIR=%~dp0
set ROOT=%SCRIPT_DIR%..
cd /d "%ROOT%"

set BUILD_DIR=build-coverage-tactical_objects
set COV_OUT=coverage_tactical_objects

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
  -DCMAKE_CXX_FLAGS="--coverage -O0 -fno-elide-constructors" ^
  -DCMAKE_EXE_LINKER_FLAGS="--coverage"
if %ERRORLEVEL% neq 0 exit /b 1

echo.
echo === Building tactical_objects tests ===
cmake --build %BUILD_DIR% -j%NUMBER_OF_PROCESSORS% ^
  --target test_tobj_types test_tobj_object_store test_tobj_milclass test_tobj_spatial ^
  test_tobj_zone test_tobj_correlation test_tobj_query test_tobj_relationship ^
  test_tobj_codec test_tobj_interest test_tobj_history test_tobj_runtime ^
  test_tobj_zone_perf test_tobj_component test_tobj_component_robustness test_tobj_component_hlr ^
  test_tobj_streaming_codec test_tobj_runtime_streaming test_tobj_interest_matching
if %ERRORLEVEL% neq 0 exit /b 1

echo.
echo === Running tests ===
for %%T in (test_tobj_types test_tobj_object_store test_tobj_milclass test_tobj_spatial test_tobj_zone test_tobj_correlation test_tobj_query test_tobj_relationship test_tobj_codec test_tobj_interest test_tobj_history test_tobj_runtime test_tobj_zone_perf test_tobj_component_robustness test_tobj_component_hlr test_tobj_streaming_codec test_tobj_runtime_streaming test_tobj_interest_matching test_tobj_component) do (
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
  --filter "pyramid/tactical_objects/src/.*" ^
  --object-directory %BUILD_DIR% ^
  --gcov-ignore-errors source_not_found ^
  --gcov-ignore-errors no_working_dir_found ^
  --gcov-ignore-parse-errors negative_hits.warn ^
  --html-details=%COV_OUT%/index.html ^
  --txt=%COV_OUT%/summary.txt

echo.
echo === Generating HLR-LLR-test traceability report ===
python "%ROOT%\scripts\gen_requirement_trace.py"
if %ERRORLEVEL% neq 0 (
    echo WARNING: gen_requirement_trace.py failed - traceability report may be missing
)

echo.
echo Coverage report: %COV_OUT%\index.html
echo Summary: %COV_OUT%\summary.txt
echo Traceability: %COV_OUT%\requirement_traceability.md
endlocal
