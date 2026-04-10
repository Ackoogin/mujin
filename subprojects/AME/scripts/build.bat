@echo off
REM Build script for ame test app
REM Uses Visual Studio 2022 generator (MSVC)
setlocal

set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%\..\..\..") do set REPO_ROOT=%%~fI

cd /d "%REPO_ROOT%"

echo === Configuring with CMake ===
cmake --preset default -DMUJIN_BUILD_PYRAMID=OFF
if %ERRORLEVEL% neq 0 (
    echo CMake configure FAILED
    exit /b 1
)

echo.
echo === Building ===
cmake --build build --config Release -j%NUMBER_OF_PROCESSORS%
if %ERRORLEVEL% neq 0 (
    echo Build FAILED
    exit /b 1
)

echo.
echo === Build complete ===
echo Run: build\subprojects\AME\src\Release\ame_test_app.exe
