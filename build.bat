@echo off
REM Build script for mujin test app
REM Uses Visual Studio 2022 generator (MSVC)

echo === Configuring with CMake ===
cmake --preset default
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
echo Run: build\src\Release\mujin_test_app.exe
