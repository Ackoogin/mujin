@echo off
REM Run ame_combined directly with full env setup and timeout
setlocal

set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%\..\..\..") do set REPO_ROOT=%%~fI

call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64
call "D:\Dev\ros2-windows\setup.bat"
call "%REPO_ROOT%\install\setup.bat"

set "PATH=%REPO_ROOT%\install\ame_ros2\lib\ame_ros2;%PATH%"
set "PATH=%REPO_ROOT%\install\ame_ros2\bin;%PATH%"
set "AMENT_PREFIX_PATH=%REPO_ROOT%\install\ame_ros2;%AMENT_PREFIX_PATH%"

cd /d "%REPO_ROOT%"

echo Launching ame_combined...
start /b "" "%REPO_ROOT%\install\ame_ros2\lib\ame_ros2\ame_combined.exe" --ros-args -p domain.pddl_file:=%REPO_ROOT:\=/%/subprojects/AME/domains/uav_search/domain.pddl -p domain.problem_file:=%REPO_ROOT:\=/%/subprojects/AME/domains/uav_search/problem.pddl > C:\Temp\mc_out.txt 2> C:\Temp\mc_err.txt

REM Wait 15 seconds then kill the process
timeout /t 15 /nobreak >nul
taskkill /f /im ame_combined.exe >nul 2>&1

echo.
echo === STDOUT ===
type C:\Temp\mc_out.txt 2>nul
echo === STDERR ===
type C:\Temp\mc_err.txt 2>nul
