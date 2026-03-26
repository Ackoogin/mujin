@echo off
REM Run ame_combined directly with full env setup and timeout
setlocal

call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64
call "D:\Dev\ros2-windows\setup.bat"
call "D:\Dev\repo\ame\install\setup.bat"

set "PATH=D:\Dev\repo\ame\install\ame_ros2\lib\ame_ros2;%PATH%"
set "PATH=D:\Dev\repo\ame\install\ame_ros2\bin;%PATH%"
set "AMENT_PREFIX_PATH=D:\Dev\repo\ame\install\ame_ros2;%AMENT_PREFIX_PATH%"

cd /d D:\Dev\repo\ame

echo Launching ame_combined...
start /b "" "D:\Dev\repo\ame\install\ame_ros2\lib\ame_ros2\ame_combined.exe" --ros-args -p domain.pddl_file:=D:/Dev/repo/ame/domains/uav_search/domain.pddl -p domain.problem_file:=D:/Dev/repo/ame/domains/uav_search/problem.pddl > C:\Temp\mc_out.txt 2> C:\Temp\mc_err.txt

REM Wait 15 seconds then kill the process
timeout /t 15 /nobreak >nul
taskkill /f /im ame_combined.exe >nul 2>&1

echo.
echo === STDOUT ===
type C:\Temp\mc_out.txt 2>nul
echo === STDERR ===
type C:\Temp\mc_err.txt 2>nul
