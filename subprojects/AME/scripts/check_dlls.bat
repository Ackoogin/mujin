@echo off
setlocal enabledelayedexpansion
REM Check DLL dependencies
set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%\..\..\..") do set REPO_ROOT=%%~fI
call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64
call "D:\Dev\ros2-windows\setup.bat"
call "%REPO_ROOT%\install\setup.bat"
set "PATH=%REPO_ROOT%\install\ame_ros2\lib\ame_ros2;%PATH%"
set "PATH=%REPO_ROOT%\install\ame_ros2\bin;%PATH%"

for %%D in (
    rclcpp_action.dll
    rclcpp_lifecycle.dll
    ame_ros2__rosidl_typesupport_cpp.dll
    rcl_action.dll
    rclcpp.dll
    lifecycle_msgs__rosidl_typesupport_cpp.dll
    rcl.dll
    rmw.dll
    std_msgs__rosidl_typesupport_cpp.dll
    rcutils.dll
    ame_ros2_lib.dll
) do (
    where %%D >nul 2>&1
    if !errorlevel! equ 0 (
        for /f "delims=" %%P in ('where %%D 2^>nul') do echo OK: %%D -^> %%P
    ) else (
        echo MISSING: %%D
    )
)
