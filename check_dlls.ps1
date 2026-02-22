# Check DLL dependencies
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\Launch-VsDevShell.ps1' -Arch amd64 -SkipAutomaticLocation
. 'D:\Dev\ros2-windows\setup.ps1'
. 'D:\Dev\repo\mujin\install\setup.ps1'
$env:PATH = 'D:\Dev\repo\mujin\install\mujin_ros2\lib\mujin_ros2;' + $env:PATH
$env:PATH = 'D:\Dev\repo\mujin\install\mujin_ros2\bin;' + $env:PATH

$dlls = @(
    'rclcpp_action.dll',
    'rclcpp_lifecycle.dll',
    'mujin_ros2__rosidl_typesupport_cpp.dll',
    'rcl_action.dll',
    'rclcpp.dll',
    'lifecycle_msgs__rosidl_typesupport_cpp.dll',
    'rcl.dll',
    'rmw.dll',
    'std_msgs__rosidl_typesupport_cpp.dll',
    'rcutils.dll',
    'mujin_ros2_lib.dll'
)

foreach ($dll in $dlls) {
    $found = Get-Command $dll -ErrorAction SilentlyContinue
    if ($found) {
        Write-Host "OK: $dll -> $($found.Source)"
    } else {
        Write-Host "MISSING: $dll"
    }
}
