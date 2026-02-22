# Run mujin_combined directly with full env setup and timeout
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\Launch-VsDevShell.ps1' -Arch amd64 -SkipAutomaticLocation
. 'D:\Dev\ros2-windows\setup.ps1'
. 'D:\Dev\repo\mujin\install\setup.ps1'

$env:PATH = 'D:\Dev\repo\mujin\install\mujin_ros2\lib\mujin_ros2;' + $env:PATH
$env:PATH = 'D:\Dev\repo\mujin\install\mujin_ros2\bin;' + $env:PATH
$env:AMENT_PREFIX_PATH = 'D:\Dev\repo\mujin\install\mujin_ros2;' + $env:AMENT_PREFIX_PATH

Set-Location D:\Dev\repo\mujin

Write-Host "Launching mujin_combined..."
$proc = Start-Process -FilePath 'D:\Dev\repo\mujin\install\mujin_ros2\lib\mujin_ros2\mujin_combined.exe' `
  -ArgumentList '--ros-args -p domain.pddl_file:=D:/Dev/repo/mujin/domains/uav_search/domain.pddl -p domain.problem_file:=D:/Dev/repo/mujin/domains/uav_search/problem.pddl' `
  -PassThru -NoNewWindow `
  -RedirectStandardOutput 'C:\Temp\mc_out.txt' `
  -RedirectStandardError  'C:\Temp\mc_err.txt'

# Wait up to 15 seconds
$proc.WaitForExit(15000) | Out-Null
if (!$proc.HasExited) {
    Write-Host "Node is running (killing after 15s)..."
    $proc.Kill()
    $proc.WaitForExit(2000) | Out-Null
} else {
    Write-Host "Process exited with code: $($proc.ExitCode) (0x$($proc.ExitCode.ToString('X8')))"
}

Write-Host ""
Write-Host "=== STDOUT ==="
Get-Content 'C:\Temp\mc_out.txt' -ErrorAction SilentlyContinue
Write-Host "=== STDERR ==="
Get-Content 'C:\Temp\mc_err.txt' -ErrorAction SilentlyContinue
