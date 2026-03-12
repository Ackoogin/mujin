# Run ROS2 ame nodes with correct environment
param(
    [string]$domain = "D:/Dev/repo/ame/domains/uav_search/domain.pddl",
    [string]$problem = "D:/Dev/repo/ame/domains/uav_search/problem.pddl",
    [int]$timeout = 30
)

& 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\Launch-VsDevShell.ps1' -Arch amd64 -SkipAutomaticLocation
. 'D:\Dev\ros2-windows\setup.ps1'
. 'D:\Dev\repo\ame\install\setup.ps1'

# Add DLL directories to PATH
$env:PATH = 'D:\Dev\repo\ame\install\ame_ros2\lib\ame_ros2;' + $env:PATH
$env:PATH = 'D:\Dev\repo\ame\install\ame_ros2\bin;' + $env:PATH

Set-Location D:\Dev\repo\ame

Write-Host "Starting ame_combined with:"
Write-Host "  domain:  $domain"
Write-Host "  problem: $problem"

ros2 run ame_ros2 ame_combined --ros-args `
  -p "domain.pddl_file:=$domain" `
  -p "domain.problem_file:=$problem"
