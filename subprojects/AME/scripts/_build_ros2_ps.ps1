# PowerShell helper: set up VS + ROS2 env, then run colcon
param([string]$Mode = "all")

$ameRoot = "D:\Dev\repo\mujin"

# Capture VS + ROS2 environment via temp batch
$envBat = @"
@echo off
set PATH=D:\Dev\ros2-windows\bin;D:\Dev\ros2-windows\.pixi\envs\default\Library\bin;%PATH%
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" > nul 2>&1
call "D:\Dev\ros2-windows\setup.bat" > nul 2>&1
set
"@
$tmpBat = [System.IO.Path]::GetTempFileName() + ".bat"
[System.IO.File]::WriteAllText($tmpBat, $envBat)
$envOutput = cmd /c $tmpBat 2>$null
Remove-Item $tmpBat -ErrorAction SilentlyContinue

# Apply env vars to current PowerShell session
foreach ($line in $envOutput) {
    $idx = $line.IndexOf('=')
    if ($idx -gt 0) {
        $k = $line.Substring(0, $idx)
        $v = $line.Substring($idx + 1)
        [System.Environment]::SetEnvironmentVariable($k, $v, 'Process')
    }
}

Write-Host "VisualStudioVersion=$env:VisualStudioVersion"
Write-Host "PATH length=$($env:PATH.Length)"
Write-Host "PYTHONPATH=$env:PYTHONPATH"

if ($Mode -eq "all" -or $Mode -eq "core") {
    Write-Host "`n=== Building ame_core ==="
    Push-Location $ameRoot
    & cmake --preset default "-DCMAKE_INSTALL_PREFIX=$ameRoot\build\install"
    if ($LASTEXITCODE -ne 0) { Write-Error "cmake configure failed"; exit 1 }
    & cmake --build "$ameRoot\build" --config Release -j $env:NUMBER_OF_PROCESSORS
    if ($LASTEXITCODE -ne 0) { Write-Error "cmake build failed"; exit 1 }
    & cmake --install "$ameRoot\build" --config Release
    if ($LASTEXITCODE -ne 0) { Write-Error "cmake install failed"; exit 1 }
    Pop-Location
}

if ($Mode -eq "all" -or $Mode -eq "ros2") {
    Write-Host "`n=== Building ame_ros2 ==="
    Set-Location $ameRoot
    & "D:\Dev\ros2-windows\.pixi\envs\default\Scripts\colcon.exe" build `
        --packages-select ame_ros2 `
        --base-paths "$ameRoot\subprojects\AME\ros2" `
        --cmake-force-configure `
        --cmake-args "-DCMAKE_PREFIX_PATH=D:/Dev/ros2-windows;D:/Dev/repo/mujin/build/install"
    if ($LASTEXITCODE -ne 0) { Write-Error "colcon build failed"; exit 1 }
}

Write-Host "`n=== Build complete ==="
