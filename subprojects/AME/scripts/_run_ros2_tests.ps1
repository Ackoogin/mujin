# Set up VS + ROS2 environment, then run all ame_ros2 test binaries
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
Remove-Item $tmpBat

# Apply env vars to current PowerShell session using $env: syntax (affects child process inheritance)
foreach ($line in $envOutput) {
    $idx = $line.IndexOf('=')
    if ($idx -gt 0) {
        $k = $line.Substring(0, $idx)
        $v = $line.Substring($idx + 1)
        Set-Item -Path "Env:$k" -Value $v
    }
}

# Add ame_ros2 DLL locations and ame_core DLLs
$env:PATH = "D:\Dev\repo\unmanned\build\ame_ros2\Release;" +
            "D:\Dev\repo\unmanned\build\ame_ros2\test\Release;" +
            "D:\Dev\repo\unmanned\install\ame_ros2\lib\ame_ros2;" +
            "D:\Dev\repo\unmanned\build\install\bin;" +
            $env:PATH

Write-Host "VisualStudioVersion=$env:VisualStudioVersion"
Write-Host "ros2 bin in PATH: $($env:PATH -like '*ros2-windows\bin*')"

$base = "D:\Dev\repo\unmanned\build\ame_ros2\test\Release"
$tests = @(
    "test_world_model_node",
    "test_executor_node",
    "test_planner_node",
    "test_full_pipeline",
    "test_extension_wiring",
    "test_e2e_pyramid_service"
)

$pass = 0
$fail = 0

foreach ($t in $tests) {
    Write-Host "`n=== $t ===" -ForegroundColor Cyan
    $exe = Join-Path $base "$t.exe"
    & $exe --gtest_brief=1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "PASSED" -ForegroundColor Green
        $pass++
    } else {
        Write-Host "FAILED (exit $LASTEXITCODE)" -ForegroundColor Red
        $fail++
    }
}

Write-Host "`n=============================" -ForegroundColor White
Write-Host "Results: $pass passed, $fail failed" -ForegroundColor $(if ($fail -eq 0) { "Green" } else { "Red" })
if ($fail -gt 0) { exit 1 }
