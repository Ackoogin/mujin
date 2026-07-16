param(
  [Parameter(Mandatory = $true)]
  [string]$SdkRoot,

  [Parameter(Mandatory = $true)]
  [string]$BuildDir,

  [ValidateSet("shared_memory", "tcp")]
  [string]$Profile = "shared_memory",

  [ValidateRange(3, 300)]
  [int]$DurationSeconds = 15
)

$ErrorActionPreference = "Stop"
$SdkRoot = (Resolve-Path -LiteralPath $SdkRoot).Path
$BuildDir = (Resolve-Path -LiteralPath $BuildDir).Path
$ConfigDir = Join-Path $PSScriptRoot "configs\$Profile"

$Coordinator = Join-Path $BuildDir "Release\agra_p3_mission_coordinator.exe"
$Sensor = Join-Path $BuildDir "Release\agra_p3_sensor_planner.exe"
$Vehicle = Join-Path $BuildDir "Release\agra_p3_vehicle_planner.exe"
foreach ($Executable in @($Coordinator, $Sensor, $Vehicle)) {
  if (-not (Test-Path -LiteralPath $Executable)) {
    throw "Missing example executable: $Executable"
  }
}

$DurationArgument = "--duration-seconds=$DurationSeconds"
$Processes = @()
try {
  Write-Host "Starting $Profile profile from $SdkRoot"
  $Processes += Start-Process -FilePath $Coordinator -WorkingDirectory $SdkRoot `
      -ArgumentList @((Join-Path $ConfigDir "mission_coordinator.routes"), $DurationArgument) `
      -NoNewWindow -PassThru
  Start-Sleep -Seconds 1
  $Processes += Start-Process -FilePath $Sensor -WorkingDirectory $SdkRoot `
      -ArgumentList @((Join-Path $ConfigDir "sensor_planner.routes"), $DurationArgument) `
      -NoNewWindow -PassThru
  $Processes += Start-Process -FilePath $Vehicle -WorkingDirectory $SdkRoot `
      -ArgumentList @((Join-Path $ConfigDir "vehicle_planner.routes"), $DurationArgument) `
      -NoNewWindow -PassThru

  $Processes | Wait-Process
  $Failures = @($Processes | Where-Object { $_.ExitCode -ne 0 })
  if ($Failures.Count -ne 0) {
    $FailureDetails = $Failures | ForEach-Object {
      "$($_.Id) (exit $($_.ExitCode))"
    }
    throw "One or more example processes failed: $($FailureDetails -join ', ')"
  }
} finally {
  foreach ($Process in $Processes) {
    if (-not $Process.HasExited) {
      Stop-Process -Id $Process.Id
    }
  }
}

Write-Host "All three processes completed successfully."
