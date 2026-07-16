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

$C2 = Join-Path $BuildDir "Release\agra_p3_c2.exe"
$MissionAutonomy = Join-Path $BuildDir "Release\agra_p3_mission_autonomy.exe"
$MissionSystem = Join-Path $BuildDir "Release\agra_p3_mission_system.exe"
foreach ($Executable in @($C2, $MissionAutonomy, $MissionSystem)) {
  if (-not (Test-Path -LiteralPath $Executable)) {
    throw "Missing example executable: $Executable"
  }
}

$SystemDurationArgument = "--duration-seconds=$($DurationSeconds + 2)"
$AutonomyDurationArgument = "--duration-seconds=$($DurationSeconds + 1)"
$C2DurationArgument = "--duration-seconds=$DurationSeconds"
$Processes = @()
try {
  Write-Host "Starting $Profile profile from $SdkRoot"
  $Processes += Start-Process -FilePath $MissionSystem -WorkingDirectory $SdkRoot `
      -ArgumentList @((Join-Path $ConfigDir "mission_system.ports"), $SystemDurationArgument) `
      -NoNewWindow -PassThru
  Start-Sleep -Seconds 1
  $Processes += Start-Process -FilePath $MissionAutonomy -WorkingDirectory $SdkRoot `
      -ArgumentList @((Join-Path $ConfigDir "mission_autonomy.ports"), $AutonomyDurationArgument) `
      -NoNewWindow -PassThru
  Start-Sleep -Seconds 1
  $Processes += Start-Process -FilePath $C2 -WorkingDirectory $SdkRoot `
      -ArgumentList @((Join-Path $ConfigDir "c2.ports"), $C2DurationArgument) `
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
