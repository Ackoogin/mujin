# test_ada_active_find_e2e.ps1 -- Cross-process ActiveFind E2E test driver (Windows).
#
# Orchestrates:
#   1. Build Ada active-find client (if gprbuild available)
#   2. Start C++ server with --no-entity (tobj_socket_server)
#   3. Wait for server to be ready (port file appears)
#   4. Start Ada active-find executable (client + evidence provider)
#   5. Wait for Ada process to exit
#   6. Terminate server
#   7. Report pass/fail
#
# Usage: scripts/test_ada_active_find_e2e.ps1 [-ServerBin PATH] [-ClientBin PATH]
# CTest: powershell -ExecutionPolicy Bypass -File script.ps1 -ServerBin PATH -ClientBin PATH

param(
  [string]$ServerBin = "",
  [string]$ClientBin = "",
  [int]$Port = 19235,
  [int]$Timeout = 20
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir = Split-Path -Parent $ScriptDir

if (-not $ServerBin) {
  $ServerBin = Join-Path $RootDir "build\tests\Release\tobj_socket_server.exe"
}
if (-not $ClientBin) {
  $ClientBin = Join-Path $RootDir "examples\ada\bin\ada_active_find_e2e.exe"
  if (-not (Test-Path $ClientBin)) {
    $ClientBin = Join-Path $RootDir "examples\ada\bin\ada_active_find_e2e"
  }
}

$PortFile = [System.IO.Path]::GetTempFileName()
$ServerProcess = $null

function Cleanup {
  if ($ServerProcess -and -not $ServerProcess.HasExited) {
    try {
      Stop-Process -Id $ServerProcess.Id -Force -ErrorAction SilentlyContinue
    } catch {}
  }
  if (Test-Path $PortFile) {
    Remove-Item $PortFile -Force -ErrorAction SilentlyContinue
  }
}

try {
  Write-Host "=== Ada ActiveFind E2E Test ==="

  # Step 1: Build Ada active-find client if gprbuild is available.
  # If the build fails we fall through to check for a pre-built binary;
  # only SKIP when no usable binary exists at all.
  $gprbuild = Get-Command gprbuild -ErrorAction SilentlyContinue
  if ($gprbuild) {
    Write-Host "[driver] Building Ada active-find client..."
    $adaDir = Join-Path $RootDir "examples\ada"
    $buildOk = $false
    try {
      Push-Location $adaDir
      $env:MUJIN_ROOT = $RootDir
      $buildOutput = & gprbuild -P ada_active_find_e2e.gpr -q 2>&1
      $buildOk = ($LASTEXITCODE -eq 0)
    } catch {
      $buildOk = $false
    } finally {
      Pop-Location -ErrorAction SilentlyContinue
    }
    if (-not $buildOk) {
      Write-Host "[driver] gprbuild failed -- falling back to pre-built binary"
      Write-Host "[driver] gprbuild output: $buildOutput"
    }
  } else {
    Write-Host "[driver] gprbuild not found -- checking for pre-built client..."
  }

  # Resolve client path (try without .exe suffix too)
  if (-not (Test-Path $ClientBin)) {
    $altClient = $ClientBin + ".exe"
    if (Test-Path $altClient) {
      $ClientBin = $altClient
    }
  }
  if (-not (Test-Path $ClientBin)) {
    Write-Host "[driver] SKIP: Ada client binary not found at $ClientBin"
    exit 0
  }

  if (-not (Test-Path $ServerBin)) {
    Write-Host "[driver] FAIL: Server binary not found at $ServerBin"
    exit 1
  }

  # Step 2: Start server with --no-entity (Ada client drives entity creation via ActiveFind)
  Write-Host "[driver] Starting server on port $Port (--no-entity)..."
  $psi = New-Object System.Diagnostics.ProcessStartInfo
  $psi.FileName = $ServerBin
  $psi.Arguments = "--port $Port --port-file `"$PortFile`" --timeout $Timeout --no-entity"
  $psi.UseShellExecute = $false
  $psi.RedirectStandardOutput = $true
  $psi.RedirectStandardError = $true
  $psi.CreateNoWindow = $true
  $ServerProcess = [System.Diagnostics.Process]::Start($psi)

  # Step 3: Wait for port file
  Write-Host "[driver] Waiting for server to write port file..."
  $maxAttempts = 50
  $attempt = 0
  while ($attempt -lt $maxAttempts) {
    Start-Sleep -Milliseconds 100
    if ((Test-Path $PortFile) -and (Get-Item $PortFile).Length -gt 0) {
      break
    }
    $attempt++
  }

  if (-not (Test-Path $PortFile) -or (Get-Item $PortFile).Length -eq 0) {
    Write-Host "[driver] FAIL: Server did not write port file within 5 seconds"
    Cleanup
    exit 1
  }

  $ActualPort = (Get-Content $PortFile -Raw).Trim()
  Write-Host "[driver] Server ready on port $ActualPort"

  # Step 4: Brief delay so server enters accept()
  Start-Sleep -Milliseconds 200

  # Step 5: Start Ada active-find client
  Write-Host "[driver] Starting Ada active-find client..."
  $clientPsi = New-Object System.Diagnostics.ProcessStartInfo
  $clientPsi.FileName = $ClientBin
  $clientPsi.Arguments = "--host 127.0.0.1 --port $ActualPort"
  $clientPsi.UseShellExecute = $false
  $clientPsi.RedirectStandardOutput = $true
  $clientPsi.RedirectStandardError = $true
  $clientPsi.CreateNoWindow = $true
  $clientProcess = [System.Diagnostics.Process]::Start($clientPsi)
  $null = $clientProcess.WaitForExit(30000)
  $ClientExit = $clientProcess.ExitCode

  # Step 6: Stop server
  Cleanup

  # Step 7: Report
  if ($ClientExit -eq 0) {
    Write-Host "[driver] PASS: Ada ActiveFind E2E -- evidence provider + entity correlation succeeded"
    exit 0
  } else {
    Write-Host "[driver] FAIL: Ada active-find client exited with code $ClientExit"
    exit 1
  }
} catch {
  Cleanup
  Write-Host "[driver] ERROR: $_"
  exit 1
}
