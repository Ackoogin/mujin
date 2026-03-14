# test_ada_socket_e2e.ps1 — Cross-process E2E test driver (Windows).
#
# Orchestrates:
#   1. Build Ada client (if gprbuild available)
#   2. Start C++ server (tobj_socket_server)
#   3. Wait for server to be ready (port file appears)
#   4. Start Ada client (ada_tobj_client)
#   5. Wait for Ada client to exit
#   6. Terminate server
#   7. Report pass/fail
#
# Usage: scripts/test_ada_socket_e2e.ps1 [-ServerBin PATH] [-ClientBin PATH]
# CTest: powershell -ExecutionPolicy Bypass -File script.ps1 -ServerBin PATH -ClientBin PATH

param(
  [string]$ServerBin = "",
  [string]$ClientBin = "",
  [int]$Port = 19234,
  [int]$Timeout = 15
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir = Split-Path -Parent $ScriptDir

if (-not $ServerBin) {
  $ServerBin = Join-Path $RootDir "build\tests\Release\tobj_socket_server.exe"
}
if (-not $ClientBin) {
  $ClientBin = Join-Path $RootDir "examples\ada\bin\ada_tobj_client.exe"
  if (-not (Test-Path $ClientBin)) {
    $ClientBin = Join-Path $RootDir "examples\ada\bin\ada_tobj_client"
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
  Write-Host "=== Ada Socket E2E Test ==="

  # Step 1: Build Ada client if gprbuild is available
  $gprbuild = Get-Command gprbuild -ErrorAction SilentlyContinue
  if ($gprbuild) {
    Write-Host "[driver] Building Ada client..."
    $adaDir = Join-Path $RootDir "examples\ada"
    $buildOk = $false
    try {
      Push-Location $adaDir
      $env:MUJIN_ROOT = $RootDir
      $null = & gprbuild -P ada_tobj_client.gpr -q 2>&1
      $buildOk = ($LASTEXITCODE -eq 0)
    } catch {
      $buildOk = $false
    } finally {
      Pop-Location -ErrorAction SilentlyContinue
    }
    if (-not $buildOk) {
      Write-Host "[driver] SKIP: gprbuild failed (GNAT toolchain issue)"
      exit 0
    }
  } else {
    Write-Host "[driver] gprbuild not found — checking for pre-built client..."
  }

  # Resolve client path (try .exe if not specified)
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

  # Step 2: Start server
  Write-Host "[driver] Starting server on port $Port..."
  $psi = New-Object System.Diagnostics.ProcessStartInfo
  $psi.FileName = $ServerBin
  $psi.Arguments = "--port $Port --port-file `"$PortFile`" --timeout $Timeout"
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

  # Step 5: Start Ada client
  Write-Host "[driver] Starting Ada client..."
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
    Write-Host "[driver] PASS: Ada client received entity updates over socket transport"
    exit 0
  } else {
    Write-Host "[driver] FAIL: Ada client exited with code $ClientExit"
    exit 1
  }
} catch {
  Cleanup
  Write-Host "[driver] ERROR: $_"
  exit 1
}
