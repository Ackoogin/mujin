# test_ada_active_find_e2e.ps1 -- Cross-process ActiveFind E2E test driver (Windows).
#
# 3-process architecture:
#   1. TacticalObjectsComponent server (--no-bridge --no-entity)
#   2. Standalone StandardBridge (dual TCP: client->server, server->Ada)
#   3. Ada active-find client (connects to bridge)
#
# Usage: scripts/test_ada_active_find_e2e.ps1 [-ServerBin PATH]
#            [-BridgeBin PATH] [-ClientBin PATH]

param(
  [string]$ServerBin = "",
  [string]$BridgeBin = "",
  [string]$ClientBin = "",
  [int]$BackendPort = 19235,
  [int]$FrontendPort = 19236,
  [int]$Timeout = 25
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir = Split-Path -Parent $ScriptDir

if (-not $ServerBin) {
  $ServerBin = Join-Path $RootDir "build\tests\Release\tobj_socket_server.exe"
}
if (-not $BridgeBin) {
  $BridgeBin = Join-Path $RootDir "build\tests\Release\standalone_bridge.exe"
}
if (-not $ClientBin) {
  $ClientBin = Join-Path $RootDir "examples\ada\bin\ada_active_find_e2e.exe"
  if (-not (Test-Path $ClientBin)) {
    $ClientBin = Join-Path $RootDir "examples\ada\bin\ada_active_find_e2e"
  }
}

$PortFile = [System.IO.Path]::GetTempFileName()
$BridgePortFile = [System.IO.Path]::GetTempFileName()
$ServerProcess = $null
$BridgeProcess = $null

function Cleanup {
  if ($BridgeProcess -and -not $BridgeProcess.HasExited) {
    try {
      Stop-Process -Id $BridgeProcess.Id -Force -ErrorAction SilentlyContinue
    } catch {}
  }
  if ($ServerProcess -and -not $ServerProcess.HasExited) {
    try {
      Stop-Process -Id $ServerProcess.Id -Force -ErrorAction SilentlyContinue
    } catch {}
  }
  if (Test-Path $PortFile) {
    Remove-Item $PortFile -Force -ErrorAction SilentlyContinue
  }
  if (Test-Path $BridgePortFile) {
    Remove-Item $BridgePortFile -Force -ErrorAction SilentlyContinue
  }
}

try {
  Write-Host "=== Ada ActiveFind E2E Test (3-process: server -> bridge -> client) ==="

  # Step 1: Build Ada active-find client if gprbuild is available.
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

  # Resolve client path
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
  if (-not (Test-Path $BridgeBin)) {
    Write-Host "[driver] FAIL: Bridge binary not found at $BridgeBin"
    exit 1
  }

  # Step 2: Start backend server with --no-bridge --no-entity
  Write-Host "[driver] Starting backend server on port $BackendPort (--no-bridge --no-entity)..."
  $psi = New-Object System.Diagnostics.ProcessStartInfo
  $psi.FileName = $ServerBin
  $psi.Arguments = "--port $BackendPort --port-file `"$PortFile`" --timeout $Timeout --no-entity --no-bridge"
  $psi.UseShellExecute = $false
  $psi.RedirectStandardOutput = $true
  $psi.RedirectStandardError = $true
  $psi.CreateNoWindow = $true
  $ServerProcess = [System.Diagnostics.Process]::Start($psi)

  # Step 3: Wait for server port file
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

  $ActualBackendPort = (Get-Content $PortFile -Raw).Trim()
  Write-Host "[driver] Backend server ready on port $ActualBackendPort"

  # Step 4: Brief delay so server enters accept()
  Start-Sleep -Milliseconds 200

  # Step 5: Start standalone bridge
  Write-Host "[driver] Starting standalone bridge (backend=$ActualBackendPort, frontend=$FrontendPort)..."
  $bpsi = New-Object System.Diagnostics.ProcessStartInfo
  $bpsi.FileName = $BridgeBin
  $bpsi.Arguments = "--backend-host 127.0.0.1 --backend-port $ActualBackendPort --frontend-port $FrontendPort --port-file `"$BridgePortFile`" --timeout $Timeout"
  $bpsi.UseShellExecute = $false
  $bpsi.RedirectStandardOutput = $true
  $bpsi.RedirectStandardError = $true
  $bpsi.CreateNoWindow = $true
  $BridgeProcess = [System.Diagnostics.Process]::Start($bpsi)

  # Step 6: Wait for bridge port file
  Write-Host "[driver] Waiting for bridge to write port file..."
  $attempt = 0
  while ($attempt -lt $maxAttempts) {
    Start-Sleep -Milliseconds 100
    if ((Test-Path $BridgePortFile) -and (Get-Item $BridgePortFile).Length -gt 0) {
      break
    }
    $attempt++
  }

  if (-not (Test-Path $BridgePortFile) -or (Get-Item $BridgePortFile).Length -eq 0) {
    Write-Host "[driver] FAIL: Bridge did not write port file within 5 seconds"
    Cleanup
    exit 1
  }

  $ActualFrontendPort = (Get-Content $BridgePortFile -Raw).Trim()
  Write-Host "[driver] Bridge ready, frontend on port $ActualFrontendPort"

  # Step 7: Brief delay so bridge enters accept()
  Start-Sleep -Milliseconds 200

  # Step 8: Start Ada active-find client (connects to bridge)
  Write-Host "[driver] Starting Ada active-find client (-> bridge port $ActualFrontendPort)..."
  $clientPsi = New-Object System.Diagnostics.ProcessStartInfo
  $clientPsi.FileName = $ClientBin
  $clientPsi.Arguments = "--host 127.0.0.1 --port $ActualFrontendPort"
  $clientPsi.UseShellExecute = $false
  $clientPsi.RedirectStandardOutput = $true
  $clientPsi.RedirectStandardError = $true
  $clientPsi.CreateNoWindow = $true
  $clientProcess = [System.Diagnostics.Process]::Start($clientPsi)
  $null = $clientProcess.WaitForExit(30000)
  $ClientExit = $clientProcess.ExitCode

  # Step 9: Cleanup
  Cleanup

  # Step 10: Report
  if ($ClientExit -eq 0) {
    Write-Host "[driver] PASS: Ada ActiveFind E2E (3-process) -- evidence + correlation via standalone bridge succeeded"
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
