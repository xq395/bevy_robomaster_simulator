[CmdletBinding()]
param(
    [string]$DeployDir = 'D:\HNU_RM\代码栈\deploy_dev',
    [string]$ConfigPath = 'D:\HNU_RM\代码栈\deploy_dev\config\sp_standard.yaml',
    [string]$ShmDir = 'F:\talos_shm',
    [string]$ModelPath = 'F:\bevy_robomaster_simulator\.deps\models\praysky_coord_noe2e_0331_640x640_fp16.xml',
    [string]$WebDir = 'F:\talos_web_debug',
    [ValidateRange(1, 65535)]
    [int]$Port = 5000,
    [ValidateRange(0.1, 30.0)]
    [double]$WebHz = 5.0,
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Config = 'Release',
    [ValidateRange(0, 63)]
    [int]$CpuCores = 0,
    [UInt64]$MaxFrames = 0,
    [switch]$Send,
    [switch]$AllowFire,
    [switch]$SingleShot,
    [ValidateRange(0, 1000)]
    [int]$FireMaxAgeMs = 0,
    [switch]$ProbeHold,
    [switch]$NormalPriority
)

$ErrorActionPreference = 'Stop'
if ($AllowFire -and -not $Send) {
    throw '-AllowFire requires -Send'
}
if ($SingleShot -and -not $AllowFire) {
    throw '-SingleShot requires -AllowFire'
}
if ($FireMaxAgeMs -gt 0 -and -not $AllowFire) {
    throw '-FireMaxAgeMs requires -AllowFire'
}

$RepoRoot = Split-Path -Parent $PSScriptRoot
$BuildRoot = Join-Path $RepoRoot '.deps\build\deploy_dev-win64'
$Executable = Join-Path $BuildRoot "talos_bridge\$Config\talos_sp_bridge.exe"
$OpenCvBin = Join-Path $RepoRoot '.deps\install\opencv\build\x64\vc16\bin'
$OpenVinoBin = 'C:\Users\34741\AppData\Local\Programs\Python\Python311\Lib\site-packages\openvino\libs'
$Python = 'C:\Users\34741\AppData\Local\Programs\Python\Python311\python.exe'
$WebScript = Join-Path $PSScriptRoot 'debug_web_windows.py'
$WebUrl = "http://127.0.0.1:$Port/"

$null = New-Item -ItemType Directory -Path $WebDir -Force

foreach ($Path in @(
    $DeployDir, $ConfigPath, $ShmDir, $ModelPath,
    $Executable, $OpenCvBin, $OpenVinoBin, $Python, $WebScript
)) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Required path does not exist: $Path"
    }
}

$LogDir = Join-Path $WebDir 'logs'
[void](New-Item -ItemType Directory -Path $LogDir -Force)
$WebOut = Join-Path $LogDir 'web_stdout.log'
$WebErr = Join-Path $LogDir 'web_stderr.log'

$env:HNU_DEBUG_WEB_DIR = (Resolve-Path -LiteralPath $WebDir).Path
$env:PATH = "$OpenCvBin;$OpenVinoBin;$env:PATH"
$WebArguments = @(
    $WebScript,
    '--deploy-dir', $DeployDir,
    '--web-dir', $WebDir,
    '--talos-shm-dir', $ShmDir,
    '--port', $Port
)
$WebProcess = Start-Process `
    -FilePath $Python `
    -ArgumentList $WebArguments `
    -WorkingDirectory $RepoRoot `
    -WindowStyle Hidden `
    -RedirectStandardOutput $WebOut `
    -RedirectStandardError $WebErr `
    -PassThru

try {
    $Ready = $false
    for ($Attempt = 0; $Attempt -lt 50; ++$Attempt) {
        if ($WebProcess.HasExited) {
            $Details = Get-Content -LiteralPath $WebErr -Raw -ErrorAction SilentlyContinue
            throw "Web server exited during startup. $Details"
        }
        try {
            $Response = Invoke-WebRequest -Uri $WebUrl -UseBasicParsing -TimeoutSec 1
            if ($Response.StatusCode -eq 200) {
                $Ready = $true
                break
            }
        } catch {
        }
        Start-Sleep -Milliseconds 100
    }
    if (-not $Ready) {
        throw "Web server did not become ready at $WebUrl"
    }

    Write-Host "HNU Vision Debug: $WebUrl"
    Write-Host 'UDP telemetry / PlotJuggler: disabled'
    Write-Host 'Gimbal command output: disabled unless -Send is supplied'

    $Arguments = @(
        $ConfigPath,
        '--shm-dir', $ShmDir,
        '--model', $ModelPath,
        '--web',
        '--web-hz', $WebHz,
        '--no-telemetry'
    )
    if ($CpuCores -gt 0) { $Arguments += @('--cpu-cores', $CpuCores) }
    if (-not $NormalPriority) { $Arguments += '--low-priority' }
    if ($MaxFrames -gt 0) { $Arguments += @('--max-frames', $MaxFrames) }
    if ($Send) { $Arguments += '--send' }
    if ($AllowFire) { $Arguments += '--allow-fire' }
    if ($SingleShot) { $Arguments += '--single-shot' }
    if ($FireMaxAgeMs -gt 0) {
        $Arguments += @('--fire-max-age-ms', $FireMaxAgeMs)
    }
    if ($ProbeHold) { $Arguments += '--probe-hold' }

    & $Executable @Arguments
    $BridgeExitCode = $LASTEXITCODE
    if ($BridgeExitCode -ne 0) {
        throw "talos_sp_bridge failed with exit code $BridgeExitCode"
    }
} finally {
    if ($WebProcess -and -not $WebProcess.HasExited) {
        Stop-Process -Id $WebProcess.Id -ErrorAction SilentlyContinue
        [void]$WebProcess.WaitForExit(3000)
    }
}