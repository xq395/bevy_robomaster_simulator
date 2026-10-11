[CmdletBinding()]
param(
    [string]$ConfigPath = 'D:\HNU_RM\代码栈\deploy_dev\config\sp_standard.yaml',
    [string]$ShmDir = 'F:\talos_shm',
    [string]$ModelPath = '',
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Config = 'Release',
    [ValidateRange(0, 63)]
    [int]$CpuCores = 8,
    [UInt64]$MaxFrames = 0,
    [switch]$Send,
    [switch]$AllowFire,
    [switch]$ProbeHold,
    [switch]$CaptureOnly,
    [switch]$NormalPriority
)

$ErrorActionPreference = 'Stop'
if ($AllowFire -and -not $Send) {
    throw '-AllowFire requires -Send'
}

$RepoRoot = Split-Path -Parent $PSScriptRoot
$BuildRoot = Join-Path $RepoRoot '.deps\build\deploy_dev-win64'
$Executable = Join-Path $BuildRoot "talos_bridge\$Config\talos_sp_bridge.exe"
$OpenCvBin = Join-Path $RepoRoot '.deps\install\opencv\build\x64\vc16\bin'
$OpenVinoBin = 'C:\Users\34741\AppData\Local\Programs\Python\Python311\Lib\site-packages\openvino\libs'

$RequiredPaths = @($Executable, $ConfigPath, $ShmDir, $OpenCvBin, $OpenVinoBin)
if (-not [string]::IsNullOrWhiteSpace($ModelPath)) {
    $RequiredPaths += $ModelPath
}
foreach ($Path in $RequiredPaths) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Required path does not exist: $Path"
    }
}

$Arguments = @($ConfigPath, '--shm-dir', $ShmDir)
if ($CpuCores -gt 0) { $Arguments += @('--cpu-cores', $CpuCores) }
if (-not $NormalPriority) { $Arguments += '--low-priority' }
if ($MaxFrames -gt 0) { $Arguments += @('--max-frames', $MaxFrames) }
if (-not [string]::IsNullOrWhiteSpace($ModelPath)) {
    $Arguments += @('--model', $ModelPath)
}
if ($CaptureOnly) { $Arguments += '--capture-only' }
if ($Send) { $Arguments += '--send' }
if ($AllowFire) { $Arguments += '--allow-fire' }
if ($ProbeHold) { $Arguments += '--probe-hold' }

$env:PATH = "$OpenCvBin;$OpenVinoBin;$env:PATH"
& $Executable @Arguments
exit $LASTEXITCODE