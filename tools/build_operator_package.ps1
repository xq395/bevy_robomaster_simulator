[CmdletBinding()]
param(
    [string]$OutputDir,
    [string]$RepoRoot = '',
    [string]$OpenCvBin = '',
    [string]$OpenVinoLibs = '',
    [string]$DeployDevDir = 'D:\HNU_RM\代码栈\deploy_dev',
    [string]$BridgeExe = '',
    [string]$ModelPath = '',
    [switch]$SkipBridge
)

$ErrorActionPreference = 'Stop'

$ToolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($RepoRoot)) { $RepoRoot = Split-Path -Parent $ToolsDir }
$PSScriptRoot = $ToolsDir

if ([string]::IsNullOrWhiteSpace($OutputDir)) {
    $OutputDir = Join-Path $RepoRoot 'dist\NHS_Operator_Terminal'
}
if ([string]::IsNullOrWhiteSpace($OpenCvBin)) {
    $OpenCvBin = Join-Path $RepoRoot '.deps\install\opencv\build\x64\vc16\bin'
}
if ([string]::IsNullOrWhiteSpace($OpenVinoLibs)) {
    $OpenVinoLibs = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\Lib\site-packages\openvino\libs'
}
if ([string]::IsNullOrWhiteSpace($BridgeExe)) {
    $BridgeExe = Join-Path $RepoRoot '.deps\build\deploy_dev-win64\talos_bridge\Release\talos_sp_bridge.exe'
}
if ([string]::IsNullOrWhiteSpace($ModelPath)) {
    $ModelPath = Join-Path $RepoRoot '.deps\models\praysky_coord_noe2e_0331_640x640_fp16.xml'
}

function Copy-Tree($source, $destination) {
    New-Item -ItemType Directory -Path $destination -Force | Out-Null
    Copy-Item -Path (Join-Path $source '*') -Destination $destination -Recurse -Force
}

function Report($path) {
    if (-not (Test-Path -LiteralPath $path)) { return 0 }
    (Get-ChildItem -Recurse -File -LiteralPath $path | Measure-Object Length -Sum).Sum
}

Write-Host "打包到: $OutputDir"
if (Test-Path -LiteralPath $OutputDir) { Remove-Item -LiteralPath $OutputDir -Recurse -Force }
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null

# Static package files (launcher, config, docs).
Copy-Tree (Join-Path $PSScriptRoot 'operator_package') $OutputDir

# Operator web app.
$appDir = Join-Path $OutputDir 'app'
New-Item -ItemType Directory -Path $appDir -Force | Out-Null
foreach ($name in 'operator_web.py', 'operator_ui.html', 'narrowband_video.py',
                  'debug_web_windows.py', 'debug_web_windows_stream.js', 'rmu_field.png') {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination $appDir -Force
}

# RMU2026 V2.0.0 protocol (proto, generated pb2, MQTT link, 3334 video, mock engine).
$rmuSrc = Join-Path $PSScriptRoot 'rmu_protocol'
$rmuDst = Join-Path $appDir 'rmu_protocol'
New-Item -ItemType Directory -Path $rmuDst -Force | Out-Null
Get-ChildItem -LiteralPath $rmuSrc -File | Where-Object { $_.Extension -in '.py', '.proto', '.md' } |
    ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $rmuDst -Force }

# Algorithm debug frontend needed by debug_web_windows.py (deploy_dir/debug_web/app.py).
$frontendDir = Join-Path $OutputDir 'web_frontend\debug_web'
New-Item -ItemType Directory -Path $frontendDir -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $DeployDevDir 'debug_web\app.py') -Destination $frontendDir -Force
Copy-Tree (Join-Path $DeployDevDir 'debug_web\static') (Join-Path $frontendDir 'static')
Copy-Tree (Join-Path $DeployDevDir 'debug_web\templates') (Join-Path $frontendDir 'templates')

# Image bridge: executable, runtime libraries, model and config.
$bridgeDir = Join-Path $OutputDir 'bridge'
New-Item -ItemType Directory -Path $bridgeDir -Force | Out-Null
if (-not $SkipBridge) {
    Copy-Item -LiteralPath $BridgeExe -Destination $bridgeDir -Force
    $dllDir = Join-Path $bridgeDir 'dlls'
    New-Item -ItemType Directory -Path $dllDir -Force | Out-Null
    foreach ($dll in 'opencv_world4100.dll', 'opencv_videoio_ffmpeg4100_64.dll', 'opencv_videoio_msmf4100_64.dll') {
        Copy-Item -LiteralPath (Join-Path $OpenCvBin $dll) -Destination $dllDir -Force
    }
    Get-ChildItem -LiteralPath $OpenVinoLibs -Filter *.dll | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $dllDir -Force
    }
    $modelDir = Join-Path $bridgeDir 'models'
    New-Item -ItemType Directory -Path $modelDir -Force | Out-Null
    Copy-Item -LiteralPath $ModelPath -Destination $modelDir -Force
    Copy-Item -LiteralPath ([System.IO.Path]::ChangeExtension($ModelPath, '.bin')) -Destination $modelDir -Force
    $configDir = Join-Path $bridgeDir 'config'
    New-Item -ItemType Directory -Path $configDir -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $DeployDevDir 'config\sp_standard.yaml') -Destination $configDir -Force
}

New-Item -ItemType Directory -Path (Join-Path $OutputDir 'runtime\debug_web') -Force | Out-Null

# Do not ship interpreter caches.
Get-ChildItem -Recurse -Directory -LiteralPath $OutputDir -Filter '__pycache__' |
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

$total = (Get-ChildItem -Recurse -File -LiteralPath $OutputDir | Measure-Object Length -Sum).Sum
Write-Host ''
Write-Host ("完成。文件 {0} 个，总计 {1:N1} MB" -f (Get-ChildItem -Recurse -File -LiteralPath $OutputDir).Count, ($total / 1MB))
Write-Host ("  操作手程序 : {0:N1} MB" -f ((Get-ChildItem -Recurse -File -LiteralPath $appDir | Measure-Object Length -Sum).Sum / 1MB))
Write-Host ("  图像桥接   : {0:N1} MB" -f ((Get-ChildItem -Recurse -File -LiteralPath $bridgeDir | Measure-Object Length -Sum).Sum / 1MB))
Write-Host "  共享目录   : 运行后在 runtime\debug_web 下生成图像共享文件"