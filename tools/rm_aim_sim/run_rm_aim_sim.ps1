[CmdletBinding()]
param(
    # rm_aim_sim 的 Godot 可执行文件；留空则依次看 $env:GODOT_BIN、项目里的 .tools/godot、PATH
    [string]$Godot = '',
    # rm_aim_sim 项目目录；留空则看 $env:RM_AIM_SIM_DIR，再退回本机默认克隆位置
    [string]$Project = '',
    [string]$Python = '',
    [string]$Channel = 'default',
    # 无界面仿真：能跑控制/状态，但没有图传
    [switch]$Headless,
    [switch]$NoSim,
    [switch]$NoTerminal,
    [switch]$Check
)

$ErrorActionPreference = 'Stop'
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Tools = Split-Path -Parent $Here
$Bridge = Join-Path $Tools 'rm_aim_sim_bridge.py'
$Web = Join-Path $Tools 'operator_web.py'
$Scenario = Join-Path $Here 'operator.cfg'

function Info($m) { Write-Host $m }
function Fail($m) { Write-Host "错误：$m" -ForegroundColor Red; exit 2 }

# Windows PowerShell 5.1 的环境变量驱动器不区分大小写：同时存在 HTTPS_PROXY 和
# https_proxy（Clash / 公司代理常见）时，Env: 驱动器会直接抛“已添加相同键”，
# Start-Process 随之失败。这些子进程都不走代理，直接清掉重复项。
foreach ($name in @('http_proxy', 'https_proxy', 'no_proxy', 'all_proxy',
        'HTTP_PROXY', 'HTTPS_PROXY', 'NO_PROXY', 'ALL_PROXY')) {
    try { [Environment]::SetEnvironmentVariable($name, $null, 'Process') } catch { }
}

if (-not $Project) { $Project = $env:RM_AIM_SIM_DIR }
if (-not $Project) {
    $candidates = @('F:\rm_aim_sim_ws\rm_aim_sim', (Join-Path $env:USERPROFILE 'rm_aim_sim'))
    $Project = $candidates | Where-Object { Test-Path -LiteralPath (Join-Path $_ 'project.godot') } |
        Select-Object -First 1
}
if (-not $Project -or -not (Test-Path -LiteralPath (Join-Path $Project 'project.godot'))) {
    Fail '找不到 rm_aim_sim 项目（需要 project.godot）。用 -Project 指定，或先 git clone。'
}

if (-not $Godot) { $Godot = $env:GODOT_BIN }
if (-not $Godot -or -not (Test-Path -LiteralPath $Godot)) {
    $found = @(
        (Join-Path $Project '.tools\godot\Godot_v4.7.2-stable_win64.exe'),
        (Join-Path $Project '.tools\godot\godot.exe')
    ) | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    if (-not $found) {
        $found = Get-ChildItem -LiteralPath 'F:\rm_aim_sim_ws\godot' -Filter 'Godot_*win64.exe' -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -notlike '*console*' } | Select-Object -First 1 -ExpandProperty FullName
    }
    if (-not $found) {
        $cmd = Get-Command godot4, godot -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($cmd) { $found = $cmd.Source }
    }
    $Godot = $found
}
if (-not $Godot -or -not (Test-Path -LiteralPath $Godot)) {
    Fail '找不到 Godot 4.x 可执行文件。用 -Godot 指定，或设 $env:GODOT_BIN。'
}

if (-not $Python) {
    $Python = (Get-Command python -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Source)
}
if (-not $Python -or -not (Test-Path -LiteralPath $Python)) { Fail '找不到 python。用 -Python 指定。' }

$version = & $Godot --version 2>&1 | Select-Object -First 1
$projectLines = Get-Content -LiteralPath (Join-Path $Project 'project.godot') -Raw
if ($projectLines -notmatch 'config_version=5') { Write-Host '警告：project.godot 不是 Godot 4 工程。' -ForegroundColor Yellow }

Info 'rm_aim_sim × 操作终端'
Info "  仿真项目 : $Project"
Info "  Godot    : $Godot"
Info "  Python   : $Python"
Info "  场景配置 : $Scenario"
Info "  IPC 通道 : $Channel"
Info ''

if ($Check) {
    & $Python -c "import PIL, json; print('Pillow ok')"
    if ($LASTEXITCODE -ne 0) { Fail '缺少 Pillow（画面需要）。pip install Pillow' }
    Info '依赖检查通过（未启动）。'
    exit 0
}

$ipcDir = Join-Path $env:TEMP ("rm_aim_sim_" + $Channel)   # Linux 上是 /dev/shm
$env:DAEDALUS_SIM_HOST = '127.0.0.1'
$env:DAEDALUS_SIM_PORT = '5001'
$env:DAEDALUS_VIDEO_HOST = '127.0.0.1'
$env:DAEDALUS_VIDEO_PORT = '5000'
$env:DAEDALUS_OPERATOR_PORT = '5002'
$env:RMU_ENABLED = '0'

$sim = $null
if (-not $NoSim) {
    $arguments = @('--path', $Project)
    if ($Headless) { $arguments = @('--headless') + $arguments }
    $arguments += @('--', '--remote', "--config=$Scenario")
    Info '启动仿真（外部控制模式）...'
    $sim = Start-Process -FilePath $Godot -ArgumentList $arguments -PassThru
    for ($i = 0; $i -lt 60; $i++) {
        if (Test-Path -LiteralPath $ipcDir) { break }
        Start-Sleep -Milliseconds 500
    }
    if (-not (Test-Path -LiteralPath $ipcDir)) { Write-Host '警告：没等到 IPC 目录，继续启动桥接（会重试连接）。' -ForegroundColor Yellow }
}

Info '启动桥接...'
$bridgeArgs = @("`"$Bridge`"", '--channel', $Channel)
if ($Headless) { $bridgeArgs += '--no-video' }
$bridgeProc = Start-Process -FilePath $Python -ArgumentList $bridgeArgs -PassThru -WindowStyle Minimized
Start-Sleep -Milliseconds 800

if (-not $NoTerminal) {
    Info '启动操作终端...'
    $webProc = Start-Process -FilePath $Python -ArgumentList @("`"$Web`"") -PassThru -WindowStyle Minimized
    Start-Sleep -Seconds 1
    Start-Process 'http://127.0.0.1:5002/'
}

Info ''
Info '  终端页面 : http://127.0.0.1:5002/'
Info '  仿真画面 : http://127.0.0.1:5000/snapshot/raw'
Info '  状态回报 : udp://127.0.0.1:5001'
Info ''
Info '注意：仿真窗口最小化后不渲染，画面会停；恢复窗口即可继续。'
Info '按 Ctrl+C 结束本脚本不会关子进程，需要手工关闭 Godot / 两个控制台窗口。'
