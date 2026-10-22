[CmdletBinding()]
param(
    # sim / aim / client / all，可多选：-Component sim,aim
    [string[]]$Component = @('all'),
    [ValidateSet('install', 'check', 'start', 'stop', 'status')]
    [string]$Action = 'start',
    [switch]$Headless,
    [switch]$NoBrowser,
    # 客户端直连官方裁判系统 / 本机 Mock
    [switch]$Rmu,
    [switch]$Mock,
    # 自瞄解算微调：留 0 表示用桥接默认（窗口 0.10 s，附加延迟 0）
    [double]$LeadWindow = 0,
    [double]$ReleaseLatency = -1,
    [string]$Channel = '',
    [string]$Python = '',
    [string]$Godot = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'lib\common.ps1')
Clear-ProxyEnv

$config = Get-BundleConfig
if (-not $Channel) { $Channel = $config.channel }

$order = @('sim', 'aim', 'client')
$selected = @()
foreach ($name in $Component) {
    $lower = $name.ToLower()
    if ($lower -eq 'all') { $selected = $order; break }
    if ($lower -in $order) { if ($lower -notin $selected) { $selected += $lower } }
    else { Write-Host "未知组件：$name（可选 sim / aim / client / all）" -ForegroundColor Red; exit 2 }
}
if (-not $selected) { $selected = $order }

$simDir = Join-Path $PSScriptRoot 'components\sim'
$aimDir = Join-Path $PSScriptRoot 'components\aim'
$clientDir = Join-Path $PSScriptRoot 'components\client'
$projectDir = Join-Path $simDir 'rm_aim_sim'

function Write-Step($text) { Write-Host ''; Write-Host "== $text" -ForegroundColor Cyan }
function Write-Ok($text) { Write-Host "   ok  $text" -ForegroundColor Green }
function Write-Warn2($text) { Write-Host "  warn $text" -ForegroundColor Yellow }
function Write-Err($text) { Write-Host "  fail $text" -ForegroundColor Red }

Write-Host 'RoboMaster 自瞄仿真部署包'
Write-Host "  根目录 : $PSScriptRoot"
Write-Host "  组件   : $($selected -join ' + ')"
Write-Host "  动作   : $Action"

switch ($Action) {

    'install' {
        Write-Step 'Python 依赖'
        $base = Resolve-Python $config $Python
        if (-not $base) {
            Write-Err '找不到 Python 3.10+。装一个 Python 3.11/3.12，或在 config.json 的 python 字段填完整路径。'
            exit 2
        }
        Write-Ok "基础解释器：$base"
        $venv = Join-Path $PSScriptRoot '.venv'
        $venvPython = Join-Path $venv 'Scripts\python.exe'
        if (-not (Test-Path -LiteralPath $venvPython)) {
            Write-Host '  创建 .venv ...'
            & $base -m venv $venv
        }
        if (-not (Test-Path -LiteralPath $venvPython)) { Write-Err '.venv 创建失败'; exit 3 }
        $requirements = Join-Path $clientDir 'requirements.txt'
        Write-Host "  安装 $requirements ..."
        & $venvPython -m pip install --disable-pip-version-check -q -r $requirements
        if ($LASTEXITCODE -ne 0) {
            # 留一个半成品 .venv 会让后面的 Resolve-Python 选中它，反而更糟。
            Write-Warn2 'pip 安装失败（可能没有网）。删掉 .venv，改用系统 Python。'
            Remove-Item -LiteralPath $venv -Recurse -Force -ErrorAction SilentlyContinue
            & $base -c "import PIL" 2>$null
            if ($LASTEXITCODE -ne 0) { Write-Err '系统 Python 也没有 Pillow：请 pip install Pillow' }
        } else {
            Write-Ok '依赖装好了（.venv）'
        }

        Write-Step 'Godot 引擎'
        $godot = Resolve-Godot $config $Godot
        if (-not $godot) {
            Write-Host '  未找到，尝试下载 ...'
            & (Join-Path $PSScriptRoot 'third_party\get_godot.ps1')
            $godot = Resolve-Godot $config $Godot
        }
        if (-not $godot) { Write-Err '没有 Godot。手动放到 third_party\Godot_v4.7.2-stable_win64.exe'; exit 3 }
        Write-Ok "引擎：$godot"

        Write-Step '仿真工程'
        if (-not (Test-Path -LiteralPath (Join-Path $projectDir 'project.godot'))) {
            Write-Err "缺少 $projectDir"; exit 3
        }
        if (-not (Test-Path -LiteralPath (Join-Path $projectDir '.godot'))) {
            Write-Host '  首次导入资源（约 20 秒）...'
            & $godot --headless --path $projectDir --import | Out-Null
        }
        Write-Ok '工程就绪'
        Write-Host ''
        Write-Host '安装完成。下一步：.\deploy.ps1            （一键启动全部）' -ForegroundColor Green
        Write-Host '           .\deploy.ps1 -Action check' -ForegroundColor Green
    }

    'check' {
        $fails = 0
        Write-Step 'Python'
        $python = Resolve-Python $config $Python
        if ($python) { Write-Ok $python } else { Write-Err '找不到 Python'; $fails++ }
        if ($python) {
            $probe = & $python -c "
mods = []
for name in ('PIL', 'flask', 'av', 'paho.mqtt.client', 'google.protobuf'):
    try:
        __import__(name); mods.append(name + '=ok')
    except Exception:
        mods.append(name + '=missing')
print(' '.join(mods))
" 2>&1
            Write-Host "   $probe"
            if ($probe -notmatch 'PIL=ok') { Write-Err 'Pillow 缺失（画面必需）'; $fails++ }
            if ($probe -notmatch 'flask=ok') { Write-Warn2 'flask 缺失：只有 RMU Mock 需要' }
            if ($probe -notmatch 'av=ok') { Write-Warn2 'av 缺失：只有 HEVC 图传解码需要' }
            if ($probe -notmatch 'paho\.mqtt\.client=ok') { Write-Warn2 'paho-mqtt 缺失：只有真裁判系统需要' }
        }

        Write-Step 'Godot 与工程'
        $godot = Resolve-Godot $config $Godot
        if ($godot) {
            $version = (& $godot --version 2>&1 | Select-Object -First 1)
            Write-Ok "$godot  ($version)"
        } else { Write-Err '找不到 Godot'; $fails++ }
        if (Test-Path -LiteralPath (Join-Path $projectDir 'project.godot')) { Write-Ok '仿真工程存在' }
        else { Write-Err '仿真工程缺失'; $fails++ }
        if (Test-Path -LiteralPath (Join-Path $projectDir '.godot')) { Write-Ok '资源已导入' }
        else { Write-Warn2 '资源未导入：首次 start 会自动导入' }

        Write-Step '端口'
        foreach ($pair in @(@($config.bind, $config.video_port, 'TCP', '画面'),
                            @($config.bind, $config.client_port, 'TCP', '页面'))) {
            if (Test-TcpPort $pair[0] $pair[1]) { Write-Warn2 "$($pair[3]) $($pair[0]):$($pair[1]) 已被占用" }
            else { Write-Ok "$($pair[3]) $($pair[0]):$($pair[1]) 空闲" }
        }
        if (Test-UdpPort $config.operator_port) { Write-Warn2 "租约 UDP $($config.operator_port) 已被占用" }
        else { Write-Ok "租约 UDP $($config.operator_port) 空闲" }

        Write-Host ''
        if ($fails -eq 0) { Write-Host '检查通过。' -ForegroundColor Green; exit 0 }
        Write-Host "有 $fails 项必须先修。" -ForegroundColor Red; exit 3
    }

    'start' {
        $started = @()
        if ($selected -contains 'sim') {
            Write-Step '仿真'
            & (Join-Path $simDir 'start.ps1') -Godot $Godot -Channel $Channel -Headless:$Headless
            if ($LASTEXITCODE -and $LASTEXITCODE -ne 0) { Write-Err '仿真没起来，终止。'; exit $LASTEXITCODE }
            $started += 'sim'
        }
        if ($selected -contains 'aim') {
            Write-Step '本机自瞄'
            $noVideo = if ($Headless) { $true } else { $false }
            & (Join-Path $aimDir 'start.ps1') -Channel $Channel -NoVideo:$noVideo `
                -LeadWindow $LeadWindow -ReleaseLatency $ReleaseLatency
            if ($LASTEXITCODE -and $LASTEXITCODE -ne 0) { Write-Err '自瞄桥接没起来，终止。'; exit $LASTEXITCODE }
            $started += 'aim'
        }
        if ($selected -contains 'client') {
            Write-Step '自定义客户端'
            & (Join-Path $clientDir 'start.ps1') -NoBrowser:$NoBrowser -Rmu:$Rmu -Mock:$Mock
            if ($LASTEXITCODE -and $LASTEXITCODE -ne 0) { Write-Err '客户端没起来，终止。'; exit $LASTEXITCODE }
            $started += 'client'
        }

        Write-Host ''
        Write-Host "已启动：$($started -join ' + ')" -ForegroundColor Green
        if ($started -contains 'client' -and -not $NoBrowser) {
            Write-Host "页面：http://$($config.bind):$($config.client_port)/"
        }
        Write-Host '停止：.\deploy.ps1 -Action stop      状态：.\deploy.ps1 -Action status'
    }

    'stop' {
        Write-Step '停止'
        $map = Read-PidMap
        $names = @('client', 'mock', 'aim', 'sim')
        foreach ($name in $names) {
            if (-not $map.Contains($name)) { continue }
            $processId = $map[$name]
            if (Test-Alive $processId) {
                Stop-Process -Id $processId -Force -ErrorAction SilentlyContinue
                Write-Ok "$name (PID $processId) 已停止"
            } else {
                Write-Host "  --  $name (PID $processId) 已不在" -ForegroundColor DarkGray
            }
            Remove-Pid $name
        }
        $left = @()
        if (Test-UdpPort $config.operator_port) { $left += "UDP $($config.operator_port)（可能是残留桥接）" }
        if (Test-TcpPort $config.bind $config.client_port) { $left += "TCP $($config.client_port)" }
        if (Test-TcpPort $config.bind $config.video_port) { $left += "TCP $($config.video_port)" }
        if ($left) { Write-Warn2 ('还在监听：' + ($left -join '、') + '（不在本包记录里，手工关）') }
    }

    'status' {
        $map = Read-PidMap
        Write-Step '进程'
        foreach ($name in @('sim', 'aim', 'client', 'mock')) {
            if (-not $map.Contains($name)) { Write-Host "  --   $name 未启动" -ForegroundColor DarkGray; continue }
            $processId = $map[$name]
            if (Test-Alive $processId) { Write-Ok "$name  PID $processId" }
            else { Write-Warn2 "$name  PID $processId 已退出" }
        }
        Write-Step '端口'
        $rows = @(
            @{ n = '租约 (aim)'; v = "udp $($config.operator_port)"; ok = (Test-UdpPort $config.operator_port) },
            @{ n = '画面 (aim)'; v = "tcp $($config.video_port)"; ok = (Test-TcpPort $config.bind $config.video_port) },
            @{ n = '页面 (client)'; v = "tcp $($config.client_port)"; ok = (Test-TcpPort $config.bind $config.client_port) }
        )
        foreach ($row in $rows) {
            if ($row.ok) { Write-Ok "$($row.n)  $($row.v)" } else { Write-Host "  --   $($row.n)  $($row.v) 未监听" -ForegroundColor DarkGray }
        }
        Write-Step '共享邮箱'
        $inbox = Get-InboxDir $Channel
        if (Test-Path -LiteralPath $inbox) { Write-Ok $inbox } else { Write-Host "  --   $inbox 不存在（仿真没跑）" -ForegroundColor DarkGray }
        Write-Step '最近日志'
        foreach ($name in @('sim', 'aim', 'client')) {
            $log = Join-Path (Get-RuntimeDir) "$name.log"
            if (Test-Path -LiteralPath $log) {
                $last = Get-Content -LiteralPath $log -Tail 3 -ErrorAction SilentlyContinue
                Write-Host "  [$name] $($last -join ' / ')"
            }
        }
    }
}
