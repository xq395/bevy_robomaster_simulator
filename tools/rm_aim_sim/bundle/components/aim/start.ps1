[CmdletBinding()]
param(
    [string]$Channel = '',
    [int]$OperatorPort = 0,
    [int]$VideoPort = 0,
    [string]$Bind = '',
    [double]$ToleranceDeg = 0,
    # 测速窗口（秒）；留空用桥接默认 0.10
    [double]$LeadWindow = 0,
    # 额外时延补偿（秒）；实测 0 最好，加正值会让 4 rad/s 的靶更差
    [double]$ReleaseLatency = -1,
    # 不要画面（无界面仿真、或只想验控制链路时）
    [switch]$NoVideo,
    [switch]$NoWait
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\..\lib\common.ps1')
Clear-ProxyEnv

$config = Get-BundleConfig
if (-not $Channel) { $Channel = $config.channel }
if (-not $OperatorPort) { $OperatorPort = $config.operator_port }
if (-not $VideoPort) { $VideoPort = $config.video_port }
if (-not $Bind) { $Bind = $config.bind }
if (-not $ToleranceDeg) { $ToleranceDeg = $config.tolerance_deg }

$python = Resolve-Python $config
if (-not $python) {
    Write-Host '错误：找不到 Python。用 deploy.ps1 -Action install，或在 config.json 里填 python。' -ForegroundColor Red
    exit 2
}

$bridge = Join-Path $PSScriptRoot 'rm_aim_sim_bridge.py'
$runtime = Get-RuntimeDir
$log = Join-Path $runtime 'aim.log'
$err = Join-Path $runtime 'aim.err.log'

$arguments = @("`"$bridge`"", '--channel', $Channel,
    '--operator-port', $OperatorPort, '--video-port', $VideoPort,
    '--bind', $Bind, '--tolerance-deg', $ToleranceDeg)
if ($LeadWindow -gt 0) { $arguments += @('--lead-window', $LeadWindow) }
if ($ReleaseLatency -ge 0) { $arguments += @('--release-latency', $ReleaseLatency) }
if ($NoVideo) { $arguments += '--no-video' }

Write-Host "启动本机自瞄：$python"
Write-Host "  邮箱 : $(Get-InboxDir $Channel)"
Write-Host "  租约 : udp://$Bind`:$OperatorPort   画面 : http://$Bind`:$VideoPort/snapshot/raw"
$process = Start-Process -FilePath $python -ArgumentList $arguments -PassThru `
    -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError $err
Save-Pid 'aim' $process.Id

if ($NoWait) {
    Write-Host "已启动（PID $($process.Id)），日志 runtime\aim.log"
    exit 0
}

if (-not (Wait-UdpPort $OperatorPort 25)) {
    Write-Host '错误：租约端口没起来。最近日志：' -ForegroundColor Red
    Get-Content -LiteralPath $err -Tail 20 -ErrorAction SilentlyContinue
    Get-Content -LiteralPath $log -Tail 20 -ErrorAction SilentlyContinue
    exit 3
}
if (-not $NoVideo -and -not (Wait-TcpPort $Bind $VideoPort 20)) {
    Write-Host '错误：画面端口没起来。' -ForegroundColor Red
    Get-Content -LiteralPath $err -Tail 20 -ErrorAction SilentlyContinue
    exit 3
}
Write-Host "就绪（PID $($process.Id)）。" -ForegroundColor Green
