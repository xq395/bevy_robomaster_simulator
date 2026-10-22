[CmdletBinding()]
param(
    # 自瞄/仿真那一侧的本机接口
    [string]$SimHost = '',
    [int]$SimPort = 0,
    [string]$VideoHost = '',
    [int]$VideoPort = 0,
    # 本页面监听端口
    [int]$ListenPort = 0,
    # 接官方裁判系统（MQTT 3333 + 图传 UDP 3334）
    [switch]$Rmu,
    [string]$RmuHost = '192.168.12.1',
    [int]$RmuPort = 3333,
    [int]$RobotId = 3,
    # 本机 Mock 裁判系统（不需要真机/引擎）
    [switch]$Mock,
    [switch]$NoBrowser,
    [switch]$NoWait
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\..\lib\common.ps1')
Clear-ProxyEnv

$config = Get-BundleConfig
if (-not $SimHost) { $SimHost = $config.bind }
if (-not $VideoHost) { $VideoHost = $config.bind }
if (-not $SimPort) { $SimPort = $config.operator_port }
if (-not $VideoPort) { $VideoPort = $config.video_port }
if (-not $ListenPort) { $ListenPort = $config.client_port }

$python = Resolve-Python $config
if (-not $python) {
    Write-Host '错误：找不到 Python。用 deploy.ps1 -Action install。' -ForegroundColor Red
    exit 2
}

$app = Join-Path $PSScriptRoot 'app'
if (-not (Test-Path -LiteralPath (Join-Path $app 'operator_web.py'))) {
    Write-Host "错误：缺少客户端程序 $app" -ForegroundColor Red
    exit 2
}

$runtime = Get-RuntimeDir
$log = Join-Path $runtime 'client.log'
$err = Join-Path $runtime 'client.err.log'

if ($Mock) {
    $Rmu = $true
    $RmuHost = '127.0.0.1'
    $RmuPort = 3343
    $mockLog = Join-Path $runtime 'mock.log'
    $mockErr = Join-Path $runtime 'mock.err.log'
    $mockArgs = @('-m', 'rmu_protocol.mock_engine', '--robot-id', $RobotId,
        '--mqtt-host', '127.0.0.1', '--mqtt-port', $RmuPort,
        '--video-to', "127.0.0.1:$VideoPort")
    Write-Host '启动本机 Mock 裁判系统...'
    $mock = Start-Process -FilePath $python -ArgumentList $mockArgs -WorkingDirectory $app -PassThru `
        -WindowStyle Hidden -RedirectStandardOutput $mockLog -RedirectStandardError $mockErr
    Save-Pid 'mock' $mock.Id
    Start-Sleep -Seconds 1
}

$env:DAEDALUS_SIM_HOST = $SimHost
$env:DAEDALUS_SIM_PORT = "$SimPort"
$env:DAEDALUS_VIDEO_HOST = $VideoHost
$env:DAEDALUS_VIDEO_PORT = "$VideoPort"
$env:DAEDALUS_OPERATOR_HOST = $config.bind
$env:DAEDALUS_OPERATOR_PORT = "$ListenPort"
if ($Rmu) {
    $env:RMU_ENABLED = '1'
    $env:RMU_MQTT_HOST = $RmuHost
    $env:RMU_MQTT_PORT = "$RmuPort"
    $env:RMU_ROBOT_ID = "$RobotId"
    $env:RMU_VIDEO_HOST = '0.0.0.0'
    $env:RMU_VIDEO_PORT = "$VideoPort"
} else {
    $env:RMU_ENABLED = '0'
}

Write-Host '启动自定义客户端...'
Write-Host "  模拟器接口 : udp://$SimHost`:$SimPort  画面 http://$VideoHost`:$VideoPort"
if ($Rmu) { Write-Host "  裁判系统   : MQTT $RmuHost`:$RmuPort clientID=$RobotId" }
$process = Start-Process -FilePath $python -ArgumentList @("`"$(Join-Path $app 'operator_web.py')`"") `
    -WorkingDirectory $app -PassThru -WindowStyle Hidden `
    -RedirectStandardOutput $log -RedirectStandardError $err
Save-Pid 'client' $process.Id

if ($NoWait) {
    Write-Host "已启动（PID $($process.Id)），日志 runtime\client.log"
    exit 0
}

if (-not (Wait-TcpPort $config.bind $ListenPort 20)) {
    Write-Host '错误：页面端口没起来。最近日志：' -ForegroundColor Red
    Get-Content -LiteralPath $err -Tail 20 -ErrorAction SilentlyContinue
    Get-Content -LiteralPath $log -Tail 20 -ErrorAction SilentlyContinue
    exit 3
}

Write-Host "就绪（PID $($process.Id)）：http://$($config.bind):$ListenPort/" -ForegroundColor Green
if (-not $NoBrowser) { Start-Process "http://$($config.bind):$ListenPort/" }
