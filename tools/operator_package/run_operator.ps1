[CmdletBinding()]
param(
    [switch]$Check,
    [switch]$NoBridge,
    # sim = Daedalus 模拟器；rmu = 官方裁判系统（MQTT 3333 + 图传 3334）；rmu-mock = 本机 Mock 裁判系统
    [ValidateSet('', 'sim', 'rmu', 'rmu-mock')]
    [string]$Mode = ''
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path

function Info($message) { Write-Host $message }
function Fail($message) { Write-Host "错误：$message" -ForegroundColor Red }
function Quote($value) { '"' + $value + '"' }

$configPath = Join-Path $Root 'operator_config.json'
if (-not (Test-Path -LiteralPath $configPath)) { Fail '缺少 operator_config.json'; exit 2 }
$Config = Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
if ([string]::IsNullOrWhiteSpace($Mode)) { $Mode = $Config.mode }
if ([string]::IsNullOrWhiteSpace($Mode)) { $Mode = 'sim' }
if ($Mode -notin 'sim', 'rmu', 'rmu-mock') { Write-Host "错误：未知 mode '$Mode'（可选 sim / rmu / rmu-mock）" -ForegroundColor Red; exit 2 }
$IsSim = $Mode -eq 'sim'

# Python: explicit config wins, then PATH, then the common per-user install.
$Python = $Config.python
if ([string]::IsNullOrWhiteSpace($Python)) {
    $Python = @(
        (Get-Command python -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty Source),
        (Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe')
    ) | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
}
if ([string]::IsNullOrWhiteSpace($Python) -or -not (Test-Path -LiteralPath $Python)) {
    Fail '未找到 Python，请在 operator_config.json 的 python 字段填写完整路径'
    exit 2
}

# The simulator and the bridge must publish/read the same folder. Env wins so a
# shared launch script can set it for both sides.
$ShmDir = $env:TALOS_SHM_DIR
if ([string]::IsNullOrWhiteSpace($ShmDir)) { $ShmDir = $Config.shm_dir }
if ([string]::IsNullOrWhiteSpace($ShmDir)) { $ShmDir = Join-Path $env:TEMP 'talos_shm' }
$env:TALOS_SHM_DIR = $ShmDir

$DebugWebDir = Join-Path $Root $Config.debug_web_dir
$env:HNU_DEBUG_WEB_DIR = $DebugWebDir

$BridgeExe = Join-Path $Root 'bridge\talos_sp_bridge.exe'
$BridgeConfig = Join-Path $Root 'bridge\config\sp_standard.yaml'
$BridgeModel = Join-Path $Root 'bridge\models\praysky_coord_noe2e_0331_640x640_fp16.xml'
$BridgeDlls = Join-Path $Root 'bridge\dlls'
$AppDir = Join-Path $Root 'app'
$WebFrontend = Join-Path $Root 'web_frontend'

$required = @(
    $AppDir,
    (Join-Path $AppDir 'operator_web.py'),
    (Join-Path $AppDir 'operator_ui.html'),
    (Join-Path $AppDir 'narrowband_video.py'),
    (Join-Path $AppDir 'rmu_protocol\rm_custom_client_pb2.py'),
    (Join-Path $AppDir 'rmu_protocol\client_link.py')
)
if ($IsSim) {
    $required += $WebFrontend, (Join-Path $AppDir 'debug_web_windows.py'), (Join-Path $WebFrontend 'debug_web\app.py')
    if ($Config.start_bridge -and -not $NoBridge) { $required += $BridgeExe, $BridgeConfig, $BridgeModel }
}
$missing = $required | Where-Object { -not (Test-Path -LiteralPath $_) }
if ($missing) {
    Fail '以下文件缺失：'
    $missing | ForEach-Object { Write-Host "  $_" }
    exit 2
}

$Rmu = $Config.rmu
$RmuMqttHost = if ($Mode -eq 'rmu-mock') { '127.0.0.1' } else { [string]$Rmu.mqtt_host }
$RmuVideoHost = if ($Mode -eq 'rmu-mock') { '127.0.0.1' } else { [string]$Rmu.video_host }

Info 'NHS 操作手终端'
Info "  模式     : $Mode"
Info "  包目录   : $Root"
Info "  Python   : $Python"
if ($IsSim) {
    Info "  共享目录 : $ShmDir"
    Info "  图像目录 : $DebugWebDir"
} else {
    Info "  裁判系统 : MQTT $RmuMqttHost`:$($Rmu.mqtt_port)  clientID=$($Rmu.robot_id)"
    Info "  图传     : UDP $RmuVideoHost`:$($Rmu.video_port)"
}
Info ''

$moduleCheck = & $Python -c "import flask, PIL, av, paho.mqtt.client, google.protobuf; import sys; sys.path.insert(0, r'$AppDir'); import rmu_protocol.rm_custom_client_pb2; print('ok')" 2>&1
if ($LASTEXITCODE -ne 0 -or ($moduleCheck -join ' ') -notmatch 'ok') {
    Fail '缺少 Python 依赖（或 protobuf 版本过低，需 6.32.1）。请先运行：'
    $moduleCheck | Select-Object -Last 3 | ForEach-Object { Write-Host "  $_" }
    Write-Host "  $(Quote $Python) -m pip install -r $(Quote (Join-Path $Root 'requirements.txt'))"
    exit 3
}

if ($Check) {
    Info '依赖检查通过（未启动服务）。'
    exit 0
}

# The bridge reads the simulator heartbeat; wait briefly so a cold start is not a hard failure.
$imagePool = Join-Path $ShmDir 'talos_ipc_image_pool'
if ($IsSim -and -not (Test-Path -LiteralPath $imagePool)) {
    Info "等待模拟器发布图像（$ShmDir）..."
    for ($i = 0; $i -lt 30; $i++) {
        if (Test-Path -LiteralPath $imagePool) { break }
        Start-Sleep -Seconds 1
    }
    if (-not (Test-Path -LiteralPath $imagePool)) {
        Info '未检测到模拟器图像。请确认模拟器已启动，且其 TALOS_SHM_DIR 与本包一致。'
    }
}

New-Item -ItemType Directory -Path $DebugWebDir -Force | Out-Null

$env:PATH = "$BridgeDlls;$env:PATH"
$env:DAEDALUS_VIDEO_SOURCE = "http://$($Config.video_host):$($Config.video_port)/snapshot/raw"
$env:DAEDALUS_SIM_HOST = $Config.sim_udp_host
$env:DAEDALUS_SIM_PORT = [string]$Config.sim_udp_port
$env:DAEDALUS_VIDEO_HOST = $Config.video_host
$env:DAEDALUS_VIDEO_PORT = [string]$Config.video_port
$env:DAEDALUS_OPERATOR_PORT = [string]$Config.operator_port
if ($IsSim) {
    $env:RMU_ENABLED = '0'
} else {
    $env:RMU_ENABLED = '1'
    $env:RMU_MQTT_HOST = $RmuMqttHost
    $env:RMU_MQTT_PORT = [string]$Rmu.mqtt_port
    $env:RMU_ROBOT_ID = [string]$Rmu.robot_id
    $env:RMU_VIDEO_HOST = $RmuVideoHost
    $env:RMU_VIDEO_PORT = [string]$Rmu.video_port
}

$started = @()
try {
    if ($Mode -eq 'rmu-mock') {
        # 本机 Mock 裁判系统：MQTT broker + 模拟数据 + HEVC 测试图传，仅监听 127.0.0.1。
        $mockArgs = "-m rmu_protocol.mock_engine --robot-id $($Rmu.robot_id) --mqtt-host 127.0.0.1" +
            " --mqtt-port $($Rmu.mqtt_port) --video-to 127.0.0.1:$($Rmu.video_port)"
        $mock = Start-Process -FilePath $Python -ArgumentList $mockArgs -WorkingDirectory $AppDir -PassThru
        $started += $mock
        Info "Mock 裁判系统已启动 (PID $($mock.Id))"
        Start-Sleep -Seconds 1
    }

    if (-not $IsSim) {
        Info '裁判系统模式：不启动图像桥接与算法调试页。'
    } elseif ($Config.start_bridge -and -not $NoBridge) {
        $bridgeArgs = "$(Quote $BridgeConfig) --shm-dir $(Quote $ShmDir) --model $(Quote $BridgeModel)" +
            " --web --web-hz $($Config.bridge_web_hz) --no-telemetry --low-priority"
        if ($Config.send) { $bridgeArgs += ' --send' }
        if ($Config.allow_fire) { $bridgeArgs += ' --allow-fire' }
        $bridge = Start-Process -FilePath $BridgeExe -ArgumentList $bridgeArgs -WorkingDirectory $Root -PassThru
        $started += $bridge
        Info "图像桥接已启动 (PID $($bridge.Id))"
    } else {
        Info '已跳过图像桥接（无实时画面）。'
    }

    if ($IsSim) {
        $webArgs = "$(Quote (Join-Path $AppDir 'debug_web_windows.py')) --deploy-dir $(Quote $WebFrontend)" +
            " --web-dir $(Quote $DebugWebDir) --talos-shm-dir $(Quote $ShmDir) --port $($Config.video_port)"
        $web = Start-Process -FilePath $Python -ArgumentList $webArgs -WorkingDirectory $Root -PassThru
        $started += $web
        Info "图像服务已启动 (PID $($web.Id))"
    }

    $operator = Start-Process -FilePath $Python -ArgumentList (Quote (Join-Path $AppDir 'operator_web.py')) -WorkingDirectory $AppDir -PassThru
    $started += $operator
    Info "操作手服务已启动 (PID $($operator.Id))"

    Info ''
    Info "操作手页面： http://127.0.0.1:$($Config.operator_port)/"
    if ($IsSim) { Info "算法调试页： http://127.0.0.1:$($Config.video_port)/" }
    else { Info '页面“视频源”选择“裁判系统图传”，右侧为裁判系统面板。' }
    Info '按 Ctrl+C 结束全部服务。'

    while ($true) {
        foreach ($process in $started) {
            if ($process.HasExited) { throw "进程 $($process.Id) 已退出（代码 $($process.ExitCode)）" }
        }
        Start-Sleep -Seconds 2
    }
} catch {
    Fail $_.Exception.Message
} finally {
    foreach ($process in $started) {
        if ($process -and -not $process.HasExited) {
            Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
        }
    }
    Info '服务已停止。'
}