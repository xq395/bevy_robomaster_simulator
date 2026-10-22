# 用 RMU 自定义协议把操作终端接到 Godot rm_aim_sim 仿真。
#
# 起三个进程（-WithBridge 时四个）：
#   1. Godot 仿真（rm_aim_sim）        —— 赛场
#   2. rmu_protocol.sim_engine         —— MQTT 3343 + 两路图传，直接读仿真原始帧：
#        官方图传 HEVC → UDP 3344；自定义数据流 0x0310（300 B × 50 Hz）→ MQTT CustomByteBlock
#   3. operator_web.py                 —— 浏览器终端 http://127.0.0.1:5002/
#   (4) rm_aim_sim_bridge.py           —— 只有要同时用模拟器通路（UDP 5001 / HTTP 5000）才需要
#
# 用法：powershell -ExecutionPolicy Bypass -File .\run_sim_rmu.ps1
#       -SimPath/-GodotPath 可覆盖；-NoWeb 不起终端；-Size/-Fps/-Kbps 调官方图传；
#       -CustomSize/-CustomFps/-CustomKbps 调 0x0310（-CustomSize none 关闭）。

param(
    [string]$GodotPath = 'F:\rm_aim_sim_ws\godot\Godot_v4.7.2-stable_win64_console.exe',
    [string]$SimPath = 'F:\rm_aim_sim_ws\rm_aim_sim',
    [string]$Config = (Join-Path $PSScriptRoot '..\rm_aim_sim\bench.cfg'),
    [int]$MqttPort = 3343,
    [int]$VideoPort = 3344,
    [int]$WebPort = 5002,
    [int]$Fps = 30,
    [int]$Kbps = 4000,
    [string]$Size = '960x720',
    [string]$CustomSize = '384x288',
    [int]$CustomFps = 15,
    [int]$CustomKbps = 96,
    [switch]$NoWeb,
    [switch]$SkipSim,
    [switch]$WithBridge
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$tools = Join-Path $root "tools"
$procs = @()

# 仿真窗口必须真的在渲染：最小化/被遮挡时 frame_post_draw 不来，图传会退回测试图。
Add-Type @"
using System; using System.Runtime.InteropServices;
public class SimWindow {
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int n);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int cx, int cy, uint flags);
}
"@ -ErrorAction SilentlyContinue

function Show-SimWindow {
    for ($i = 0; $i -lt 40; $i++) {
        $process = Get-Process Godot_v4.7.2-stable_win64 -ErrorAction SilentlyContinue |
            Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
        if ($process) {
            # SW_RESTORE=9, HWND_TOPMOST=-1, SWP_NOMOVE|SWP_NOSIZE=3
            [SimWindow]::ShowWindow($process.MainWindowHandle, 9) | Out-Null
            [SimWindow]::SetWindowPos($process.MainWindowHandle, [IntPtr](-1), 0, 0, 0, 0, 3) | Out-Null
            return
        }
        Start-Sleep -Milliseconds 250
    }
}

function Start-Child([string]$file, [string[]]$arguments, [string]$workdir, [switch]$Visible) {
    $style = if ($Visible) { 'Normal' } else { 'Hidden' }
    $p = Start-Process -FilePath $file -ArgumentList $arguments -WorkingDirectory $workdir -PassThru -WindowStyle $style
    $script:procs += $p
    return $p
}

Write-Host "[1/3] Godot 仿真…" -ForegroundColor Cyan
if ($SkipSim) {
    Write-Host "      已有仿真在跑（-SkipSim），跳过启动" -ForegroundColor DarkGray
    Show-SimWindow
} else {
    Start-Child $GodotPath @('--path', $SimPath, '--', '--remote', "--config=$Config", '--quit-after-seconds=7200') $SimPath -Visible | Out-Null
    Start-Sleep -Seconds 4
    Show-SimWindow
    Start-Sleep -Seconds 3
}

if ($WithBridge) {
    Write-Host "[+]   模拟器通路桥接（UDP 5001 / HTTP 5000）…" -ForegroundColor Cyan
    Start-Child 'python' @('tools\rm_aim_sim_bridge.py', '--backend', 'truth') $root | Out-Null
    Start-Sleep -Seconds 3
}

Write-Host "[2/3] RMU SIM 引擎（MQTT $MqttPort，图传 UDP $VideoPort，0x0310 $CustomSize）…" -ForegroundColor Cyan
$env:PYTHONPATH = Join-Path $root 'tools'
$env:PYTHONIOENCODING = 'utf-8'
Start-Child 'python' @('-m', 'rmu_protocol.sim_engine', '--mqtt-port', "$MqttPort",
    '--video-to', "127.0.0.1:$VideoPort", '--size', $Size, '--fps', "$Fps", '--kbps', "$Kbps",
    '--custom-video', $CustomSize, '--custom-fps', "$CustomFps", '--custom-kbps', "$CustomKbps") $tools | Out-Null
Start-Sleep -Seconds 2

if (-not $NoWeb) {
    Write-Host "[3/3] 操作终端 http://127.0.0.1:$WebPort/ …" -ForegroundColor Cyan
    $env:RMU_ENABLED = '1'
    $env:RMU_MQTT_HOST = '127.0.0.1'
    $env:RMU_MQTT_PORT = "$MqttPort"
    $env:RMU_VIDEO_HOST = '127.0.0.1'
    $env:RMU_VIDEO_PORT = "$VideoPort"
    $env:RMU_ROBOT_ID = '3'
    $env:DAEDALUS_OPERATOR_PORT = "$WebPort"
    $env:OPERATOR_WEB_URL = "http://127.0.0.1:$WebPort/"
    Start-Child 'python' @('operator_web.py') $tools | Out-Null
}

Write-Host ""
Write-Host "已启动 $($script:procs.Count) 个进程（PID: $($script:procs.Id -join ', ')）" -ForegroundColor Green
Write-Host "终端地址 http://127.0.0.1:$WebPort/  ·  关窗口后按 Ctrl+C 收尾" -ForegroundColor Green
Write-Host "注意：仿真窗口不能最小化（最小化不渲染 → 没有图传；引擎会尝试自动恢复）。" -ForegroundColor Yellow
try {
    Wait-Process -Id $script:procs.Id
} finally {
    foreach ($p in $script:procs) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
}




