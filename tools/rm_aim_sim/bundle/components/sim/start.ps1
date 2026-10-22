[CmdletBinding()]
param(
    [string]$Godot = '',
    [string]$Scenario = '',
    [string]$Channel = '',
    # 无界面：只跑运动/云台/状态，不出图（自瞄链路仍可验）
    [switch]$Headless,
    [switch]$NoWait
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\..\lib\common.ps1')
Clear-ProxyEnv

$config = Get-BundleConfig
if (-not $Channel) { $Channel = $config.channel }
if (-not $Scenario) { $Scenario = Join-Path $PSScriptRoot 'operator.cfg' }

$project = Join-Path $PSScriptRoot 'rm_aim_sim'
if (-not (Test-Path -LiteralPath (Join-Path $project 'project.godot'))) {
    Write-Host "错误：缺少仿真工程 $project" -ForegroundColor Red
    exit 2
}

if (-not $Godot) { $Godot = Resolve-Godot $config }
if (-not $Godot) {
    Write-Host '错误：找不到 Godot。跑一次 third_party\get_godot.ps1，或用 -Godot 指定。' -ForegroundColor Red
    exit 2
}

if (-not (Test-Path -LiteralPath (Join-Path $project '.godot'))) {
    Write-Host '首次运行，导入资源（约 20 秒）...'
    & $Godot --headless --path $project --import | Out-Null
}

$arguments = @('--path', $project)
if ($Headless) { $arguments = @('--headless') + $arguments }
$arguments += @('--', '--remote', "--config=$Scenario")

$runtime = Get-RuntimeDir
$log = Join-Path $runtime 'sim.log'
$err = Join-Path $runtime 'sim.err.log'

Write-Host "启动仿真：$Godot"
Write-Host "  工程 : $project"
Write-Host "  场景 : $Scenario"
Write-Host "  邮箱 : $(Get-InboxDir $Channel)"
$process = Start-Process -FilePath $Godot -ArgumentList $arguments -PassThru `
    -RedirectStandardOutput $log -RedirectStandardError $err
Save-Pid 'sim' $process.Id

if ($NoWait) {
    Write-Host "已启动（PID $($process.Id)），日志 runtime\sim.log"
    exit 0
}

Start-Sleep -Seconds 2
if ($process.HasExited) {
    Write-Host "错误：仿真启动即退出（代码 $($process.ExitCode)）。" -ForegroundColor Red
    Write-Host '可能是同通道已有实例，或 Godot 版本不匹配。最近日志：'
    Get-Content -LiteralPath $err -Tail 15 -ErrorAction SilentlyContinue
    Get-Content -LiteralPath $log -Tail 15 -ErrorAction SilentlyContinue
    exit 3
}

if (-not (Wait-Inbox $Channel 40)) {
    Write-Host '警告：没等到共享邮箱目录，后续组件可能连不上。' -ForegroundColor Yellow
} else {
    Write-Host "就绪（PID $($process.Id)）。窗口最小化后不出图，恢复窗口即可继续。" -ForegroundColor Green
}
