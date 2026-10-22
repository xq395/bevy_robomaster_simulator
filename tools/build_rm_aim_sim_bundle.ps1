[CmdletBinding()]
param(
    # 不带 180 MB 的 Godot 引擎（对方跑 third_party\get_godot.ps1 自取）
    [switch]$NoEngine,
    # 只生成目录，不压 zip
    [switch]$NoZip,
    [string]$RmAimSim = 'F:\rm_aim_sim_ws\rm_aim_sim',
    [string]$GodotExe = 'F:\rm_aim_sim_ws\godot\Godot_v4.7.2-stable_win64.exe'
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$template = Join-Path $PSScriptRoot 'rm_aim_sim\bundle'
$out = Join-Path $repo 'dist\rm_aim_sim_bundle'

function Step($text) { Write-Host ''; Write-Host "== $text" -ForegroundColor Cyan }
function Copy-Tree($source, $destination) {
    if (-not (Test-Path -LiteralPath $destination)) { New-Item -ItemType Directory -Path $destination -Force | Out-Null }
    Copy-Item -Path (Join-Path $source '*') -Destination $destination -Recurse -Force
}

Step "输出目录 $out"
if (Test-Path -LiteralPath $out) {
    Remove-Item -LiteralPath $out -Recurse -Force -ErrorAction SilentlyContinue
}
New-Item -ItemType Directory -Path $out -Force | Out-Null

Step '模板（脚本、配置、文档）'
Copy-Tree $template $out
Copy-Item -LiteralPath (Join-Path $repo 'docs\rm_aim_sim_integration.md') `
    -Destination (Join-Path $out 'docs\INTEGRATION.md') -Force

Step '客户端'
$appDir = Join-Path $out 'components\client\app'
New-Item -ItemType Directory -Path $appDir -Force | Out-Null
foreach ($name in 'operator_web.py', 'operator_ui.html', 'narrowband_video.py') {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination $appDir -Force
}
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'operator_package\requirements.txt') `
    -Destination (Join-Path $out 'components\client\requirements.txt') -Force
$rmuDst = Join-Path $appDir 'rmu_protocol'
New-Item -ItemType Directory -Path $rmuDst -Force | Out-Null
Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'rmu_protocol') -File |
    Where-Object { $_.Extension -in '.py', '.proto', '.md' } |
    ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $rmuDst -Force }
Write-Host ("   {0} 个文件" -f (Get-ChildItem -LiteralPath $appDir -Recurse -File | Measure-Object).Count)

Step '本机自瞄'
$aimDir = Join-Path $out 'components\aim'
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'rm_aim_sim_bridge.py') -Destination $aimDir -Force
# 基准/归因脚本一起带上，队友能自己复核命中率
foreach ($name in 'bench_hit_rate.py', 'diag_lead.py', 'attribute_misses.py',
                  'bench.cfg', 'bench_hard.cfg', 'sweep.sh') {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot "rm_aim_sim\$name") -Destination $aimDir -Force
}

Step '仿真工程'
$simDst = Join-Path $out 'components\sim\rm_aim_sim'
New-Item -ItemType Directory -Path $simDst -Force | Out-Null
if (-not (Test-Path -LiteralPath (Join-Path $RmAimSim 'project.godot'))) {
    throw "找不到 rm_aim_sim 工程：$RmAimSim（用 -RmAimSim 指定）"
}
# 不带 .git 与 .godot：前者是上游历史，后者是导入缓存，首次运行会自动重建
& robocopy $RmAimSim $simDst /E /XD .git .godot /NFL /NDL /NJH /NJS /NP | Out-Null
if ($LASTEXITCODE -ge 8) { throw "robocopy 失败（$LASTEXITCODE）" }
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'rm_aim_sim\operator.cfg') `
    -Destination (Join-Path $out 'components\sim') -Force
$patchDir = Join-Path $out 'components\sim\patches'
New-Item -ItemType Directory -Path $patchDir -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'rm_aim_sim\0001-local-ipc-cross-platform.patch') `
    -Destination $patchDir -Force

Step 'Godot 引擎'
$engineDst = Join-Path $out 'third_party'
if ($NoEngine) {
    Write-Host '   -NoEngine：跳过（部署时跑 third_party\get_godot.ps1）'
} elseif (Test-Path -LiteralPath $GodotExe) {
    Copy-Item -LiteralPath $GodotExe -Destination $engineDst -Force
    Write-Host '   已放入 Godot 4.7.2'
} else {
    Write-Host "   警告：没找到 $GodotExe，包内不带引擎。" -ForegroundColor Yellow
}

Step '统计'
$files = Get-ChildItem -LiteralPath $out -Recurse -File
$size = ($files | Measure-Object Length -Sum).Sum / 1MB
Write-Host ("   {0} 个文件，{1:N1} MB" -f $files.Count, $size)

if ($NoZip) {
    Write-Host ''
    Write-Host "目录已生成：$out" -ForegroundColor Green
    exit 0
}

Step '压缩'
$stamp = Get-Date -Format 'yyyyMMdd'
$zip = Join-Path $repo ("dist\rm_aim_sim_bundle_{0}.zip" -f $stamp)
if (Test-Path -LiteralPath $zip) { Remove-Item -LiteralPath $zip -Force -ErrorAction SilentlyContinue }
# tar.exe 自带多线程式流拷，比 Compress-Archive 快一个量级。
# .venv / runtime 是本机产物（依赖、日志、PID），不进包。
& tar -a -c -f $zip --exclude=.venv --exclude=runtime -C (Split-Path -Parent $out) (Split-Path -Leaf $out)
if ($LASTEXITCODE -ne 0) {
    Write-Host '   tar 不可用，回退 Compress-Archive（慢）'
    Compress-Archive -Path (Join-Path $out '*') -DestinationPath $zip -Force
}
$zipSize = (Get-Item -LiteralPath $zip).Length / 1MB
Write-Host ''
Write-Host ("完成：{0}（{1:N1} MB）" -f $zip, $zipSize) -ForegroundColor Green
Write-Host ("目录：{0}" -f $out)
