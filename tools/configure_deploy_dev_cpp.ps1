[CmdletBinding()]
param(
    [string]$DeployDir = 'D:\HNU_RM\代码栈\deploy_dev',
    [string]$BuildDir = '',
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Config = 'Release',
    [string[]]$BuildTarget = @()
)

$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $RepoRoot '.deps\build\deploy_dev-win64'
}

$OpenCvDir = Join-Path $RepoRoot '.deps\install\opencv\build\x64\vc16\lib'
$EigenDir = Join-Path $RepoRoot '.deps\install\eigen-3.4.1\share\eigen3\cmake'
$YamlDir = Join-Path $RepoRoot '.deps\install\yaml-cpp-0.8.0\lib\cmake\yaml-cpp'
$OpenVinoDir = 'C:\Users\34741\AppData\Local\Programs\Python\Python311\Lib\site-packages\openvino\cmake'
$Bootstrap = Join-Path $RepoRoot 'cmake\deploy_dev_windows_bootstrap.cmake'

$RequiredPaths = @($DeployDir, $OpenCvDir, $EigenDir, $YamlDir, $OpenVinoDir, $Bootstrap)
foreach ($Path in $RequiredPaths) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Required path does not exist: $Path"
    }
}

$ConfigureArgs = @(
    '-S', $DeployDir,
    '-B', $BuildDir,
    '-G', 'Visual Studio 16 2019',
    '-A', 'x64',
    "-DOpenCV_DIR=$OpenCvDir",
    "-DEigen3_DIR=$EigenDir",
    "-Dyaml-cpp_DIR=$YamlDir",
    "-DOpenVINO_DIR=$OpenVinoDir",
    "-DCMAKE_PROJECT_INCLUDE=$Bootstrap",
    '-DCMAKE_CXX_FLAGS=',
    '-DCMAKE_EXE_LINKER_FLAGS=',
    '-DBUILD_APPLICATIONS=ON',
    '-DBUILD_TESTS=ON'
)

& cmake @ConfigureArgs
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed with exit code $LASTEXITCODE"
}

foreach ($Target in $BuildTarget) {
    & cmake --build $BuildDir --config $Config --target $Target -- /m:2 /nr:false
    if ($LASTEXITCODE -ne 0) {
        throw "Build target '$Target' failed with exit code $LASTEXITCODE"
    }
}

Write-Host "deploy_dev C++ build directory: $BuildDir"