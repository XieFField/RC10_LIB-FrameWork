$ErrorActionPreference = 'Stop'

$testsRoot = Split-Path -Parent $PSScriptRoot
$aiRoot = Split-Path -Parent $testsRoot
$projectRoot = Split-Path -Parent $aiRoot
$repo = Join-Path $projectRoot 'RC10_LIB-FrameWork'
$outDir = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$exe = Join-Path $outDir ("test_swerve_core_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$moduleInc = Join-Path $repo 'RC10_LIB\Module\Inc'
$moduleSrcSwerve = Join-Path $repo 'RC10_LIB\Module\Src\Module_ChassisSwerve.cpp'
$testSrc = Join-Path $PSScriptRoot 'test_swerve_core.cpp'

& 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
    '-std=c++17' `
    '-I' $moduleInc `
    $moduleSrcSwerve `
    $testSrc `
    '-o' $exe

& $exe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
