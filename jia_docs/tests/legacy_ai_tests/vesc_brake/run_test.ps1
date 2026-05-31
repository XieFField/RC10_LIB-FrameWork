$ErrorActionPreference = 'Stop'

$testsRoot = Split-Path -Parent $PSScriptRoot
$docsRoot = Split-Path -Parent $testsRoot
$jiaDocsRoot = Split-Path -Parent $docsRoot
$repo = Split-Path -Parent $jiaDocsRoot
$outDir = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$exe = Join-Path $outDir ("test_motor_vesc_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $PSScriptRoot 'stubs'
$motorInc = Join-Path $repo 'RC10_LIB\\Motor\\Inc'
$motorSrc = Join-Path $repo 'RC10_LIB\\Motor\\Src\\Motor_VESC.cpp'
$testSrc = Join-Path $PSScriptRoot 'test_motor_vesc.cpp'

& 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
    '-std=c++17' `
    '-I' $stubDir `
    '-I' $motorInc `
    $motorSrc `
    $testSrc `
    '-o' $exe

& $exe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
Write-Host 'vesc_brake test: PASS'
