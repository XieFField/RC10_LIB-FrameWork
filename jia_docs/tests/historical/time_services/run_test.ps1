$ErrorActionPreference = 'Stop'

$testsRoot = Split-Path -Parent $PSScriptRoot
$aiRoot = Split-Path -Parent $testsRoot
$projectRoot = Split-Path -Parent $aiRoot
$repo = Join-Path $projectRoot 'RC10_LIB-FrameWork'
$outDir = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$exe = Join-Path $outDir ("test_time_services_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $PSScriptRoot 'stubs'
$bspInc = Join-Path $repo 'RC10_LIB\BSP_Driver\Inc'
$bspSrcDwt = Join-Path $repo 'RC10_LIB\BSP_Driver\Src\BSP_TimeDwt.cpp'
$bspSrcUs64 = Join-Path $repo 'RC10_LIB\BSP_Driver\Src\BSP_TimeUs64.cpp'
$testSrc = Join-Path $PSScriptRoot 'test_time_services.cpp'

& 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
    '-std=c++17' `
    '-I' $bspInc `
    '-I' $stubDir `
    $bspSrcDwt `
    $bspSrcUs64 `
    $testSrc `
    '-o' $exe

& $exe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
