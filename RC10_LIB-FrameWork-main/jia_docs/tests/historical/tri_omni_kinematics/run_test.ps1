$ErrorActionPreference = 'Stop'

$testsRoot = Split-Path -Parent $PSScriptRoot
$aiRoot = Split-Path -Parent $testsRoot
$projectRoot = Split-Path -Parent $aiRoot
$repo = Join-Path $projectRoot 'RC10_LIB-FrameWork'
$outDir = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$exe = Join-Path $outDir ("test_tri_omni_kinematics_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $PSScriptRoot 'stubs'
$moduleInc = Join-Path $repo 'RC10_LIB\Module\Inc'
$moduleSrcBase = Join-Path $repo 'RC10_LIB\Module\Src\Module_ChassisBase.cpp'
$moduleSrcOmni = Join-Path $repo 'RC10_LIB\Module\Src\Module_ChassisOmni.cpp'
$motorInc = Join-Path $repo 'RC10_LIB\Motor\Inc'
$appInc = Join-Path $repo 'RC10_LIB\APP\Inc'
$appSrcTool = Join-Path $repo 'RC10_LIB\APP\Src\APP_tool.cpp'
$testSrc = Join-Path $PSScriptRoot 'test_tri_omni_kinematics.cpp'

& 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
    '-std=c++17' `
    '-I' $stubDir `
    '-I' $moduleInc `
    '-I' $motorInc `
    '-I' $appInc `
    $moduleSrcBase `
    $moduleSrcOmni `
    $appSrcTool `
    $testSrc `
    '-o' $exe

& $exe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
