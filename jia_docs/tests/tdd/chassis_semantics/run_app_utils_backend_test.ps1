$ErrorActionPreference = 'Stop'

$testsDir = $PSScriptRoot
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $testsDir)))
$outDir = Join-Path $testsDir 'build'
if (-not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

$stubDir = Join-Path $testsDir 'stubs'
$appInc = Join-Path $root 'RC10_LIB\APP\Inc'
$testSrc = Join-Path $testsDir 'test_app_utils_backend.cpp'

function Invoke-MathBackendTest {
    param(
        [Parameter(Mandatory = $true)]
        [string] $BackendName,
        [Parameter(Mandatory = $true)]
        [string] $BackendDefine
    )

    $exe = Join-Path $outDir ("test_app_utils_backend_{0}_{1}.exe" -f $BackendName, [System.Guid]::NewGuid().ToString('N'))

    & 'C:\Qt\Tools\mingw1310_64\bin\g++.exe' `
        '-std=c++17' `
        '-ffunction-sections' `
        '-fdata-sections' `
        '-I' $stubDir `
        '-I' $appInc `
        '-DJIA_APP_MATH_MODE_STD=0' `
        '-DJIA_APP_MATH_MODE_DSP=1' `
        ('-D' + $BackendDefine) `
        $testSrc `
        '-Wl,--gc-sections' `
        '-o' $exe

    if (-not (Test-Path $exe)) {
        throw "compile failed, executable not generated: $exe"
    }

    & $exe
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

Invoke-MathBackendTest -BackendName 'std' -BackendDefine 'JIA_APP_MATH_MODE=JIA_APP_MATH_MODE_STD'
Invoke-MathBackendTest -BackendName 'dsp' -BackendDefine 'JIA_APP_MATH_MODE=JIA_APP_MATH_MODE_DSP'
