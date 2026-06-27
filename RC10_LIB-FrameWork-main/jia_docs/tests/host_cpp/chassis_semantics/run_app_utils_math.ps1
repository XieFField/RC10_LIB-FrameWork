$ErrorActionPreference = 'Stop'

$suiteDir = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $suiteDir)))
$sharedScript = Join-Path (Split-Path -Parent $suiteDir) '..\shared\common.ps1'
. $sharedScript

$outDir = New-JiaBuildDir -SuiteDir $suiteDir
$stubDir = Join-Path $suiteDir 'stubs'
$appInc = Join-Path $repoRoot 'RC10_LIB\APP\Inc'
$testSrc = Join-Path $suiteDir 'test_app_utils_math.cpp'

function Invoke-MathCase {
    param(
        [Parameter(Mandatory = $true)][string] $BackendName,
        [Parameter(Mandatory = $true)][string] $BackendDefine
    )

    $exe = Join-Path $outDir ("test_app_utils_math_{0}_{1}.exe" -f $BackendName, [System.Guid]::NewGuid().ToString('N'))
    Invoke-JiaGppCompile -Arguments @(
        '-std=c++17',
        '-ffunction-sections',
        '-fdata-sections',
        '-I', $stubDir,
        '-I', $appInc,
        '-DJIA_APP_MATH_MODE_STD=0',
        '-DJIA_APP_MATH_MODE_DSP=1',
        ('-D' + $BackendDefine),
        $testSrc,
        '-Wl,--gc-sections'
    ) -OutputPath $exe

    Invoke-JiaNativeExecutable -Path $exe
}

Invoke-MathCase -BackendName 'std' -BackendDefine 'JIA_APP_MATH_MODE=JIA_APP_MATH_MODE_STD'
Invoke-MathCase -BackendName 'dsp' -BackendDefine 'JIA_APP_MATH_MODE=JIA_APP_MATH_MODE_DSP'
