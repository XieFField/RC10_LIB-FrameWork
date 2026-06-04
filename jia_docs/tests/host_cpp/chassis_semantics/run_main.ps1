$ErrorActionPreference = 'Stop'

$suiteDir = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $suiteDir)))
$sharedScript = Join-Path (Split-Path -Parent $suiteDir) '..\shared\common.ps1'
. $sharedScript

$outDir = New-JiaBuildDir -SuiteDir $suiteDir
$exe = Join-Path $outDir ("test_chassis_semantics_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $suiteDir 'stubs'
$doctestInc = Join-Path (Split-Path -Parent (Split-Path -Parent $suiteDir)) 'third_party\doctest'
$userInc = Join-Path $repoRoot 'User\Setup\Inc'
$appInc = Join-Path $repoRoot 'RC10_LIB\APP\Inc'
$testSupportSrc = Join-Path $suiteDir 'stubs_src\test_host_globals.cpp'
$chassisSrc = Join-Path $repoRoot 'User\Setup\Src\chassis.cpp'
$testMainSrc = Join-Path $suiteDir 'test_chassis_semantics_main.cpp'
$testSrc = Join-Path $suiteDir 'test_chassis_semantics.cpp'

Invoke-JiaGppCompile -Arguments @(
    '-std=c++17',
    '-ffunction-sections',
    '-fdata-sections',
    '-I', $stubDir,
    '-I', $doctestInc,
    '-I', $userInc,
    '-I', $appInc,
    $testSupportSrc,
    $chassisSrc,
    $testMainSrc,
    $testSrc,
    '-Wl,--gc-sections'
) -OutputPath $exe

Invoke-JiaNativeExecutable -Path $exe -Arguments @(
    '--reporters=console',
    '--success=false',
    '--duration=false',
    '--no-version'
)
& (Join-Path $suiteDir 'run_app_utils_backend.ps1')
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
