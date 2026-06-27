$ErrorActionPreference = 'Stop'

$suiteDir = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $suiteDir)))
$sharedScript = Join-Path (Split-Path -Parent $suiteDir) '..\shared\common.ps1'
. $sharedScript

$outDir = New-JiaBuildDir -SuiteDir $suiteDir
$exe = Join-Path $outDir ("test_chassis_semantics_slim_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $suiteDir 'stubs'
$userInc = Join-Path $repoRoot 'User\Setup\Inc'
$appInc = Join-Path $repoRoot 'RC10_LIB\APP\Inc'
$testSupportSrc = Join-Path $suiteDir 'stubs_src\test_host_globals.cpp'
$chassisSrc = Join-Path $repoRoot 'User\Setup\Src\chassis.cpp'
$smokeSrc = Join-Path $suiteDir 'test_chassis_semantics_slim_smoke.cpp'

$compileArgs = @(
    '-std=c++17',
    '-Os',
    '-ffunction-sections',
    '-fdata-sections',
    '-DJIA_CHASSIS_PROFILE=JIA_CHASSIS_PROFILE_RUNTIME_MIN',
    '-I', $stubDir,
    '-I', $userInc,
    '-I', $appInc,
    $testSupportSrc,
    $chassisSrc,
    $smokeSrc,
    '-Wl,--gc-sections'
)

Invoke-JiaGppCompile -Arguments $compileArgs -OutputPath $exe
Invoke-JiaNativeExecutable -Path $exe
