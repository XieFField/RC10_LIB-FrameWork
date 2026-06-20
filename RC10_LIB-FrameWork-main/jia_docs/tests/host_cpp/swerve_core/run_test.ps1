$ErrorActionPreference = 'Stop'

$suiteDir = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $suiteDir)))
$sharedScript = Join-Path (Split-Path -Parent $suiteDir) '..\shared\common.ps1'
. $sharedScript

$outDir = New-JiaBuildDir -SuiteDir $suiteDir
$exe = Join-Path $outDir ("test_swerve_core_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$moduleInc = Join-Path $repoRoot 'RC10_LIB\Module\Inc'
$moduleSrcSwerve = Join-Path $repoRoot 'RC10_LIB\Module\Src\Module_ChassisSwerve.cpp'
$testSrc = Join-Path $suiteDir 'test_swerve_core.cpp'

Invoke-JiaGppCompile -Arguments @(
    '-std=c++17',
    '-I', $moduleInc,
    $moduleSrcSwerve,
    $testSrc
) -OutputPath $exe

Invoke-JiaNativeExecutable -Path $exe
