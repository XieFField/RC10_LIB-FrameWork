$ErrorActionPreference = 'Stop'

$suiteDir = $PSScriptRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $suiteDir)))
$sharedScript = Join-Path (Split-Path -Parent $suiteDir) '..\shared\common.ps1'
. $sharedScript

$outDir = New-JiaBuildDir -SuiteDir $suiteDir
$exe = Join-Path $outDir ("test_motor_el05_{0}.exe" -f [System.Guid]::NewGuid().ToString('N'))
$stubDir = Join-Path $suiteDir 'stubs'
$motorInc = Join-Path $repoRoot 'RC10_LIB\Motor\Inc'
$motorSrc = Join-Path $repoRoot 'RC10_LIB\Motor\Src\Motor_EL05.cpp'
$testSrc = Join-Path $suiteDir 'test_motor_el05.cpp'

Invoke-JiaGppCompile -Arguments @(
    '-std=c++17',
    '-I', $stubDir,
    '-I', $motorInc,
    $motorSrc,
    $testSrc
) -OutputPath $exe

Invoke-JiaNativeExecutable -Path $exe
Write-Host 'el05_position_mode test: PASS'
