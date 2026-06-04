param(
    [Parameter(Mandatory = $true)][string] $SuiteId
)

$ErrorActionPreference = 'Stop'
$testsRoot = Split-Path -Parent $PSScriptRoot
$runner = Join-Path $testsRoot 'run_tests.ps1'

Write-Host "compat wrapper: redirecting to unified tests runner for suite '$SuiteId'"
& powershell -ExecutionPolicy Bypass -File $runner -Suite $SuiteId
exit $LASTEXITCODE
