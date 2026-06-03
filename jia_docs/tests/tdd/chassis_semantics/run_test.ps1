$ErrorActionPreference = 'Stop'
$compat = Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'shared\compat_wrapper.ps1'
& powershell -ExecutionPolicy Bypass -File $compat -SuiteId 'host_cpp.chassis_semantics.main'
exit $LASTEXITCODE
