param(
    [ValidateSet('list', 'smoke', 'active', 'extended', 'historical', 'full')]
    [string] $Mode = 'smoke',
    [ValidateSet('all', 'host_cpp', 'python_semantics', 'historical')]
    [string] $Kind = 'all',
    [string] $Suite
)

$ErrorActionPreference = 'Stop'
$testsRoot = $PSScriptRoot
$registryPath = Join-Path $testsRoot 'tests.yaml'
$repoRoot = Split-Path -Parent (Split-Path -Parent $testsRoot)

if (-not (Test-Path $registryPath)) {
    throw "tests registry not found: $registryPath"
}

function Convert-SimpleScalar {
    param([string] $Value)

    $trimmed = $Value.Trim()
    if ($trimmed -eq '[]') {
        return @()
    }

    if (($trimmed.StartsWith("'") -and $trimmed.EndsWith("'")) -or ($trimmed.StartsWith('"') -and $trimmed.EndsWith('"'))) {
        return $trimmed.Substring(1, $trimmed.Length - 2)
    }

    return $trimmed
}

function Read-TestRegistry {
    $lines = Get-Content -Path $registryPath -Encoding utf8
    $suites = @()
    $current = $null
    $currentList = $null

    foreach ($rawLine in $lines) {
        $line = $rawLine.TrimEnd()
        if (-not $line -or $line.TrimStart().StartsWith('#')) {
            continue
        }

        if ($line -eq 'version: 1' -or $line -eq 'suites:') {
            continue
        }

        if ($line.StartsWith('  - id: ')) {
            if ($current) {
                $suites += [pscustomobject]$current
            }

            $current = @{
                id = Convert-SimpleScalar ($line.Substring(8))
                aliases = @()
            }
            $currentList = $null
            continue
        }

        if (-not $current) {
            continue
        }

        if ($line.Trim() -eq 'aliases:') {
            $currentList = 'aliases'
            continue
        }

        if ($line.StartsWith('      - ')) {
            if ($currentList) {
                $current[$currentList] += Convert-SimpleScalar ($line.Substring(8))
            }
            continue
        }

        if ($line.StartsWith('    ')) {
            $currentList = $null
            $pair = $line.Trim()
            $index = $pair.IndexOf(':')
            if ($index -lt 0) {
                continue
            }

            $key = $pair.Substring(0, $index).Trim()
            $value = $pair.Substring($index + 1).Trim()
            $current[$key] = Convert-SimpleScalar $value
        }
    }

    if ($current) {
        $suites += [pscustomobject]$current
    }

    return [pscustomobject]@{
        version = 1
        suites = $suites
    }
}

function Should-IncludeSuite {
    param($Entry)

    if ($Suite) {
        return $Entry.id -eq $Suite
    }

    if ($Kind -ne 'all' -and $Entry.kind -ne $Kind) {
        return $false
    }

    switch ($Mode) {
        'list' { return $true }
        'smoke' { return $Entry.default_level -eq 'smoke' }
        'active' { return @('smoke', 'active') -contains $Entry.default_level }
        'extended' { return @('smoke', 'active', 'extended') -contains $Entry.default_level }
        'historical' { return $Entry.default_level -eq 'historical' }
        'full' { return $true }
    }

    return $false
}

function Invoke-RegisteredSuite {
    param($Entry)

    Write-Host ("==> running {0}" -f $Entry.id)

    if ($Entry.command.StartsWith('powershell ')) {
        $parts = $Entry.command -split '-File ', 2
        $scriptAndArgs = $parts[1]
        $tokens = $scriptAndArgs -split ' '
        $script = Join-Path $repoRoot $tokens[0]
        $extra = @()
        if ($tokens.Length -gt 1) {
            $extra = $tokens[1..($tokens.Length - 1)]
        }
        & powershell -ExecutionPolicy Bypass -File $script @extra
    }
    elseif ($Entry.command.StartsWith('python ')) {
        $args = $Entry.command.Substring(7) -split ' '
        & python @args
    }
    else {
        throw "unsupported command format: $($Entry.command)"
    }

    $nativeExitCode = if ($null -eq $LASTEXITCODE) { 0 } else { [int]$LASTEXITCODE }
    if ($Entry.expected_result -eq 'xfail') {
        if ($nativeExitCode -eq 0) {
            throw "suite $($Entry.id) unexpectedly passed but is marked xfail"
        }
        Write-Host ("xfail confirmed: {0}" -f $Entry.id)
        return
    }

    if ($nativeExitCode -ne 0) {
        exit $nativeExitCode
    }
}

$registry = Read-TestRegistry
$selected = @($registry.suites | Where-Object { Should-IncludeSuite $_ })

if (-not $selected) {
    throw 'no suites matched the requested selection'
}

if ($Mode -eq 'list') {
    $selected | ForEach-Object {
        Write-Host ("{0} [{1}/{2}] => {3}" -f $_.id, $_.kind, $_.default_level, $_.command)
    }
    exit 0
}

foreach ($entry in $selected) {
    Invoke-RegisteredSuite -Entry $entry
}

Write-Host 'all selected suites completed'
