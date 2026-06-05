$ErrorActionPreference = 'Stop'

function Get-JiaTestsRoot {
    return Split-Path -Parent $PSScriptRoot
}

function Get-JiaRepoRoot {
    return Split-Path -Parent (Split-Path -Parent (Get-JiaTestsRoot))
}

function Get-JiaArtifactsRoot {
    $path = Join-Path (Get-JiaTestsRoot) 'artifacts'
    if (-not (Test-Path $path)) {
        New-Item -ItemType Directory -Path $path | Out-Null
    }
    return $path
}

function New-JiaBuildDir {
    param([Parameter(Mandatory = $true)][string] $SuiteDir)

    $path = Join-Path $SuiteDir 'build'
    if (-not (Test-Path $path)) {
        New-Item -ItemType Directory -Path $path | Out-Null
    }
    return $path
}

function Get-JiaGppPath {
    $candidates = @(
        'C:\Qt\Tools\mingw1310_64\bin\g++.exe',
        'C:\Qt\Tools\mingw1120_64\bin\g++.exe'
    )

    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    $cmd = Get-Command g++ -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }

    throw 'g++ compiler not found. Install MinGW g++ or update Get-JiaGppPath candidates.'
}

function Invoke-JiaGppCompile {
    param(
        [Parameter(Mandatory = $true)][string[]] $Arguments,
        [Parameter(Mandatory = $true)][string] $OutputPath
    )

    $gpp = Get-JiaGppPath
    & $gpp @Arguments '-o' $OutputPath
    if (-not (Test-Path $OutputPath)) {
        throw "compile failed, executable not generated: $OutputPath"
    }
}

function Invoke-JiaNativeExecutable {
    param(
        [Parameter(Mandatory = $true)][string] $Path,
        [string[]] $Arguments = @()
    )

    $gppBin = Split-Path -Parent (Get-JiaGppPath)
    $oldPath = $env:PATH
    try {
        # Native host tests must load the MinGW runtime that matches the compiler.
        # Put the selected g++ bin dir first so another toolchain's DLLs do not win PATH lookup.
        $env:PATH = "$gppBin;$oldPath"
        & $Path @Arguments
        $nativeExitCode = if ($null -eq $LASTEXITCODE) { 0 } else { [int]$LASTEXITCODE }
        if ($nativeExitCode -ne 0) {
            exit $nativeExitCode
        }
    }
    finally {
        $env:PATH = $oldPath
    }
}
