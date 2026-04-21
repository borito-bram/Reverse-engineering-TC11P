param(
    [string]$SourceDir,
    [string]$SourceFile
)

$ErrorActionPreference = "Stop"

function Resolve-SourceFile {
    param(
        [string]$Dir,
        [string]$File
    )

    if ($File) {
        if ([System.IO.Path]::IsPathRooted($File)) {
            if (-not (Test-Path -LiteralPath $File)) {
                throw "Source file not found: $File"
            }
            return (Resolve-Path -LiteralPath $File).Path
        }

        $candidate = Join-Path $Dir $File
        if (-not (Test-Path -LiteralPath $candidate)) {
            throw "Source file not found: $candidate"
        }
        return (Resolve-Path -LiteralPath $candidate).Path
    }

    $files = Get-ChildItem -Path $Dir -Filter "*.c" -File | Sort-Object Name
    if (-not $files) {
        throw "No .c files found in folder: $Dir"
    }

    if ($files.Count -gt 1) {
        throw "Multiple .c files found in folder. Pass -SourceFile explicitly."
    }

    return $files[0].FullName
}

function Find-KeilTool {
    param([string]$ToolName)

    $candidates = New-Object System.Collections.Generic.List[string]

    if (-not [string]::IsNullOrWhiteSpace($env:KEIL_ROOT)) {
        $candidates.Add((Join-Path $env:KEIL_ROOT "C51\BIN\$ToolName"))
        $candidates.Add((Join-Path $env:KEIL_ROOT "BIN\$ToolName"))
    }

    $candidates.Add("C:\Keil_v5\C51\BIN\$ToolName")

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }

    throw "$ToolName was not found. Set KEIL_ROOT or install Keil C51 under C:\Keil_v5."
}

if ([string]::IsNullOrWhiteSpace($SourceDir)) {
    if ($PSCommandPath) {
        $SourceDir = Split-Path -Parent $PSCommandPath
    }
    else {
        $SourceDir = "."
    }
}

if (-not (Test-Path -LiteralPath $SourceDir)) {
    throw "SourceDir not found: $SourceDir"
}

$resolvedSourceDir = (Resolve-Path -LiteralPath $SourceDir).Path
$resolvedSourceFile = Resolve-SourceFile -Dir $resolvedSourceDir -File $SourceFile
$baseName = [System.IO.Path]::GetFileNameWithoutExtension($resolvedSourceFile)

$compiler = Find-KeilTool -ToolName "C51.EXE"
$linker = Find-KeilTool -ToolName "BL51.EXE"
$hexTool = Find-KeilTool -ToolName "OH51.EXE"

Write-Host "Building with Keil C51:" -ForegroundColor Cyan
Write-Host "  Source: $resolvedSourceFile"
Write-Host "  C51:    $compiler"

Push-Location $resolvedSourceDir
try {
    & $compiler $resolvedSourceFile OBJECTEXTEND DEBUG
    if ($LASTEXITCODE -ne 0) {
        throw "C51 compilation failed with exit code $LASTEXITCODE"
    }

    & $linker "$baseName.OBJ" TO $baseName
    if ($LASTEXITCODE -ne 0) {
        throw "BL51 link failed with exit code $LASTEXITCODE"
    }

    & $hexTool $baseName
    if ($LASTEXITCODE -ne 0) {
        throw "OH51 hex conversion failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

$hexPath = Join-Path $resolvedSourceDir ("{0}.hex" -f $baseName)
if (-not (Test-Path -LiteralPath $hexPath)) {
    throw "Expected HEX file was not produced: $hexPath"
}

Write-Host "HEX created: $hexPath" -ForegroundColor Green