param(
    [string]$SourceDir,
    [string]$SourceFile,
    [string]$BuildDir = "$env:TEMP\tc11p_build_exact"
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($SourceDir)) {
    if ($PSCommandPath) {
        $SourceDir = Split-Path -Parent $PSCommandPath
    }
    else {
        $SourceDir = "."
    }
}

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
    if (-not $files -or $files.Count -eq 0) {
        throw "No .c files found in folder: $Dir"
    }

    if ($files.Count -gt 1) {
        Write-Host "Multiple .c files found. Using first by name:" -ForegroundColor Yellow
        $files | ForEach-Object { Write-Host "  - $($_.Name)" -ForegroundColor Yellow }
    }

    return $files[0].FullName
}

if (-not (Test-Path -LiteralPath $SourceDir)) {
    throw "SourceDir not found: $SourceDir"
}

$resolvedSourceDir = (Resolve-Path -LiteralPath $SourceDir).Path
$resolvedSourceFile = Resolve-SourceFile -Dir $resolvedSourceDir -File $SourceFile
$baseName = [System.IO.Path]::GetFileNameWithoutExtension($resolvedSourceFile)

New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$resolvedBuildDir = (Resolve-Path -LiteralPath $BuildDir).Path

$buildSource = Join-Path $resolvedBuildDir ("{0}.c" -f $baseName)
$buildHeader = Join-Path $resolvedBuildDir "REG51.H"
$buildIhx = Join-Path $resolvedBuildDir ("{0}.ihx" -f $baseName)
$buildHex = Join-Path $resolvedBuildDir ("{0}.hex" -f $baseName)
$outHex = Join-Path ([System.IO.Path]::GetDirectoryName($resolvedSourceFile)) ("{0}.hex" -f $baseName)

Copy-Item -LiteralPath $resolvedSourceFile -Destination $buildSource -Force

$shim = @'
/* SDCC shim for Keil REG51.H */
#include <8051.h>
'@
Set-Content -LiteralPath $buildHeader -Value $shim -Encoding ASCII

$sdccBin = "C:\Program Files\SDCC\bin"
if (Test-Path -LiteralPath $sdccBin) {
    $env:PATH = "$sdccBin;$env:PATH"
}

if (-not (Get-Command sdcc -ErrorAction SilentlyContinue)) {
    throw "sdcc was not found. Install SDCC or add it to PATH."
}

if (-not (Get-Command packihx -ErrorAction SilentlyContinue)) {
    throw "packihx was not found. Ensure SDCC tools are available in PATH."
}

Write-Host "Building: $resolvedSourceFile"
Write-Host "Work dir: $resolvedBuildDir"

sdcc -mmcs51 --model-small --iram-size 256 --xram-size 2048 --code-size 61440 -I "$resolvedBuildDir" -o "$resolvedBuildDir\" "$buildSource"
if ($LASTEXITCODE -ne 0) {
    throw "sdcc compilation failed with exit code $LASTEXITCODE"
}

packihx "$buildIhx" > "$buildHex"
if ($LASTEXITCODE -ne 0) {
    throw "packihx failed with exit code $LASTEXITCODE"
}

Copy-Item -LiteralPath $buildHex -Destination $outHex -Force

Write-Host "HEX created: $outHex" -ForegroundColor Green
