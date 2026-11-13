
param(
    [string]$RackExe = "$env:ProgramFiles\VCV\Rack2\Rack.exe",
    [double]$Zoom = 1.0,
    [string]$PluginSlug = "AGMM"
)

$ErrorActionPreference = "Stop"

# Rack directories
$RackUserDir = Join-Path $env:LOCALAPPDATA "Rack2"
$ScreenshotRoot = Join-Path $RackUserDir "screenshots"
$SourceDir = Join-Path $ScreenshotRoot $PluginSlug

# Destination inside AGMM repository
$Destination = Join-Path $PSScriptRoot "docs\screenshots"

Write-Host "`n==================================" -ForegroundColor Cyan
Write-Host " AGMM MODULE SCREENSHOT GENERATOR"
Write-Host "==================================" -ForegroundColor Cyan

# Verify Rack executable
if (!(Test-Path $RackExe)) {
    throw "Rack.exe not found: $RackExe"
}

# Remove previous AGMM screenshots
if (Test-Path $SourceDir) {
    Remove-Item $SourceDir -Recurse -Force
}

# Generate screenshots
Write-Host "`nGenerating module screenshots..." -ForegroundColor Yellow

& $RackExe --screenshot $Zoom

if ($LASTEXITCODE -ne 0) {
    throw "Rack screenshot generation failed."
}

# Verify output
if (!(Test-Path $SourceDir)) {
    throw "Screenshot directory not found: $SourceDir. Check PluginSlug."
}

# Create documentation directory
New-Item -ItemType Directory -Force -Path $Destination | Out-Null

# Copy screenshots
$Images = Get-ChildItem $SourceDir -Filter "*.png"

foreach ($Image in $Images) {
    Copy-Item $Image.FullName $Destination -Force
    Write-Host "Saved: $($Image.Name)" -ForegroundColor Green
}

Write-Host "`n==================================" -ForegroundColor Cyan
Write-Host "Completed: $($Images.Count) screenshots"
Write-Host "Destination: $Destination"
Write-Host "==================================" -ForegroundColor Cyan
