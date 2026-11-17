
$ErrorActionPreference = "Stop"

$RackDir = "C:\Program Files\VCV\Rack2Free"
$Shots = "$env:LOCALAPPDATA\Rack2\screenshots\AGMM"
$Downloads = Join-Path $env:USERPROFILE "Downloads"
$ZipFile = Join-Path $Downloads "AGMM-Screenshots.zip"

Write-Host "`n=== AGMM Screenshot Export ===" -ForegroundColor Cyan

# Clean previous screenshots
Write-Host "Cleaning previous screenshots..."
Remove-Item $Shots -Recurse -Force -ErrorAction SilentlyContinue

# Run Rack screenshot generation
Write-Host "Generating screenshots..."
Push-Location $RackDir

try {
    & ".\Rack.exe" --screenshot 2.0
}
finally {
    Pop-Location
}

# Verify screenshots
if (!(Test-Path $Shots) -or !(Get-ChildItem $Shots -File -Recurse)) {
    throw "No screenshots were generated!"
}

# Create ZIP archive
Write-Host "Creating ZIP archive..."
Remove-Item $ZipFile -Force -ErrorAction SilentlyContinue

Compress-Archive -Path "$Shots\*" `
    -DestinationPath $ZipFile `
    -CompressionLevel Optimal

Write-Host "`nExport completed!" -ForegroundColor Green
Write-Host "ZIP: $ZipFile" -ForegroundColor Yellow

# Open Downloads folder
explorer.exe $Downloads
