$RackDir = "C:\Program Files\VCV\Rack2Free"
$Shots = "$env:LOCALAPPDATA\Rack2\screenshots\AGMM"

Remove-Item $Shots -Recurse -Force -ErrorAction SilentlyContinue

Push-Location $RackDir
.\Rack.exe --screenshot 2.0
Pop-Location