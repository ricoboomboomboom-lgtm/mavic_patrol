param(
  [Parameter(Mandatory = $true)][string]$World,
  [Parameter(Mandatory = $true)][string]$Controller,
  [Parameter(Mandatory = $true)][string]$Out,
  [string]$Mission = "",
  [string]$Radius = ""
)
$ErrorActionPreference = "Stop"
if ($Controller -ne "mavic_manual" -and $Controller -ne "mavic2pro_controller") {
  throw "Unknown controller: $Controller"
}
$text = [IO.File]::ReadAllText($World)
$replaced = $text.Replace('controller "mavic_manual"', "controller `"$Controller`"")
if ($replaced -eq $text -and $Controller -ne "mavic_manual") {
  throw "Controller field was not found in $World"
}
$utf8 = New-Object System.Text.UTF8Encoding $false
[IO.File]::WriteAllText($Out, $replaced, $utf8)
if ($Mission -ne "") {
  $allowed = @("area", "route", "object", "watch")
  if ($allowed -notcontains $Mission) {
    throw "Unknown mission: $Mission"
  }
  $root = Split-Path -Parent (Split-Path -Parent $World)
  $missionFile = Join-Path $root "controllers\mavic2pro_controller\mission.txt"
  [IO.File]::WriteAllText($missionFile, ($Mission + "`n"), $utf8)
  if ($Radius -ne "") {
    $allowedRadius = @("4", "6", "10", "20")
    if ($allowedRadius -notcontains $Radius) {
      throw "Unknown radius: $Radius"
    }
    $objectFile = Join-Path $root "controllers\mavic2pro_controller\missions\object.mission"
    $missionText = [IO.File]::ReadAllText($objectFile)
    $updated = [regex]::Replace($missionText, '(?m)^radius\s*=\s*\d+(?:\.\d+)?', "radius = $Radius")
    if ($updated -eq $missionText) {
      throw "radius line was not found in object.mission"
    }
    [IO.File]::WriteAllText($objectFile, $updated, $utf8)
  }
}
