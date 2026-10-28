param(
    [Parameter(Mandatory)] [string] $OutputDirectory,
    [Parameter(Mandatory)] [string] $ReferenceDirectory,
    [ValidateSet('Debug', 'Release')] [string] $Configuration = 'Release',
    [string[]] $Scenes = @(),
    # Small sizes (for example -Width 64 -Height 64 -Spp 4) make an every-scene check fast;
    # references must have been rendered with the same settings.
    [ValidateRange(1, 16384)] [int] $Width = 512,
    [ValidateRange(1, 16384)] [int] $Height = 512,
    [ValidateRange(1, 1048576)] [int] $Spp = 64
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$exe = Join-Path $root "x64\$Configuration\GyunityApp.exe"
$output = [IO.Path]::GetFullPath($OutputDirectory)
$reference = (Resolve-Path -LiteralPath $ReferenceDirectory).Path
if (-not (Test-Path -LiteralPath $exe)) { throw "Build $Configuration|x64 before running this script." }
if ($output -eq $reference) { throw 'Use separate output and reference directories.' }

# Get names from the same registry used for rendering; never patch main.cpp.
$registered = @(& $exe --list-scenes)
if ($LASTEXITCODE -ne 0 -or $registered.Count -eq 0) { throw 'Cannot list registered scenes.' }
$scenes = if ($Scenes.Count -gt 0) { $Scenes } else { $registered }
foreach ($scene in $scenes) {
    if ($scene -notin $registered) { throw "Unknown scene requested for regression: $scene" }
}
foreach ($scene in $scenes) {
    foreach ($accel in 'bvh', 'kd') {
        $baseline = Join-Path $reference "old.$scene.$accel.png"
        if (-not (Test-Path -LiteralPath $baseline)) { throw "Missing reference: $baseline" }
    }
}
New-Item -ItemType Directory -Force -Path $output | Out-Null
$results = @()
Push-Location (Join-Path $root 'GyunityApp')
try {
    foreach ($scene in $scenes) {
        foreach ($accel in 'bvh', 'kd') {
            $image = Join-Path $output "registry.$scene.$accel.png"
            $log = Join-Path $output "registry.$scene.$accel.log"
            & $exe --scene $scene --accel $accel --out $image --width $Width --height $Height --spp $Spp --max-depth 8 *> $log
            if ($LASTEXITCODE -ne 0) { throw "Render failed: $scene/$accel; see $log" }
            $baseline = Join-Path $reference "old.$scene.$accel.png"
            $identical = (Get-FileHash -LiteralPath $image -Algorithm SHA256).Hash -eq
                (Get-FileHash -LiteralPath $baseline -Algorithm SHA256).Hash
            $results += [pscustomobject]@{ Scene = $scene; Accelerator = $accel; Identical = $identical }
            "$scene/$accel identical=$identical"
        }
    }
}
finally {
    Pop-Location
    $results | Export-Csv -LiteralPath (Join-Path $output 'comparisons.csv') -NoTypeInformation
}
if ($results | Where-Object { -not $_.Identical }) { throw 'Scene regression failed: PNG bytes differ.' }
