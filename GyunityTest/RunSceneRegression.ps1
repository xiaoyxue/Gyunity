param(
    [Parameter(Mandatory)] [string] $OutputDirectory,
    [Parameter(Mandatory)] [string] $ReferenceDirectory,
    [ValidateSet('Debug', 'Release')] [string] $Configuration = 'Release'
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$exe = Join-Path $root "x64\$Configuration\GyunityApp.exe"
$output = [IO.Path]::GetFullPath($OutputDirectory)
$reference = (Resolve-Path -LiteralPath $ReferenceDirectory).Path
if (-not (Test-Path -LiteralPath $exe)) { throw "Build $Configuration|x64 before running this script." }
if ($output -eq $reference) { throw 'Use separate output and reference directories.' }

# Get names from the same registry used for rendering; never patch main.cpp.
$scenes = @(& $exe --list-scenes)
if ($LASTEXITCODE -ne 0 -or $scenes.Count -eq 0) { throw 'Cannot list registered scenes.' }
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
            & $exe --scene $scene --accel $accel --out $image --width 512 --height 512 --spp 64 --max-depth 8 *> $log
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
