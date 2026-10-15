$ErrorActionPreference = 'Stop'

$externalPath = Join-Path -Path $PSScriptRoot -ChildPath 'external'
$buildPath = Join-Path -Path $externalPath -ChildPath 'build'

if (Test-Path -LiteralPath $buildPath) {
    Remove-Item -LiteralPath $buildPath -Recurse -Force
}

cmake -S $externalPath -B $buildPath '-DCMAKE_POLICY_VERSION_MINIMUM:STRING=3.10'
if ($LASTEXITCODE -ne 0) {
    throw "CMake configuration failed with exit code $LASTEXITCODE."
}

foreach ($configuration in 'Debug', 'Release') {
    cmake --build $buildPath --config $configuration
    if ($LASTEXITCODE -ne 0) {
        throw "CMake $configuration build failed with exit code $LASTEXITCODE."
    }
}
