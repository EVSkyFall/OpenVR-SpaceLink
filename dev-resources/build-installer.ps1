param(
    [string]$BuildDir = (Join-Path (Split-Path -Parent $PSScriptRoot) 'out\build\x64-release'),
    [string]$OutDir = (Join-Path (Split-Path -Parent $PSScriptRoot) 'out\installer'),
    [string]$Makensis
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$repoDir = Split-Path -Parent $PSScriptRoot
$BuildDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($BuildDir)
$OutDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutDir)

if (-not $Makensis) {
    $compiler = Get-Command -Name 'makensis' -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($compiler) {
        $Makensis = $compiler.Source
    } else {
        foreach ($programFilesDir in @(${env:ProgramFiles(x86)}, $env:ProgramFiles)) {
            if ($programFilesDir) {
                $candidate = Join-Path $programFilesDir 'NSIS\makensis.exe'
                if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                    $Makensis = $candidate
                    break
                }
            }
        }
        if (-not $Makensis) {
            throw 'NSIS compiler not found. Supply -Makensis or install makensis on PATH or in Program Files\NSIS.'
        }
    }
}

$binInputs = @(
    (Join-Path $repoDir 'resources\LICENSE.txt'),
    (Join-Path $repoDir 'resources\LICENSE'),
    (Join-Path $repoDir 'resources\LICENSES'),
    (Join-Path $repoDir 'resources\manifest.vrmanifest'),
    (Join-Path $repoDir 'resources\icon.png'),
    (Join-Path $BuildDir 'OpenVR-SpaceOverride.exe'),
    (Join-Path $repoDir '3rdparty\OpenVR\bin\win64\openvr_api.dll')
)
$driverSource = Join-Path $PSScriptRoot 'driver'
$driverBinary = Join-Path $BuildDir 'driver_spaceoverride.dll'
$installerScript = Join-Path $PSScriptRoot 'installer.nsi'
foreach ($inputFile in ($binInputs + @($driverBinary, $installerScript))) {
    if (-not (Test-Path -LiteralPath $inputFile -PathType Leaf)) {
        throw "Missing installer input: $inputFile"
    }
}
if (-not (Test-Path -LiteralPath $driverSource -PathType Container)) {
    throw "Missing installer input directory: $driverSource"
}

$stageDir = [IO.Path]::GetFullPath((Join-Path $OutDir 'stage'))
$outPrefix = $OutDir.TrimEnd('\', '/') + [IO.Path]::DirectorySeparatorChar
if (-not $stageDir.StartsWith($outPrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw "Staging path is outside the output directory: $stageDir"
}
if (Test-Path -LiteralPath $stageDir) {
    Remove-Item -LiteralPath $stageDir -Recurse -Force
}
$binDir = Join-Path $stageDir 'bin'
New-Item -ItemType Directory -Path $binDir -Force | Out-Null
foreach ($inputFile in $binInputs) {
    Copy-Item -LiteralPath $inputFile -Destination $binDir
}
Copy-Item -LiteralPath $driverSource -Destination $stageDir -Recurse
$driverDir = Join-Path $stageDir 'driver'
Get-ChildItem -LiteralPath $driverDir -Filter '.gitkeep' -Recurse -File -Force | Remove-Item -Force
$driverBinDir = Join-Path $driverDir 'bin\win64'
New-Item -ItemType Directory -Path $driverBinDir -Force | Out-Null
Copy-Item -LiteralPath $driverBinary -Destination $driverBinDir

& $Makensis /V4 "/DFILES_DIR=$binDir" "/DDRIVER_DIR=$driverDir" "/DLICENSE_FILE=$binDir\LICENSE.txt" "/DOUT_DIR=$OutDir" $installerScript
exit $LASTEXITCODE
