$gitHash = git rev-parse HEAD
$gitTag = git describe --tags --always --dirty

$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
$vsInstallationPath = & $vswhere -latest -products * -property installationPath
$msbuildPath = Join-Path $vsInstallationPath "MSBuild\Current\Bin\MSBuild.exe"

# Check if msbuild.exe exists at the constructed path
if (Test-Path $msbuildPath) {
    Write-Host "Found msbuild.exe at: $msbuildPath"
} else {
    Write-Error "msbuild.exe not found at: $msbuildPath"
    break
}

@"
#pragma once

#define GIT_HASH "$gitHash"
#define GIT_TAG "$gitTag"
"@ | Out-File -FilePath git_info.h -Encoding ascii

write-host "`nBuilding...`n"
# Set alias for msbuild
Set-Alias msbuild $msbuildPath

# Run msbuild with desired parameters
& MSBuild PixElv.sln -property:Configuration=Release -t:rebuild