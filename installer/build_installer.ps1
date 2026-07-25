# Builds the signed LEparagliding installer into installer/Output/.
# Prerequisites: Inno Setup 6, Windows SDK signtool, and the COBOD
# International A/S SafeNet token certificate (see CLAUDE.md).
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent

Push-Location $repo
try {
    # Reconfigure so installer/version.iss embeds the current HEAD commit,
    # then build Release (also runs windeployqt and the OCCT deployment).
    cmake --preset windows
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
    cmake --build --preset release
    if ($LASTEXITCODE) { exit $LASTEXITCODE }
} finally {
    Pop-Location
}

# ISCC.exe does not read the Inno Setup IDE's SignTools registry config, so
# the SafeNet definition is passed via /S. The argument is quote-free (8.3
# short path for signtool, certificate selected by thumbprint) because
# PowerShell mangles embedded quotes in native-command arguments.
$sign = '/SSafeNet=C:\PROGRA~2\WI3CF2~1\10\bin\100261~1.0\x86\signtool.exe sign /sha1 C8CD08A6B254958D769848CC047F1C7E79FC3A84 /tr http://timestamp.sectigo.com /td sha256 /fd sha256 $p'
& "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" $sign "$PSScriptRoot\LEparagliding-installer.iss"
if ($LASTEXITCODE) { exit $LASTEXITCODE }

$installer = Get-ChildItem "$PSScriptRoot\Output\*.exe" |
    Sort-Object LastWriteTime | Select-Object -Last 1
$signature = Get-AuthenticodeSignature $installer.FullName
"$($signature.Status): $($installer.FullName)"
if ($signature.Status -ne 'Valid') { exit 1 }
