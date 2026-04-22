# Finds the installed VSIX extension directory for LDraw Visualiser in the main VS instance.
# Searches per-user Extensions folders and the global VS IDE Extensions folder.
# Outputs the directory path to stdout (for MSBuild ConsoleOutput capture).
param()

# Per-user: %LOCALAPPDATA%\Microsoft\VisualStudio\18.0_*\Extensions\
# Global:   C:\Program Files\Microsoft Visual Studio\18\Enterprise\Common7\IDE\Extensions\
$extensions_dirs = @()

# Collect per-user extension directories (skip Exp instances)
Get-ChildItem "$env:LOCALAPPDATA\Microsoft\VisualStudio\18.0_*" -Directory -ErrorAction SilentlyContinue |
	Where-Object { $_.Name -notmatch 'Exp$' } |
	ForEach-Object {
		$ext = Join-Path $_.FullName 'Extensions'
		if (Test-Path $ext) { $extensions_dirs += $ext }
	}

# Add global VS IDE Extensions folder
$global_ext = 'C:\Program Files\Microsoft Visual Studio\18\Enterprise\Common7\IDE\Extensions'
if (Test-Path $global_ext) { $extensions_dirs += $global_ext }

foreach ($dir in $extensions_dirs)
{
	Get-ChildItem $dir -Recurse -Filter 'Rylogic.LDrawVisualiser.dll' -ErrorAction SilentlyContinue |
		Select-Object -First 1 -ExpandProperty DirectoryName |
		ForEach-Object { Write-Output $_; exit 0 }
}
