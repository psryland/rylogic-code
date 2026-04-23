<#
.SYNOPSIS
	Replaces installed Rylogic.LDrawVisualiser VSIX binaries with freshly built ones.

.DESCRIPTION
	Finds all locally installed copies of the Rylogic.LDrawVisualiser extension across
	supported Visual Studio versions (2022 / 2026) — both per-user and global — then
	prompts:
	  1. Whether to deploy Debug or Release build output.
	  2. Which discovered installs to overwrite.
	Selected installs are then overwritten with the chosen build's DLLs and PDBs.

	Run this manually when you want to push a fresh local build into VS without
	reinstalling the .vsix (e.g. between code edits during iterative development).

.PARAMETER Configuration
	Optional. 'Debug' or 'Release'. If omitted, the script prompts.

.PARAMETER All
	Optional. If specified, all discovered installs are overwritten without prompting
	for selection (the configuration prompt still applies unless -Configuration is set).

.EXAMPLE
	.\ReplaceLocalInstall.ps1
	Interactive: prompts for both configuration and install selection.

.EXAMPLE
	.\ReplaceLocalInstall.ps1 -Configuration Debug -All
	Non-interactive: deploys Debug binaries to every discovered install.
#>
param(
	[ValidateSet('Debug', 'Release')]
	[string] $Configuration,

	[switch] $All
)

$ErrorActionPreference = 'Stop'

# Files to copy. PDBs are included for both configs (Release builds also produce them).
$artifacts = @(
	'Rylogic.LDrawVisualiser.dll', 'Rylogic.LDrawVisualiser.pdb',
	'Rylogic.Core.dll',            'Rylogic.Core.pdb',
	'Rylogic.Gfx.dll',             'Rylogic.Gfx.pdb',
	'Rylogic.Gui.WPF.dll',         'Rylogic.Gui.WPF.pdb',
	'Rylogic.Windows.dll',         'Rylogic.Windows.pdb'
)

# Project root and TFM (must match TargetFrameworks in the csproj).
$project_root = Split-Path -Parent $PSScriptRoot
$tfm = 'net481'

# Find all installed extension copies.
function Find-Installs
{
	$installs = @()

	# Per-user: %LOCALAPPDATA%\Microsoft\VisualStudio\<ver>_*\Extensions\<hash>\Rylogic.LDrawVisualiser.dll
	# Skip Experimental hives (the F5-debug instance) — those are managed by VS itself.
	foreach ($ver in @('17.0', '18.0'))
	{
		$user_root = Join-Path $env:LOCALAPPDATA "Microsoft\VisualStudio\${ver}_*"
		Get-ChildItem $user_root -Directory -ErrorAction SilentlyContinue |
			Where-Object { $_.Name -notmatch 'Exp$' } |
			ForEach-Object {
				$ext_root = Join-Path $_.FullName 'Extensions'
				if (-not (Test-Path $ext_root)) { return }
				Get-ChildItem $ext_root -Recurse -Filter 'Rylogic.LDrawVisualiser.dll' -ErrorAction SilentlyContinue |
					ForEach-Object {
						$installs += [pscustomobject]@{
							Scope   = 'User'
							Version = "VS $ver"
							Hive    = $_.FullName -replace '.*\\(\d+\.\d+_[^\\]+).*', '$1'
							Path    = $_.DirectoryName
						}
					}
			}
	}

	# Global: C:\Program Files\Microsoft Visual Studio\<year>\<edition>\Common7\IDE\Extensions\<hash>\
	foreach ($year in @('2022', '18'))
	{
		foreach ($edition in @('Enterprise', 'Professional', 'Community'))
		{
			$global_ext = "C:\Program Files\Microsoft Visual Studio\$year\$edition\Common7\IDE\Extensions"
			if (-not (Test-Path $global_ext)) { continue }
			Get-ChildItem $global_ext -Recurse -Filter 'Rylogic.LDrawVisualiser.dll' -ErrorAction SilentlyContinue |
				ForEach-Object {
					$installs += [pscustomobject]@{
						Scope   = 'Global'
						Version = "VS $year"
						Hive    = $edition
						Path    = $_.DirectoryName
					}
				}
		}
	}

	return $installs
}

# Prompt for configuration when not supplied.
function Get-Configuration
{
	if ($Configuration) { return $Configuration }

	Write-Host ''
	Write-Host 'Which build configuration to deploy?' -ForegroundColor Cyan
	Write-Host '  [1] Debug'
	Write-Host '  [2] Release'
	while ($true)
	{
		$answer = Read-Host 'Choice (1/2)'
		switch ($answer)
		{
			'1' { return 'Debug' }
			'2' { return 'Release' }
			default { Write-Host "Enter 1 or 2." -ForegroundColor Yellow }
		}
	}
}

# Prompt the user to pick which installs to overwrite. Returns the selected subset.
function Select-Installs([object[]] $installs)
{
	if ($All) { return $installs }

	Write-Host ''
	Write-Host 'Discovered installs:' -ForegroundColor Cyan
	for ($i = 0; $i -lt $installs.Count; ++$i)
	{
		$it = $installs[$i]
		Write-Host ("  [{0}] {1,-6} {2,-7} {3,-12} {4}" -f ($i + 1), $it.Scope, $it.Version, $it.Hive, $it.Path)
	}
	Write-Host '  [A] All'
	Write-Host '  [Q] Quit (cancel)'

	while ($true)
	{
		$answer = Read-Host 'Select installs (comma-separated indices, or A / Q)'
		$answer = $answer.Trim()
		if ($answer -match '^[Qq]$') { return @() }
		if ($answer -match '^[Aa]$') { return $installs }

		# Parse comma-separated indices.
		$indices = $answer -split '[,\s]+' | Where-Object { $_ -ne '' }
		$selected = @()
		$valid = $true
		foreach ($idx in $indices)
		{
			if ($idx -notmatch '^\d+$') { $valid = $false; break }
			$n = [int]$idx
			if ($n -lt 1 -or $n -gt $installs.Count) { $valid = $false; break }
			$selected += $installs[$n - 1]
		}
		if ($valid -and $selected.Count -gt 0) { return $selected }

		Write-Host "Invalid selection. Use indices like '1,3' or 'A' / 'Q'." -ForegroundColor Yellow
	}
}

# --- Main ---

$installs = Find-Installs
if ($installs.Count -eq 0)
{
	Write-Host 'No installed Rylogic.LDrawVisualiser extension found.' -ForegroundColor Yellow
	Write-Host 'Install the .vsix first (double-click bin\<Config>\net481\Rylogic.LDrawVisualiser.vsix).'
	exit 1
}

$config = Get-Configuration
$source_dir = Join-Path $project_root "bin\$config\$tfm"
if (-not (Test-Path $source_dir))
{
	Write-Host "Build output not found: $source_dir" -ForegroundColor Red
	Write-Host "Build the project in '$config' configuration first."
	exit 1
}

# Verify the primary artifact exists before doing anything destructive.
$primary = Join-Path $source_dir 'Rylogic.LDrawVisualiser.dll'
if (-not (Test-Path $primary))
{
	Write-Host "Missing build artifact: $primary" -ForegroundColor Red
	exit 1
}

$targets = Select-Installs $installs
if ($targets.Count -eq 0)
{
	Write-Host 'Cancelled — no installs selected.' -ForegroundColor Yellow
	exit 0
}

Write-Host ''
Write-Host "Deploying '$config' build to $($targets.Count) install(s)..." -ForegroundColor Cyan

$any_failed = $false
foreach ($target in $targets)
{
	Write-Host ''
	Write-Host "  -> $($target.Path)" -ForegroundColor Green
	foreach ($file in $artifacts)
	{
		$src = Join-Path $source_dir $file
		if (-not (Test-Path $src))
		{
			Write-Host "     skip (not built): $file" -ForegroundColor DarkGray
			continue
		}
		$dst = Join-Path $target.Path $file
		try
		{
			Copy-Item -LiteralPath $src -Destination $dst -Force
			Write-Host "     copied: $file"
		}
		catch
		{
			# Most common cause: VS is running and has the DLL loaded.
			Write-Host "     FAILED: $file — $($_.Exception.Message)" -ForegroundColor Red
			$any_failed = $true
		}
	}
}

Write-Host ''
if ($any_failed)
{
	Write-Host 'Done (with errors). Close Visual Studio and retry if files were locked.' -ForegroundColor Yellow
	exit 1
}
Write-Host 'Done. Restart Visual Studio to pick up changes.' -ForegroundColor Green
