<#
.SYNOPSIS
	First-run bootstrap for building the Rylogic solution on a fresh machine.

.DESCRIPTION
	Prepares a clean checkout so that 'Rylogic.sln' / 'script/Build.csx' can build:
	  1. Verifies the .NET SDK is available.
	  2. Installs (or updates) the 'dotnet-script' global tool and ensures the global
	     tools directory is on PATH for the current session.

	The external SDK dependencies are NOT fetched here - they are pulled lazily. Each
	native project's pre-build step runs the relevant 'sdk/<name>/_get.csx' via
	dotnet-script if its SDK is missing, and 'Build.csx' pre-fetches the core SDKs.
	This script just makes 'dotnet-script' available so those steps can run.

.EXAMPLE
	pwsh -File .\script\Setup.ps1
	Installs the dotnet-script global tool (if missing) and confirms it is on PATH.
#>
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

function Write-Step($msg) { Write-Host "==> $msg" -ForegroundColor Cyan }
function Write-Ok($msg)   { Write-Host "    $msg" -ForegroundColor Green }
function Write-Note($msg) { Write-Host "    $msg" -ForegroundColor DarkGray }

# ---------------------------------------------------------------------------
# Step 1: Verify the .NET SDK is installed
# ---------------------------------------------------------------------------
Write-Step 'Checking for the .NET SDK...'
$dotnet = Get-Command dotnet -ErrorAction SilentlyContinue
if ($null -eq $dotnet)
{
	throw "The .NET SDK was not found on PATH. Install it from https://dotnet.microsoft.com/download and re-run this script."
}
$dotnetVersion = (& dotnet --version).Trim()
Write-Ok ".NET SDK $dotnetVersion"

# ---------------------------------------------------------------------------
# Step 2: Ensure the global tools directory is on PATH, then install dotnet-script
# ---------------------------------------------------------------------------
# Global tools install to %USERPROFILE%\.dotnet\tools. On a fresh machine this is
# often not on PATH until the shell is restarted, so add it for this session.
$toolsDir = Join-Path $env:USERPROFILE '.dotnet\tools'
if ($env:PATH -notlike "*$toolsDir*")
{
	$env:PATH = "$toolsDir;$env:PATH"
	Write-Note "Added $toolsDir to PATH for this session."
}

Write-Step 'Ensuring the dotnet-script tool is installed...'
$hasDotnetScript = $null -ne (Get-Command dotnet-script -ErrorAction SilentlyContinue)
if (-not $hasDotnetScript)
{
	Write-Note 'Installing dotnet-script (global tool)...'
	& dotnet tool install -g dotnet-script
	if ($LASTEXITCODE -ne 0)
	{
		throw "Failed to install the dotnet-script global tool (exit code $LASTEXITCODE)."
	}
}
else
{
	# Best-effort update; ignore failures (e.g. offline) since a working tool already exists.
	Write-Note 'dotnet-script already installed; attempting update...'
	& dotnet tool update -g dotnet-script *> $null
}

# Confirm it is now resolvable
if ($null -eq (Get-Command dotnet-script -ErrorAction SilentlyContinue))
{
	throw "dotnet-script is still not on PATH. Open a new terminal (so $toolsDir is picked up) and re-run this script."
}
Write-Ok 'dotnet-script is available.'

Write-Host ''
Write-Step 'Setup complete.'
Write-Ok "Build the solution with:  dotnet-script .\script\Build.csx"
Write-Ok 'or open Rylogic.sln in Visual Studio.'
Write-Note 'SDK dependencies are fetched automatically on first build.'
