param(
	[string]$Exe = "projects\tests\physics-sandbox\obj\x64\Debug\physics-sandbox.exe",
	[string]$Scene = "projects\tests\physics-sandbox\scenes\brick_pyramid.json",
	[int]$Steps = 600,
	[int]$Report = 120,
	[string[]]$Substeps = @("1", "2", "4"),
	[int]$SolverIterations = -1,
	[int]$PositionIterations = -1,
	[double]$MinActivePhysicsMs = 1.0,
	[string]$OutputCsv = "projects\tests\physics-sandbox\dump\pyramid_substep_benchmark.csv"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repo_root = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\..\.."))
Set-Location $repo_root

if (!(Test-Path $Exe))
{
	throw "Physics sandbox executable not found: $Exe. Build projects\tests\physics-sandbox\physics-sandbox.vcxproj first."
}
if (!(Test-Path $Scene))
{
	throw "Scene file not found: $Scene"
}

function Convert-ToDouble
{
	param([string]$Value)
	return [double]::Parse($Value, [Globalization.CultureInfo]::InvariantCulture)
}

function Convert-ToIntList
{
	param([string[]]$Values)

	# powershell.exe -File passes "1,2,4" as one string, while interactive PowerShell can bind @(1,2,4) as many strings.
	# Normalize both forms so benchmark sweeps do not accidentally become one culture-parsed integer like 124.
	$list = @()
	foreach ($value in $Values)
	{
		foreach ($part in $value.Split(",", [System.StringSplitOptions]::RemoveEmptyEntries))
		{
			$list += [int]::Parse($part.Trim(), [Globalization.CultureInfo]::InvariantCulture)
		}
	}
	return $list
}

function Invoke-PyramidMetricRun
{
	param([int]$SubstepCount)

	$log = Join-Path $env:APPDATA "RylogicPhysicsSandbox\scene_diagnostic.log"
	$out = Join-Path $env:TEMP "rylogic_pyramid_${SubstepCount}_$([guid]::NewGuid()).out"
	$err = Join-Path $env:TEMP "rylogic_pyramid_${SubstepCount}_$([guid]::NewGuid()).err"
	Remove-Item $log,$out,$err -ErrorAction SilentlyContinue

	# Use the scene diagnostic pyramid metric so stability is measured as whole-structure lattice preservation.
	$args = @(
		"-scenediag",
		"-scene", $Scene,
		"-steps", "$Steps",
		"-report", "$Report",
		"-pyramid_metric",
		"-substeps", "$SubstepCount"
	)
	if ($SolverIterations -ge 0)
	{
		$args += @("-solver_iterations", "$SolverIterations")
	}
	if ($PositionIterations -ge 0)
	{
		$args += @("-position_iterations", "$PositionIterations")
	}

	$watch = [System.Diagnostics.Stopwatch]::StartNew()
	$process = Start-Process -FilePath $Exe -ArgumentList $args -WorkingDirectory $repo_root -RedirectStandardOutput $out -RedirectStandardError $err -PassThru
	$process.WaitForExit()
	$process.Refresh()
	$watch.Stop()

	if ($null -ne $process.ExitCode -and $process.ExitCode -ne 0)
	{
		$stderr = if (Test-Path $err) { Get-Content $err -Raw } else { "" }
		throw "Pyramid benchmark run failed for substeps=$SubstepCount exit=$($process.ExitCode)`n$stderr"
	}

	# The sandbox is a Windows-subsystem executable, so its diagnostic console output is not reliably captured by Start-Process redirection.
	# The scene diagnostic log is truncated per run by the sandbox and is therefore the stable source for the benchmark rows.
	if (!(Test-Path $log))
	{
		$stderr = if (Test-Path $err) { Get-Content $err -Raw } else { "" }
		throw "Scene diagnostic log was not created for substeps=$SubstepCount`n$stderr"
	}

	# The final pyramid row is the settled stability sample. Cost is taken from the last non-trivial interval, because once the
	# pyramid sleeps the normal engine skip path hides the resolver cost that we want to compare.
	$lines = Get-Content $log
	$pyramid_lines = @($lines | Where-Object { $_ -like "pyramid step=*" })
	if ($pyramid_lines.Count -eq 0)
	{
		throw "No pyramid metric rows found for substeps=$SubstepCount"
	}

	$pattern = "^pyramid step=\s*(?<step>\d+)\s+t=\s*(?<time>[-+0-9.]+)\s+count=(?<count>\d+)\s+rms=\s*(?<rms>[-+0-9.]+)\s+rms_xy=\s*(?<rms_xy>[-+0-9.]+)\s+max_xy=\s*(?<max_xy>[-+0-9.]+)\s+max_drop=\s*(?<max_drop>[-+0-9.]+)\s+top_z=\s*(?<top_z>[-+0-9.]+)\s+top_err=\s*(?<top_err>[-+0-9.]+)\s+fallen=(?<fallen>\d+)\s+sleeping=(?<sleeping>\d+)\s+ke=\s*(?<ke>[-+0-9.]+)\s+pairs=(?<pairs>\d+)\s+contacts=(?<contacts>\d+)\s+physics_ms=\s*(?<physics_ms>[-+0-9.]+)\s+fps=\s*(?<fps>[-+0-9.]+)"
	$rows = @()
	foreach ($line in $pyramid_lines)
	{
		$match = [regex]::Match($line, $pattern)
		if (!$match.Success)
		{
			throw "Could not parse pyramid metric row for substeps=$SubstepCount`n$line"
		}

		$rows += [pscustomobject]@{
			Step = [int]$match.Groups["step"].Value
			TimeS = Convert-ToDouble $match.Groups["time"].Value
			BodyCount = [int]$match.Groups["count"].Value
			RmsPose = Convert-ToDouble $match.Groups["rms"].Value
			RmsXY = Convert-ToDouble $match.Groups["rms_xy"].Value
			MaxXY = Convert-ToDouble $match.Groups["max_xy"].Value
			MaxDrop = Convert-ToDouble $match.Groups["max_drop"].Value
			TopZ = Convert-ToDouble $match.Groups["top_z"].Value
			TopErr = Convert-ToDouble $match.Groups["top_err"].Value
			Fallen = [int]$match.Groups["fallen"].Value
			Sleeping = [int]$match.Groups["sleeping"].Value
			KineticEnergy = Convert-ToDouble $match.Groups["ke"].Value
			Pairs = [int]$match.Groups["pairs"].Value
			Contacts = [int]$match.Groups["contacts"].Value
			PhysicsMs = Convert-ToDouble $match.Groups["physics_ms"].Value
			Fps = Convert-ToDouble $match.Groups["fps"].Value
			RawLine = $line
		}
	}

	$final_row = $rows[-1]
	$cost_rows = @($rows | Where-Object { $_.PhysicsMs -ge $MinActivePhysicsMs })
	$cost_row = if ($cost_rows.Count -ne 0) { $cost_rows[-1] } else { $final_row }

	return [pscustomobject]@{
		Substeps = $SubstepCount
		Step = $final_row.Step
		TimeS = $final_row.TimeS
		BodyCount = $final_row.BodyCount
		RmsPose = $final_row.RmsPose
		RmsXY = $final_row.RmsXY
		MaxXY = $final_row.MaxXY
		MaxDrop = $final_row.MaxDrop
		TopZ = $final_row.TopZ
		TopErr = $final_row.TopErr
		Fallen = $final_row.Fallen
		Sleeping = $final_row.Sleeping
		KineticEnergy = $final_row.KineticEnergy
		Pairs = $final_row.Pairs
		Contacts = $final_row.Contacts
		CostStep = $cost_row.Step
		CostPhysicsMs = $cost_row.PhysicsMs
		CostFps = $cost_row.Fps
		CostContacts = $cost_row.Contacts
		WallSeconds = [math]::Round($watch.Elapsed.TotalSeconds, 3)
		Args = ($args -join " ")
		LogPath = $log
		RawFinalLine = $final_row.RawLine
		RawCostLine = $cost_row.RawLine
	}
}

$substep_counts = Convert-ToIntList $Substeps
$results = @()
foreach ($substep_count in $substep_counts)
{
	$results += Invoke-PyramidMetricRun $substep_count
}

$baseline = $results | Where-Object { $_.Substeps -eq 1 } | Select-Object -First 1
if ($null -eq $baseline)
{
	throw "Substeps must include 1 so the benchmark has a baseline"
}

$summary = foreach ($result in $results)
{
	# Compare every run to substeps=1: lower whole-pyramid RMS pose error is better, lower physics time is cheaper.
	$stability_gain = if ($baseline.RmsPose -ne 0.0) { 1.0 - ($result.RmsPose / $baseline.RmsPose) } else { [double]::NaN }
	$cost_ratio = if ($baseline.CostPhysicsMs -ne 0.0) { $result.CostPhysicsMs / $baseline.CostPhysicsMs } else { [double]::NaN }
	[pscustomobject]@{
		Substeps = $result.Substeps
		FinalStep = $result.Step
		CostStep = $result.CostStep
		RmsPose = $result.RmsPose
		RmsXY = $result.RmsXY
		MaxXY = $result.MaxXY
		MaxDrop = $result.MaxDrop
		TopErr = $result.TopErr
		Fallen = $result.Fallen
		Sleeping = $result.Sleeping
		KE = $result.KineticEnergy
		StabilityGainPct = [math]::Round(100.0 * $stability_gain, 2)
		PhysicsMs = $result.CostPhysicsMs
		Fps = $result.CostFps
		CostRatio = [math]::Round($cost_ratio, 3)
		Contacts = $result.CostContacts
		WallSeconds = $result.WallSeconds
	}
}

$output_dir = Split-Path -Parent $OutputCsv
if ($output_dir)
{
	New-Item -ItemType Directory -Path $output_dir -Force | Out-Null
}

$summary | Export-Csv -Path $OutputCsv -NoTypeInformation
$summary | Format-Table -Property Substeps,FinalStep,CostStep,RmsPose,StabilityGainPct,MaxXY,Fallen,Sleeping,PhysicsMs,Fps,CostRatio -AutoSize
Write-Host ""
Write-Host "Pyramid benchmark CSV: $([System.IO.Path]::GetFullPath($OutputCsv))"
