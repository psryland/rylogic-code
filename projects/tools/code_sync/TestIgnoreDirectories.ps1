param(
	[Parameter(Mandatory)][string]$Executable,
	[string]$OutputDirectory = (Join-Path ([IO.Path]::GetTempPath()) ("code-sync-ignore-" + [guid]::NewGuid().ToString('N')))
)
$ErrorActionPreference = 'Stop'
$exe = (Resolve-Path -LiteralPath $Executable).Path
if (Test-Path -LiteralPath $OutputDirectory) { throw 'Use a new test output directory.' }
$root = [IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $root | Out-Null
$truth = "// PR_CODE_SYNC_BEGIN(ignore_test, source_of_truth)`nint current_value;`n// PR_CODE_SYNC_END()`n"
$checks = 0

# Check exit status and diagnostics rather than treating any failure as the expected duplicate.
function Invoke-Check([string[]]$Arguments, [int]$ExpectedExit, [string]$ExpectedText)
{
	$output = & $exe @Arguments 2>&1 | Out-String
	$exit_code = $LASTEXITCODE
	if ($exit_code -ne $ExpectedExit -or ($ExpectedText -and -not $output.Contains($ExpectedText)))
	{
		throw "Expected exit $ExpectedExit and '$ExpectedText', got $exit_code`: $output"
	}
	$script:checks++
}

# Duplicate truth blocks make traversal observable without changing any input files.
$cases = @(
	@('nested\obj\snapshot', @('obj', 'bin'), 0),
	@('nested\BIN\snapshot', @('obj', 'bin'), 0),
	@('nested\ObJ', @('OBJ'), 0),
	@('nested\objects', @('obj', 'bin'), 1),
	@('nested\binary', @('obj', 'bin'), 1),
	@('nested\scratch-one', @('scratch-*'), 0),
	@('nested\scratch-', @('scratch-*'), 0),
	@('nested\cache1', @('cache?'), 0),
	@('nested\cache', @('cache?'), 1),
	@('nested\cache12', @('cache?'), 1),
	@('nested\ababa', @('*aba'), 0),
	@('nested\ababa', @('*abb'), 1),
	@('nested\obj', @('**o**?**'), 0),
	@('nested\obj', @(), 1)
)
foreach ($case in $cases)
{
	$scan = Join-Path $root ("case-" + $checks)
	$duplicate_dir = Join-Path $scan $case[0]
	New-Item -ItemType Directory -Path $duplicate_dir -Force | Out-Null
	[IO.File]::WriteAllText((Join-Path $scan 'truth.h'), $truth)
	$duplicate = Join-Path $duplicate_dir 'copy.h'
	[IO.File]::WriteAllText($duplicate, $truth)
	$before = (Get-Item -LiteralPath $duplicate).LastWriteTimeUtc.Ticks
	$arguments = @($scan)
	foreach ($pattern in $case[1]) { $arguments += @('--ignore-dir', $pattern) }
	$expected_text = if ($case[2] -eq 1) { 'Duplicate source_of_truth' } else { '' }
	Invoke-Check $arguments $case[2] $expected_text
	if ([IO.File]::ReadAllText($duplicate) -ne $truth -or (Get-Item -LiteralPath $duplicate).LastWriteTimeUtc.Ticks -ne $before) { throw 'Duplicate fixture changed.' }
}

# Every root receives exclusions, but an explicitly supplied root itself is not ignored.
$first = Join-Path $root 'roots\first'
$second = Join-Path $root 'roots\obj'
New-Item -ItemType Directory -Path (Join-Path $first 'bin'), (Join-Path $second 'obj') -Force | Out-Null
[IO.File]::WriteAllText((Join-Path $first 'truth.h'), $truth)
[IO.File]::WriteAllText((Join-Path $first 'bin\copy.h'), $truth)
[IO.File]::WriteAllText((Join-Path $second 'obj\copy.h'), $truth)
Invoke-Check @($first, $second, '--ignore-dir', 'obj', '--ignore-dir', 'bin') 0 ''
[IO.File]::WriteAllText((Join-Path $second 'truth.h'), $truth)
Invoke-Check @($first, $second, '--ignore-dir', 'obj', '--ignore-dir', 'bin') 1 'Duplicate source_of_truth'

# File names are not directory names, and ordinary source synchronisation still runs.
$sync = Join-Path $root 'sync'
New-Item -ItemType Directory -Path $sync | Out-Null
[IO.File]::WriteAllText((Join-Path $sync 'truth.h'), $truth)
$reference = Join-Path $sync 'obj.cs'
[IO.File]::WriteAllText($reference, "// PR_CODE_SYNC_BEGIN(ignore_test)`nint old_value;`n// PR_CODE_SYNC_END()`n")
[IO.File]::SetLastWriteTimeUtc($reference, [DateTime]::UtcNow.AddDays(-1))
Invoke-Check @($sync, '--ignore-dir', 'obj*', '--ignore-dir', 'bin') 0 ''
if (-not [IO.File]::ReadAllText($reference).Contains('int current_value;')) { throw 'Ordinary reference was not synchronised.' }

# Invalid options must fail even when a fresh stamp would otherwise bypass the scan.
$stamp = Join-Path $root 'fresh.stamp'
[IO.File]::WriteAllText($stamp, 'test stamp')
foreach ($options in @(
	@('--ignore-dir'),
	@('--ignore-dir', '--verbose'),
	@('--ignore-dir', ''),
	@('--ignore-dir', 'obj\child'),
	@('--ignore-dir', 'obj/child'),
	@('--ignore-dir', 'C:obj')
))
{
	Invoke-Check (@($sync, '--stamp', $stamp) + $options) 1 '--ignore-dir requires'
}
Invoke-Check @($sync, '--ignore-dri', 'obj') 1 'Unknown or incomplete option'
"CodeSync directory-ignore checks passed: $checks. Fixtures: $root"
