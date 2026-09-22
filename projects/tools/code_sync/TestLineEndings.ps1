param(
	[Parameter(Mandatory)][string]$Executable,
	[string]$OutputDirectory = (Join-Path ([IO.Path]::GetTempPath()) ("code-sync-line-endings-" + [guid]::NewGuid().ToString('N')))
)
$ErrorActionPreference = 'Stop'
$exe = (Resolve-Path -LiteralPath $Executable).Path
if (Test-Path -LiteralPath $OutputDirectory) { throw 'Use a new test output directory.' }
$root = [IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $root | Out-Null
$utf8 = [Text.UTF8Encoding]::new($false)

# Use one LF truth and references with distinct destination formatting contracts.
$truth = "// PR_CODE_SYNC_BEGIN(line_ending_test, source_of_truth)`nint current_value;`n// PR_CODE_SYNC_END()`n"
$crlf_reference = "prefix  `r`n// PR_CODE_SYNC_BEGIN(line_ending_test)`r`nint old_value;`r`n// PR_CODE_SYNC_END()`r`nsuffix`t `r`n"
$lf_reference = "prefix  `n// PR_CODE_SYNC_BEGIN(line_ending_test)`nint old_value;`n// PR_CODE_SYNC_END()`nsuffix`t "
[IO.File]::WriteAllText((Join-Path $root 'truth.h'), $truth, $utf8)
[IO.File]::WriteAllText((Join-Path $root 'crlf.h'), $crlf_reference, $utf8)
[IO.File]::WriteAllText((Join-Path $root 'lf.h'), $lf_reference, $utf8)
[IO.File]::SetLastWriteTimeUtc((Join-Path $root 'crlf.h'), [DateTime]::UtcNow.AddDays(-1))
[IO.File]::SetLastWriteTimeUtc((Join-Path $root 'lf.h'), [DateTime]::UtcNow.AddDays(-1))

# Synchronize both references through the production command surface.
$output = & $exe $root 2>&1 | Out-String
if ($LASTEXITCODE -ne 0) { throw "CodeSync failed: $output" }

# Preserve CRLF and the final newline without changing unrelated trailing whitespace.
$crlf = [IO.File]::ReadAllText((Join-Path $root 'crlf.h'))
if ([regex]::IsMatch($crlf, "(?<!`r)`n") -or [regex]::IsMatch($crlf, "`r(?!`n)")) { throw 'CRLF reference contains normalized or mixed line endings.' }
if (-not $crlf.EndsWith("suffix`t `r`n")) { throw 'CRLF reference lost its final newline or unrelated trailing whitespace.' }
if (-not $crlf.Contains('int current_value;')) { throw 'CRLF reference was not synchronized.' }

# Preserve LF and the absence of a final newline without changing unrelated trailing whitespace.
$lf = [IO.File]::ReadAllText((Join-Path $root 'lf.h'))
if ($lf.Contains("`r")) { throw 'LF reference contains CRLF line endings.' }
if ($lf.EndsWith("`n")) { throw 'LF reference gained a final newline.' }
if (-not $lf.EndsWith("suffix`t ")) { throw 'LF reference lost unrelated trailing whitespace.' }
if (-not $lf.Contains('int current_value;')) { throw 'LF reference was not synchronized.' }

"CodeSync line-ending checks passed. Fixtures: $root"
