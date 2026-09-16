param(
	[string]$OutputDirectory = (Join-Path ([IO.Path]::GetTempPath()) ("rylogic-tool-bootstrap-" + [guid]::NewGuid().ToString('N')))
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$vs_project = Join-Path $repo 'projects\tools\VSRedirector\VSRedirector.csproj'
$native_project = Join-Path $repo 'projects\tools\code_sync\code_sync.vcxproj'
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$msbuild = & $vswhere -latest -version '[18.0,19.0)' -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find 'MSBuild\Current\Bin\amd64\MSBuild.exe'
if (-not $msbuild -or -not (Test-Path -LiteralPath $msbuild)) { throw 'VS 2026 C++ MSBuild is required.' }
if (Test-Path -LiteralPath $OutputDirectory) { throw 'Use a new, empty test output directory.' }
$root = [IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $root | Out-Null

# Fail on the real child exit code, including deliberately induced deployment failures.
function Invoke-Checked([string]$Exe, [string[]]$Arguments, [int]$ExpectedExit = 0)
{
	$output = & $Exe @Arguments 2>&1 | Out-String
	$exit_code = $LASTEXITCODE
	$output | Add-Content -LiteralPath (Join-Path $root 'build.log')
	if ($exit_code -ne $ExpectedExit) { throw "Expected exit $ExpectedExit, got $exit_code from $Exe`n$output" }
	return $output
}

# Include timestamps as well as bytes: skipping only the executable is not sufficient.
function Get-Snapshot([string]$Directory)
{
	if (-not (Test-Path -LiteralPath $Directory)) { return '<absent>' }
	return @(Get-ChildItem -LiteralPath $Directory -File -Recurse | Sort-Object FullName | ForEach-Object {
		'{0}|{1}|{2}|{3}' -f $_.FullName.Substring($Directory.Length), (Get-FileHash -LiteralPath $_.FullName).Hash, $_.Length, $_.LastWriteTimeUtc.Ticks
	}) -join "`n"
}

function Get-Configuration([string]$AssemblyPath)
{
	$assembly = [Reflection.Assembly]::Load([IO.File]::ReadAllBytes($AssemblyPath))
	return ($assembly.GetCustomAttributesData() | Where-Object { $_.AttributeType.Name -eq 'AssemblyConfigurationAttribute' }).ConstructorArguments[0].Value
}

$live_vs = Join-Path $repo 'tools\VSRedirector'
$live_native = Join-Path $repo 'tools\code_sync'
$live_vs_before = Get-Snapshot $live_vs
$live_native_before = Get-Snapshot $live_native
$tools = Join-Path $root 'tools with spaces'
$vs_payload = Join-Path $tools 'VSRedirector'
$native_payload = Join-Path $tools 'code_sync'
$property = "-p:RepositoryToolsDirectory=$tools"
$prebuild_check = "-p:DirectoryBuildTargetsPath=$(Join-Path $PSScriptRoot 'VerifyRepositoryToolPrebuild.targets')"

# Missing tools must be ready before the Debug consumer's build events or compiler, even with CodeSync disabled.
$first_build = Invoke-Checked dotnet @('build', $vs_project, '-c', 'Debug', '-t:Rebuild', '--nologo', '-v', 'minimal', $property, $prebuild_check)
if (-not $first_build.Contains('Verified both tools before consumer compilation (Debug).')) { throw 'Managed prebuild assertion did not run.' }
if ((Get-Configuration (Join-Path $repo 'projects\tools\VSRedirector\bin\Debug\net10.0-windows\VSRedirector.dll')) -ne 'Debug') { throw 'Consumer changed configuration.' }
if ((Get-Configuration (Join-Path $vs_payload 'VSRedirector.dll')) -ne 'Release') { throw 'Bootstrap was not Release.' }
foreach ($name in @('VSRedirector.exe', 'VSRedirector.dll', 'VSRedirector.deps.json', 'VSRedirector.runtimeconfig.json'))
{
	if (-not (Test-Path -LiteralPath (Join-Path $vs_payload $name))) { throw "Missing runtime file: $name" }
}
$vs_before = Get-Snapshot $vs_payload
$native_before = Get-Snapshot $native_payload
$repeat = Invoke-Checked dotnet @('build', $vs_project, '-c', 'Debug', '--no-restore', '--nologo', '-v', 'minimal', '-t:Rebuild', $property)
if ($repeat.Contains('Bootstrapping missing') -or (Get-Snapshot $vs_payload) -ne $vs_before -or (Get-Snapshot $native_payload) -ne $native_before) { throw 'Repeat build refreshed a tool payload.' }

# Native Debug and Release builds must also leave both existing deployments alone.
Invoke-Checked (Join-Path $native_payload 'code_sync.exe') @('--help') | Out-Null
Invoke-Checked $msbuild @($native_project, '/p:Configuration=Debug', '/p:Platform=x64', '/nologo', '/v:minimal', $property) | Out-Null
Invoke-Checked $msbuild @($native_project, '/p:Configuration=Release', '/p:Platform=x64', '/nologo', '/v:minimal', $property) | Out-Null
if ((Get-Snapshot $native_payload) -ne $native_before -or (Get-Snapshot $vs_payload) -ne $vs_before) { throw 'Native build refreshed an existing deployment.' }

# A native build from an empty tools root must bootstrap both tools before compilation in desktop MSBuild.
$desktop_tools = Join-Path $root 'desktop host tools'
$native_build = Invoke-Checked $msbuild @($native_project, '/t:Rebuild', '/p:Configuration=Debug', '/p:Platform=x64', "-p:RepositoryToolsDirectory=$desktop_tools", $prebuild_check, '/nologo', '/v:minimal')
if (-not $native_build.Contains('Verified both tools before consumer compilation (Debug).')) { throw 'Native prebuild assertion did not run.' }
if ((Get-Configuration (Join-Path $desktop_tools 'VSRedirector\VSRedirector.dll')) -ne 'Release') { throw 'Native consumer did not bootstrap Release VSRedirector.' }
Invoke-Checked (Join-Path $desktop_tools 'code_sync\code_sync.exe') @('--help') | Out-Null

# Run the consumer hook itself with a future-dated private stamp, so its source scan is guaranteed to skip.
$consumer_tools = Join-Path $root 'consumer tools'
$stamp = Join-Path $root 'skip-source-scan.stamp'
[IO.File]::WriteAllText($stamp, 'test-only skip')
[IO.File]::SetLastWriteTimeUtc($stamp, [DateTime]::UtcNow.AddDays(1))
Invoke-Checked dotnet @('msbuild', $vs_project, '-t:CodeSyncTarget', '-p:Configuration=Debug', '-p:RunCodeSync=true', "-p:CodeSyncStamp=$stamp", "-p:RepositoryToolsDirectory=$consumer_tools", '-nologo', '-v:minimal') | Out-Null
if (-not (Test-Path -LiteralPath (Join-Path $consumer_tools 'code_sync\code_sync.exe'))) { throw 'Consumer hook did not bootstrap its missing tool.' }
if (-not (Test-Path -LiteralPath (Join-Path $consumer_tools 'VSRedirector\VSRedirector.exe'))) { throw 'Consumer hook did not bootstrap VSRedirector.' }

# IDE evaluation and explicitly disabled bootstrap must not launch child builds or deploy tools.
$disabled_tools = Join-Path $root 'disabled tools'
foreach ($mode in @('-p:DesignTimeBuild=true', '-p:BootstrapRepositoryTools=false'))
{
	Invoke-Checked dotnet @('msbuild', $vs_project, '-t:EnsureRepositoryTools', $mode, "-p:RepositoryToolsDirectory=$disabled_tools", '-nologo', '-v:minimal') | Out-Null
	if (Test-Path -LiteralPath $disabled_tools) { throw 'Disabled or design-time bootstrap deployed tools.' }
}

# Explicit native deployment may overwrite the isolated copy, but never either live deployment.
Invoke-Checked $msbuild @($native_project, '/t:Deploy', '/p:Configuration=Release', '/p:Platform=x64', '/p:BootstrapRepositoryTools=false', '/nologo', '/v:minimal', $property) | Out-Null
$release_exe = Join-Path $repo 'projects\tools\code_sync\obj\x64\Release\code_sync.exe'
if ((Get-FileHash -LiteralPath (Join-Path $native_payload 'code_sync.exe')).Hash -ne (Get-FileHash -LiteralPath $release_exe).Hash) { throw 'Explicit native deployment did not copy the built Release executable.' }

# An executable alone is still the no-overwrite gate: do not quietly repair missing companion files.
$sentinel_tools = Join-Path $root 'sentinel tools'
$sentinel_dir = Join-Path $sentinel_tools 'VSRedirector'
New-Item -ItemType Directory -Path $sentinel_dir | Out-Null
[IO.File]::WriteAllText((Join-Path $sentinel_dir 'VSRedirector.exe'), 'existing deployment marker')
New-Item -ItemType Directory -Path (Join-Path $sentinel_tools 'code_sync') | Out-Null
[IO.File]::WriteAllText((Join-Path $sentinel_tools 'code_sync\code_sync.exe'), 'existing deployment marker')
$sentinel_before = Get-Snapshot $sentinel_dir
Invoke-Checked dotnet @('msbuild', $vs_project, '-t:EnsureRepositoryTools', '-p:Configuration=Debug', "-p:RepositoryToolsDirectory=$sentinel_tools", '-nologo', '-v:minimal') | Out-Null
if ((Get-Snapshot $sentinel_dir) -ne $sentinel_before) { throw 'Existing executable did not protect the whole payload.' }

# A failed companion copy must not publish the executable marker; a subsequent retry can complete.
$failure_tools = Join-Path $root 'partial tools'
$failure_dir = Join-Path $failure_tools 'VSRedirector'
New-Item -ItemType Directory -Path $failure_dir | Out-Null
$locked_file = [IO.File]::Open((Join-Path $failure_dir 'VSRedirector.dll'), [IO.FileMode]::Create, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
$bootstrap_args = @('build', $vs_project, '-t:Rebuild', '-c', 'Debug', $prebuild_check, "-p:RepositoryToolsDirectory=$failure_tools", '-nologo', '-v:minimal')
try
{
	$failure = Invoke-Checked dotnet $bootstrap_args 1
	if (-not $failure.Contains('Could not bootstrap VSRedirector') -or -not $failure.Contains('VSRedirector.dll')) { throw "Unexpected failure instead of the locked companion copy: $failure" }
	if (Test-Path -LiteralPath (Join-Path $failure_dir 'VSRedirector.exe')) { throw 'Failed deployment left a success marker.' }
	if ($failure.Contains('Verified both tools before consumer compilation')) { throw 'Consumer proceeded after bootstrap failed.' }
}
finally { $locked_file.Dispose() }
Invoke-Checked dotnet $bootstrap_args | Out-Null
if ((Get-Configuration (Join-Path $failure_dir 'VSRedirector.dll')) -ne 'Release') { throw 'Retry did not deploy the Release payload.' }
if (-not (Test-Path -LiteralPath (Join-Path $failure_dir 'VSRedirector.exe'))) { throw 'Retry did not complete.' }
if (Get-ChildItem -LiteralPath $root -Directory -Recurse -Force | Where-Object Name -Like '*.bootstrap-*') { throw 'Bootstrap scratch directories leaked.' }
if ((Get-Snapshot $live_vs) -ne $live_vs_before -or (Get-Snapshot $live_native) -ne $live_native_before) { throw 'An isolated test modified a live deployment.' }
"All tool bootstrap checks passed. Evidence and isolated payloads: $root"
