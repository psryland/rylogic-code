# Checks the release source. Local mode only creates and pushes a tag when explicitly requested.
[CmdletBinding(DefaultParameterSetName = 'Local')]
param(
	[Parameter(ParameterSetName = 'Local')]
	[switch]$PushTag,
	[Parameter(Mandatory, ParameterSetName = 'CI')]
	[switch]$CI
)

$ErrorActionPreference = 'Stop'
$repository = 'psryland/rylogic-code'
$repo_root = Split-Path -Parent $PSScriptRoot

# Read Git state without continuing after a failed command.
function Invoke-ReleaseGit([string]$workspace, [string[]]$arguments)
{
	# Keep errors tied to the checkout whose state could not be established.
	$result = & git -C $workspace @arguments
	if ($LASTEXITCODE -ne 0)
	{
		throw "git $($arguments -join ' ') failed in '$workspace' (exit $LASTEXITCODE)."
	}
	$result
}

# Read every page of open pull requests, including drafts and non-main targets.
function Get-OpenPullRequests
{
	# A failed or unauthorized query must never look like an empty PR list.
	$result = & gh api --hostname github.com --paginate "repos/$repository/pulls?state=open&per_page=100" --jq '.[] | "#\(.number) \(.html_url)"'
	if ($LASTEXITCODE -ne 0)
	{
		throw "Could not check open pull requests for '$repository' (exit $LASTEXITCODE)."
	}
	$result
}

# Confirm the tagged source belongs to main without access to the release operator's PC.
if ($CI)
{
	# The tag need not remain the tip after an approval wait, but it must remain in main's history.
	$commit = Invoke-ReleaseGit $repo_root @('rev-parse', 'HEAD')
	$status = & gh api --hostname github.com "repos/$repository/compare/$commit...main" --jq '.status'
	if ($LASTEXITCODE -ne 0 -or $status -cnotin @('ahead', 'identical'))
	{
		throw "Could not verify release commit '$commit' is on '$repository' main (comparison: '$status')."
	}

	# Re-query GitHub rather than trusting the state seen by the local preflight.
	$open_prs = @(Get-OpenPullRequests)
	if ($open_prs.Count -ne 0)
	{
		throw "Open pull requests block this release:`n$($open_prs -join "`n")"
	}
	return
}

# Discover the numbered sibling checkouts as well as the primary checkout.
$workspace_name = '^rylogic-code[0-9]*$'
if ((Split-Path -Leaf $repo_root) -notmatch $workspace_name)
{
	throw "Run this command from a rylogic-code checkout or one of its numbered siblings."
}
$workspaces = @(Get-ChildItem -LiteralPath (Split-Path -Parent $repo_root) -Directory -Force | Where-Object Name -Match $workspace_name | Sort-Object Name)
$remote_pattern = '^https://github\.com/psryland/rylogic-code(?:\.git)?/?$|^(?:git@github\.com:|ssh://git@github\.com/)psryland/rylogic-code(?:\.git)?$'
$blockers = [System.Collections.Generic.List[string]]::new()

# Use the same current GitHub main commit as the baseline for every checkout.
$main = & gh api --hostname github.com "repos/$repository/git/ref/heads/main" --jq '.object.sha'
if ($LASTEXITCODE -ne 0 -or $main -cnotmatch '^[0-9a-f]{40}$')
{
	throw "Could not resolve '$repository' main."
}

foreach ($workspace in $workspaces)
{
	# Reject a misnamed directory or wrong remote rather than silently skipping a workspace.
	$path = $workspace.FullName
	$top_level = Invoke-ReleaseGit $path @('rev-parse', '--show-toplevel')
	$origin = Invoke-ReleaseGit $path @('remote', 'get-url', 'origin')
	if ([IO.Path]::GetFullPath($top_level) -ne $path -or $origin -notmatch $remote_pattern)
	{
		throw "'$path' is not a '$repository' checkout with the expected origin."
	}

	# Fetch objects without moving local branches. A changing main invalidates the shared baseline.
	Invoke-ReleaseGit $path @('fetch', '--quiet', '--no-tags', '--no-recurse-submodules', 'origin', 'refs/heads/main') | Out-Null
	if ((Invoke-ReleaseGit $path @('rev-parse', 'FETCH_HEAD')) -cne $main)
	{
		throw "GitHub main changed during the preflight or '$path' fetched a different main. Run the preflight again."
	}

	# Report tracked, staged and non-ignored untracked changes, plus commits reachable only from local branches or a detached HEAD.
	$changes = @(Invoke-ReleaseGit $path @('status', '--porcelain=v1', '--untracked-files=all', '--ignore-submodules=none'))
	if ($changes.Count -ne 0)
	{
		$blockers.Add("'$path' has uncommitted changes:`n$($changes -join "`n")")
	}
	$branches = @(Invoke-ReleaseGit $path @('for-each-ref', "--no-merged=$main", '--format=%(refname:short) (%(objectname:short))', 'refs/heads'))
	if ($branches.Count -ne 0)
	{
		$blockers.Add("'$path' has branches not merged into main:`n$($branches -join "`n")")
	}
	$detached_commits = Invoke-ReleaseGit $path @('rev-list', '--count', 'HEAD', '--not', $main, '--branches')
	if ($detached_commits -ne '0')
	{
		$blockers.Add("'$path' has $detached_commits commit(s) not on main or any local branch at a detached HEAD.")
	}
}

# Only the current GitHub main tip may be used to start a new release.
$source = Invoke-ReleaseGit $repo_root @('rev-parse', 'HEAD')
if ($source -cne $main)
{
	$blockers.Add("The release checkout must be at GitHub main ($main); its HEAD is '$source'.")
}
$open_prs = @(Get-OpenPullRequests)
if ($open_prs.Count -ne 0)
{
	$blockers.Add("Open pull requests block this release:`n$($open_prs -join "`n")")
}
if ($blockers.Count -ne 0)
{
	throw "Release preflight failed:`n$($blockers -join "`n`n")"
}

# Derive the tag from committed source so it cannot disagree with the workflow's package version.
[xml]$props = (Invoke-ReleaseGit $repo_root @('show', "${main}:Directory.Build.props")) -join "`n"
$version_nodes = $props.SelectNodes('/Project/PropertyGroup/RylogicLibraryVersion')
if ($version_nodes.Count -ne 1 -or $version_nodes[0].InnerText.Trim() -cnotmatch '^[0-9]+\.[0-9]+\.[0-9]+$')
{
	throw 'Directory.Build.props must define exactly one stable RylogicLibraryVersion.'
}
$tag = "rylogic-v$($version_nodes[0].InnerText.Trim())"
Write-Output "Release preflight passed for $tag at $main across $($workspaces.Count) checkouts."
if ($PushTag)
{
	# Publish the exact checked commit without leaving a local tag if the push fails.
	$push_origin = @(Invoke-ReleaseGit $repo_root @('remote', 'get-url', '--push', '--all', 'origin'))
	if ($push_origin.Count -ne 1 -or $push_origin[0] -notmatch $remote_pattern)
	{
		throw "Origin must have one push URL for '$repository'."
	}
	$existing_tag = Invoke-ReleaseGit $repo_root @('ls-remote', '--refs', $push_origin[0], "refs/tags/$tag")
	if ($existing_tag)
	{
		# An existing tag cannot start a new release.
		throw "Release tag '$tag' already exists on GitHub."
	}
	Invoke-ReleaseGit $repo_root @('push', '--no-follow-tags', 'origin', "${main}:refs/tags/$tag")
}
