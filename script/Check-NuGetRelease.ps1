# Reject release commits outside GitHub main and open pull requests targeting main.
$ErrorActionPreference = 'Stop'
$repository = 'psryland/rylogic-code'

# The tagged commit must remain on main, but main can advance during deployment approval.
$commit = & git -C (Split-Path -Parent $PSScriptRoot) rev-parse HEAD
if ($LASTEXITCODE -ne 0)
{
	throw "Could not resolve the release commit (exit $LASTEXITCODE)."
}
$status = & gh api --hostname github.com "repos/$repository/compare/$commit...main" --jq '.status'
if ($LASTEXITCODE -ne 0 -or $status -cnotin @('ahead', 'identical'))
{
	throw "Could not verify release commit '$commit' is on '$repository' main (comparison: '$status')."
}

# Include drafts and all result pages. Fail closed if GitHub cannot report the PR state.
$open_prs = @(& gh api --hostname github.com --paginate "repos/$repository/pulls?state=open&base=main&per_page=100" --jq '.[] | "#\(.number) \(.html_url)"')
if ($LASTEXITCODE -ne 0)
{
	throw "Could not check open pull requests targeting '$repository' main (exit $LASTEXITCODE)."
}
if ($open_prs.Count -ne 0)
{
	throw "Open pull requests targeting main block this release:`n$($open_prs -join "`n")"
}
