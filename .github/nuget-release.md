# NuGet release setup

Rylogic packages are built from an immutable version tag and published by
`.github/workflows/publish-nuget.yml`. Local publishing is intentionally disabled.

## One-time repository setup

1. Create the GitHub environment `nuget-release` and configure at least one required reviewer.
2. Add the repository Actions variable `NUGET_USER` containing the nuget.org account username.
3. In the nuget.org account, create a Trusted Publishing policy with:
   - repository owner: `psryland`
   - repository: `rylogic-code`
   - workflow: `publish-nuget.yml`
   - environment: `nuget-release`

The workflow requests an OIDC token only in the approval-protected publish job. `NuGet/login@v1`
exchanges it for a short-lived API key; no persistent nuget.org API key is stored in GitHub.

## Publishing a release

1. Set `RylogicLibraryVersion` in `Directory.Build.props` to a stable three-part version.
2. Merge the release changes and ensure the normal build workflows pass.
3. Create and push the matching product-prefixed version tag, such as `rylogic-v2.1.3`.
4. Review the package artifact produced by the build job, then approve the `nuget-release`
   environment deployment.

The workflow rebuilds every package from the tagged commit, validates the exact archive inventory,
restores representative .NET 10 and .NET Framework 4.8.1 consumers, and publishes without
`--skip-duplicate`. Reusing an existing version therefore fails instead of silently succeeding.
Packages are currently unsigned.

## Package sources

The repository `nuget.config` defines only Rylogic's local package directories and inherits its
public package source from user configuration. Managed Microsoft PCs should enable the protected
NuGet proxy and disable direct nuget.org access at user scope; unmanaged PCs may use nuget.org.
GitHub-hosted workflows replace the inherited configuration with nuget.org as their sole source.

## Adding or retiring an assembly

Every canonical `projects\rylogic\<Name>\<Name>.csproj` must explicitly set `IsPackable`.
For a public package, set it to `true` and add a `RylogicAssembly` builder in `script/Build.csx`.
For a retained internal or deprecated assembly, set it to `false`. `AllRylogic` rejects missing or
contradictory classifications, and the workflow discovers the resulting package set from the
validated release manifest rather than a hardcoded package count.
