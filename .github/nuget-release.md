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

The workflow requests an OIDC token only in the approval-protected publish job and the tag-restricted
signing build job. `NuGet/login@v1` exchanges it for a short-lived API key; no persistent nuget.org
API key is stored in GitHub.

## Code signing

Release binaries (managed DLLs, and native runtime DLLs and tools) are Authenticode-signed with Azure
Artifact Signing (account `rylogic-signing`, certificate profile `rylogic`, in the Rylogic tenant).
`.nupkg` files are not NuGet-signed: Artifact Signing certificates rotate every few days and so cannot
be registered with nuget.org, which adds its own repository signature instead. `Tools.SignBinaries`
in `script/Tools.csx` signs through the Sign CLI (`dotnet tool install -g sign --prerelease`).

CI setup (already done):
- Entra app `rylogic-code-signing-ci` in the Rylogic tenant, with a federated credential for subject
  `repo:psryland/rylogic-code:environment:code-signing` and the *Artifact Signing Certificate Profile
  Signer* role on the signing account. It has no subscription access.
- GitHub environment `code-signing`, limited to `rylogic-v*` tags, with environment variables
  `AZURE_CODESIGN_CLIENT_ID` and `AZURE_CODESIGN_TENANT_ID`.

A signing failure fails the CI build. Local builds sign when the Azure CLI is logged in to the Rylogic
tenant and otherwise warn and continue unsigned. To keep a separate Azure CLI profile for signing, log
in with `$env:AZURE_CONFIG_DIR = '<dir>'; az login --tenant <rylogic-tenant>` and set
`"CodeSign_AzureConfigDir": "<dir>"` in `script/UserVars.json`.

## Publishing a release

1. Set `RylogicLibraryVersion` in `Directory.Build.props` to a stable three-part version.
2. Merge the release changes and ensure the normal build workflows pass.
3. Create and push the matching product-prefixed version tag, such as `rylogic-v2.1.3`.
4. Review the package artifact produced by the build job, then approve the `nuget-release`
   environment deployment.

The workflow first builds the tagged native package into its private local feed so Debug managed
projects restore the same unpublished version, then rebuilds every package from the tagged commit,
validates the exact archive inventory, requires both Debug and Release native link assets, restores
representative .NET 10 and .NET Framework 4.8.1 consumers, and publishes without `--skip-duplicate`.
Reusing an existing version therefore fails instead of silently succeeding.

## Package sources

The repository `nuget.config` defines Rylogic's local package directories and the protected Microsoft
NuGet proxy (`MicrosoftProtectedNuGet`). Managed Microsoft PCs should also disable direct nuget.org
access at user scope. GitHub-hosted workflows replace the inherited configuration with nuget.org as
their sole source. Local Debug development packages are described in
[`docs/local-dev-packages.md`](../docs/local-dev-packages.md).

## Adding or retiring an assembly

Every canonical `projects\rylogic\<Name>\<Name>.csproj` must explicitly set `IsPackable`.
For a public package, set it to `true` and add a `RylogicAssembly` builder in `script/Build.csx`.
For a retained internal or deprecated assembly, set it to `false`. `AllRylogic` rejects missing or
contradictory classifications, and the workflow discovers the resulting package set from the
validated release manifest rather than a hardcoded package count.
