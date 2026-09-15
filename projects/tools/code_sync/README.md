# code_sync

Native code-block synchronisation tool used by `Directory.Build.targets`. The deployed executable is
`E:\Rylogic\Code\tools\code_sync\code_sync.exe`.

## Missing-only Release deployment

When a build needs code_sync and the deployed executable is missing, `EnsureCodeSync` builds and deploys a separate **Release x64** copy before invoking it.
The requesting build remains in its original configuration, including Debug. Building code_sync itself also bootstraps the missing deployment.

If the deployed executable exists, automatic deployment does **nothing**: no byte comparison, freshness check, rebuild of the Release tool, overwrite,
or timestamp change. Building changed Debug or Release sources does not refresh an existing deployed binary. This deliberately leaves tool updates under
manual control rather than repeatedly dirtying a tracked executable.

The bootstrap uses VS 2026 MSBuild with the C++ workload, found through Visual Studio Installer's `vswhere.exe`. `RepositoryToolMSBuild` can specify that
MSBuild executable explicitly. The tool links the static C++ runtime (`/MT` in Release); no app-local runtime DLLs are required.

Bootstrap outputs/intermediates are isolated from ordinary builds. Both CodeSync and recursive deployment hooks are disabled in the bootstrap build.
Parallel requests are serialised, and the deployed executable is published atomically as the completion marker. Failed first deployment never leaves a
partial executable that would make the next build incorrectly skip bootstrap. The same shared task handles VSRedirector and its managed runtime payload.

## Build and manually redeploy

From `E:\Rylogic\Code`:

```powershell
$msbuild = 'C:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe'

# Ordinary development build; an existing deployment is untouched
& $msbuild projects\tools\code_sync\code_sync.vcxproj /p:Configuration=Debug /p:Platform=x64 /nologo /v:minimal

# Explicitly build Release and overwrite the deployed executable
& $msbuild projects\tools\code_sync\code_sync.vcxproj /t:Deploy /p:Configuration=Release /p:Platform=x64 /p:BootstrapRepositoryTools=false /nologo /v:minimal
```

Wait for any active code_sync invocation to finish before manual redeployment. `Deploy` is an explicit target, not an automatic post-build overwrite hook.
The native Release build output is `projects\tools\code_sync\obj\x64\Release\code_sync.exe`.
Building this utility does not invoke CodeSync on the repository's sources.

## Isolated verification

```powershell
pwsh -NoProfile -File build\TestRepositoryToolBootstrap.ps1
```

The script uses a unique temporary deployment root, exercises Debug-to-Release bootstrap, verifies byte-and-timestamp no-op repeats, checks full
VSRedirector runtime deployment, and induces a companion-copy failure before retrying. It also checks the actual consumer hook with a private skip-scan
stamp and explicit native deployment in staging. Live `tools\code_sync` and `tools\VSRedirector` payloads must remain unchanged. Test output is retained for
inspection; use `-OutputDirectory <new-directory>` to select a location.

`RepositoryToolsDirectory` redirects both automatic and native explicit deployment for testing. `BootstrapRepositoryTools=false` disables automatic
bootstrap; if a consumer still needs a missing code_sync, its invocation fails rather than silently ignoring synchronisation.
