# code_sync

Native code-block synchronisation tool used by `Directory.Build.targets`. The deployed executable is
`E:\Rylogic\Code\tools\code_sync\code_sync.exe`.

## Directory exclusions

Pass `--ignore-dir <pattern>` repeatedly to skip matching directory subtrees beneath every supplied scan root:

```powershell
& tools\code_sync\code_sync.exe projects include --ignore-dir obj --ignore-dir bin
& tools\code_sync\code_sync.exe projects --ignore-dir 'scratch-*' --ignore-dir 'cache?'
```

Patterns match the whole directory name, case-insensitively, at any depth. `*` matches zero or more characters and `?` matches one character.
Patterns are not paths or `.gitignore` rules: no separators, negation, or character classes are interpreted. Quote wildcard patterns when invoking from a shell.
There are no default exclusions; explicitly supplied scan roots are always scanned, even if their own name matches a pattern. Only child directories are pruned,
not files with matching names. `Directory.Build.targets` passes `obj` and `bin` to exclude generated output trees while leaving names such as `objects` intact.

Existing deployed executables must be manually updated before using the new option; the missing-only prebuild target deliberately does not refresh them.
The older `script\CodeSync.csx` implementation is not used by this build hook and does not accept this option.

After a Debug build, run isolated directory-ignore checks with:

```powershell
pwsh -NoProfile -File projects\tools\code_sync\TestIgnoreDirectories.ps1 -Executable projects\tools\code_sync\obj\x64\Debug\code_sync.exe
```

These cover nested exclusions, case-insensitive wildcard matching, multiple roots, unchanged ignored files, normal reference updates, and invalid arguments.
They do not scan repository sources.

## Missing-only Release deployment

Every normal C# or C++ build runs the shared `EnsureRepositoryTools` prebuild target, which builds and deploys missing **Release** copies of both tools
before build events, source generation, or compilation. code_sync is built as **Release x64**. The requesting build remains in its original configuration,
including Debug. This applies even when CodeSync is disabled and when building either utility itself; IDE design-time builds do not bootstrap tools.
A clean checkout needs both VS 2026 C++ tools and the .NET 10 SDK, even for a C#-only or native-only build. There is no automatic post-build deployment.

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

The script uses a unique temporary deployment root and asserts that both tools exist before managed and native consumer compilation.
It exercises Debug-to-Release bootstrap, verifies byte-and-timestamp no-op repeats, checks full VSRedirector runtime deployment, and induces a
companion-copy failure before retrying. It also checks the actual consumer hook with a private skip-scan
stamp and explicit native deployment in staging. Live `tools\code_sync` and `tools\VSRedirector` payloads must remain unchanged. Test output is retained for
inspection; use `-OutputDirectory <new-directory>` to select a location.

`RepositoryToolsDirectory` redirects both automatic and native explicit deployment for testing. `BootstrapRepositoryTools=false` disables automatic
bootstrap; if a consumer still needs a missing code_sync, its invocation fails rather than silently ignoring synchronisation.
