# VSRedirector

A one-shot Windows external-editor executable that routes **one existing file** to the right Visual Studio instance. No VS extension, resident service,
automatic elevation, or package dependencies.

## Build and run

From the repository root, with the .NET 10 SDK installed:

```powershell
dotnet build projects\tools\VSRedirector\VSRedirector.csproj -c Debug
```

Debug builds run the inline `PR_UNITTESTS` checks and fail if a check fails. The tool's build disables repository-wide CodeSync so building this tool does not
modify unrelated sources. It is intentionally built directly, without adding it to the main solution.

Stable deployed executable for GitKraken:

```text
tools\VSRedirector\VSRedirector.exe
```

Every normal C# or C++ build runs the shared `EnsureRepositoryTools` prebuild target, which builds and deploys missing **Release** copies of both
VSRedirector and code_sync before build events, source generation, or compilation. This also applies when CodeSync is disabled, but not during IDE
design-time builds. A clean checkout therefore needs both the .NET 10 SDK and VS 2026 C++ tools even for a C# build.
The requesting build keeps its own configuration. If the executable already exists, automatic deployment does nothing to the **entire** payload: no Release rebuild,
companion-file refresh, timestamp touch, or freshness/version check. Missing companions in an existing deployment require explicit manual redeployment.

Bootstrap builds use isolated output/intermediate directories, disable recursive bootstrap and CodeSync, and publish the executable only after all runtime
files are ready. Parallel bootstrap requests are serialised. There is no automatic post-build deployment.
Keep the executable with its adjacent `.dll`, `.deps.json`, and `.runtimeconfig.json` files. Running requires the .NET 10 runtime; it is not self-contained.

Manual redeployment, after any running VSRedirector invocation has finished:

```powershell
dotnet publish projects\tools\VSRedirector\VSRedirector.csproj -c Release -f net10.0-windows -o tools\VSRedirector -p:BootstrapRepositoryTools=false
```

This explicit command refreshes the runtime payload even when the deployed executable exists. No GitKraken preferences are changed.
For isolated deployment checks, override `RepositoryToolsDirectory` with another absolute directory. `BootstrapRepositoryTools=false` disables automatic
bootstrap without hiding deployed binaries from Git. The development Debug executable remains under `projects\tools\VSRedirector\bin\Debug\net10.0-windows`.

```powershell
# Plain file open
& .\tools\VSRedirector\VSRedirector.exe 'E:\src with spaces\file.cs'

# Devenv-compatible open-at-line input (one-based line number)
& .\tools\VSRedirector\VSRedirector.exe /Edit 'E:\src with spaces\file.cs' /Command 'Edit.Goto 42'

# Read-only routing report: never opens, activates, navigates, or starts Visual Studio
dotnet .\tools\VSRedirector\VSRedirector.dll --inspect 'E:\src with spaces\file.cs' /Command 'Edit.Goto 42'

# Run the Debug checks explicitly
dotnet .\projects\tools\VSRedirector\bin\Debug\net10.0-windows\VSRedirector.dll --self-test

# Isolated missing-only bootstrap checks for both tools (retains logs and test payloads in a unique temporary directory)
pwsh -NoProfile -File build\TestRepositoryToolBootstrap.ps1
```

`/Edit` is optional input syntax only: it **does not** enable devenv's instance-selection behavior. `/Command` accepts only `Edit.Goto` and a positive
32-bit line number; arbitrary commands, multiple files, folders, and invented `/line` options are rejected. Switches are case-insensitive and may use
`-Edit` / `-Command` instead. Line navigation opens the text view and calls `TextSelection.GotoLine(line, false)` on the returned document, in both reused and
new instances. Requests past the end of the live editor buffer fail with the requested line and available line count; the opened file is not silently
redirected to another line. Unsupported text views also fail explicitly. Devenv compatibility here describes the input syntax, not every native
`Edit.Goto` edge case: the linked `/Command` and Go To documentation do not specify out-of-range behavior.

This is a GUI-subsystem executable, so GitKraken does not get a flashing console. Failures return nonzero and show a message box. Use `dotnet ...dll --inspect`
for terminal diagnostics with captured stdout/stderr and no message box.

## GitKraken setup

In **Preferences > External Tools > External Editor**, select **`<Custom>`**, then choose the executable path above. Select the executable itself, not
`dotnet.exe`, the DLL, or a command string. Use the file's **Edit This File** action, not **Open Repo in External Editor** (the latter supplies a folder).
No GitKraken preferences are changed by building or running this tool.

The installed GitKraken UI's `<Custom>` option and the official documentation's **custom path** setting have been checked. No supported external-editor
file/line placeholder syntax has been established, so do **not** append guessed `{file}`, `%l`, `$LINE`, or `/Command` text to the executable path.
The **CLI arguments** field under **Coding Agent** belongs to coding agents, not the external editor.
The custom editor's settings were not selected/changed during inspection, and GitKraken-to-executable argument delivery remains a live verification step.
Open-at-line is available through the direct command shown above; it does not imply that GitKraken forwards its diff cursor line to custom editors.

## Routing

1. Prefer an instance whose loaded solution has the exact file in an actual project, even if another instance already has it open as a loose document.
2. Among containing solutions, prefer those with the exact document already open. Resolve any remaining tie using the topmost matching VS window.
3. If no loaded solution contains the file, reuse an instance with that exact document open, with the same window-order tie-break.
4. Otherwise discover the newest complete, launchable, **stable** Visual Studio IDE with Visual Studio Installer's `vswhere.exe`, and start a **new** instance
   opening just the file. Preview releases and Build Tools are excluded. Numeric installation versions are compared; no VS release is hardcoded.

The approved window stacking-order tie-break is **approximate recency**, not historical most-recent activation. Owned popups count with their main window.
Windows allows z-order changes without activation; process start time, PID, and ROT enumeration order are not recency measures.
A tied candidate with no visible window rank produces an error rather than an arbitrary choice.

Identity uses normalized, case-insensitive Windows full paths, not basenames, repository names, directory proximity, or common ancestors. Different clones
remain distinct. Symlinks, junctions, hardlinks, short names, and mapped-drive/UNC aliases are not unified; use consistent path spellings.

Membership uses DTE's loaded-solution lookup, verifies the item's full file names and real project owner, and traverses nested projects/dependent items if a
solution-folder item shadows the actual member. Linked files retain their real full paths. Solution folders, miscellaneous-file projects, and unloaded
projects do not establish membership. An open synthetic solution with no solution path, as used by loose-file windows, does not establish membership either.
The indexed `FileNames` property is read through the declared `EnvDTE.ProjectItem` dispatch interface: managed project systems can expose a different default
dispatch interface from native C++ projects. Project systems that fail DTE inspection produce errors, not guessed matches. Binding defects are reported
separately from busy or access-denied errors. Finish solution/project loading before invoking the redirector.

Reuse calls the selected instance's DTE object directly. Fallback starts the selected `devenv.exe` with no arguments, then binds only that new process's ROT
object and opens the file through DTE. File paths never pass through devenv's command-line parser; `/Edit` cannot redirect the launch to another instance.
Shell execution starts that exact executable without inheriting the worker's result pipes, so the IDE can outlive the redirector without blocking its exit.
If that new instance unexpectedly restores a solution, navigation fails explicitly and leaves it untouched rather than closing user state.
The IDE's startup/project-picker window may also appear according to its existing startup preferences; the redirector does not change those preferences.

## Failure boundaries and verification

All same-Windows-session `devenv` processes must be inspectable before routing. Missing ROT registration, elevation/access denial, unsupported automation,
busy/modal IDEs, and processes appearing/disappearing during discovery are not treated as no-match. Run GitKraken and VS as the same normal user, finish
startup, close modal dialogs, and retry. The redirector does not auto-elevate.

COM runs in a short-lived STA worker. Rejected/busy calls retry for up to three seconds; the supervisor bounds the complete operation at 90 seconds.
`vswhere` has a ten-second timeout and a new IDE has up to 60 seconds to register. Only owned helper processes are terminated on timeout, never VS.
An error after launch or during navigation can leave a new IDE or an already opened file; inspect it before retrying. Windows can restrict foreground focus
even when opening succeeds. No failure triggers a second, speculative launch.

Automated checks cover routing priorities/ties, full-path clone isolation, nested/linked membership and solution-item exclusion, numeric stable-version
selection, Preview/Build Tools exclusion, argument parsing, argument-free IDE startup, and document-specific line navigation (including
`E:\Copilot\copilot-skills\copilot-instructions.md` and paths containing spaces and `&`). A real worker/child-process test checks that both result pipes close
while the launched child remains alive. Read-only discovery has confirmed the
original scenario: the Rylogic solution in VS2026 wins over its file already open loose in VS2022, and an unmatched file selects installed stable VS2026
18.10 rather than VS2022 or Build Tools.

Live verification of the executable has also opened `projects\rylogic\Rylogic.Gfx\src\Program.cs` in the existing VS2026 Rylogic instance and navigated to
line 10, confirmed independently through DTE's active document and `TextSelection.CurrentLine`. The same file contains only 19 lines, so line 42 is
explicitly rejected after the file is opened. File contents, the solution, and the set of VS processes were preserved; no documents or instances were closed.

The deployed executable's fallback has also been verified with `E:\Copilot\copilot-skills\copilot-instructions.md`: it started a new VS2026 process, returned
success with both result pipes closed, and Paul confirmed the file opened correctly before closing the new window. The existing solution instances were
preserved. VS also showed its startup/project picker.

**Live verification remains:** Paul should check GitKraken's Edit This File action after choosing the executable. GitKraken-to-executable argument delivery
has not been verified end-to-end; direct executable tests are not a claim of that integration.

## References

- [GitKraken External Tools preferences](https://help.gitkraken.com/gitkraken-desktop/preferences/#how-external-tool-preferences-work)
- [Devenv command-line switches](https://learn.microsoft.com/en-us/visualstudio/ide/reference/devenv-command-line-switches)
- [Devenv /Edit selects the most recently opened instance](https://learn.microsoft.com/en-us/visualstudio/ide/reference/edit-devenv-exe)
- [Devenv /Command](https://learn.microsoft.com/en-us/visualstudio/ide/reference/command-devenv-exe)
- [File.OpenFile command](https://learn.microsoft.com/en-us/visualstudio/ide/reference/open-file-command)
- [DTE FindProjectItem](https://learn.microsoft.com/en-us/dotnet/api/envdte._solution.findprojectitem)
- [DTE TextSelection.GotoLine](https://learn.microsoft.com/en-us/dotnet/api/envdte.textselection.gotoline)
- [SetWindowPos: activation and z-order are distinct](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-setwindowpos)
