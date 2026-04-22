# LDraw Visualiser — VSIX Development Setup

## Iterative Development via Post-Build Deploy

Instead of reinstalling the `.vsix` for every change, the project includes a post-build
target (`DeployToMainVS`) that copies debug binaries directly over the installed extension.

### Setup (one-time)

1. **Build the project** in Debug to produce the `.vsix`:
   - Build from Visual Studio, or:
   ```powershell
   dotnet build projects/apps/Rylogic.LDrawVisualiser/Rylogic.LDrawVisualiser.csproj -c Debug
   ```

2. **Install the `.vsix`** into your main VS instance:
   - Double-click `bin\Debug\net481\Rylogic.LDrawVisualiser.vsix`, or
   - Use **Extensions → Manage Extensions → Install from VSIX...**
   - This creates the extension metadata (catalog, manifest, registry entries) that VS needs.

3. **Restart VS.** The extension is now installed and the post-build target will find it.

### Development Workflow

1. **Edit** the extension code.
2. **Build** (Debug) — the `DeployToMainVS` target automatically copies updated DLLs over the
   installed extension. Build output will show:
   ```
   Deployed debug binaries to: C:\Users\...\Extensions\<hash>\
   ```
3. **Restart VS** to pick up the changes.
4. **F5** still works for debugging in the Experimental Instance as usual.

### What Gets Copied

The post-build target copies these assemblies to the installed extension folder:
- `Rylogic.LDrawVisualiser.dll` + `.pdb`
- `Rylogic.Core.dll` + `.pdb`
- `Rylogic.Gfx.dll` + `.pdb`
- `Rylogic.Gui.WPF.dll` + `.pdb`
- `Rylogic.Windows.dll` + `.pdb`

### Notes

- The deploy target only runs in **Debug** configuration.
- It auto-detects the installed extension folder by scanning for the extension GUID in
  non-Experimental VS instances under `%LOCALAPPDATA%\Microsoft\VisualStudio\18.0_*`.
- If the extension is not installed, build output shows a warning — install the `.vsix` first.
- After bumping the version in `source.extension.vsixmanifest`, reinstall the `.vsix` so VS
  updates its metadata. Subsequent builds will resume copying over the new install.
- Works from any workspace (the target uses `$(TargetDir)` relative paths).
