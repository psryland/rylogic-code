# LDraw Visualiser — VSIX Development Setup

## Iterative Development via Manual Deploy

To avoid reinstalling the `.vsix` for every change, build the project then run the
`ReplaceLocalInstall.ps1` script to copy the freshly built binaries over an installed
copy of the extension. Deploy is no longer triggered automatically by the build —
it's a deliberate action.

### Setup (one-time)

1. **Build the project** in Debug to produce the `.vsix`:
   - Build from Visual Studio, or:
   ```powershell
   dotnet build projects/apps/Rylogic.LDrawVisualiser/Rylogic.LDrawVisualiser.csproj -c Debug
   ```

2. **Install the `.vsix`** into your main VS instance(s):
   - Double-click `bin\Debug\net481\Rylogic.LDrawVisualiser.vsix`, or
   - Use **Extensions → Manage Extensions → Install from VSIX...**
   - This creates the extension metadata (catalog, manifest, registry entries) that VS needs.
   - The extension is compatible with both VS 2022 (17.x) and VS 2026 (18.x).

3. **Restart VS.** The extension is now installed and `ReplaceLocalInstall.ps1` will find it.

### Development Workflow

1. **Edit** the extension code.
2. **Build** (Debug or Release) — produces fresh DLLs in `bin\<Config>\net481\`.
3. **Close Visual Studio** (otherwise the loaded DLLs are locked).
4. **Run the deploy script:**
   ```powershell
   .\script\ReplaceLocalInstall.ps1
   ```
   You'll be prompted:
   - Which configuration to deploy (`Debug` or `Release`).
   - Which discovered installs to overwrite (multi-select; `A` for all).

   Non-interactive examples:
   ```powershell
   .\script\ReplaceLocalInstall.ps1 -Configuration Debug -All
   .\script\ReplaceLocalInstall.ps1 -Configuration Release
   ```
5. **Restart VS** to pick up the changes.
6. **F5** still works for debugging in the Experimental Instance as usual (Exp hives are
   intentionally skipped by the deploy script — VS manages them itself).

### What Gets Copied

The script copies these assemblies (and their PDBs) from `bin\<Config>\net481\`:
- `Rylogic.LDrawVisualiser.dll`
- `Rylogic.Core.dll`
- `Rylogic.Gfx.dll`
- `Rylogic.Gui.WPF.dll`
- `Rylogic.Windows.dll`

### What Gets Searched

The script looks for installs in:
- Per-user hives: `%LOCALAPPDATA%\Microsoft\VisualStudio\{17.0,18.0}_*\Extensions\`
  (Experimental hives are skipped.)
- Global installs: `C:\Program Files\Microsoft Visual Studio\{2022,18}\{Enterprise,Professional,Community}\Common7\IDE\Extensions\`

### Notes

- After bumping the version in `source.extension.vsixmanifest`, reinstall the `.vsix`
  so VS updates its metadata. Subsequent script runs will resume copying over the new install.
- If the script reports "FAILED — file in use", close VS and retry.
