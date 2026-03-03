# Copilot Instructions for Rylogic Codebase

## Build Commands

### Full Solution Build
```powershell
# Using dotnet-script (requires: dotnet tool install -g dotnet-script)
dotnet-script ./script/Build.csx

# Or open Rylogic.sln in Visual Studio
```

### Building Specific Projects
```powershell
# Build specific project(s) via the build script
dotnet-script ./script/Build.csx -project <ProjectName> -build

# Available projects: Sqlite3, Scintilla, Audio, Fbx, View3d, P3d, RylogicCore, RylogicDB,
#   RylogicDirectShow, RylogicGfx, RylogicGuiWPF, RylogicNet, RylogicScintilla, RylogicWindows,
#   Csex, LDraw, RyLogViewer, RylogicTextAligner, AllNative, AllManaged, AllRylogic, All

# Build with specific platform and configuration
dotnet-script ./script/Build.csx -project View3d -platform x64 -config Release -build

# Rebuild (clean + build)
dotnet-script ./script/Build.csx -project RylogicCore -rebuild

# Build a single C# project directly with dotnet CLI
dotnet build projects/rylogic/Rylogic.Core/Rylogic.Core.csproj
```

### Deploy and Publish
```powershell
# Build and deploy to /lib folder
dotnet-script ./script/Build.csx -project AllNative -build -deploy

# Publish to NuGet
dotnet-script ./script/Build.csx -project RylogicCore -deploy -publish
```

## Testing

### C# Unit Tests
Tests are **embedded inline in source files** (not in separate test files), guarded by `#if PR_UNITTESTS`. Each assembly includes a `Program.Main()` entry point that discovers and runs all embedded tests. Tests run automatically as a post-build step in Debug configuration.
```csharp
// At the bottom of a source file, e.g. Rylogic.Core/src/Common/Base64.cs:
#if PR_UNITTESTS
[TestFixture] public class TestBase64
{
    [Test] public void Base64()
    {
        // assertions...
    }
}
#endif
```
Run tests via:
```powershell
# Run all tests in a compiled assembly
dotnet-script ./script/RunUnitTests.csx <TargetPath> <is_managed> [dependencies...]

# Example:
dotnet-script ./script/RunUnitTests.csx "projects/rylogic/Rylogic.Core/bin/Debug/net9.0-windows/Rylogic.Core.dll" true

# Build + run tests in one step (tests auto-run on Debug build)
dotnet build projects/rylogic/Rylogic.Core/Rylogic.Core.csproj -c Debug
```
Individual test filtering is not supported — tests run as a batch per assembly.

### C++ Unit Tests
Tests use a lightweight custom framework (`include/pr/common/unittests.h`), also inline:
```cpp
#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::unittests
{
    PRUnitTest(TestName)
    {
        PR_EXPECT(1 + 1 == 2);
        PR_THROWS(bad_call(), std::exception);
    }
}
#endif
```
Compile with `/DPR_UNITTESTS=1`. The `projects/tests/unittests/` project includes all test headers.

### Python Tests (py-rylogic)
```powershell
cd projects/rylogic/py-rylogic
pip install .[dev]
pytest                    # Run all tests
pytest tests/test_file.py # Run single test file
```

### TypeScript (VS Code Extensions)
```powershell
cd typescript/projects/Rylogic.TextAligner
npm install && npm run compile
```

## Tools

### CEX — Console Extensions (`projects/tools/cex`)
The `cex` utility provides GUI automation commands that Copilot can use to interact with and verify Windows GUI applications. The tool is a `/SUBSYSTEM:Windows` app (no console window). Build it with:
```powershell
& "C:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe" `
    projects/tools/cex/cex.vcxproj /p:Configuration=Release /p:Platform=x64 /nologo /verbosity:minimal
```
The built executable is at `projects/tools/cex/obj/x64/Release/cex.exe`. A deployed copy is at `C:\Tools\cex\cex.exe`.

**To discover all available commands and their options**, run:
```powershell
Start-Process "C:\Tools\cex\cex.exe" -ArgumentList "-rtfm" -RedirectStandardOutput "dump\cex-manual.md" -Wait -NoNewWindow
```
Then read `dump\cex-manual.md` for complete markdown documentation of every command.

Because cex is a GUI-subsystem app, always invoke it via `Start-Process` with `-RedirectStandardOutput` to capture output:
```powershell
Start-Process -FilePath ".\cex.exe" -ArgumentList "<args>" -NoNewWindow -Wait -PassThru `
    -RedirectStandardOutput "out.txt" -RedirectStandardError "err.txt"
```

#### Screenshot — Capture Window Images
```powershell
# Capture all visible windows of a process to PNG files
cex -screenshot -p <process-name> -o <output-directory>

# Use -bitblt for GPU-rendered apps (Electron, Chromium) — captures from screen DC
cex -screenshot -p <process-name> -o <output-directory> -bitblt

# Use -all to include hidden/minimised windows
cex -screenshot -p <process-name> -o <output-directory> -all

# Use -scale to reduce image size (e.g. 0.25 for quarter size, saves tokens for AI input)
cex -screenshot -p <process-name> -o <output-directory> -scale 0.25
```
Output files are named `<process>.<window-title>.png`. Use the `view` tool on the PNG to visually inspect the result.

#### Send Keys — Keyboard Input Simulation
```powershell
# Type text into a GUI application (uses SendInput with KEYEVENTF_UNICODE)
cex -send_keys "text to type" -p <process-name>

# Target a specific window (default: largest window)
cex -send_keys "text" -p <process-name> -w <window-name>

# Control typing speed (default: 10 keys/sec)
cex -send_keys "text" -p <process-name> -rate 20
```
Characters are sent as raw unicode — this does not parse key-combo escape sequences like `{CTRL+A}`.

#### Send Mouse — Mouse Input Simulation
```powershell
# Send mouse events to a GUI application (coordinates are client-area relative)
cex -send_mouse 100,100 -b LeftClick  -p <process-name>
cex -send_mouse 100,100 -b RightClick -p <process-name>

# Drag sequence: button down, move, button up
cex -send_mouse 100,100 -b LeftDown -p <process-name>
cex -send_mouse 200,200 -b Move     -p <process-name>
cex -send_mouse 200,200 -b LeftUp   -p <process-name>

# Target a specific window (default: largest window)
cex -send_mouse 100,100 -b LeftClick -p <process-name> -w <window-name>
```

#### Visual Verification Workflow
Copilot can close the loop on GUI automation by capturing and viewing screenshots:
1. Perform an action (e.g. `send_keys` to type into Notepad)
2. Capture a screenshot of the target window (`-screenshot`)
3. View the PNG with the `view` tool to visually confirm the result

This is useful for debugging GUI interactions without requiring the user to manually verify.

#### Automate — Scripted Mouse/Keyboard Commands
```powershell
# Execute a script of commands from a file
cex -automate -p <process-name> -f <script-file>

# Or pipe from stdin
echo "key ctrl+a" | cex -automate -p <process-name>
```
Script commands (one per line, `#` comments):
- **Mouse:** `move x,y`, `click x,y [button]`, `down x,y [button]`, `up [button]`, `drag x1,y1 x2,y2 [N]`
- **Drawing:** `line x1,y1 x2,y2`, `circle cx,cy r [N]`, `arc cx,cy r a0 a1 [N]`, `fill_circle cx,cy r [N]`
- **Keyboard:** `type text...`, `key combo` (e.g. `key ctrl+a`, `key shift+delete`, `key f5`)
- **Timing:** `delay ms`

All coordinates are client-area relative. Angles are in degrees.

#### Shutdown Process — Graceful Close
```powershell
# Gracefully close a process by sending WM_CLOSE
cex -shutdown_process -p <process-name>

# Wait up to 10 seconds for exit (default: 5000ms)
cex -shutdown_process -p <process-name> -timeout 10000
```

#### List Windows — Enumerate Process Windows
```powershell
# List all visible windows of a process (HWND, size, title)
cex -list_windows -p <process-name>

# Include hidden/minimised windows
cex -list_windows -p <process-name> -all
```

#### Read Text — UI Automation Text Extraction
```powershell
# Read text content from a window's UI element tree
cex -read_text -p <process-name>

# Limit tree depth (default: 5)
cex -read_text -p <process-name> -depth 3
```
Outputs the element tree with control types, names, and text values. Useful for reading dialog messages, button labels, and text fields without screenshots.

#### Find Element — Locate UI Elements by Name
```powershell
# Find a UI element and get its bounding rectangle in client coordinates
cex -find_element -name "OK" -p <process-name>

# Search deeper (default depth: 8)
cex -find_element -name "Save" -p <process-name> -depth 12
```
Returns the element's control type, name, client-area position, size, and center point. Use the center coordinates with `send_mouse` or `automate` to click elements by name.

#### Wait Window — Wait for Window to Appear
```powershell
# Wait for a window to appear (default timeout: 30s)
cex -wait_window -p <process-name>

# Wait for a specific window title
cex -wait_window -p <process-name> -w "Save As" -timeout 5000
```

## Architecture

### Language Mix
- **C++**: Native libraries in `/include/pr/` (header-heavy, template-based, C++20)
- **C#/.NET**: Managed assemblies in `/projects/rylogic/` targeting `net9.0-windows` and `net481`
- **TypeScript**: VS Code extensions in `/typescript/projects/`
- **Python**: Python bindings in `/projects/rylogic/py-rylogic/`

### Core .NET Libraries Hierarchy
```
Rylogic.Core      → Base library (extensions, math, containers, utilities)
Rylogic.Windows   → Windows-specific helpers (depends on Core)
Rylogic.Gui.WPF   → WPF UI components (depends on Core, Windows)
Rylogic.Gfx       → Graphics/View3d wrapper (depends on Core, Native DLLs)
Rylogic.Scintilla → Scintilla editor wrapper (depends on Core, Gui.WPF, Windows)
```

### Native DLLs
- `view3d-12.dll` - DirectX 12 3D graphics engine (the main native library)
- `audio.dll`, `fbx.dll`, `gltf.dll` - Supporting native libraries
- Built via MSBuild (`.vcxproj`), deployed to `/lib/{platform}/{config}/`

### C++ Header Library
All C++ code lives under `/include/pr/` as a header-heavy library. All `#include` paths are relative to `/include/`:
```cpp
#include "pr/common/assert.h"
#include "pr/maths/maths.h"
#include "pr/view3d-12/view3d-dll.h"
```

### C++ forward.h Convention
Each C++ library has a `forward.h` that acts as a **pseudo-precompiled header**. It includes all external/foreign dependencies, forward declarations, type aliases, and configuration for that library. **All other headers within a library only include sibling headers** — never headers from outside the library directly. This keeps compile-time dependencies manageable.
```
include/pr/maths/forward.h     → STL, concepts, forward decls for all math types
include/pr/view3d-12/forward.h → STL, DirectX, 200+ forward decls for rendering types
include/pr/physics2/forward.h  → STL, spatial algebra types, forward decls
```

### C++ Library Dependency Layers
```
Layer 0: common, macros, meta, str, container (no pr/ dependencies)
Layer 1: maths (depends on common)
Layer 2: collision, geometry, camera (depend on maths)
Layer 3: physics2 (depends on collision + maths)
Layer 4: view3d-12 (depends on most lower layers — camera, geometry, gfx, maths, etc.)
```

## Conventions

### Naming (applies to both C++ and C#)
- **Classes, methods, properties, enums**: `PascalCase`
- **Private/internal fields**: `m_field_name` (lowercase with underscores, `m_` prefix)
- **Local variables**: `snake_case`
- **Interfaces**: `I` prefix (e.g., `IMyInterface`)
- **No camelCase** — this is a strong preference throughout the codebase
- C++ namespaces: `pr::maths`, `pr::ldraw`, `pr::collision`, etc.

### Coding Style
- **Tabs** for indentation in both C++ and C#
- **Allman braces** (opening brace on its own line) in C#
- Prefer `var` in C# and `auto` in C++ unless the type change is deliberate
- **East const** (postfix): `Type const&` not `const Type&`
- **For loops**: use `!=` and pre-increment: `for (int i = 0; i != 10; ++i)` — `!=` is less defensive, `++i` is more optimal for non-trivial iterators
- File-scoped namespaces in C#
- Comments should explain "why", not "what" — add a blank line before comment blocks
- C# nullable references enabled (`<Nullable>Enable</Nullable>`)

### P/Invoke Pattern
Win32 interop in `Rylogic.Core/src/Win32/` uses a trailing-underscore convention:
```csharp
public static bool AllocConsole() => AllocConsole_();
[DllImport("kernel32.dll", EntryPoint = "AllocConsole", SetLastError = true)]
private static extern bool AllocConsole_();
```
The public method wraps the private `_`-suffixed extern.

### `@copilot` Comments
Any comment containing `@copilot` is an instruction, question, or context directed at Copilot. Treat these as actionable input during sessions.

### Target Frameworks
- Primary: `net9.0-windows`
- Legacy: `net481`
- C++ toolset: `v145` (VS 2026), Windows SDK 10.0

## Build System Details

### Prerequisites
- Windows (this project is Windows-only)
- Visual Studio 2026 (version 18) with C++ workload (for native projects) — v145 toolset
- Visual Studio 2022 also installed (v143 toolset available but not preferred)
- .NET 9 SDK
- `dotnet-script` tool: `dotnet tool install -g dotnet-script`

### MSBuild Path
Always use the VS 2026 MSBuild for native C++ builds:
```powershell
$msbuild = "C:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe"
& $msbuild <project.vcxproj> /p:Configuration=Release /p:Platform=x64 /nologo /verbosity:minimal
```
Do **not** use the VS 2022 MSBuild (`C:\Program Files\Microsoft Visual Studio\2022\...`) — all native libraries in `/lib/` are built with the v145 toolset and linking against them with v143 causes `LNK1047`/`C1047` errors.

### Configuration Files
- `Directory.Build.props` — Shared MSBuild properties for all C# and C++ projects
- `Directory.Packages.props` — Centralized NuGet package versions
- `.editorconfig` — Code style and formatting rules
- `/script/UserVars.csx` or `/script/UserVars.json` — Local build path customization

### CI (GitHub Actions)
- **C# builds** (`build-csharp-projects.yml`): Builds all C# projects on `windows-latest`. Removes `.vcxproj` (C++) and VSIX projects since the runner lacks the full VS C++ toolset.
- **Native builds** (`build-native-projects.yml`): Builds C++ projects with MSBuild (v143 toolset on CI runner). Removes C# projects first.
- Both workflows trigger on push/PR to `main` with path filters.
