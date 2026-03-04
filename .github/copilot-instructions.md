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

### P/Invoke Pattern
Win32 interop in `Rylogic.Core/src/Win32/` uses a trailing-underscore convention:
```csharp
public static bool AllocConsole() => AllocConsole_();
[DllImport("kernel32.dll", EntryPoint = "AllocConsole", SetLastError = true)]
private static extern bool AllocConsole_();
```
The public method wraps the private `_`-suffixed extern.

### C++ Namespaces
`pr::maths`, `pr::ldraw`, `pr::collision`, etc.

### Target Frameworks
- Primary: `net9.0-windows`
- Legacy: `net481`
- C++ toolset: `v145` (VS 2026), Windows SDK 10.0
- C# nullable references enabled (`<Nullable>Enable</Nullable>`)

## Build System Details

### Prerequisites
- Windows (this project is Windows-only)
- .NET 9 SDK
- `dotnet-script` tool: `dotnet tool install -g dotnet-script`

### Configuration Files
- `Directory.Build.props` — Shared MSBuild properties for all C# and C++ projects
- `Directory.Packages.props` — Centralized NuGet package versions
- `.editorconfig` — Code style and formatting rules
- `/script/UserVars.csx` or `/script/UserVars.json` — Local build path customization

### CI (GitHub Actions)
- **C# builds** (`build-csharp-projects.yml`): Builds all C# projects on `windows-latest`. Removes `.vcxproj` (C++) and VSIX projects since the runner lacks the full VS C++ toolset.
- **Native builds** (`build-native-projects.yml`): Builds C++ projects with MSBuild (v143 toolset on CI runner). Removes C# projects first.
- Both workflows trigger on push/PR to `main` with path filters.
