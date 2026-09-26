# Rylogic Lib project organisation

This repository contains the Rylogic Ltd code.

## Project structure

	rylogic_code
	|-- art
	|   |-- icons       - Art assets
	|   |-- pngs        - Art assets
	|   |-- 3d models   - Model assets
	|   |-- etc...
	|
	|-- build           - Project files and property sheets
	|   |-- props       - Property sheets
	|   |-- target      - msbuild target files
	|   |-- etc...      - Miscellaneous files
	|
	|-- include         - Public headers and interfaces. Users of this library should add this directory as an include path.
	|   |-- pr            All rylogic library code uses includes relative to '/rylogic_code/include', e.g. #include "pr/common/..."
	|       |-- common
	|       |-- etc...
	|
	|-- projects        - C/C++/C# Projects
	|   |-- apps        - Applications/ideas some complete, most half baked
	|   |   |-- LDraw
	|   |   |-- RyLogViewer
	|   |   |-- Rylogic.TextAligner
	|   |   |-- etc...
	|   |-- rylogic     - Rylogic core libraries and assemblies
	|   |   |-- Rylogic.Core
	|   |   |-- Rylogic.Gui.WPF
	|   |   |-- view3d
	|   |   |-- etc...
	|   |-- tests       - Unit test projects and other tests
	|   |-- tools       - Helper utility projects
	|
	|-- typescript      - Typescript Projects
	|   |-- Rylogic.TextAligner
	|   |-- etc...
	|
	|-- script            - Scripts used in the build process.
	|   |-- UserVars.csx  - User variables required to build the library.
	|   |-- UserVars.json - Local customisation of user variables.
	|   |-- Tools.csx     - Helper functions
	|   |-- etc...
	|
	|-- sdk             - Third party libraries and source
	|-- tools           - Handy binaries
	|-- miscellaneous   - HowTos, binary templates, licenses, and other random stuff
	|
	|-- obj             - Generated directory containing all object files for native projects
	|-- lib             - Generated directory containing compiled libraries
	|-- bin             - Generated directory containing compiled executables
	|
    |-- Rylogic.sln - "Everything" solution file

## Building

This project is only used on windows. Compiling requires MSBuild, dotnet, dotnet-script.
Follow these steps to build:

- Pull to a clean directory,
- Run the first-run bootstrap: `pwsh -File ./script/Setup.ps1`
  This installs the `dotnet-script` tool (if missing) and ensures it is on PATH.
- Customise _/script/UserVars.csx_ or _/script/UserVars.json_ as needed,
- Run `dotnet-script ./script/Build.csx` to build projects from the command line, or, open _Rylogic.sln_ in Visual Studio.

SDK dependencies are fetched automatically: `Build.csx` pre-fetches the core SDKs, and each
native project's pre-build step runs the relevant `sdk/<name>/_get.csx` via dotnet-script if
its SDK is missing (the AI stack, Vulkan + llama.cpp, additionally requires CMake on PATH).

For faster local Visual Studio builds, copy `Directory.Build.user.props.template` to `Directory.Build.user.props` and enable the opt-in settings there.

This repo is actively developed, often refactored, and frequently broken. It is public so that the source for my released projects is publicly available.

### PIX instrumentation and capture modes

`build\targets\WinPixEventRuntime.targets` enables `PR_PIX_ENABLED` by default in Debug.
This enables CPU and command-list event instrumentation, not a capture engine.
`pr::compute::pix::LoadDll()` loads only the optional `WinPixEventRuntime.dll`;
neither renderer nor standalone compute-device construction loads a capturer.

- **External timing capture:** run the application normally and attach PIX in timing-capture mode.
  Keep GPU timings enabled. Neither `WinPixGpuCapturer.dll` nor an application-loaded
  `WinPixTimingCapturer.dll` is needed for this workflow.
- **GPU capture:** launch through PIX for GPU capture, or explicitly call
  `pr::compute::pix::LoadLatestWinPixGpuCapturer()` at application startup, before
  **any** D3D12 API call. This includes adapter capability checks, `DefaultAdapter()`
  with software fallback, and construction of a renderer or standalone `Gpu`.
  Existing `BeginCapture`/`CaptureScope` call sites require this startup setup;
  they cannot load the capturer after the device already exists.
- **Programmatic timing capture:** explicitly load `WinPixTimingCapturer.dll` using
  `PIXLoadLatestWinPixTimingCapturerLibrary()` and follow PIX's timing-capture
  elevation requirements. Do not also load the GPU capturer.

Choose the capture mode before creating GPU objects and use a fresh process when
changing modes; do not unload a capture engine while its wrapped objects remain alive.
The debugger-controlled captures in `window.cpp` and physics `engine.cpp`, and the
`PR_RDR12_DEBUG_RAYCAST` path, retain the same explicit GPU-capture prerequisite.
Separating capture engines does not itself prove that a particular driver/runtime
produces named GPU intervals: verify those in a fresh timing capture.

See Microsoft's [instrumentation documentation](https://devblogs.microsoft.com/pix/winpixeventruntime/)
and [programmatic capture prerequisites](https://devblogs.microsoft.com/pix/programmatic-capture/).

#### Detailed PIX instrumentation

Detailed PIX instrumentation is retained as a supported diagnostic capability and is **off by default**.
Set `PR_PIX_DETAIL=1` **before process startup** in a PIX-enabled build (enabled by default in Debug)
to enable the fine-grained regions. Leave it unset or set it to `0` for ordinary instrumentation.
Only the exact value `1` enables detail, and the setting is cached on first use; restart the process after changing it.
When `PR_PIX_ENABLED=0`, detail is disabled regardless of the environment setting.
With detail disabled, no detail events, markers, diagnostic counter copies or associated readback allocations are recorded.
Ordinary PIX regions, including purpose-specific sort names, remain available independently of detailed instrumentation.
Enabling detail adds
markers, small counter copies, and their resource transitions, not a different solver,
sort algorithm, dispatch count, or queue policy; compare runs with the same setting.

The three physics sort purposes are `Physics::SortBroadphaseEndpoints`,
`Physics::SortContactPriority`, and `Physics::SortCoupledContactEndpoints`.
Their current inputs are respectively twice the body count, the contact sort capacity,
and twice the coupled-contact capacity. Inactive entries are still part of the latter
two sorted ranges: a live contact count is not the number of keys actually sorted.
The generic GPU-counted overload reports a capacity explicitly; the physics callers use
the CPU-known input-count overload.

Stable `RadixSort::Pass`, `SweepUp`, `Scan`, `SweepDown`, dispatch and barrier regions
separate the four byte passes. Associated PIX markers carry radix shift, input count,
partition size and dispatch dimensions. `Resolve::*` regions distinguish cache clearing,
key generation, priority propagation, colouring, warm start, position and velocity
iterations, colour batches and their barriers. Changing indices are markers, not region names.

`Physics::Step` carries engine identity, simulation time, elapsed time and substep count;
`Physics::Substep` carries its index. After the existing completion fence,
`Physics::ResolveCounts` CPU markers identify the same engine/time/substep and report
the main solve's GPU-generated pair/contact counts and indirect dispatch dimensions.
These diagnostic copies share the already submitted job and its completion fence;
there is no extra submission or CPU/GPU synchronization round trip. They do not describe
selective-refresh subsets. `Physics::Completed` reports frame maxima and must not be
mistaken for individual substep counts.

Aggregate durations by stable purpose/phase and match their metadata before comparing.
GPU event spans include possible preemption, resource ordering and queue interference;
they are not exclusive shader instruction time. Inspect the marked dispatches and barriers
in a separate GPU capture if an expensive phase persists with rendering submissions disabled.
Verify actual viewport size, capture loss, unrelated GPU consumers and clock state for each run.

## License

- [licence](miscellaneous/licenses/license.txt)
