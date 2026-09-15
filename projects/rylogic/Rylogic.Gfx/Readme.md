# Rylogic.Gfx

This assembly is an interop wrapper for the native View3d dll

## Procedural atmosphere

`View3d.ProceduralSky` owns a Z-up GPU sky shared with native `pr::rdr12::ProceduralSky`
(`pr/view3d-12/scene/procedural_sky.h`). Create it after `View3d.Create()`, add its `Skybox` object to a window,
and remove that object before disposing the sky. Creation, `Update`, `Blend`, rendering and disposal must be sequenced
on the render owner; the View3d context must outlive the sky.

```csharp
using var sky = new View3d.ProceduralSky("Atmosphere", new v4(0.4f, 0.6f, 1, 0), new v4(1, 0.95f, 0.85f, 1), 1);
window.AddObject(sky.Skybox);
// Before the next render, update direction toward the sun, linear RGB colour, and nonnegative intensity.
sky.Update(new v4(1, 0, 0, 0), new v4(1, 0.5f, 0.2f, 1), 0.5f);
window.Render();
window.RemoveObject(sky.Skybox);
```

The shared shader covers daylight, twilight, night, a sun disc and halo, and the below-horizon fade.
It uses precompiled renderer shader bytecode and one persistent background triangle, with only a small parameter
upload per draw. No CPU image generation, intermediate texture files, runtime shader compiler, or sky-specific render loop is needed.
Perspective and orthographic cameras are supported; object transforms and camera translation do not move the sky.
It renders at far depth after opaque geometry without writing depth.

`sky.Blend(background, weight, world_to_sky, world_to_background)` optionally blends a `View3d.CubeMap` into the
atmosphere in that same draw. Weight 0 is exactly the cubemap and weight 1 is exactly the atmosphere; the shader
interpolates linear RGB before output encoding. The caller owns timing and easing. The two finite orthonormal
matrices rotate current scene directions into the atmosphere's and cubemap's world frames; translation and scale
are not accepted. The cubemap's own `m_cube2w` orientation is also respected. Independent direction frames let a
caller change scene coordinates without rotating either background. The native sky retains the source texture
even if its managed wrapper is disposed. `Blend(null, 1, ..., ...)` releases that reference and selects the standalone
atmosphere. Invalid weights, rotations, or disposed wrappers are rejected without changing the previous state.
The blend uses a dedicated descriptor binding, leaving material textures and the reflection environment untouched.

The sky does **not** generate a reflection cube map. Set `window.EnvironmentMap = null` when switching from an
authored environment to this sky unless a separate reflection source is intended. The C ABI exposes
`View3D_ObjectCreateProceduralSky`, `View3D_ObjectUpdateProceduralSky` and `View3D_ObjectBlendProceduralSky`;
destruction uses `View3D_ObjectDelete`.

## Rylogic.Gfx.UI (View3DUI managed API)

`Rylogic.Gfx.UI` is a WPF-free managed wrapper over the native `view3d-ui.dll`. It exposes copyable, value-typed
descriptors identified by stable `ControlId`/`ResourceId`/`StyleId`/`TemplateId` handles instead of a managed UI
element identity contract; there are no per-frame managed callbacks.

- **Ownership**: `UiRuntime` loads `view3d-ui.dll`, validates that every native ABI struct size matches its managed
  mirror (see `Native.cs`/`Types.cs`), and creates `UiContext` instances via `CreateContext(config, device, window)`.
  `UiRuntime` and `UiContext` are both `IDisposable`; disposing a runtime does not require its contexts to already be
  disposed, but a context must not outlive the runtime that created it.
- **Threading**: `UiRuntime` and the contexts it creates are affine to the OS thread that constructed the runtime.
  `CreateContext` and `Dispose` (and all `UiContext` operations) throw `InvalidOperationException` if called from any
  other thread - this mirrors the native library's own single-threaded contract and is enforced entirely in managed
  code before any native call is made.
- **Revisions**: all tree mutation goes through `UiTransactionBuilder`, which batches upserts/removes/reorders and
  resource/style/template additions/removals into a single native call via `Apply(context, base_revision, revision)`.
  `base_revision` must equal the context's currently accepted revision and `revision` must be exactly
  `base_revision + 1`; otherwise the native library rejects the whole transaction atomically (`EStatus.StaleRevision`
  or `EStatus.InvalidArgument`, surfaced as a `View3dUiException`) and the accepted revision is left unchanged.
- **Event draining**: `UiContext.DrainEvents()` (or the `Span<UiEvent>` overload for a caller-owned buffer) removes and
  returns all pending `UiEvent` records since the last drain, each carrying the originating `ControlId`, `EEventKind`,
  the revision/sequence/edit-generation it was raised against, and a decoded payload string.
- **Semantics**: `UiContext.CaptureSemantics()` (or its `Span<UiSemanticNode>` overload) returns a flattened snapshot of
  the accessible-semantics tree as of the most recent `Update(ViewportState)` call - call `Update` at least once after
  applying a transaction before capturing semantics that should reflect it.
- **Diagnostics**: `UiContext.GetDiagnostics()` returns native `UiDiagnostics` counters (accepted revision, control
  count, rejected-transaction attempts, etc.) for direct inspection; native status codes are always surfaced as a
  `View3dUiException` carrying the originating `EStatus`, never swallowed.

### JSON documents (`Rylogic.Gfx.UI.Json`)

`Json.UiDocument.Parse(string json)` is a managed-only conversion layer - the native library never parses JSON. It
walks a `System.Text.Json` document tree and produces the exact same typed descriptors
(`UiControlDesc`/`UiResourceDesc`/`UiStyleDesc`/`UiTemplateDesc`) that runtime code builds directly, so
`UiDocument.ToTransactionBuilder()` produces a `UiTransactionBuilder` equivalent to one built by hand. Only the closed
initial control/layout/style/template/resource/event/semantic vocabulary is accepted; there is no arbitrary drawing or
control registration. Every malformed document, unknown enum name, dangling id reference (style/template/font
resource), duplicate id, zero control id, or oversized template part list is rejected with a `UiJsonException` naming
the offending JSON path (e.g. `$.tree[0].style_id`) before any native call would ever be attempted.
`UiDocument.Serialize()` writes a deterministic canonical form with stable collection/property order and explicit values,
so equivalent documents converge to byte-identical JSON.
