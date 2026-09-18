# Rylogic.Gfx

This assembly is an interop wrapper for the native View3d dll

## Procedural surface materials

`View3d.ProceduralSurface` adds GPU-evaluated albedo, UV-free normal perturbation, and roughness to an ordinary
PBR nugget. It samples deterministic 3D value noise from object or world coordinates, so it does not allocate,
generate, tile, or wrap a `Texture2D`. The five `Soil`, `Grass`, `Sand`, `Rock`, and `Snow` presets are editable
parameter factories rather than renderer or application semantics.

```csharp
var surface = View3d.ProceduralSurface.Preset(View3d.EProceduralSurfacePreset.Rock);
surface.m_feature_scale = 2.5f;
surface.m_seed = 0x12345678;
surface.m_coordinate_space = View3d.EProceduralCoordinateSpace.World;
surface.m_coordinate_origin = new v4(1000000, 0, 0, 1);
surface.m_colour0 = new Colour32(0xFF202428);
surface.m_colour1 = new Colour32(0xFF495057);
surface.m_colour2 = new Colour32(0xFF737B80);
surface.m_colour3 = new Colour32(0xFFB3B5AE);
surface.m_normal_strength = 0.65f;
surface.m_roughness_min = 0.5f;
surface.m_roughness_max = 0.9f;

// 'geometry' can be any ordinary View3D object; no authored UV or tangent stream is required.
geometry.NuggetProceduralSurface = surface;
```

The caller owns feature scale, seed, coordinate frame and origin, anisotropic axis scale, palette, normal strength,
and roughness range. World coordinates are continuous across meshes whose transforms describe the same world
positions. Object coordinates intentionally move with each object. Translation is split into a wrapped 32-bit
lattice cell plus a small fractional coordinate before upload; set `m_coordinate_origin` near very large working
coordinates to preserve local detail. The hash domain repeats only after `2^32` lattice cells per axis.

Forward opaque/alpha rendering and its reflection-attribute side buffer evaluate all three channels. Shadow-map
and ray-cast paths retain their geometry behavior because neither shades visible material channels. Deferred
G-buffer/direct-lighting and DXR secondary-hit material shading reject procedural surfaces with a View3D diagnostic
rather than silently substituting a different surface. Camera-visible raster output remains usable while ray
tracing is disabled.

The native ABI exposes `View3D_ProceduralSurfacePreset`,
`View3D_ObjectNuggetProceduralSurfaceGet/Set/Clear`, `pr::view3d::ProceduralSurface`, and matching preset/coordinate
enums. Assignment promotes an ordinary nugget material to PBR while retaining its base colour and any compatible
roughness/two-sided components. The material is model-owned, matching the existing nugget tint/flag APIs, so the
assignment affects every instance sharing that model.
Assignment preserves alpha blending when promoting an ordinary transparent nugget. Materials with custom shader
overlays are rejected because replacing those stages with the stock PBR shader would silently discard caller code.

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
  the revision/sequence/edit-generation it was raised against, a decoded payload string, and an optional typed numeric payload.
- **Semantics**: `UiContext.CaptureSemantics()` (or its `Span<UiSemanticNode>` overload) returns a flattened snapshot of
  the accessible-semantics tree as of the most recent `Update(ViewportState)` call - call `Update` at least once after
  applying a transaction before capturing semantics that should reflect it.
- **Diagnostics**: `UiContext.GetDiagnostics()` returns native `UiDiagnostics` counters (accepted revision, control
  count, rejected-transaction attempts, etc.) for direct inspection; native status codes are always surfaced as a
  `View3dUiException` carrying the originating `EStatus`, never swallowed.

### Visibility

`UiControlDesc.Visibility` uses `EVisibility.Visible` (default), `Hidden`, or `Collapsed`. Visible controls draw, accept input, and occupy layout space.
Hidden suppresses the whole subtree's drawing/input but retains its allocation. Collapsed removes the subtree's layout extent, margins, padding, and stack spacing.
Changing an ancestor's visibility also affects descendant input and semantics. An unavailable subtree loses focus, pressed/captured interaction, and composition;
call `Update` after submitting the transaction to refresh layout and semantic snapshots.

The current native ABI is version `0x00060000` / struct version `6`; use matching managed and native binaries. Replace the old boolean `Visible` property with `Visibility`:
`true` becomes `EVisibility.Visible`, and `false` becomes `EVisibility.Hidden` to preserve behavior. Select `Collapsed` explicitly to remove space.
No auto-sizing or clipping behavior is implied.

### ProgressBar

`UiControlDesc.Type = EControlType.ProgressBar` is a horizontal, retained, read-only progress indicator. `Value` is finite normalized completion in `[0, 1]`
(default `0`), including while `IsIndeterminate` is true. `IsIndeterminate` defaults to false; true displays activity without implying a percentage.
Update the descriptor through the usual transaction path when completion or mode changes. Set `StyleVisual`'s `foreground` constructor argument
(`m_foreground`) for the indicator and `fill` (`m_fill`) for the track. Border, corner radius, opacity, and foreground state transitions use the normal style path.
Custom styles must specify their indicator colour; the built-in default style supplies a blue indicator. Explicit layout width/height are required as for other controls.

Activity is a quarter-width indicator travelling smoothly back and forth within the track every 1200 ms, driven entirely by `ViewportState.m_time_ms`.
Hosts must keep supplying finite time and updating/rendering frames; no new transaction, event, managed animation callback, or percentage update is needed.
Hidden/Collapsed ancestors suppress its drawing normally. It is hit-test transparent and does not block host input; modal input ownership belongs to the host.
Put any visible label in a separate Text control. `Name` and `Description` supply accessible labeling.

Semantics expose `Role = EControlType.ProgressBar`, `ProgressValue`, and `IsIndeterminate`, plus percentage `Value` text only in determinate mode.
UI Automation exposes ProgressBar control type and a read-only RangeValue pattern with minimum 0 and maximum 1 only while determinate.
Activity animation does not change accessible completion or emit input events. Optional templates require `PART_Track` and `PART_Indicator`.
ABI 6 extends ControlDesc, Event, and SemanticNode beyond the earlier progress/style fields; rebuild native UI clients and refresh the managed wrapper and native runtime together.

### Slider

`UiControlDesc.Type = EControlType.Slider` is a reusable horizontal finite scalar. Set finite `Minimum`, `Maximum`, `Value`, and `Step`; defaults are `0`, `1`,
`0`, and `0.1`. Maximum must be greater than Minimum, Value must lie in the inclusive range, and Step must be positive and no larger than the range.
Native and JSON descriptor validation reject invalid values atomically and never clamp caller data.

The accepted descriptor remains authoritative. Click-to-position, captured drag, Left/Down, Right/Up, Home/End, and accessibility SetValue produce
`EEventKind.ValueChangeProposed`. `UiEvent.HasNumericValue` is true and `UiEvent.ProposedValue` exposes the typed `double` proposal without parsing text or depending
on culture. The proposal retains normal accepted-revision and event-sequence ordering and coalesces per slider. Apply the accepted/normalized value in a later
transaction; until then visuals and semantics continue to report the previous `UiControlDesc.Value`.

The built-in template uses `PART_Track` and `PART_Thumb`. Style `fill` paints the track and `foreground` paints the accepted-range indicator and thumb. Slider uses
the normal layout/DPI, focus traversal, hover/pressed/focused/disabled channels, pointer capture, and visibility model. Disabled sliders emit no proposals.
`UiSemanticNode` reports `Role = EControlType.Slider`, accepted `RangeValue`, `RangeMinimum`, `RangeMaximum`, `RangeStep`, and SetValue/Focus actions.
UI Automation exposes Slider with RangeValue; disabled state makes it read-only.

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

Schema version `2` represents visibility as `"visibility": "Visible"`, `"Hidden"`, or `"Collapsed"` and defaults to Visible when omitted.
Schema version `1` and the old `"visible"` boolean are rejected, not automatically converted.
ProgressBar adds optional `"value": 0.25`, `"is_indeterminate": true`, and style visual `"foreground": "#00FF00FF"` properties within schema 2;
no existing property changes meaning. Canonical serialization writes these fields explicitly.
Slider adds optional `"minimum"`, `"maximum"`, and `"step"` alongside `"value"` within schema 2; canonical serialization writes them explicitly.
