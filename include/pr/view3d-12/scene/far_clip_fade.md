# Far clip fade

`Scene::FarClipFadeProperties` owns an opt-in world-opacity policy. A new scene is disabled, with
`start_fraction = 0.9` and `end_fraction = 0.99`. Both fractions must be finite and satisfy
`0 <= start_fraction < end_fraction < 1`, including when disabled. Enabling also requires a finite,
positive camera far depth and a representable interval ending strictly before that depth.

## API

Native clients set `scene.FarClipFadeProperties(FarClipFadeProps{true, 0.9f, 0.99f})`.
The DLL exports `View3D_FarClipFadePropertiesGet/Set`, using a 12-byte struct containing a `BOOL`
and two floats. The setter reports errors through the existing error callback, returns `FALSE` on
failure, and does not partially replace the settings. Window changes raise `Rendering_FarClipFade`
and invalidate the view.

Managed clients use:

```csharp
window.FarClipFadeProperties = new View3d.FarClipFadeProps(true, 0.9f, 0.99f);
```

`new FarClipFadeProps()` and `FarClipFadeProps.Default()` supply valid disabled settings.
The zero-initialized `default(FarClipFadeProps)` has an invalid zero-width range and is rejected
by the setter. The managed setter validates locally and throws if native validation rejects the change.
The DLL/window accessors delegate to the scene; they do not maintain another copy of the settings.

## Rendering contract

Depth is the negative camera-space Z of the fragment's emitted world position, not radial distance.
The existing scene camera supplies the transform and absolute far plane, so camera motion, projection
changes, and clip-plane updates automatically change the interval. Opacity is
`1 - smoothstep(start_fraction * far_depth, end_fraction * far_depth, forward_depth)`.
It reaches zero before the hardware far plane.

* Near opaque fragments keep normal depth writes and full coverage.
* Opaque fragments beyond the fade start are discarded from that pass, then submitted to the existing
  alpha K-buffer with fade opacity. Their full-coverage source alpha is preserved; PBR cutout masking
  happens before collection.
* Ordinary transparent fragments multiply their material opacity by fade opacity. Both kinds of
  fragments enter the same depth-sorted K-buffer before its one resolve, regardless of submission order.
  Near opaque depth still rejects hidden layers.
* Skybox and `PostAlpha` sort groups, and retained UI host passes, are excluded. Background colour is
  not faded. Shadows and picking remain geometric rather than using visual fade opacity.

Explicit object sort groups survive unrelated flag changes, including visibility, bounds, picking,
and shadow exclusions. Only changes to `NoZTest`/`NoZWrite` select automatic depth-policy ordering;
`NoZTest` takes precedence when both are enabled. Disabling both clears that automatic group override.
The procedural sky therefore keeps its `Skybox` classification when its exclusion flags are applied.

The K-buffer's existing limits also apply to faded opaque coverage: a bounded number of stored
layers, quantized opacity, and **single-sample alpha coverage**, even when opaque rendering uses MSAA.
Alpha rejection uses the nearest opaque MSAA depth sample. Thus fade-band silhouettes have the same
coverage limitations as existing transparency; this feature does not implement per-sample alpha.

## Supported pipelines and cost

Stock simple, PBR, and multi-UV PBR pixel families have explicit opt-in opaque, reflection-attribute,
and alpha-collect variants. Compatible custom vertex stages (including world-position deformation)
remain supported when they emit the stock forward inputs and retain the stock root signature.
Unsupported custom pixel shaders, custom root signatures, and unknown material passes fail explicitly
instead of silently bypassing the fade. A forward alpha K-buffer is required.

Disabled rendering uses the original pixel entry points and pass count: no fade pixel calculations,
extra raster passes, or geometry allocations. Enabled rendering adds one linear traversal of the
existing opaque draw list, reusing its meshes and the existing alpha storage. For rigid stock vertex
pipelines, conservative existing model bounds suppress near-only recollection. Invalid bounds,
skinning, custom vertex/geometry/tessellation stages, and non-affine transforms bypass that optimization.
No mesh copying or per-frame vertex scan is required.

The forward nugget constants reuse three reserved padding floats without changing their size or
existing offsets. Rebuild compatible application shaders against the matching headers when deploying
the updated Native/Gfx package pair.

## Validation

Build `projects\tests\view3d-fade-tests\view3d-fade-tests.vcxproj` with VS 2026, v145, Debug/x64, then
run its `obj\x64\Debug\view3d-fade-tests.exe`. The isolated invisible-window fixture uses the freshly
built DLL and reads rendered pixels at 1x and 4x MSAA. It covers defaults and invalid inputs,
disabled-image equality, the opacity ramp, crossing primitives, off-axis orthographic/perspective
depth, camera updates, custom vertex deformation, PBR, material alpha, sorted overlap, opaque
occlusion, existing alpha edge coverage, sky/PostAlpha exclusions, retained final UI, picking, and
explicit custom pixel/root-signature rejection. Repeated scene-handoff tests remove a rendered custom-PS
object, enable fade without an intervening render, and composite world alpha over a real procedural sky.
They also verify that visibility and exclusion flags preserve sky/overlay sort groups and that depth
flag transitions retain their automatic ordering. GPU debug-layer errors fail the fixture when the
debug interface is available.

Build `projects\rylogic\Rylogic.Gfx\Rylogic.Gfx.csproj` in Debug to run inline managed validation
and ABI-layout tests on both target frameworks. Packaging/deployment is a separate step.
