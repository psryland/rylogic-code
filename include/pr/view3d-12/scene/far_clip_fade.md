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

Depth is the negative camera-space Z of the stored depth sample, not radial distance. The scene camera
supplies the projection and absolute far plane, so camera motion, projection changes, and clip-plane
updates automatically change the interval. The fade weight is
`smoothstep(start_fraction * far_depth, end_fraction * far_depth, forward_depth)`; it reaches one
before the hardware far plane. The fade is a blend towards the background, not a change in opacity.
Faded geometry still writes depth and still hides everything behind it.

The opaque pass is split at the `Skybox` sort group:

1. Opaque world groups (before `Skybox`) draw normally, with no fade work in their pixel shaders.
2. A full-screen pass reads the opaque depth buffer for every MSAA sample and computes the fade weight.
   * With `Skybox` objects present, it writes `1 - fade` into the colour target's alpha channel.
     The `Skybox` objects are then drawn with a viewport depth range pinned to the fade start depth,
     so the reversed-depth `GREATER_EQUAL` test passes only on samples at or beyond the fade start. Their colour
     blends as `src * (1 - dst_alpha) + dst * dst_alpha`. Empty samples (depth 0) receive the full background.
   * Without `Skybox` objects, it blends the window clear colour directly by the fade weight.
3. `PostOpaques` groups draw after the background and are not faded.

Transparent layers stored in the alpha K-buffer multiply their opacity by `1 - fade` when they are
collected, so they vanish over the same interval. Without a K-buffer, alpha draws are not faded.
`PreAlpha` and `PostAlpha` groups, and retained UI host passes, are not faded. Shadows and picking
remain geometric rather than using the visual fade.

Explicit object sort groups survive unrelated flag changes, including visibility, bounds, picking,
and shadow exclusions. Only changes to `NoZTest`/`NoZWrite` select automatic depth-policy ordering;
`NoZTest` takes precedence when both are enabled. Disabling both clears that automatic group override.
The procedural sky therefore keeps its `Skybox` classification when its exclusion flags are applied.

Limitations:

* Opaque `Skybox` objects do not stack: each one blends against the faded scene, not against the others.
* The fade uses View3D's reversed depth (1 at the near plane, cleared to 0 at the far plane, `GREATER` comparisons).
* The ray-traced reflection attributes still mark faded far pixels as reflective surfaces.

## Supported pipelines and cost

The fade does not depend on the pixel shader that drew the geometry, so every pipeline is supported,
including custom pixel shaders, custom root signatures, and procedural pixel families. Each forward
pixel family has only three entry points: opaque, reflection attributes, and alpha collect.

Disabled rendering adds no passes and no pixel shader work. Enabled rendering adds one full-screen
pass that reads the depth buffer per sample, plus one extra draw of the `Skybox` objects. The fade
range is passed in `CBufFrame::far_fade` (see `forward_cbuf.hlsli`); the K-buffer collect reads it.

## Validation

Build `projects\tests\view3d-12-tests\view3d-12-tests.vcxproj` with VS 2026, v145, Debug/x64, then
run `obj\x64\Debug\view3d-12-tests.exe View3d12_FarClipFadeNumeric View3d12_FarClipFadeRender View3d12_SceneHandoff`.
The invisible-window fixture uses the freshly built DLL and reads rendered pixels at 1x and 4x MSAA.
It covers defaults and invalid inputs, disabled-image equality, the fade ramp, crossing primitives,
off-axis orthographic/perspective depth, camera updates, custom vertex and pixel shaders, PBR,
K-buffer alpha fading, occlusion of farther alpha by faded opaque geometry, MSAA edge coverage,
blending into a `Skybox` object and into the clear colour, `PostAlpha` exclusion, retained final UI,
and picking. Repeated scene-handoff tests blend faded world geometry into a real procedural sky and
verify that flags preserve sky/overlay sort groups. GPU debug-layer errors fail the fixture when the
debug interface is available.

Build `projects\rylogic\Rylogic.Gfx\Rylogic.Gfx.csproj` in Debug to run inline managed validation
and ABI-layout tests on both target frameworks. Packaging/deployment is a separate step.
