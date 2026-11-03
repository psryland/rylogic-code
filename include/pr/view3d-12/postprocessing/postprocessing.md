# Post-processing

`Scene::PostEffects()` owns a set of opt-in screen-space effects that run on the scene's composited colour.
Each effect has its own typed settings struct with an `m_enabled` flag. A new scene has every effect disabled.

Effects currently available:

| Effect | Settings | Purpose |
|--------|----------|---------|
| Ambient occlusion | `AmbientOcclusionProps` | Darkens opaque surfaces in creases and corners where nearby geometry blocks ambient light |
| Underwater | `UnderwaterProps` | Whole-screen tint, distance fog below the water surface, and a moving "looking through water" distortion |

Ambient occlusion runs at a different point in the frame from the other effects; see [Ambient occlusion](#ambient-occlusion).

## Pipeline position

Effects run in the frame's `PostEffects` command list. This list executes after the depth resolve and before
the world and final overlays:

```
main -> world_depth -> resolve -> post -> composite -> depth_resolve -> post_effects -> world_overlay -> final_overlay -> present
```

So effects see the final opaque and transparent world colour, and the resolved (single-sample) scene depth.
World overlays and retained screen UI are never post-processed.

Enabled effects always run in one fixed, engine-defined order, whatever order they were enabled in. Each pass reads
the previous result and writes the next one:

1. Copy the scene colour into scratch target A.
2. Pass 1 reads A and writes B, pass 2 reads B and writes A, and so on.
3. The last pass writes straight back into the scene colour.

One effect needs only one scratch target and one full-screen draw. The second scratch target exists only while two
or more effects are enabled. Every pass covers the scene's viewport and scissor, so several scenes that share a
window each process only their own region.

## Cost

- **Disabled:** no GPU resources are held, no commands are recorded, and the depth resolve is not requested for
  post-processing. The scratch targets are released when the last effect is disabled.
- **Enabled:** one colour copy, plus one full-screen triangle per enabled effect. Targets and pipeline states are
  created on first use and reused, and they are recreated only when the back buffer size or format changes.
- The depth resolve is shared with other users in the same frame; it is recorded at most once per frame.

## API

Native clients:

```cpp
auto props = scene.PostEffects().Underwater();
props.m_enabled = true;
props.m_visibility = 25.0f;
scene.PostEffects().Underwater(props); // throws std::invalid_argument for invalid settings
```

The DLL exports `View3D_PostEffectUnderwaterGet/Set`. The setter reports errors through the error callback,
returns `FALSE` on failure, and does not partially replace the settings. Window changes raise
`Rendering_PostEffects` and invalidate the view.

Managed clients:

```csharp
var props = window.PostEffectUnderwater;
props.Enabled = true;
props.Visibility = 25f;
window.PostEffectUnderwater = props;
```

`new View3d.UnderwaterProps()` and `UnderwaterProps.Default()` supply valid disabled settings. The zero-initialised
`default(UnderwaterProps)` has zero visibility and frequency, so the setter rejects it. The managed setter
validates locally and throws if native validation rejects the change.

## Underwater

Without a surface plane, the caller decides when the camera is submerged (for example, by comparing the camera
position with a sampled water height) and enables or disables the effect. With a surface plane, the effect finds where
the camera's near plane meets the surface once per frame on the CPU:

- **Whole near plane above the surface:** the pass is skipped. No commands are recorded, but the effect keeps its
  resources, so the camera can move in and out of the water without recreating them.
- **Whole near plane below the surface and deeper than `m_fade_depth`:** every pixel gets the full effect, as if there
  were no waterline.
- **Otherwise:** the surface or the fade region crosses the near plane, and the effect strength depends on the depth of
  each pixel's near-plane point below the surface. Pixels above the waterline keep the scene colour. With a fade depth,
  the strength rises smoothly (smoothstep) from none at the surface to full at `m_fade_depth` below it, so the waterline
  has a soft edge; without one, a one-pixel blend smooths a sharp edge. This per-pixel test runs only in this case,
  selected by a uniform branch.

So an application can leave the effect enabled with a surface plane whenever water may be near the camera.

| Setting | Default | Meaning |
|---------|---------|---------|
| `m_tint` | `FFA6D9F2` | sRGB colour multiplied into the scene colour |
| `m_fog_colour` | `FF0A384D` | sRGB colour that distant surfaces fade towards; alpha is the fog strength (largest fog amount) |
| `m_visibility` | `40` | World distance at which fog hides 95% of a surface (must be > 0; +infinity = no fog) |
| `m_distortion_amplitude` | `0.002` | Largest screen offset, as a fraction of the viewport height (>= 0, 0 = no distortion) |
| `m_distortion_frequency` | `6` | Ripples per viewport height (must be > 0) |
| `m_distortion_speed` | `0.25` | Animation cycles per second (>= 0, 0 = still) |
| `m_surface` | zero | World-space water surface plane, normal pointing out of the water (zero = no surface) |
| `m_fade_depth` | `0` | Depth below the surface over which the effect fades in (>= 0, 0 = sharp waterline) |
| `m_waterline_offsets` | zeros | 8x8 heights of the real surface above `m_surface` on the near plane (finite) |

For each pixel:

1. The screen position is offset by a sum of sine waves, then clamped to the viewport. The offset is corrected for
   aspect ratio, so ripples are round. When the surface crosses the near plane, an offset that would read from above
   the waterline is dropped, so above-water colour does not leak into the water.
2. The scene colour is read at the offset position. Alpha is kept unchanged.
3. The view ray to the surface is rebuilt from the resolved depth, using the inverse of the camera projection.
   Perspective rays start at the camera; orthographic rays start on the camera plane. Pixels without geometry (the
   background) have an unbounded ray.
4. The fog distance is the length of the ray that is under water. Without a surface plane this is the whole ray, and
   an unbounded ray is infinitely long, so the background is fully fogged. With a surface plane, only the part of the
   ray below the plane counts. So terrain and sky seen through the surface from below keep their colour (with the tint
   and distortion), while an unbounded ray that never rises through the surface is still fully fogged.
5. `fog = fog_colour.a * (1 - exp(-3 * distance / visibility))` and `colour = lerp(colour * tint, fog_colour, fog)`.
   The blend happens in linear colour space. With infinite visibility, steps 3 to 5 are skipped and only the tint
   applies, for applications that fog the scene in their own shaders.

The waterline follows `m_waterline_offsets`: the heights of the real surface above the plane, measured along its normal,
at an 8x8 grid of evenly spaced points on the near plane. Sample `(i, j)` is at index `j * 8 + i` and at viewport position
`(i, j) / 7`, with `(0, 0)` at the top-left corner. Each pixel interpolates the four nearest samples (bilinear), so the
waterline can follow waves that cross the near plane. All zeros gives a straight waterline on the plane. The CPU
classification uses the smallest and largest offsets, so the skipped and full-strength cases stay exact. For a wavy
surface, pass a plane near the water height at the camera, and the sampled wave heights relative to it. The offsets
move only the waterline; the fog still uses the plane. The effect does not produce refraction or total internal
reflection at the surface.

Depth is read at the same distorted position as the colour, so the fog always matches the surface it covers.

The distortion is animated by the engine's real-time clock. The effect does not request new frames; the caller must
keep rendering (for example, from a game loop) for the motion to be visible. The phase is wrapped to one cycle
before it is sent to the GPU, so precision does not degrade after long run times.

## Ambient occlusion

Screen-space ambient occlusion estimates, for each opaque pixel, the cosine-weighted fraction of the hemisphere above the surface that is
not blocked by nearby geometry, and multiplies the opaque colour by it. It uses only the scene depth, so it needs no normals or extra
geometry passes.

### Pipeline position

Ambient occlusion is recorded in its own command list in the `post` stage, after the MSAA resolve and before transparent surfaces are
composited:

```
main -> resolve (opaque colour, 1x) -> post (ray tracing, then ambient occlusion) -> composite (transparent surfaces) -> ...
```

- It reads the multi-sampled depth buffer (sample 0 only) and darkens the resolved single-sample opaque colour. The MSAA sample count
  does not change its cost, only the resolution does.
- Transparent surfaces are composited over the darkened colour. They are not darkened themselves, and they do not darken what is behind
  them, because they are not in the opaque depth.
- Ray-traced results (reflections, shadows) are part of the opaque colour, so they are darkened too. Ray tracing does not change how the
  occlusion is estimated.
- World overlays, screen UI, and pixels without geometry (the background) are not changed.

### Passes

1. **Prefilter:** convert the depth buffer to linear view depth at the AO resolution (half or full). Each AO texel takes the depth of the
   top-left pixel of its block, so the upsample can compare exactly the same depth.
2. **Main:** for each texel, rebuild the view-space position and normal from depth, then search for the highest occluder ("horizon") on
   both sides of the texel in a few planes ("slices") that contain the view vector. The visible arc between the horizons is integrated
   with a cosine weight. Slices are spread evenly in angle around the view vector, and their screen direction is derived from the
   projection, so flat surfaces seen at an angle by a perspective camera are not darkened. A 4x4 pattern varies the slice angle and the
   sample positions between neighbouring texels.
3. **Blur:** average each 4x4 block of texels with similar depth. The block contains each pattern cell once, so the pattern cancels on
   flat surfaces, and depth edges do not leak.
4. **Apply:** upsample to full resolution, choosing the AO texels whose depth matches the pixel, and multiply into the opaque colour
   (or write grey scale in the debug view).

### Settings

| Setting | Default | Meaning |
|---------|---------|---------|
| `m_radius` | `0.5` | World-space distance within which geometry occludes a point (> 0) |
| `m_intensity` | `1` | The visibility `v` becomes `pow(v, intensity)`; 0 = no darkening, larger = darker (>= 0) |
| `m_falloff` | `0.6` | Fraction of the radius, at its outer end, over which occluders fade out; in (0, 1] |
| `m_max_screen_radius` | `0.1` | Largest search radius as a fraction of the viewport height; limits cost and halos near the camera (> 0) |
| `m_quality` | `Medium` | `Low` = 1 slice x 2 samples each way, `Medium` = 2 x 3, `High` = 3 x 4 |
| `m_resolution` | `Half` | `Half` = one AO texel per 2x2 pixels; `Full` = one per pixel, about four times the cost |
| `m_debug_view` | `false` | Show the occlusion term as grey scale instead of applying it |

Choose `m_radius` to match the scale of the creases that should darken, for example a fraction of a character's height. Occluders
farther than the radius do not contribute, so the effect stays local. When disabled, all AO resources are released and no commands
are recorded.

### Cost

Measured by `View3d12_AmbientOcclusionCost` at 3840x2160 with 4x MSAA over a dense perspective box field (Debug build, D3D12 debug
layer active, so release builds will be faster):

| Resolution | Low | Medium | High |
|------------|-----|--------|------|
| Half | 0.65 ms | 0.68 ms | 0.87 ms |
| Full | 1.15 ms | 1.55 ms | 1.79 ms |

The default (half resolution, medium quality) is under 1 ms at 4K. The cost is mostly proportional to the AO texel count and the
slices x samples product. `m_max_screen_radius` bounds the texture cache footprint of near-camera texels.

### API

```cpp
auto props = scene.PostEffects().AmbientOcclusion();
props.m_enabled = true;
props.m_radius = 0.3f;
scene.PostEffects().AmbientOcclusion(props); // throws std::invalid_argument for invalid settings
```

The DLL exports `View3D_AmbientOcclusionGet/Set` with the same error behaviour as the underwater effect. Managed clients use
`window.AmbientOcclusion` with `View3d.AmbientOcclusionProps`.

## Adding an effect

1. Add a settings struct next to `UnderwaterProps` in `post_processing.h`, with `m_enabled`, defaults,
   `Validate()`, and an `operator ==`. Add get/set members to `PostProcessing`.
2. Add a shader in `src/shaders/hlsl/postprocessing/` with a shared `*_cbuf.hlsli` constant buffer. Reuse
   `VSPostEffect` for the full-screen triangle. Register the entry points in `view3d-12.vcxproj` and `shader.h/.cpp`.
3. Add a `Record<Effect>` pass function and its PSO. Put it into the pass list in `PostProcessing::Render` at its
   canonical position, and grow the pass array. Passes read `t0` (colour) and `t1` (depth) and write the given
   render target; they must not assume which target is the scene colour.
4. Add `View3D_PostEffect<Effect>Get/Set` to the DLL, a matching window accessor, and a C# struct and property in
   `Rylogic.Gfx`. Keep the DLL and C# struct layouts identical and cover them with layout tests.
5. Add GPU pixel tests to `view3d-12-tests` (`src\view3d-12\render_tests.cpp`).

The fixed order puts whole-screen colour effects (such as underwater) last. Ambient occlusion is the exception: it
must darken only opaque surfaces, so it runs before transparent compositing rather than in this pass list.

## Validation

Build `projects\tests\view3d-12-tests\view3d-12-tests.vcxproj` (VS 2026, v145, Debug/x64), then run
`obj\x64\Debug\view3d-12-tests.exe View3d12_PostEffects`. The invisible-window fixture reads rendered pixels at
1x and 4x MSAA. It covers defaults, invalid-settings rejection, disabled-image equality, tint, depth fog,
background fog, surface-plane fog (orthographic and perspective, from above and below the plane), the waterline
split, the fade depth, and the skipped pass above the water, overlay exclusion,
and restoring the image when the effect is disabled. GPU debug-layer errors fail
the fixture.

Run `obj\x64\Debug\view3d-12-tests.exe View3d12_AmbientOcclusion` for the ambient occlusion tests (1x and 4x MSAA): defaults and
validation, no darkening of flat surfaces (including off-axis perspective), darkening beside a raised strip at every quality and
resolution, intensity, the debug view, a transparent pane, and exact restoration when disabled. The same filter also runs
`View3d12_AmbientOcclusionCost`, which fails if the default settings cost 1 ms or more at 4K with 4x MSAA. in Debug to run the inline managed validation and
struct-layout tests.
