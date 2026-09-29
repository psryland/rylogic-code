# Post-processing

`Scene::PostEffects()` owns a set of opt-in screen-space effects that run on the scene's composited colour.
Each effect has its own typed settings struct with an `m_enabled` flag. A new scene has every effect disabled.

Effects currently available:

| Effect | Settings | Purpose |
|--------|----------|---------|
| Underwater | `UnderwaterProps` | Whole-screen tint, depth-based distance fog, and a moving "looking through water" distortion |

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

The caller decides when the camera is submerged (for example, by comparing the camera position with a sampled
water height) and enables or disables the effect. The effect does not test the camera against any water surface.

| Setting | Default | Meaning |
|---------|---------|---------|
| `m_tint` | `FFA6D9F2` | sRGB colour multiplied into the scene colour |
| `m_fog_colour` | `FF0A384D` | sRGB colour that distant surfaces fade towards |
| `m_visibility` | `40` | World distance at which fog hides 95% of a surface (must be > 0) |
| `m_distortion_amplitude` | `0.002` | Largest screen offset, as a fraction of the viewport height (>= 0, 0 = no distortion) |
| `m_distortion_frequency` | `6` | Ripples per viewport height (must be > 0) |
| `m_distortion_speed` | `0.25` | Animation cycles per second (>= 0, 0 = still) |

For each pixel:

1. The screen position is offset by a sum of sine waves, then clamped to the viewport. The offset is corrected for
   aspect ratio, so ripples are round.
2. The scene colour is read at the offset position. Alpha is kept unchanged.
3. The distance to the surface is found from the resolved depth, using the inverse of the camera projection. It is
   the straight-line distance from the camera for perspective cameras, and the forward depth for orthographic
   cameras. Pixels without geometry (the background) are treated as infinitely far away, so they are fully fogged.
4. `fog = 1 - exp(-3 * distance / visibility)` and `colour = lerp(colour * tint, fog_colour, fog)`.
   The blend happens in linear colour space.

Depth is read at the same distorted position as the colour, so the fog always matches the surface it covers.

The distortion is animated by the engine's real-time clock. The effect does not request new frames; the caller must
keep rendering (for example, from a game loop) for the motion to be visible. The phase is wrapped to one cycle
before it is sent to the GPU, so precision does not degrade after long run times.

## Adding an effect

1. Add a settings struct next to `UnderwaterProps` in `post_processing.h`, with `m_enabled`, defaults,
   `Validate()`, and a defaulted `operator ==`. Add get/set members to `PostProcessing`.
2. Add a shader in `src/shaders/hlsl/postprocessing/` with a shared `*_cbuf.hlsli` constant buffer. Reuse
   `VSPostEffect` for the full-screen triangle. Register the entry points in `view3d-12.vcxproj` and `shader.h/.cpp`.
3. Add a `Record<Effect>` pass function and its PSO. Put it into the pass list in `PostProcessing::Render` at its
   canonical position, and grow the pass array. Passes read `t0` (colour) and `t1` (depth) and write the given
   render target; they must not assume which target is the scene colour.
4. Add `View3D_PostEffect<Effect>Get/Set` to the DLL, a matching window accessor, and a C# struct and property in
   `Rylogic.Gfx`. Keep the DLL and C# struct layouts identical and cover them with layout tests.
5. Add GPU pixel tests to `view3d-fade-tests`.

The planned order puts depth-dependent effects that need a clean image first (ambient occlusion, depth of field,
motion blur) and whole-screen colour effects (such as underwater) last.

## Validation

Build `projects\tests\view3d-fade-tests\view3d-fade-tests.vcxproj` (VS 2026, v145, Debug/x64), then run
`obj\x64\Debug\view3d-fade-tests.exe --post-effects`. The invisible-window fixture reads rendered pixels at
1x and 4x MSAA. It covers defaults, invalid-settings rejection, disabled-image equality, tint, depth fog,
background fog, overlay exclusion, and restoring the image when the effect is disabled. GPU debug-layer errors fail
the fixture.

Build `projects\rylogic\Rylogic.Gfx\Rylogic.Gfx.csproj` in Debug to run the inline managed validation and
struct-layout tests.
