# Physics sandbox

## Sampled terrain stress demo

Choose **Demos -> Stress and Scaling -> Sampled Terrain (1,000 Bodies)**, or load
`scenes\terrain_samples_1000.json` (`terrain-samples-1000` catalogue command). It deterministically
spawns 200 each of boxes, spheres, capsules, octahedral polytopes and triangles, all 1 kg,
in 25 separated rows of 40 objects over a 30 x 18 m footprint at heights 5-6 m.
Outer dimensions are at most about 0.7 m. Gravity is -9.81 Z, elasticity 0.05, friction 0.3,
with four internal substeps and capacity for 65,536 collision pairs/contacts.

This demo uses the shared **0.16 m surface-spacing default**, as do terrain, buoyancy and the surface overlay.
Volume sampling is unchanged. The overlay now uses the same density as this demo's terrain sampling; it remains a
geometry display rather than a list of active contacts. All feature points are retained by the shared plan builder, not decimated.

| Primitive (200 each) | Samples/body at 0.1 m | Samples/body at 0.16 m |
| --- | ---: | ---: |
| Box | 294 | 150 |
| Sphere | 486 | 216 |
| Capsule | 230 | 125 |
| Octahedral polytope | 880 | 464 |
| Triangle | 182 | 90 |
| **Total for 1,000 bodies** | **414,400** | **209,000 (50.43%)** |

Counts are actual shared-plan populations; there is no per-query execution counter.
The area-density estimate `0.1*sqrt(2)` gives 230,000 samples here, so quantized subdivisions
and feature contributions require a slightly larger spacing to approach half the original count.

A bounded Release A-B-B-A comparison on an RTX 3080 Ti (600 frames/run, four substeps,
first 120 frames excluded) measured median warmed interval means of **4.20 -> 2.03 ms**
for terrain GPU work and **26.14 -> 23.79 ms** for whole physics. These are headless timings,
not interactive FPS. Final report-boundary penetration on the independent 0.05 m references
rose from **5.4-5.9 mm to 10.4-10.9 mm**; all runs retained 1,000 finite bodies with downhill
movement and no capacity failure. Neither this reference grid nor report cadence proves a
global worst-case penetration bound. The reduced-density terrain demo was visually approved.

The asset-free terrain has 4 m regional wavelength and 1.6 m amplitude, with the other spatial
bands disabled and the fixed family datum offset near zero. Its many slopes and hollows allow
spheres/capsules to roll and other shapes to slide/tumble. There is no artificial settling
damping, rolling resistance, water or hidden ground collider. The render mesh uses the same
canonical recipe/height/normal as collision but is only a visual approximation.

The existing `scene.terrain` block now installs physical terrain as well as its preview.
`surface_spacing` is finite and positive and inherits the shared **0.16 m** default. An explicit scene override affects
terrain only, not buoyancy or overlays.
Optional `recipe` settings are `sea_level_bias`, `uplift_height`, `mountain_base_height`,
`supported_coordinate_abs`, and the bands `regional_base`, `region_selector`, `region_uplift`,
`plains`, `hills`, `mountains`, `domain_warp`. Bands accept `amplitude`, `wavelength`, `octaves`,
`lacunarity`, `persistence`; mountains also accept `roundness`, `weight_gain`. Domain-warp amplitude
is measured in metres. The complete demo JSON is a reproducible example.
Invalid recipes/spacing fail scene loading. See `include\pr\physics\terrain\landscape\README.md`
for coverage, manifold/resource limits, ownership and discrete/substep limitations.

For a renderer-free bounded run:

```text
physics-sandbox.exe -scenediag -scene projects\tests\physics-sandbox\scenes\terrain_samples_1000.json -steps 600 -report 60 -engine_profile
```

`terrain_metric` rows measure retained dynamic bodies, sleeping/downhill counts, displacement,
height drop, finite state, mechanical energy and maximum normal penetration on an independent
0.05 m surface reference grid using CPU FP64 terrain queries. These checks happen at report
boundaries, not continuously, and are excluded from timed physics steps. Profile CSV adds
`terrain_gpu_ms` (GPU timestamps across all substeps); other timings remain host durations.
Omit `-scan` for timing because contact-event collection has a significant independent cost.
Headless physics rate does not measure interactive/rendered FPS. Sample overlays start off.
Capsule surface points/normals are supported; capsule volume sampling remains unsupported as described below.

## Rigid-body display

Thick lines are rendered as capsules with a cylindrical segment and hemispherical ends. The renderer reuses one exact mesh for each distinct
length/radius pair, rather than stretching a unit capsule: a 2 m segment with a 0.1 m radius extends 1.1 m from its centre in either axial
direction. Boxes, spheres and zero-radius lines still use shared meshes scaled per instance. Graphics do not change collision geometry.

## Sample overlays

The **View → Overlays and diagnostics** submenu has three independent checked options:

- **Surface samples + normals**: amber sample points and green outward-normal segments, 0.1 scene units long.
- **Volume samples**: blue interior sample points.
- **Sleeping-body transparency**: the existing reduced-opacity sleep diagnostic, enabled by default. Turning it off restores the ordinary body colour/opacity
  regardless of sleep state. It applies to rigid bodies and articulation links, preserves contact-priority colours, and never changes or wakes physics state.

All three options work together with either **View → Normal** or **View → Contact Priority**, and retain their selections across scene reload/reset.
The two sample overlays start disabled and include sleeping dynamic rigid bodies and movable articulation links.
Static bodies/ground, fixed-world articulation chains and shapeless links are excluded. A fixed joint below a movable joint is included.
No buoyancy registration, water or submerged state is required. The **B** key retains its separate buoyancy wetness/culling/force diagnostic.

## Display sampling

This is a geometry visualization, not a new physics sampling policy.

- Surface geometry comes from `physics::surface::BuildPlan` and `EmitSurfaceSample` at the shared default spacing **0.16 scene units**.
  See `include/pr/physics/surface/README.md` for its feature, area-weight and coverage contracts. Every surviving sample is drawn; there is no display decimation.
  Corners/edges retain separate face-normal segments, rather than a single averaged normal.
- The display uses the default spacing explicitly: immutable per-registration buoyancy plans do not expose their spacing through a query API.
  Changing a buoyancy configuration therefore does not change these geometric overlays.
- Volume geometry uses the existing buoyancy `DistributeCounts`, `BuildVolumeSampleTable`, `SampleIndex` and `EmitVolumeSample` functions:
  **8192 samples per collision hull**, allocated in proportion to child volumes. Existing per-child rounding and overlap ownership may reduce the final count.
  Display seed/hull identity is **0** so the same shape can share one cached model. The distribution is the existing one, but need not match a registered hull's exact sample identities.
- Polytopes lacking volume tetrahedra get an owned, temporary exact face-fan decomposition through the existing collision builder, only when volume display is requested.
  Original collision geometry, physics caches, volume counts/distribution, forces and sleep state are not modified.
- Compound children already store shape-to-root transforms. These are applied once during geometry creation; rigid-body or link root-to-world transforms
  are applied by persistent renderer instances. The array's transform is not applied again.
- Existing compound ownership rules remove enclosed/overlapped surface contributions and volume samples owned by earlier children. These are geometry ownership
  tests, not wetness filters. The display inherits their finite epsilon and coincident-boundary behavior.

Surface display supports boxes, spheres, capsules/thin lines, triangles, convex polytopes and nested arrays of those primitives.
Every leaf's shape-to-root transform is applied once. Capsules show their true outward normals; zero-area points/segments show positions without invented normal lines.
Surface ownership includes capsule containment, so enclosed sibling surfaces are not displayed.
Volume display retains its existing box/sphere/polytope emitter and flat-array traversal; triangles have no interior samples.
Lines/capsules and nested arrays remain explicitly unsupported for volume display. No capsule volume distribution is approximated or added.
Unsupported geometry, invalid plans, allocation failures or unrepresentable renderer buffers are reported
in the status bar with the affected target count, overlay type and first reason. Surface and volume models are built and fail independently:
enabling unsupported capsule volume display does not hide its valid surface points/normals. An affected overlay is not partially displayed within a hull,
and failed unchanged overlays are not retried every frame.
Very large valid shapes may take time and memory to visualize; this is not a throughput promise.

## Cache and renderer ownership

The scene owns one immutable model pair per collision-shape pointer and separate persistent instances per target. CPU sample vectors are discarded after upload.
Steady frames only select targets, look up cached objects and update transforms; sampling/model construction happens on first use or after invalidation.
The two options invalidate the cache when changed. Scene replacement clears it before releasing shapes; `Body::Shape` changes mark it stale.
Direct mutation of otherwise immutable shape storage must also call `m_sample_overlays.Invalidate()`.

The UI completes GPU work and clears draw lists before reset/load/test-scenario replacement or destruction. Deferred overlay invalidation is consumed after
clearing the previous draw lists and waiting for the GPU. Instances are not retired when bodies sleep or move offscreen.
Overlays disable depth testing/writing and explicitly use the post-alpha sort group so opaque geometry does not hide interior points.

For a shape with `S` emitted samples and `K` convex children, first-use ownership filtering performs at most `O(S*K)` containment queries, plus each query's
primitive cost. Surface-plan setup is inherited from the shared sampler; volume polytope CDF lookup is logarithmic in tetrahedron count.
Uploaded geometry and transient generation storage are `O(S)`. Steady-frame overlay work is expected `O(number of targets)` hash lookup/transform work,
excluding rendering; articulation mobility is cached at scene construction.

## Validation and manual check

The sandbox owns `SampleOverlayTests`, `SampleOverlayMenuTests` and `SceneSampleOverlayTests`; select them with
`physics-sandbox.exe -unittests SampleOverlayTests SampleOverlayMenuTests SceneSampleOverlayTests`.
They cover menu independence, sleeping/static selection, articulation mobility, compound transforms and incident normals, ownership,
unchanged volume emission, surface-only polytope derivation, deterministic output, real renderer model sharing and reset/shape invalidation.
Sleeping-transparency tests also check its enabled default, option independence, preserved RGB/source opacity and unchanged sleeping/force state.
An actual off-screen GPU readback checks surface points, normals and volume points inside an opaque occluder. Its paired negative cases deliberately
select the pre-opaque sort group and verify the same geometry is hidden, proving draw order rather than merely inspecting depth flags.
Capsule tests also check shared-emitter parity, nested transforms, surface-union ownership, thin-line points-only rendering, and persistent surface instances
when the independent volume overlay reports an unsupported-shape error.

Paul should visually check both options individually and together in a mixed/compound scene, with each base view mode and a sleeping body.
Check that normals point outward, volume points remain visible through the regular object rendering, and resetting or loading a scene with both options
enabled updates the clouds without leaving old objects. No automated screenshot or interactive-app driving is required.
Toggle **Sleeping-body transparency** off/on with sleeping rigid bodies and articulation links; verify their ordinary appearance is restored without waking them.
