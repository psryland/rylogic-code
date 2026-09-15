# Physics sandbox sample overlays

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

- Surface geometry comes from `physics::surface::BuildPlan` and `EmitSurfaceSample` at the shared default spacing **0.1 scene units**.
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

Supported shapes are boxes, spheres, triangles, convex polytopes, and arrays of those primitives. Triangles have no interior volume samples.
Unsupported shapes (including line collision envelopes and nested arrays), invalid plans, allocation failures or unrepresentable renderer buffers are reported
in the status bar with the affected target count and first reason. An affected hull is not partially displayed, and failed unchanged shapes are not retried every frame.
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

Paul should visually check both options individually and together in a mixed/compound scene, with each base view mode and a sleeping body.
Check that normals point outward, volume points remain visible through the regular object rendering, and resetting or loading a scene with both options
enabled updates the clouds without leaving old objects. No automated screenshot or interactive-app driving is required.
Toggle **Sleeping-body transparency** off/on with sleeping rigid bodies and articulation links; verify their ordinary appearance is restored without waking them.
