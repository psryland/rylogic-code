# Rylogic.Physics

Managed, allocation-conscious ownership and stepping APIs for the Rylogic rigid-body physics engine.

`Physics` owns the native DLL context. Each `Engine` owns a rigid-body world, private GPU queue and synchronization resources, and all `Shape` and `RigidBody`
objects created through it. Mutable and lifetime operations must run on the OS thread that created the engine; completed snapshots, events, configuration, and
diagnostics use caller-owned buffers or immutable values.

An engine can create its own D3D12 device or take an independent COM reference from a `Rylogic.D3D12.DeviceLease`. The engine never borrows a naked device
pointer beyond creation, and drains pending GPU work before checkpoint or disposal. `BeginStep`/`CompleteStep` support explicit host scheduling while `Step`
provides the synchronous path.

The package depends on `Rylogic.Core`, `Rylogic.D3D12`, and `Rylogic.Native`. It does not depend on Rylogic.Gfx, View3D, WPF, or application-specific code.

`Engine.SetTerrain(TerrainConfiguration?)` installs a copied procedural terrain definition; null removes it. Zero surface spacing selects the native shared
sampling default. Replacement requires the engine's owner thread and no pending step. The native DLL embeds both terrain and common shader includes, so
terrain compute requires neither a source checkout nor a loaded View3D DLL.

Terrain-equipped engines currently reject native checkpoint capture and import because that format does not serialize terrain. Applications that own
recipe/body persistence must reconstruct terrain before restoring body construction and completed dynamics; they must not treat an incomplete native
checkpoint as a full world save.

`Engine.SetCylindricalBoundary(CylindricalBoundaryConfiguration?)` independently installs an inward-facing, infinite-height cylinder centered at
`(centre_x, centre_y)`. Radius and all envelope distances are metres. It supports the same physical primitive surfaces, transformed compound leaves, and
articulation proxies as terrain. Contacts carry each leaf's material and the configured wall material through the ordinary solver; there is no position clamp,
finite-height rim, or polygonal ring. Terrain and boundary share one owned shapeless static endpoint, shape plans, instance stream, and contact reduction.

The defaults are **0.05 m** boundary spacing, **0.1 m** maximum horizontal surface motion per internal substep, and a **0.25 m gross penetration-rejection
cutoff**. That cutoff is not acceptable overlap or a spawn/restore allowance. Vertical translation is unrestricted. Terrain retains its independent **0.16 m**
default spacing; adding a finer wall does not increase terrain sample density. See the [native surface contract](../../../include/pr/physics/surface/README.md#sampled-cylindrical-world-boundary)
for motion bounds, configuration validation, actual-surface containment, transactional failure, caching, and checkpoint requirements.

Gravity commands apply force for one frame, not persistent acceleration. Submit gravity each frame. A nonzero initial `BodyOptions.Gravity` also adds force
for the first frame, so do not apply it twice when using a per-frame gravity command.
