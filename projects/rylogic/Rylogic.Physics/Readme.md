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
`(centre_x, centre_y)`. Radius and sampling spacing are metres. It supports the same physical primitive surfaces, transformed compound leaves, and
articulation proxies as terrain. Contacts carry each leaf's material and the configured wall material through the ordinary solver; there is no position clamp,
finite-height rim, or polygonal ring. Terrain and boundary share one owned shapeless static endpoint, shape plans, instance stream, and contact reduction.

Boundary spacing defaults to **0.05 m**, independently of terrain's **0.16 m**. Contacts depend on valid current geometry, not speed, timestep or a
penetration cutoff. The entire cylinder exterior is forbidden; deep exterior samples remain contacts. This is discrete detection, not continuous
time-of-impact detection. Callers own timestep/accuracy choices and accepted spawn, restore and completed-state overlap.
See the [native surface contract](../../../include/pr/physics/surface/README.md#sampled-cylindrical-world-boundary) for numerical/resource requirements,
failure-source diagnostics, caching and checkpoint requirements. API version 3 removes the motion/penetration fields; the boundary descriptor's size
is 40 bytes. Descriptor-header version 2 is unchanged because it is also embedded in native checkpoint format 3. No serialized layout changes, and
native checkpoints still exclude configured world surfaces.

Gravity commands apply force for one frame, not persistent acceleration. Submit gravity each frame. A nonzero initial `BodyOptions.Gravity` also adds force
for the first frame, so do not apply it twice when using a per-frame gravity command.

`CopyEvents` reports procedural terrain and boundary collisions as `EPhysicsEvent.WorldContact`. Exactly one body handle is invalid: it identifies the
engine-owned world endpoint, not a missing user body. Ordinary `Contact` events retain two valid body handles. Both use world-space points, an A-to-B normal,
leaf materials, and the zero-based generating substep. The shapeless world endpoint has no compound child identity. These are generated contact records,
not normal-load or solved-impulse measurements; contact absence is not an airborne guarantee for sleeping bodies or an incomplete event stream.
World-contact geometry retains the generating pose before positional correction; it is not remapped to the final body snapshot.
