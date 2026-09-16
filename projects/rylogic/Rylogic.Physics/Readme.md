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

Gravity commands apply force for one frame, not persistent acceleration. Submit gravity each frame. A nonzero initial `BodyOptions.Gravity` also adds force
for the first frame, so do not apply it twice when using a per-frame gravity command.
