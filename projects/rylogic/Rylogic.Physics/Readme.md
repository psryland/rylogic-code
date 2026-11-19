# Rylogic.Physics

Managed, allocation-conscious ownership and stepping APIs for the Rylogic rigid-body physics engine.

`Physics` owns the native DLL context. Each `Engine` owns a rigid-body world, private GPU job and synchronization resources, and all `Shape` and `RigidBody`
objects created through it. Mutable and lifetime operations must run on the OS thread that created the engine; completed snapshots, events, configuration, and
diagnostics use caller-owned buffers or immutable values.

An engine can create its own D3D12 device or take an independent COM reference from a `Rylogic.D3D12.DeviceLease`. The engine never borrows a naked device
pointer beyond creation, and drains pending GPU work before checkpoint or disposal. `BeginStep`/`CompleteStep` support explicit host scheduling while `Step`
provides the synchronous path.

## Explicit GPU cleanup

The native and managed runtimes must use the same ABI version (**9**). Engines using the same explicitly supplied device may share a queue, but their jobs,
fences and resources remain independently owned.

`Engine.TryDispose(timeout_ms)` permanently stops its work. `-1` requests an infinite wait; other nonnegative values
are millisecond budgets. Success invalidates its managed and native identities. A timeout returns `false`; another failure throws. Either leaves the same
owner intact for another close attempt, not for resumed simulation. `Dispose()` explicitly chooses the infinite wait. No cleanup path executes a partly
recorded command list, replays a submission, or terminates the process to handle a GPU error. Device removal permits resource retirement.

`Physics.TryDispose` shares one budget across its engines. A refused final-context release also leaves its token valid. SafeHandle finalizers do not wait
for GPU completion: unfinished engines remain in the native registry. Call `Physics.CollectAbandoned(timeout_ms)` to retry cleanup explicitly; it returns
the number of retained owners plus busy engines still requiring inspection, and throws on another error. Collection does not wait behind active engine
operations, even with an infinite GPU budget; retry when those operations finish. A new `Physics` instance can collect abandoned work after the original runtime has been
finalized. This never collects a live caller-owned object. Ignoring failed cleanup deliberately retains memory until a later collection or process exit.

Native callers use the timeout argument of `Physics_EngineDestroy`, the checked `Physics_Shutdown` status, and
`Physics_CollectAbandoned`. Do not unload the DLL while it owns retained work. Initialization error callbacks are synchronous and are not retained for shutdown.

Direct C++ users retain an engine and its dependent resources until `TryClose` succeeds, then destroy them. See `GpuJob::TryClose`.
The atmosphere solver lives in the Rylogic game-systems library; its managed wrapper is `Rylogic.Gfx.View3d.Atmosphere`.

## Simulation and dependencies

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

Gravity is a persistent acceleration: `BodyOptions.Gravity`, `SetGravity` and `SetState` store it, and every step applies it to each awake body.
Setting it again replaces the value; it never adds a second force.

`CopyEvents` reports procedural terrain and boundary collisions as `EPhysicsEvent.WorldContact`. Exactly one body handle is invalid: it identifies the
engine-owned world endpoint, not a missing user body. Ordinary `Contact` events retain two valid body handles. Both use world-space points, an A-to-B normal,
leaf materials, and the zero-based generating substep. The shapeless world endpoint has no compound child identity. These are generated contact records,
not normal-load or solved-impulse measurements; contact absence is not an airborne guarantee for sleeping bodies or an incomplete event stream.
World-contact geometry retains the generating pose before positional correction; it is not remapped to the final body snapshot.
