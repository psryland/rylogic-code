# Rylogic.Physics

Managed, allocation-conscious ownership and stepping APIs for the Rylogic rigid-body physics engine.

`Physics` owns the native DLL context. Each `Engine` owns a rigid-body world, private GPU queue and synchronization resources, and all `Shape` and `RigidBody`
objects created through it. Mutable and lifetime operations must run on the OS thread that created the engine; completed snapshots, events, configuration, and
diagnostics use caller-owned buffers or immutable values.

An engine can create its own D3D12 device or take an independent COM reference from a `Rylogic.D3D12.DeviceLease`. The engine never borrows a naked device
pointer beyond creation, and drains pending GPU work before checkpoint or disposal. `BeginStep`/`CompleteStep` support explicit host scheduling while `Step`
provides the synchronous path.

`BeginStep` and `Step` require finite positive elapsed seconds and a positive representable `float` duration after substep division. Invalid timing is
rejected before applying body commands, consuming forces or kinematic targets, or replacing completed snapshots, events and counters. Zero elapsed time
is not a maintenance operation; callers must not use stepping to clear forces without advancing physical time.

API version **0x000E0000** combines the coordinated timing/owner-tagged rejection contract with host startup and explicit recovery.
Checkpoint version **6** reflects the config layout without per-engine process diagnostics; older checkpoints are rejected, not reinterpreted.
Descriptor-header version remains 3. Public quaternion-joint exposure and separate position/velocity wire dimensions remain deferred; the DLL creates axes joints.
`EPhysicsEvent.ConstraintRejection` reports the first non-terminal rejection per persistent owner per completed frame. An ordinary owner is identified by
its stable native slot and generation, with a constraint handle. A coupled owner uses a frame-local island index, generation zero and no invented constraint
handle. `AdmissionRejected` has no numerical reason or numerical details. `SolveRejected` preserves the applicable phase, reason, actual numerical attempt
count, relaxation and merit; later no-op retry dispatches do not increase that count. A later accepted solve does not erase the earlier report.
`CopyEvents` is non-draining, and an undersized destination receives no partial copy. Fatal solver/capacity failures belong to terminal diagnostics rather
than this stream; no terminal failure alone is not proof that a frame reached caller-owned publication.

Constraint breaks sample final accepted loads once per completed internal substep, after main and selective work. Peaks cover those samples, not rejected
proposals or transient accepted intermediate solves. Historical-only caches do not contribute. Thresholds remain strict, and the uploaded graph stays an
inactive scheduling superset after a break until the next frame rebuild. These ownership rules cover persistent constraints, not separate contact warm caches.

The package depends on `Rylogic.Core`, `Rylogic.D3D12`, and `Rylogic.Native`. It does not depend on Rylogic.Gfx, View3D, WPF, or application-specific code.

The host may call `Physics.ConfigureProcessDiagnostics(EProcessDiagnostics.DebugLayer)` once at startup, before **any** module creates a D3D12 device.
`GpuValidation` additionally enables GPU instrumentation. `None` leaves externally enabled diagnostics unchanged. No `Physics` instance is needed.
Repeated/late requests visible to the DLL fail, and unavailable interfaces fail before any setting changes; cross-module ordering remains the host's responsibility.
Engine construction never changes process policy. Borrowed devices retain their owner's message settings.

`Engine.RetireInterruptedWork()` explicitly retires GPU work interrupted by a failed call. It runs on the owner thread between frames, makes one attempt,
and never replays a frame. Recording/resource-releasing operations remain blocked until success; disposal remains valid.
A failed operation reports its primary error first and appends its own failed-retirement reason. Old recovery errors are not attributed to later calls.

Atmosphere input rejections preserve the existing atmosphere. A failure during potentially partial atmosphere work instead invalidates that owner:
later atmosphere operations report `InternalError`, and the caller must destroy it and create a new one. A nested callback's input-rejection exception
cannot disguise a partial outer operation as a harmless rejection. Native atmosphere methods return explicit input checks for this distinction.

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
failure-source diagnostics, caching and checkpoint requirements. The boundary descriptor is 40 bytes and uses the current descriptor-header version 3.
Native checkpoints still exclude configured world surfaces.

Gravity is a persistent acceleration: `BodyOptions.Gravity`, `SetGravity` and `SetState` store it, and every step applies it to each awake body.
Setting it again replaces the value; it never adds a second force.

`CopyEvents` reports procedural terrain and boundary collisions as `EPhysicsEvent.WorldContact`. Exactly one body handle is invalid: it identifies the
engine-owned world endpoint, not a missing user body. Ordinary `Contact` events retain two valid body handles. Both use world-space points, an A-to-B normal,
leaf materials, and the zero-based generating substep. The shapeless world endpoint has no compound child identity. These are generated contact records,
not normal-load or solved-impulse measurements; contact absence is not an airborne guarantee for sleeping bodies or an incomplete event stream.
World-contact geometry retains the generating pose before positional correction; it is not remapped to the final body snapshot.
