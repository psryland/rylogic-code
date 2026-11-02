# Physics Kinematic Bodies and Collision Filtering Plan

Status: **In progress** (2026-11-02). Branch `pr/physics-kinematic-bodies`.

Labels used in this document:

- **[Verified]** — checked in the current code. File and approximate line references are given.
- **[Proposal]** — a design decision for this plan. It is not implemented.

## Summary

The engine accepts `EMotionType::Kinematic`, but it does not implement kinematic behaviour. A kinematic body today is a static body (infinite mass) that the caller can
teleport. It has zero velocity in contacts, it cannot wake sleeping bodies, and it cannot carry objects by friction. There is also no public way to stop two bodies
from colliding, although an internal pair-exclusion table already exists for constraints and articulations.

This plan adds:

1. Real kinematic bodies: infinite mass in the solver, no gravity or forces, and a pose driven by targets. The engine derives the body velocity from target-to-target
   motion, so contacts and friction see the true velocity.
2. Runtime switching between Dynamic and Kinematic, with a defined state reset and a release velocity.
3. Per-body collision filtering: layer/mask bits plus explicit ignore pairs. The ignore pairs reuse the existing exclusion table.
4. Contact impulses in events, so a game can later feed the collisions of a carried object back to the carrier. The feedback design itself is out of scope.

## Use Case

Petri (`E:\Rylogic\petri1`) has an Actor. Physically, an Actor is a dynamic capsule with a D6 posture constraint (`adapters\Petri.Physics.Rylogic\src\ActorDynamics.cs`).
Toys are dynamic bodies created in `ActorDynamics.AddToy` (~line 1388), for example a 60 g hollow sphere with radius 0.15 m. Petri reads contacts with
`Engine.CopyEvents` (`ReadSupport`, ~line 920). [Verified]

Required behaviour when an Actor carries a toy:

| Phase | Toy behaviour |
| --- | --- |
| Pick up | Switch the toy from Dynamic to Kinematic. Ignore collisions between the toy and the carrier's capsule. |
| Carry | Each step, set the toy's target pose from the hands. The toy collides with the world and other bodies, pushes them, and carries them by friction. |
| Release | Switch back to Dynamic with the hands' velocity. Collision with the carrier comes back (maybe after a short delay, see open questions). |
| Later | The toy's contacts push back on the carrier. This is a separate large task. This plan only provides the data and hooks. |

## Current State

### Public ABI and managed API [Verified]

- `include\pr\physics\physics-dll.h`: `PHYSICS_API_VERSION = 0x00070000`, `PHYSICS_STRUCT_VERSION = 2`, `PHYSICS_CHECKPOINT_VERSION = 3`.
- `EMotionType { Static = 0, Dynamic = 1, Kinematic = 2 }`. `EBodyFlags { Enabled = 1, Sleeping = 2, NeverSleep = 4 }`.
- `ECommand` has `SetKinematicTransform = 7` and `SetEnabled = 8`. There is no command to change the motion type.
- `BodyDesc`, `BodyState` and `BodySnapshot` have a `motion_type` field. None of them has collision filter fields.
- `EConstraintFlags::CollideConnected` and `EArticulationLinkFlags::CollideParent/CollideSelf` are the only public collision filters.
- `Event` has normal, points, depth, materials, child indices and substep index. It has **no impulse and no relative velocity**.
- Managed `Rylogic.Physics\src\Types.cs` mirrors these: `EMotionType`, `BodyOptions.MotionType` (default Dynamic), `BodyCommand.SetKinematicTransform(...)`,
  `RigidBodyState.m_motion_type`, and `PhysicsEvent` (same fields as the native `Event`). `RigidBody.SetState(RigidBodyState)` sends a full `BodyState`.

### Interop [Verified]

`projects\rylogic\physics\src\dll\interop.cpp`:

- `BodyRecord` (~line 223) stores `m_motion_type`. The native `RigidBody` has **no motion type**. The type is only stored, and only checked by `SetKinematicTransform`.
- `Physics_BodyCreate` (~line 3107) starts the body with `Inertia::Infinite()` and computes mass only when the type is Dynamic. So Static and Kinematic bodies both get
  infinite mass. `RigidBody::SetMassProperties` (`rigid_body.cpp` ~line 444) then sets `ERigidBodyStateFlags::Static`.
- `ApplyBodyState` (~line 1655) copies the inertia from the caller's state and stores the motion type separately. A switch to Kinematic through `SetState` keeps a
  finite inertia, so the body still behaves as dynamic. A switch to Dynamic needs the caller to give a valid inertia.
- `ApplyCommands` (~line 1700): `SetKinematicTransform` checks the type, then sets `O2W` directly. This is a teleport. No velocity is derived.
- `BeginStep` (~line 1795) skips disabled bodies. The snapshot fill is at ~line 1894. The checkpoint stores one `PhysicsBodyState` per body (~line 1969).

### Solver, integration, broadphase and sleep [Verified]

All under `projects\rylogic\physics\src\`:

- Body velocity is always `InertiaInvWS * MomentumWS` (`rigid_body.cpp`). The GPU body (`compute\physics_types.hlsli` ~line 168) stores `momentum_ang/lin`, inverse
  inertia, and `os_com_and_invmass.w`. Pack/unpack is in `compute\physics_types.h` (~lines 394-443).
- `integrate.hlsl` `CSIntegrate` (~line 207): half-kick, sleep check, drift with `v = invmass * momentum`. With `invmass = 0` the body never moves. Gravity is zero
  for infinite-mass bodies (`RigidBody::GravityForceWS`, ~line 312).
- `resolve.hlsl` `BodyVelocityAtPoint` (~line 269) uses `invmass * momentum`, so an infinite-mass body always has **zero contact velocity**. Pushing comes only from
  penetration correction after a teleport, and friction cannot carry objects. Position correction is split by inverse mass (~line 700), so infinite-mass bodies are
  not moved. The warm-start cache is keyed by body pair and child (~lines 423-466). `warmstart_impulse` holds the accumulated impulse.
- The same velocity-from-momentum rule is repeated in: `coupled_contact.hlsl` (~lines 301, 506), `selective.hlsl` (~line 71), `frame_output.hlsl`
  `CollisionEventRelativeVelocity` (~line 61), `sleep.hlsl` `LowVelocity` (~line 55), `integrate.hlsl` `LowVelocity` (~line 157), and the constraint solvers
  (`constraint_solver.hlsl` ~line 86 and the `coupled_constraint_*` files), which test the Static flag.
- `sweep.hlsl`: a body is dynamic when `invmass > 0` and not Static. `StorePair` drops a pair when **neither** body is effectively awake. So kinematic-vs-static and
  kinematic-vs-kinematic pairs are already dropped. But a kinematic body touching a **sleeping** dynamic body is also dropped, so it can never wake it.
- `sleep.hlsl`: islands only contain dynamic bodies. `CSDisturbIslands` only uses awake dynamic bodies, so kinematic bodies cannot disturb sleeping islands. The host
  side is `integrator\engine_buffer_cache.h` (`CountAwakeDynamic`, `PackSleepIsland`, ~lines 226-236).
- World/terrain contacts (`surface\gpu_world_contacts.cpp` ~line 233) skip bodies with `invmass == 0`.

### Existing pair exclusion [Verified]

- `GpuCollisionExclusion { body_idx_a_plus_one, body_idx_b_plus_one }` is an open-addressed hash table. It is built by `BuildGpuCollisionExclusions`
  (`constraint\constraint_gpu.cpp` ~line 284) from enabled D6 constraints without `CollideConnected` and from articulation parent links (`engine.cpp` ~lines 790-870).
- It is uploaded every step and tested in `sweep.hlsl` through `GpuSortAndSweep::Sweep(..., collision_exclusions)` (`sweep_gpu.cpp`).
- The only public way to use it today is a D6 constraint with all axes Free and `CollideConnected` unset. This works but costs a constraint row set per pair.

### Step order [Verified]

`integrator\engine.cpp` `BeginStepInternal` (~lines 675-1095): Pack, Upload, then per substep: seed forces, water and external forces, Integrate(dt), articulation
integrate, sleep/wake, broadphase (with exclusions), collide, resolve, selective refresh (last substep), sleep update, capture events. Then readback. `CompleteStep`
unpacks. `dt = elapsed / substep_count`.

### Corrections to the earlier notes

| Earlier note | Actual behaviour |
| --- | --- |
| Kinematic bodies are integrated like dynamic bodies. | Bodies created as Kinematic have infinite mass and the Static flag, so they are **not integrated**. They act as static bodies that can be teleported. Only a body switched to Kinematic by `SetState` with finite inertia is integrated like a dynamic body. |
| Gravity is applied to kinematic bodies. | Not for bodies created as Kinematic (infinite mass gives zero gravity). Yes for the `SetState` case above. |
| There is no collision filtering. | There is no **public** per-body filter. An internal pair-exclusion table exists and is used by constraints and articulations. |

## Goals

- A kinematic body follows caller targets exactly at the end of each step.
- Contacts and friction see the kinematic body's real velocity. It pushes dynamic bodies and carries them, but it is never pushed.
- No gravity, forces or impulses change a kinematic body. No contacts are made between kinematic and static, kinematic, or world (terrain/boundary) surfaces.
- A moving kinematic body wakes sleeping bodies it touches. A still kinematic body lets bodies around it sleep.
- Motion type can be switched at runtime with a clear, simple state reset and a release velocity.
- Per-body collision filtering, working on the GPU broadphase and world contacts.
- Events carry enough data (impulse, relative velocity) to build carrier feedback later.
- Checkpoints round-trip all new state.

## Non-Goals

- Two-way coupling between a carried object and its carrier. Only options and hooks are listed (see Contact Feedback).
- Per-shape (per compound child) filtering. Per-body filtering is enough for the use case.
- Kinematic articulation links.
- Continuous collision detection for fast kinematic motion. Large target jumps can still tunnel; callers own the speed limits.
- Kinematic bodies inside D6 constraints beyond treating them as infinite-mass, velocity-driven anchors (this follows from the shared velocity helper).

## Kinematic Semantics [Proposal]

### Representation

- Add `ERigidBodyStateFlags::Kinematic = 1 << 4` in `include\pr\physics\rigid_body\state_flags.h` and mirror it in `compute\physics_types.hlsli`. The native
  `RigidBody` becomes the owner of the motion type. `BodyRecord::m_motion_type` is removed and the ABI value is derived from the flags.
- A kinematic body has infinite mass and inertia (`invmass = 0`, zero inverse inertia). The Static flag is **clear**; Static means "never moves".
- The kinematic velocity (twist: angular velocity, and linear velocity of the centre of mass, in world space) is stored in the `momentum_ang/lin` slots. These slots
  have no use when `invmass = 0`, so the GPU body size and layout stay the same. On the CPU, `RigidBody` gets a clear accessor (`KinematicTwistWS`) rather than
  reusing the name "momentum".
- The body keeps its finite mass properties in a separate field (`m_dynamic_inertia`, CPU only) so a switch back to Dynamic can restore them without the caller.

### One velocity helper

Add `BodyTwistWS(GpuRigidBody body)` (HLSL) and `RigidBody::VelocityWS()` (CPU, already the single path) that return the stored twist for kinematic bodies and
`Iinv * momentum` otherwise. Replace every velocity-from-momentum site listed in Current State with this helper. This one change gives correct contact velocity,
friction carry, restitution, event relative velocity, and constraint anchor velocity.

### Targets and integration

| Item | Rule |
| --- | --- |
| Target command | `SetKinematicTarget(body, o2w)`. Applied in `BeginStep`, where `elapsed` is known. The twist is `(target - current) / elapsed`: linear from the centre-of-mass move, angular from the log map of `R_target * R_current^T`. |
| Velocity command | `SetVelocity` on a kinematic body sets the twist directly (constant-velocity motion with no target). |
| No target this step | The twist from the last step is kept, so the body moves on at constant velocity. Callers that want it to stop must send a zero velocity or a target equal to the current pose. |
| Substeps | Each substep drifts the pose by `twist * dt` (rotation by the exp map). After `n` substeps the pose equals the target to rounding error. `CompleteStep` (unpack) snaps the pose exactly to the stored target. |
| Forces | `CSSeedWorkingForces`, external forces, water and gravity skip kinematic bodies. `SetForce`, `ApplyForce`, `ApplyImpulse` and `SetMomentum` on a kinematic body are caller errors (return `InvalidArgument`). |
| Teleport | `SetTransform` on a kinematic body moves it with no velocity (twist unchanged). This is the current `SetKinematicTransform` behaviour. |

`SetKinematicTransform` is renamed to `SetKinematicTarget` because it no longer teleports. The enum value is kept in place but the meaning changes; the API version is bumped.

### Contacts

- Kinematic vs static, kinematic vs kinematic, and kinematic vs world (terrain, cylindrical boundary, water): no contacts. Broadphase already drops the first two once
  "awake" means "dynamic and awake". The world-contact gather already skips `invmass == 0`.
- Kinematic vs dynamic: normal contact. Inverse mass is zero on the kinematic side, so all impulse goes to the dynamic body. Position correction moves only the dynamic body.
- Shock propagation and contact links already skip `invmass <= 0` bodies; keep that.

### Sleep and islands

| Rule | Detail |
| --- | --- |
| Islands | Kinematic bodies never join islands (same as static). `PackSleepIsland` and the island union passes skip them. |
| Own sleep | A kinematic body is "moving" when its twist is non-zero. It has no sleep timer; it is asleep exactly when it is not moving. |
| Pair admission | In `sweep.hlsl` a pair is kept if either body is an awake dynamic body **or a moving kinematic body**. |
| Waking others | `CSDisturbIslands` treats a moving kinematic body like an awake dynamic body, so it wakes islands it overlaps. |
| Staying asleep | A still kinematic body does not keep neighbours awake; they can sleep on it like on static ground. |
| GPU work count | `CountAwakeDynamic` counts moving kinematic bodies so the step is not skipped while one moves. |

### Events

- Contacts with a kinematic body are reported as normal `Contact` events.
- Sleep/Wake events are not sent for kinematic bodies (their "sleep" is just "twist is zero").
- Snapshots report the twist as `velocity` and zero `momentum` for kinematic bodies.

## Switching Motion Type [Proposal]

Add an explicit command `ECommand::SetMotionType` (`flags` holds the new `EMotionType`, `value` holds the twist to use after the switch). `BodyState.motion_type` in
`Physics_BodyStateSet` uses the same code path, so both ways give the same result.

| Switch | State changes |
| --- | --- |
| Dynamic → Kinematic | Save the dynamic inertia. Set infinite inertia and the Kinematic flag. Clear forces and momentum. Twist = `value` (usually zero, or the current velocity). Remove the body from its island and wake that island. Drop warm-start entries that involve the body. Clear the Sleeping flag. |
| Kinematic → Dynamic | Restore the saved inertia (or compute it from the shape if none was saved). Clear the Kinematic flag. Set momentum from `value` (the release velocity) and clear forces. Drop warm-start entries for the body. Wake the body. |
| Any → Static | Infinite inertia, Static flag, zero momentum and twist, leave islands, drop warm-start entries. |
| Static → Dynamic/Kinematic | As above, with inertia computed from the shape and the body's mass settings. |

Notes:

- Warm-start entries are dropped because their impulses were computed with the old mass. In practice, the switch writes a per-body "reset" bit that the resolve pass uses
  to ignore cache hits for that body in the next step. This avoids a CPU walk of the GPU cache.
- `ApplyBodyState` is fixed so the stored inertia always agrees with the motion type: for Kinematic and Static the supplied inertia is saved as the dynamic inertia and the
  body uses infinite inertia.
- `BodyDesc` gains no new mass fields. A body created as Kinematic computes its dynamic inertia from `mass_mode`, `mass_distribution` and `mass_or_density` (today these
  are ignored for non-Dynamic types), so a later switch to Dynamic needs no extra data.
- Release velocity: the game sends `SetMotionType(Dynamic, hand_twist)`. The caller always gives the release velocity (see Decisions).

## Collision Filtering

### Options

| Option | Description | Pros | Cons |
| --- | --- | --- | --- |
| A. Layer/mask bits per body | `collision_layer` (bits the body is in) and `collision_mask` (bits it collides with). A pair collides when `(a.layer & b.mask) && (b.layer & a.mask)`. | One test in `StorePair`, no tables, also works for world contacts. Good for broad rules (e.g. "debris ignores debris"). | 32 groups only. Cannot express "this toy ignores this one Actor" when there are many Actors without using up bits. |
| B. Layer/mask per shape (compound child) | As A, per child. | Finer control. | Needs the filter in narrowphase per child pair, more GPU data. Not needed now. |
| C. Explicit ignore pairs | `Physics_BodyCollisionIgnore(a, b, ignore)`. | Exact. Reuses the existing exclusion table and tests. Matches the carrier/toy case directly. | Table rebuild cost grows with the number of pairs (small here). |
| D. Parent/attachment relation | Each body has an optional `attached_to` body; a body never collides with its attachment. | Simple for carrying. | A special case of C with one pair per body; would also suggest transform following, which this plan does not want. |

### Recommendation [Proposal]

Implement **A (body layer/mask) and C (ignore pairs)**. Do not implement B or D now.

- C is the minimum for the use case. It is exact, it reuses `GpuCollisionExclusion` and its tests, and it has no limit on the number of carriers.
- A is very cheap (one AND test), covers common game rules, and is the only option that can filter terrain/world contacts, where there is no second body handle.
- D is just C with one pair, so it adds a concept without adding power.

### Data and ABI [Proposal]

- `BodyDesc`, `BodyState`, `BodySnapshot`: add `uint32 collision_layer` (default `1`) and `uint32 collision_mask` (default `0xFFFFFFFF`). Add an engine-level
  `world_collision_layer` (default `1`) in `PhysicsEngineConfig` for terrain, boundary and water contacts.
- New functions:
  - `Physics_BodyCollisionIgnoreSet(engine, body_a, body_b, int ignore)` — add or remove an ignore pair. Idempotent.
  - `Physics_BodyCollisionIgnoreCount/Copy` — for checkpoint and debugging.
- Interop keeps a set of ignore pairs keyed by stable body handles. Destroying a body removes its pairs. Disabled bodies keep their pairs.

### Where the filter is applied [Proposal]

| Stage | Change |
| --- | --- |
| Pack (`engine.cpp`, `physics_types.h`) | Pack layer and mask into the GPU body. Reuse spare bits only if safe; otherwise add one `uint2 collision_filter` field (8 bytes) to `GpuRigidBody` and update the size `static_assert`s. Articulation proxies get layer/mask from their articulation (default all). |
| Exclusion table (`constraint_gpu.cpp`, `engine.cpp`) | Append public ignore pairs (mapped to dense body indices) to the same list as constraint and articulation exclusions before `BuildGpuCollisionExclusions`. |
| Broadphase (`sweep.hlsl`) | In `StorePair`, after the awake test: reject when the layer/mask test fails, then the existing `CollisionExcluded` lookup. The filtered sweep entry point is used whenever the table is not empty. |
| World contacts (`gpu_world_contacts.cpp/.hlsl`) | Skip a body when `(body.layer & world.mask) == 0 || (world.layer & body.mask) == 0`. |
| CPU reference (`broadphase_brute.cpp`, CPU narrowphase in `engine.cpp`) | Same test, so CPU and GPU tests agree. |

Filtering is done only in broadphase. Narrowphase, resolve and events need no change because filtered pairs never reach them.

## Contact Feedback for Carried Objects

This section lists options and hooks only. The design is a future task.

### Hooks this plan adds [Proposal]

- `Event` gains `normal_impulse` (sum over points of the accumulated normal impulse in the substep, from `warmstart_impulse`), `tangent_impulse` (v4), and
  `relative_velocity` (v4, at the first contact point, along A to B). Managed `PhysicsEvent` gets the same fields.
- Events where one body is kinematic are reported like any other contact, so the game can find all contacts of the carried toy by its handle.
- Optional: `Physics_BodyContactImpulseGet(body)` returns the total impulse and moment on a body over the last step, summed on the GPU. This avoids walking all events.
  It is useful for kinematic bodies, where the solver throws the impulse away.

### Options for the future design

| Option | Idea | Concerns |
| --- | --- | --- |
| Impulse relay | Next frame, the game applies the toy's contact impulses (and moments about the hand) to the carrier with `ApplyImpulse`. | One frame of latency. The kinematic toy always "wins" contacts, so impulses can be very large when the toy is driven into heavy or static objects. |
| Hold by constraint | Keep the toy dynamic and hold it with a D6 constraint to the carrier's hand frame (limits, motors, break force). | Two-way coupling for free. Light toys may jitter with a high mass ratio. Pose tracking is less exact than kinematic. |
| Compound attachment | Add the toy as a child shape of the carrier body while carried. | Exact coupling, but changes the carrier's shape and mass every pick-up/drop. Needs shape rebuilds. |
| Hybrid | Kinematic while free; switch to a constraint when a contact impulse passes a limit. | Two code paths, switching logic. |

### Open questions

1. Should the toy's push on heavy or static objects be limited (a maximum impulse), so a carried toy cannot push the world with infinite force?
2. Is one frame of latency acceptable for feedback, or is in-step coupling needed?
3. How should the carrier's posture controller treat feedback impulses: as a disturbance to resist, or as a reason to drop the toy?
4. Should a toy be able to hit a **different** Actor while carried (yes by default), and how does that Actor respond?
5. Per-substep events or per-step sums for feedback?

## ABI and Managed API Changes [Proposal]

| Item | Change |
| --- | --- |
| Versions | `PHYSICS_API_VERSION` → `0x00080000`, `PHYSICS_STRUCT_VERSION` → 3, `PHYSICS_CHECKPOINT_VERSION` → 4. No compatibility readers for old structs or checkpoints. |
| `ECommand` | Rename `SetKinematicTransform` → `SetKinematicTarget` (value 7). Add `SetMotionType` (11). |
| `BodyDesc` | Add `collision_layer`, `collision_mask`. Mass fields are now used for Kinematic too. |
| `BodyState` / `BodySnapshot` | Add `collision_layer`, `collision_mask`. `velocity` reports the twist for kinematic bodies. |
| `Event` | Add `normal_impulse`, `tangent_impulse`, `relative_velocity`. |
| `PhysicsEngineConfig` | Add `world_collision_layer`, `world_collision_mask`. |
| Functions | `Physics_BodyCollisionIgnoreSet`, `Physics_BodyCollisionIgnoreCount`, `Physics_BodyCollisionIgnoreCopy`, optional `Physics_BodyContactImpulseGet`. |
| Checkpoint | Store kinematic twist, saved dynamic inertia, stored target, layer/mask per body, and the ignore-pair list. |
| Managed `Types.cs` | `BodyOptions.CollisionLayer/CollisionMask`; `BodyCommand.SetKinematicTarget(...)`, `BodyCommand.SetMotionType(body, EMotionType, SpatialVector velocity)`; new fields in `RigidBodyState`, `BodySnapshot`, `PhysicsEvent`, `EngineOptions`. |
| Managed `RigidBody.cs` / `Engine.cs` | `RigidBody.SetCollisionIgnored(RigidBody other, bool ignore)` and `Engine.SetCollisionIgnored(BodyHandle, BodyHandle, bool)`. `Native.cs` gets the new imports and struct fields. |

Petri usage after this plan:

```csharp
// Pick up
engine.ApplyCommands([BodyCommand.SetMotionType(toy.Handle, EMotionType.Kinematic, SpatialVector.Zero)]);
toy.SetCollisionIgnored(carrier_capsule, true);

// Each step while carried
commands.Add(BodyCommand.SetKinematicTarget(toy.Handle, hand_o2w));

// Release
commands.Add(BodyCommand.SetMotionType(toy.Handle, EMotionType.Dynamic, hand_twist));
toy.SetCollisionIgnored(carrier_capsule, false);
```

## Test Plan [Proposal]

Native tests go in `projects\rylogic\physics\src\unittests`. New kinematic tests go in a new `kinematic_body_tests.cpp`; filter tests extend `collision_pairs_tests.cpp`
and `constraint_gpu_tests.cpp`; ABI tests extend `physics_dll_tests.cpp` (use `MakeBody`, ~line 362). Managed tests extend `Rylogic.Physics\src\Tests.cs`.

| # | Test | Pass condition |
| --- | --- | --- |
| 1 | Kinematic ignores gravity and forces | Pose unchanged after steps with gravity; `ApplyForce` returns an error. |
| 2 | Target reached with substeps | With 1, 4 and 8 substeps, the end pose equals the target (position < 1e-5 m, angle < 1e-5 rad). |
| 3 | Derived twist | Snapshot velocity equals `(target - start) / elapsed` for linear and angular moves. |
| 4 | Push | A kinematic slab moving at 1 m/s into a resting box: the box leaves with about 1 m/s (restitution 0); the slab path is unchanged. |
| 5 | Friction carry | A box on a kinematic platform moving sideways moves with the platform within a small slip. |
| 6 | No kinematic-static/kinematic/world contacts | Overlapping kinematic and static/kinematic bodies and terrain produce no contact events. |
| 7 | Wake | A sleeping stack is woken by a moving kinematic body. |
| 8 | Sleep beside still kinematic | A stack resting on a still kinematic platform goes to sleep. |
| 9 | Dynamic → Kinematic → Dynamic | Inertia is restored exactly; release velocity is applied; no momentum leaks from before the switch. |
| 10 | Warm-start reset | After a switch, the next step uses no cached impulse for the body (check via resolve test hook). |
| 11 | Island removal | Switching a sleeping island member to Kinematic wakes the rest of the island. |
| 12 | Ignore pair (GPU) | Two overlapping dynamic bodies with an ignore pair make no contacts; removing the pair restores contacts. Also with a kinematic body. |
| 13 | Ignore pair lifetime | Destroying one body removes its pairs; handle reuse does not inherit them. |
| 14 | Layer/mask matrix | All four combinations of layer/mask bits for two bodies, plus world contacts. |
| 15 | CPU/GPU agreement | The CPU reference broadphase and the GPU sweep produce the same filtered pairs. |
| 16 | Event impulses | A box resting on ground reports `normal_impulse ≈ m g dt` per substep; relative velocity is correct for a kinematic hit. |
| 17 | Checkpoint round trip | Kinematic twist, target, saved inertia, layer/mask and ignore pairs survive write/read and give identical next steps. |
| 18 | Managed API | `SetMotionType`, `SetKinematicTarget`, `SetCollisionIgnored` and new fields round-trip through `Tests.cs`. |
| 19 | Carry scenario | A capsule "carrier" with an ignore pair and a kinematic toy driven along a path: no carrier contacts, toy pushes a third body, release velocity matches. |

## Milestones [Proposal]

Each milestone builds and passes its own tests before the next one starts.

| # | Milestone | Main touch points |
| --- | --- | --- |
| 1 | Motion type in the native body | `state_flags.h` (Kinematic flag), `rigid_body.h/.cpp` (motion type, twist accessor, saved dynamic inertia, `SetMotionType`), `physics_types.hlsli` (flag mirror), `interop.cpp` (`BodyCreate` mass for Kinematic, `ApplyBodyState`, remove `BodyRecord::m_motion_type`). Tests 1, 9 (CPU part). |
| 2 | Velocity helper | `BodyTwistWS` in `physics_types.hlsli`; use it in `resolve.hlsl`, `coupled_contact.hlsl`, `selective.hlsl`, `frame_output.hlsl`, `sleep.hlsl`, `integrate.hlsl`, `constraint_solver.hlsl`, `coupled_constraint_*.hlsl`; CPU mirrors in `impulse.cpp` and `engine.cpp` CPU resolve. |
| 3 | Kinematic integration and targets | `integrate.hlsl` (drift by twist, skip forces), `physics_types.h` pack/unpack (twist in momentum slots, snap to target), `interop.cpp` `ApplyCommands`/`BeginStep` (target → twist with `elapsed`), force producers skip kinematic. Tests 2-5. |
| 4 | Pairs, sleep and wake | `sweep.hlsl` (moving kinematic counts as awake), `sleep.hlsl` (disturb islands, skip kinematic), `engine_buffer_cache.h` (`CountAwakeDynamic`, `PackSleepIsland`), `gpu_world_contacts.cpp` (keep skipping kinematic). Tests 6-8. |
| 5 | Motion switch reset | `SetMotionType` command in `physics-dll.h`/`interop.cpp`; per-body warm-start reset bit in `GpuRigidBody` flags and `resolve.hlsl` cache lookup; island removal. Tests 9-11. |
| 6 | Ignore pairs | `interop.cpp` pair set and API; `engine.cpp` merge into exclusions; `constraint_gpu.cpp` (accept extra pairs); `physics-dll.h`. Tests 12-13. |
| 7 | Layer/mask | `physics-dll.h` structs and config; `physics_types.hlsli/.h` filter field; `sweep.hlsl`, `gpu_world_contacts.*`, `broadphase_brute.cpp`. Tests 14-15. |
| 8 | Event impulses | `frame_output.hlsl` (impulse and relative velocity), `physics-dll.h` `Event`, interop event copy, optional per-body impulse sum. Test 16. |
| 9 | Checkpoint and managed API | `interop.cpp` checkpoint (version 4); `Types.cs`, `Native.cs`, `RigidBody.cs`, `Engine.cs`, `Tests.cs`. Tests 17-19. |
| 10 | Petri adoption (separate repo) | `ActorDynamics.cs`: pick-up, carry and release using the new API. Carrier feedback stays a later task. |

### Progress

- [x] Milestone 1: `KinematicBodyTests` (CPU) and `PhysicsDllKinematicTests` (ABI) pass. Sandbox `-scenediag` stack scene is unchanged.
- [x] Milestone 2: GPU shaders read velocity through `body_twist.hlsli`. `KinematicPushesDynamicCpu` and `KinematicPushesDynamicGpu` pass. Sandbox `-scenediag` stack scene is unchanged.
- [x] Milestone 3: kinematic bodies drift by their twist on the CPU and GPU (`KinematicDriftFollowsTwist`, `KinematicDriftGpu`), `SetKinematicTarget` reaches its pose with substeps (`TargetReachedWithSubsteps`), and a moving platform carries a box by friction (`PlatformCarriesBoxByFriction`). Sandbox `-scenediag` stack scene is unchanged.
- [x] Milestone 4: the broadphase rejects kinematic-static and kinematic-kinematic pairs, and a moving kinematic body admits pairs with sleeping bodies and disturbs their islands (`KinematicSleepTests`: `NoKinematicStaticOrKinematicPairs`, `MovingKinematicWakesSleepingIsland`, `SleepOnStillKinematicPlatform`).
- [x] Sandbox demo: scene JSON accepts `"kinematic": true`, and `scenes\kinematic_pusher.json` (demo `kinematic-pusher`) has a spinning paddle, a moving platform and a turntable. `-scenediag` shows the kinematic bodies push sleeping boxes (worst depth 0.05 m over 600 steps). Extend it with ignore pairs and layer/mask after Milestones 6-7.

## Decisions

Resolved open questions (2026-11-02):

1. Release velocity: the caller always gives it in the `SetMotionType` value. There is no `KeepVelocity` flag.
2. Re-enabling collision after release is game logic. The caller removes the ignore pair when ready. There is no "ignore until separated" engine option.
3. A kinematic body with no new target keeps moving at its last twist.
4. No maximum kinematic push impulse in this plan. It is deferred to the carrier-feedback task.
5. Layer and mask are 32-bit.
