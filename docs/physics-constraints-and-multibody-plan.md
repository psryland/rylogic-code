# GPU Constraint and Multibody Physics Architecture

**Status:** Implemented; corrective review complete
**Original design date:** 2026-08-25
**Original acceptance date:** 2026-08-28

## Summary

The implemented design is a hybrid, matrix-free impulse system:

- Use graph-coloured block projected Gauss-Seidel (PGS) for contacts and constraints involving only ordinary rigid bodies.
- Retain Featherstone's articulated-body algorithm (ABA) as the reduced-coordinate fast path for tree articulations.
- Use projected block Jacobi/proximal impulse iteration backed by impulse-form ABA for constraints that touch articulation links.
- Couple the rigid and articulation solvers with multiplicative outer sweeps.
- Correct positional drift through pseudo velocities and generalized-coordinate projection, separately from physical momentum.
- Record all internal substeps into one GPU submission, then perform one contiguous readback and one CPU wait per rendered frame.

This preserves the engine's impulse-based design, supports arbitrary constraint topology without assembling a global LCP matrix, and allows users who do not create
constraints or articulations to avoid their CPU, GPU, and memory costs.

## Implemented Outcome

The production implementation includes the native C++ API, versioned ABI 2.0 DLL surface, compute-shader solvers, diagnostics, unit and GPU-oracle coverage,
and 20 runtime-editable JSON physics-sandbox demonstrations.

The following measurements record the original acceptance run, not the subsequent corrective review.

| Design requirement | Original verified outcome |
|---|---|
| Fixed-iteration scaling | 50,004 rows solved in 21.683 ms and 100,002 rows in 43.290 ms on an RTX 3080 Ti: 2.00x time for 2.00x rows |
| One host boundary per frame | Every 120-frame demo soak reported one command-list submission, one fence wait, and one GPU-to-CPU readback copy per frame, including 2- and 4-substep demos |
| Optional feature cost | Empty constraint and articulation inputs allocate and dispatch no constraint, articulation, or coupled-solver resources; collision-event and multi-substep force storage are also demand-created |
| Featherstone fast path | Fixed- and floating-root trees run force ABA and bounded implicit-midpoint integration without entering the arbitrary-constraint lane |
| Dynamics preservation | Dense ABA oracles, zero-torque Coriolis cases, free-floating conservation, torque-free rigid motion, and Dzhanibekov regressions pass |
| Robust arbitrary constraints | D6 joints, loops, rigid/tree and tree/tree coupling, link contacts, split position correction, breakage, and bounded non-convergence diagnostics pass targeted suites |
| Real-time demonstrations | All 20 Release demos exceeded 40 FPS, spanning 40.44-279.16 physics FPS in the final warmed headless acceptance run |
| Legacy performance | 19 of 22 baseline scenes remained within the 10% tolerance or improved; the other three generated 31.8-33.3% more contacts while taking 16.0-17.7% more time, improving contact throughput |

The original native validation passed all 1,188 Quick tests and the complete targeted internal-substep, coupled-engine, collision-pair, compound-shape, and
articulation-midpoint suites. Subsequent review exposed gaps in those gates, including world-load rotation, off-diagonal bounded projection, scale sensitivity,
sleeping collision participants, and shared position correction. Independent analytical regressions now cover these cases; the architecture below describes
the corrected implementation.

## Goals

The system provides:

- Linear work and memory per fixed solver iteration.
- A high-performance ABA path for explicitly declared tree articulations.
- Arbitrary graph topology through external constraint edges.
- One GPU readback and one CPU synchronization per rendered frame, regardless of internal substep count.
- Orthogonality between dynamics and force-producing systems such as gravity, buoyancy, drag, thrusters, and user force shaders.
- A clean native API and matching generational DLL handles.
- Deterministic CPU reference behavior and numerically stable, tolerance-based GPU behavior.
- No secular energy gain for supported passive systems.
- Bounded, diagnostic failure for contradictory, singular, or insufficiently converged systems.
- Initial practical capacity of approximately 10,000 bodies and 100,000 scalar constraint rows.

## Deliberate Limitations

The current implementation does not:

- Assemble or factor a global LCP/KKT matrix in production.
- Guarantee exact fixed-accuracy convergence in linear time for every topology.
- Automatically recognise an arbitrary rigid constraint graph as an articulation.
- Permit CPU state-dependent force callbacks between internal GPU substeps while retaining one readback.
- Permit arbitrary topology changes while a submitted frame is pending.
- Replace continuous collision detection.
- Guarantee that physically contradictory constraints can be satisfied.

## Legacy Baseline and Implemented Integration Points

The production step at the start of this work was:

```text
NewFrame
Pack
Upload
ExternalForces
Integrate
SleepWake
BroadPhase
Collide
Resolve
SelectiveRefresh
SleepUpdate
Readback
Submit
```

`CompleteStep` waits for the submitted work and unpacks the readback.

The rigid-body integration remains a full kick-drift-kick step before contact resolution. The ordinary rigid-body angular drift stores world angular momentum and
recomputes angular velocity through orientation-dependent world inertia. Non-isotropic bodies use symplectic or implicit-midpoint angular drift, which preserves
the torque-free gyroscopic dynamics required for the Dzhanibekov effect.

The resolver still uses graph-coloured sequential impulses, but colour exhaustion no longer aliases conflicting blocks into the final parallel colour. Excess
work is routed through a coherent fallback, so high-degree graphs trade parallelism for correctness instead of racing on shared bodies.

Constraint, articulation, coupled-contact, coupled-constraint, break-state, collision-event, and multi-substep force resources are created only when their
owning feature first participates. `EngineFeatureStats` and ABI `Diagnostics` expose both logical bytes and retained high-water allocation for every optional lane.

## Constraint Model

### Scalar rows and blocks

Collect rigid-body twists, floating-base velocities, and articulation joint rates into generalized velocity $v$.

A scalar constraint row has Jacobian $J$, giving relative constraint velocity:

$$
\dot C = Jv
$$

Typical row counts are:

| Constraint | Typical scalar rows |
|---|---:|
| Distance | 1 |
| Contact normal | 1 |
| Contact friction | 2 tangential rows |
| Ball/socket | 3 |
| Hinge | 5 |
| Slider | 5 |
| Weld | 6 |
| Active hinge limit or motor | 1 selected/additional row |

Related rows are compiled into one constraint block. Contacts are three-row normal/friction-cone blocks, and multi-axis joints are solved as small
blocks where this materially improves conditioning.

The unconstrained prediction produces $v^*$. Accumulated impulse $\lambda$ changes velocity by:

$$
v = v^* + M^{-1}J^T\lambda
$$

$M^{-1}$ is an operator rather than an assembled global matrix:

- For an ordinary rigid body it is inverse mass plus world-space inverse inertia.
- For an articulation it is an impulse-form ABA traversal.

The constraint-space response is:

$$
A = JM^{-1}J^T
$$

Production code never assembles the full $A$.

### Block PGS update

For block $i$, define:

$$
r_i = J_i v - v_{\mathrm{target},i} + b_i + \Gamma_i\lambda_i
$$

where:

- $v_{\mathrm{target}}$ represents a motor or restitution target.
- $b$ represents soft-constraint bias.
- $\Gamma$ is compliance regularization.
- Hard passive rows use $v_{\mathrm{target}}=0$, $b=0$, and $\Gamma=0$ in the physical-velocity solve.

The local response is:

$$
K_{ii} = J_iM^{-1}J_i^T
$$

A block PGS update is:

$$
\lambda_i' =
\Pi_{\mathcal K_i}^{K_{ii}+\Gamma_i}
\left[
\lambda_i
-
\omega(K_{ii}+\Gamma_i)^{-1}r_i
\right]
$$

followed immediately by:

$$
v \mathrel{+}= M^{-1}J_i^T(\lambda_i'-\lambda_i)
$$

$\Pi_{\mathcal K}^{K+\Gamma}$ projects onto the joint's allowed accumulated impulse set in the response metric:

- A bilateral lock is unbounded.
- A lower limit or contact normal uses $\lambda \geq 0$.
- A motor uses $-F_{\max}h \leq \lambda \leq F_{\max}h$.

Off-diagonal response couples the rows. Clamping each component after multiplying by the block inverse is not this projection: saturating a motor can disturb
another locked row. The bounded active-set solve fixes saturated impulses and resolves the remaining free rows, with at most six rows and 64 working-set
transitions per block. A feasible candidate returns immediately. Numerical failure rejects the complete update and is reported through the existing frame
diagnostics rather than silently accepting a clamp.

Coulomb contacts require a different local solve:

$$
\lambda_n \geq 0,\qquad \|\lambda_t\| \leq \mu\lambda_n
$$

The normal impulse obeys scalar nonpenetration complementarity. Given that normal impulse, tangent impulses are projected into the friction disc. Four bounded
local sweeps update the normal and tangent responses, followed by endpoint-degree relaxation of the complete feasible change. A joint-style response-metric
projection onto the whole friction cone is not used: minimizing kinetic energy over that cone can invent normal support from pure tangential slip.

### Contact cache identity and reduction order

Warm-cache equality includes both packed body indices, both full leaf indices and the quantised local centroid, not just its hash. Packed indices are
frame-local: Engine records the exact ordered persistent rigid-body and articulation identities after frame filtering and adding its owned world endpoint.
A changed layout invalidates both main and selective caches, including replacement, reorder, omission and empty-frame transitions. A tree's immutable
topology makes its identity and order sufficient for its link slots; callers need not preserve submission order.
Every submission requires non-null owners with unique body identities and unique tree identities, whether or not constraints are supplied.
A rigid-body copy retains its identity and can replace that owner after relocation; submitting both copies as independent bodies is outside this domain.

Finite binary32 coordinates use a 1/64 m grid, rounding half-cells toward positive infinity and identifying signed zeros. Large coordinates already on that grid retain their original
bits without an overflowing scale or a narrowing integer conversion. Each nonzero stored contact claims its own empty slot in a power-of-two table at
most half full; lookup visits the complete chain. An integer avalanche folds high coordinate bits into low power-of-two bucket bits, and the centroid key
is computed once per lookup/store rather than once per probe. Duplicate identities select one impulse by rounded squared magnitude and then raw component bits.
This is an insertion-order-independent priority, not an energy certificate. Existing normal projection, motion-reset filtering and physical admission remain.

Zero velocity sweeps skip warm preload/application and cache promotion at the resolver owner. Retirement selects the already-cleared output cache, so guesses
cannot become physical response or a newly accepted cache without a solve. Raw helper load/store fixtures are separate from this publication contract.

Coupled-contact endpoints are first grouped by target, then sorted once by immutable body/leaf, material, geometry and endpoint-side inputs. The in-place
per-segment heap sort uses O(k log k) work and constant scratch; it does not assume short contact lists or use append indices as tie-breaks. Inactive manifold
storage and mutable impulse accumulators do not determine the order. Repeated target reductions use this fixed order rather than atomic append order.
These are cache and reduction guarantees, not a claim that every solver path or whole simulation is bitwise deterministic.

### Native ragdoll workload and geometry screens

The dispersed and pile sandbox inputs each retain 100 ten-link trees, 500 typed spherical-angular limits/dampers, 400 angular-X D6 limits and
400 separate angular-X D6 dampers. Their authored joint excitation remains distinct: 3 N*m/kg for dispersed and 1.5 N*m/kg for pile, with each tree's
original period and phase. Scene-owned actuation samples the submitted frame's starting time and applies mass-scaled sinusoidal forces in joint
velocity coordinates; a spherical quaternion has three such coordinates, not four. Zero amplitude leaves independently supplied forces unchanged.
Both inputs retain 3/10/1 substeps/velocity sweeps/position sweeps; runtime receipts also bind beta 0.2 and maximum position speed 2 m/s.

Committed-state geometry screens include rigid/rigid, rigid/link, inter-tree and enabled nonparent self pairs, rather than only articulation/ground.
They apply collision-layer masks, explicit ignored pairs, the native enabled/nonbroken constraint exclusion table, and separate parent/self policies.
Full-scene endpoint indices include sleeping trees even when a submitted GPU frame omits them. Conservative bounds prune narrow-phase work.
Compound leaves already carry root-space transforms; array recursion does not multiply array frames into them again.

The ordinary geometry diagnostic also reads unwrapped `JointPosition` values of one-axis revolute joints. `hinge_domain` reports their count,
`max_abs_hinge_q` in radians and whether every coordinate lies strictly inside `(-pi, pi)`; `worst_hinges` retains excursions across all measured steps,
including steps between reports. The converted workloads have four such pure-X hinges per tree. This is a necessary domain check for that workload's
scalar-to-principal-angle equivalence, not a general restriction on valid revolute motion or a sufficient proof for arbitrary joint frames.
A full turn plus 0.2 rad is outside this domain even though the D6 principal rotation-vector coordinate can report a legal 0.2 rad.

These screens and the existing kinetic-energy sum are not an external-work/potential/dissipation ledger. Headless `-engine_profile` bypasses geometry
screening and provides characterization only, not physical-quality or rendered 60-FPS acceptance. Ordinary sandbox startup makes no process-validation
request; the unit-test host requests the debug layer. Diagnostic receipts record the actual device's debug-layer state and runtime build diagnostics.

Headless GPU capture uses `-pix_capture <new-file.wpix> -pix_capture_step N`, with a one-based tick inside the requested step range.
The GPU capturer loads before scene/device creation; capture brackets one synchronous `Scene::Step`, including its completion fence, without a Present.
With `PR_PIX_DETAIL=1`, named velocity/transaction/ABA pass regions include their dependency barriers. PIX replay intervals are not exclusive shader
time or comparable live ordinary timing. Leave `PR_GPU_STAMPS` unset for these captures: its fixed-slot-lag harvesting is not a completion fence.

The same detail flag copies island attempt counters and transaction outcomes after **every** coupled velocity sweep, before the next begin resets them.
CPU aggregation occurs only after the existing job completion fence. `coupled_work` separates recorded velocity dispatches (including guarded no-ops),
started/selected numerical trials, accepted/rejected **owner-sweeps**, zero-attempt accepted owner-sweeps (a subset, including retirement), and distinct
first-rejection owners. `has_snapshots=false` can mean disabled detail, zero sweeps, or no owners; it does not claim an observed zero workload.
Started trials increment at the Pending-only begin-trial kernel, before later cache/admission checks; they do not prove every later candidate ran.
The first-rejection event bank is not a cumulative rejection counter and does not determine how many other owner-sweeps accepted.
`-engine_profile` also emits primary articulation state each tick without refreshing derived kinematics: `articulation_root` contains step/tree,
the three rotation columns, position, then root-frame angular/linear velocity; `articulation_joint` contains step/tree/link, q/v counts, then q and v.
These round-trip float records retain spherical quaternions and unwrapped scalar hinges. Neither their presence nor transaction acceptance certifies
physical trajectory quality, external work, conservation, or rendered throughput.

Host submission counts and prepare/execute/reset totals include every existing between-substep `Flush` plus the final `Submit`/`Complete`.
Three substeps therefore produce three queue submissions and one final host wait, without changing the cadence. These are host costs, not GPU
execution durations; earlier characterization rows counted only the final submission and omitted prefix-flush phase costs.

Initialization admission evaluates the Free-to-Current work once when its chosen reference is Free, reusing both signed value and error for the
identical descent comparison. Other references retain their own evaluation. Range arithmetic, finite checks, and both admission gates are unchanged.

### Soft constraints

For stiffness $k$, damping $d$, timestep $h$, and positional error $C$:

$$
\gamma = \frac{1}{h(d+hk)}
$$

$$
b = Chk\gamma
$$

The row uses $K+\gamma$. This is an implicit spring-damper formulation and avoids applying a large explicit Baumgarte velocity to represent stiffness.
Compliance can be exposed to callers instead of stiffness, with $k=1/\mathrm{compliance}$.

### Position drift correction

Hard passive rows should not receive real-velocity position bias. Drift is solved separately using pseudo velocities $u$:

$$
v_{\mathrm{target,pseudo}} =
-\operatorname{clamp}
\left(
\frac{\beta C}{h},
-v_{\max,\mathrm{correction}},
v_{\max,\mathrm{correction}}
\right)
$$

Pseudo impulses modify only:

- Ordinary rigid-body transforms.
- Articulation root pose.
- Articulation generalized coordinates $q$.

They never modify physical $v$ or $\dot q$. After an articulation correction, link transforms are regenerated through forward kinematics. Link proxies must
never be corrected independently of the articulation tree.

Position projection can alter potential energy despite leaving kinetic momentum unchanged. Corrections must therefore be capped, measured, and kept small by
using internal substeps.

For an articulation participant with simultaneous contact degree $d$, the first fixed-manifold sweep uses
$\rho\min(1,1/\max(\beta d,1))$ relaxation and later sweeps use $\rho/d$. A coherent equal-depth response therefore moves by at most $\rho C$ on the
accelerated sweep, independent of $d$, so $\rho\leq1$ introduces no intentional support gap. The maximum correction speed can only reduce this bound.
A contact that first requires correction on a later sweep always uses the conservative $\rho/d$ rule. Each update projects the new total normal impulse
onto the nonnegative half-line; its change may be negative when another contact supplies some of the required separation. Preventing this release violates
complementarity even when all contacts reach their minimum separation speed. Pseudo state and position impulses are cleared before the next frame.

For a fixed normal response $D$, target $u$, and total pseudo impulse $\lambda$, the law is $\lambda\geq0$, $r=D\lambda-u\geq0$, and
$\lambda_i r_i=0$. The native shared-body fixture realizes $D=((1,.5),(.5,2))$ and $u=(.5,2)$ through two unit-mass floating roots and a fixed endpoint.
Its accelerated first sweep has $\lambda=(.5,1)$; subsequent projected sweeps must release the first support and approach $(0,1)$.
The fixture uses 32 sweeps to test this frozen linear problem, independently reconstructing impulses from root momentum responses. It then applies the
contact-produced pseudo state once, refreshes link proxies, and checks both root/proxy poses against `dt*pseudo_speed`, with physical speeds unchanged.
This does not establish convergence with the scene's one-position-iteration setting. Transaction acceptance still checks finite state, not a final global
complementarity tolerance.

### Restitution and motors

Restitution target velocity is calculated once from the pre-solve normal velocity:

$$
v_{\mathrm{target}} = -e\min(v_{n,\mathrm{pre}},0)
$$

It is not recalculated after each iteration, preventing repeated bounce amplification.

Motor descriptors provide:

- Target velocity and optional target position.
- Stiffness and damping.
- Maximum force or torque.
- Maximum impulse.
- Optional maximum work or power per frame.

### Passive work

For a passive candidate block impulse $\delta$, the kinetic-energy change is:

$$
\Delta T = \delta^T Jv + \frac{1}{2}\delta^T A\delta
$$

An exact sequential local update with a zero target should not increase kinetic energy. This is a CPU-oracle invariant and a debug-GPU diagnostic.

## Solver Architecture

### Rigid-only lane

Contacts and user constraints between ordinary rigid bodies are compiled into one block list and solved together.

Each block is graph-coloured by its dynamic endpoints. Blocks sharing a dynamic body cannot execute in the same colour. One GPU invocation owns the complete
block so friction or joint rows within that block do not race one another.

Colour exhaustion must be explicit:

1. Never alias an already occupied colour.
2. Initially route the entire rigid solve through a coherent projected-Jacobi fallback if any block cannot be coloured.
3. Later build rigid connected components with GPU union-find and route only affected components through the fallback.
4. Never mix stale Jacobi overflow updates with PGS updates that concurrently modify the same body.

The existing contact-only route remains the normal fast path after this correctness fix.

### Pure articulation lane

An explicitly created articulation selects a spanning tree. Its unconstrained dynamics use the full Featherstone ABA, including all velocity-product bias terms.

For link $i$:

$$
v_i = X_i v_{\mathrm{parent}} + S_i\dot q_i
$$

The kinematic bias is:

$$
c_i = v_i \mathbin{\times} (S_i\dot q_i)
$$

The spatial bias force contains:

$$
p_i^A = v_i \mathbin{\times^*}(I_i v_i) - f_{i,\mathrm{external}}
$$

These terms preserve Coriolis, centrifugal, and gyroscopic effects. ABA must not be reduced to merely applying $H^{-1}\tau$.

During the inward pass:

$$
U_i = I_i^A S_i
$$

$$
D_i = S_i^T U_i
$$

$$
u_i = \tau_i-S_i^T p_i^A
$$

$$
I_i^a = I_i^A-U_iD_i^{-1}U_i^T
$$

The outward pass calculates $\ddot q_i$, link accelerations, and updated velocities. This path is $O(L+D)$.

The production integrator evaluates ABA inside a bounded implicit-midpoint fixed-point solve:

$$
\dot q_{m}=\dot q_n+\frac{\Delta t}{2}\ddot q(q_m,\dot q_m)
$$

$$
q_m=q_n+\frac{\Delta t}{2}\dot q_m
$$

$$
\dot q_{n+1}=\dot q_n+\Delta t\,\ddot q(q_m,\dot q_m)
\qquad
q_{n+1}=q_n+\Delta t\,\dot q_m
$$

Floating-root translation and orientation use the same midpoint velocity, with orientation advanced by the quaternion exponential map rather than component-wise
addition. A fixed iteration ceiling keeps frame work bounded. Non-finite state, a singular joint/root solve, or failure to meet the residual threshold sets a
sticky GPU status; the CPU publishes none of the frame's articulation results unless every returned tree is valid. This transactional boundary prevents a
partially failed forest from entering the caller's state.

The fused GPU kernel assigns one invocation to one articulation. That invocation reuses global tree scratch across the start-state solve and all midpoint
iterations, so fixed iteration work remains linear in links and scalar degrees of freedom without multiplying command-list barriers by tree depth.

Any loop-closing joint remains an external constraint edge. An arbitrary mechanism is therefore represented as:

- One or more reduced-coordinate spanning trees.
- External constraints connecting links, trees, ordinary rigid bodies, or world.

A tree created as ordinary six-degree-of-freedom constraints is not automatically converted into an articulation. Selecting the ABA representation is an
explicit API decision.

### Articulation-coupled lane

A constraint touching an articulation cannot apply independent inverse inertia to the contacted link proxy. The complete tree responds.

For these rows:

$$
A = JH^{-1}J^T
$$

where $H^{-1}$ is applied through impulse-form ABA.

The projected proximal iteration is:

$$
\lambda^{k+1} =
\Pi_{\mathcal K}^{P}
\left[
\lambda^k-\omega P^{-1}r(\lambda^k)
\right]
$$

Each iteration:

1. Evaluates every coupled block residual using current rigid and link velocities.
2. Projects a candidate block impulse.
3. Writes two endpoint wrench contributions per block.
4. Gathers contributions through prebuilt endpoint adjacency.
5. Applies ordinary inverse inertia once per participating rigid body.
6. Runs one impulse ABA per participating articulation.
7. Updates generalized/link velocities and accumulated impulses.

The lane is simultaneous so an articulation never receives several conflicting partial ABA updates.

A GPU-resident backtracking check evaluates the convex constraint merit per coupled island. If a candidate increases the merit, the relaxation factor is reduced
and the iteration retried. After a bounded retry count, the solver retains the last accepted finite state and emits a non-convergence event.

### Multiplicative coupling

For a mixed rigid/articulation island:

```text
Prepare current geometry, response factors and persistent owner context

repeat outer_iteration_count:
    Stage and publish ordinary blocks in dependency order
    UAV barrier
    Stage, solve and publish each articulation-coupled island privately
    UAV barrier
```

A rigid body participating in both lanes receives the coupled update before the next PGS sweep. The two systems therefore converge toward one combined fixed
point rather than behaving as disconnected solvers.

Special cases remain cheap:

| Active features | Work |
|---|---|
| Contacts only | Existing rigid contact path |
| Rigid constraints, no articulations | Unified coloured block PGS |
| Pure tree, no external rows | One ABA dynamics pass |
| Tree with contacts or loops | ABA prediction plus coupled iterations |
| Colour-overflow rigid component | Coherent projected-Jacobi fallback |

## Exact Frame Pipeline

### Legacy-to-implemented mapping

| Legacy phase | Implemented behavior |
|---|---|
| `NewFrame` | Reset frame output and establish frame/substep timing |
| `Pack` | Pack ordinary bodies, bind stable endpoints, and append hidden articulation link proxies |
| `Upload` | Upload bodies plus only dirty constraint/articulation descriptors |
| `ExternalForces` | Move inside every GPU substep; seed forces before invoking force modules |
| `Integrate` | Preserve ordinary-body KDK prediction and run tree-consistent ABA articulation prediction |
| `SleepWake` | Wake complete constrained islands and complete articulations |
| `BroadPhase` | Include current articulation link proxy bounds |
| `Collide` | Generate contacts against ordinary bodies and link proxies |
| `Resolve` | Compile current-pose rows, prepare phase-specific inverses, and commit ordinary/coupled persistent transactions; contact warm ownership is separate |
| Position solve | Correct rigid transforms and articulation generalized coordinates |
| `SelectiveRefresh` | Refresh affected contacts after correction and recompute residuals |
| `SleepUpdate` | Include constraints, motors, external link wrenches, and unresolved residuals |
| `Readback` | Gather selected output into one contiguous GPU buffer |
| `Submit` | Submit the complete frame once |
| `CompleteStep` | Wait once, unpack once, and deliver queued events |

### Recorded command sequence

```text
BeginStep(frame_dt, substep_count)
    Validate topology and endpoint lifetimes
    Build BodyId/LinkHandle -> current GPU index remap
    Pack ordinary bodies, articulation forest, hidden link proxies, and constraints
    Upload body state, dirty constraint ranges, and articulation topology/state
    If substep_count > 1:
        Lazily create the force-replay pipeline and upload 32 B/body of immutable CPU-authored force
    Establish the bounded packed frame-output layout
    Upload/reset only the 64-byte frame-output header

    for substep_index in [0, substep_count):
        if substep_index > 0:
            Reset collision counters
            CSSeedWorkingForces from immutable frame-force storage

        Record each GPU external-force module
        CSIntegrateOrdinaryBodies

        CSIntegrateArticulationsImplicitMidpoint
            one invocation serially traverses one articulation
            re-express live world-at-COM loads for each trial orientation
            solve start acceleration and bounded midpoint iterations
            publish final generalized state and link proxies transactionally

        CSSleepWake
        CSBroadPhase
        CSNarrowPhase

        CSCompileContactBlocks
        CSCompilePersistentConstraintBlocks
        CSBuildEndpointAdjacency
        Prepare persistent acceptance epochs and private working storage
        Apply contact warm caches only

        # Coupled-lane branch shown; rigid-only work runs Position before Velocity.
        repeat outer_iteration_count:
            repeat ordinary_dependency_levels:
                Stage and publish complete ordinary blocks
                UAV barrier

            Recover private coupled free state and admit compatible coefficient guesses
            Run unchanged bounded projected/ABA numerical solve on private state
            Admit and publish the complete coupled result
            Capture first rejection per owner before position scratch reuse
            UAV barrier

        Initialize shared detached position state for the fixed manifold
        Assign ordinary position colours/fallback and prepare position inverses
        repeat position_iteration_count:
            Solve ordinary contacts and rigid joints against shared rigid pseudo velocities
            Solve articulated contacts and coupled joints against shared link/generalized pseudo state
        Apply converged pseudo state once
        CSForwardKinematics
        CSWriteLinkProxyTransformsAndBounds

        CSSelectiveContactRefresh, repeating prepare/solve/apply for its new manifold when enabled
        Sample final accepted completed-substep loads and latch breaks
        CSSleepUpdate
        if collision subscribers exist:
            CSCompactCollisionEventsAndReserveRange
            CSAppendCompactCollisionEvents
        else:
            CSPrepareSubstepCounters

    GPU-to-GPU CopyBufferRegion final ordinary-body range into packed output
    GPU-to-GPU CopyBufferRegion optional rigid numerical-failure word into header padding
    GPU-to-GPU CopyBufferRegion optional break and owner-tagged rejection ranges
    CSGatherFrameArticulations when articulations exist
    GPU-to-CPU CopyBufferRegion(one contiguous packed output)
    ExecuteCommandLists once

CompleteStep()
    Wait for one fence
    Map one readback allocation
    Unpack bodies, articulations, counters, and events
```

Pure-tree integration uses one articulation dispatch per internal substep. One invocation owns each articulation's disjoint packed ranges and serially performs
its inward/outward ABA traversals and bounded implicit-midpoint iterations. Tree-sized state remains in global buffers and is streamed one link at a time,
rather than being retained in registers or group-shared memory. This avoids replaying a depth-dependent dispatch schedule for every nonlinear iteration while
still exposing parallelism across independent articulations. The breadth schedules and level kernels remain available for the coupled impulse-ABA passes,
where many constraint endpoints must be gathered before one tree response.

There are multiple compute dispatches, UAV barriers, and GPU-to-GPU copies in the complete frame, but there is no CPU wait, state readback, or resubmission
between internal substeps. Only the final contiguous copy targets CPU-visible readback memory.

## External Forces and Orthogonality

Every solver node has a per-substep working spatial wrench:

1. The body upload supplies CPU-authored force, torque, and gravity directly for substep zero.
2. When the frame has more than one internal substep, retain the authored force and torque in optional 32-byte-per-body immutable storage.
3. Before each later substep, restore that immutable force and torque after resetting transient collision counters.
4. Let GPU modules add state-dependent forces for the current substep.
5. Retain the working wrench until force integration completes, then clear it for the next substep.

Articulation link proxies follow this lifecycle even though the ordinary-body integration dispatch skips them.

Articulations retain separate local link-origin follower wrenches and world-at-COM force/torque samples. Each implicit-midpoint ABA evaluation re-expresses the
world samples using its trial orientation; it does not freeze the initial local representation of gravity. Local follower wrenches remain local. External
modules still sample state once per substep, not once per nonlinear trial. Configuration-only mobility factorization neither reevaluates forces nor overwrites
the accepted acceleration; immediate impulse response reconstructs its link-velocity cache from the current generalized velocities.

Buoyancy remains independent of the dynamics representation:

- It reads geometry, transform, velocity, and fluid state.
- It writes a spatial wrench to the target node.
- It does not know whether the target is an ordinary rigid body or an articulation link.
- The ABA inward pass maps link wrenches into generalized forces.

Gravity, drag, thrusters, springs, fields, and user force shaders use the same seam.

A CPU callback cannot inspect intermediate GPU state while preserving the one-readback contract. Force sources are therefore classified as:

| Evaluation mode | Internal substeps |
|---|---|
| Frame-constant CPU data | Uploaded in body state for substep zero; retained once and reapplied only when later substeps exist |
| GPU per-substep module | Records a shader dispatch for every substep |
| CPU state-dependent callback | Rejected when internal one-readback substeps are requested |

## Events, Breakage, and Capacity

Collision, constraint-break, non-convergence, and capacity-overflow events are appended to bounded GPU queues with their substep index.

A persistent constraint broken during one substep is disabled in GPU state for later substeps of the same frame. The CPU receives the event after final readback.

Known persistent row capacity is validated before submission. Unpredictable contact overflow is clamped safely and flagged within the active substep. The GPU
never waits for the CPU to resize a buffer.

## Public API Shape

The native API retains the engine's non-owning ordinary-body model while using stable identities for persistent endpoints. A GPU buffer index never becomes a
persistent API identity.

```cpp
namespace pr::physics
{
	enum class EConstraintAxisMode
	{
		Free,
		Locked,
		Limited,
		Driven,
	};

	struct ConstraintHandle
	{
		uint32_t m_index;
		uint32_t m_generation;
	};

	struct LinkHandle
	{
		uint32_t m_index;
		uint32_t m_generation;
	};

	struct BodyRef
	{
		static BodyRef World();
		static BodyRef Rigid(RigidBody const& body);
		static BodyRef Link(Articulation const& articulation, LinkHandle link);
	};

	struct BodyFrame
	{
		BodyRef m_body;
		m4x4 m_constraint_to_body;
	};

	struct ConstraintAxisDesc
	{
		EConstraintAxisMode m_mode;
		Range<float> m_limits;
		float m_target_position;
		float m_target_velocity;
		float m_stiffness;
		float m_damping;
		float m_max_force;
	};

	struct D6ConstraintParameters
	{
		std::array<ConstraintAxisDesc, 3> m_linear;
		std::array<ConstraintAxisDesc, 3> m_angular;
	};

	struct SphericalAngularConstraintParameters
	{
		Range<float> m_swing_x;
		Range<float> m_swing_z;
		Range<float> m_twist;
		float m_damping;
	};

	struct ConstraintDesc
	{
		BodyFrame m_frame_a;
		BodyFrame m_frame_b;
		std::variant<D6ConstraintParameters, SphericalAngularConstraintParameters> m_parameters;
		float m_break_force;
		float m_break_torque;
		bool m_collide_connected;
		bool m_enabled;
	};

	class ConstraintSet
	{
	public:
		ConstraintHandle Add(ConstraintDesc const& desc);
		ConstraintHandle Add(BallSocketConstraintDesc const& desc);
		ConstraintHandle Add(HingeConstraintDesc const& desc);
		ConstraintHandle Add(SliderConstraintDesc const& desc);
		ConstraintHandle Add(WeldConstraintDesc const& desc);

		void Update(ConstraintHandle handle, ConstraintDesc const& desc);
		void SetEnabled(ConstraintHandle handle, bool enabled);
		void Repair(ConstraintHandle handle);
		void Remove(ConstraintHandle handle);
		ConstraintDesc const& Get(ConstraintHandle handle) const;
		bool Contains(ConstraintHandle handle) const;
		bool IsBroken(ConstraintHandle handle) const;
	};

	struct Engine::StepInput
	{
		std::span<RigidBody*> m_bodies;
		std::span<Articulation*> m_articulations;
		ConstraintSet const* m_constraints = nullptr;
		float m_elapsed_seconds = 0.0f;
		int m_substep_count = 1;
		double m_time_s = 0.0;
	};
}
```

`ConstraintDesc` owns one typed parameter alternative, with D6 as its default. `D6()` provides checked access to D6 parameters.
Named joint descriptors construct the D6 alternative. Spherical angular constraints share persistent ownership and CPU/rigid-GPU/coupled-GPU row
solving with D6; their preparation keeps ellipse/twist gradients separate from physical viscous damping.

Additional API rules:

- Anchors are body-local or link-local, never ambiguous world snapshots.
- Linear, angular, force, torque, compliance, and damping units are documented independently.
- Constraint and link handles are generational.
- Every rigid body has a stable `BodyId`; remapping uses it rather than current vector position.
- Missing or duplicate endpoint identities fail `BeginStep` before submission.
- Constraint and articulation topology cannot change while a step is pending.
- Parameter changes upload as merged dirty stable-slot ranges.
- Link self-collision and connected-body collision are configurable: an adjacent parent/child pair requires the child link's `collide_parent`, while a
  non-adjacent pair in the same articulation requires `collide_self` on both links.
- Per-row impulses remain GPU-resident unless diagnostics explicitly request them.
- `ConstraintsBroken` samples final accepted completed-substep loads; `ConstraintRejections` retains the first non-terminal rejection per owner per frame.
  Later acceptance does not erase that rejection. A broken constituent stays inert in the uploaded scheduling superset until next-frame repacking.
- `LastFeatureStats()` reports active counts, retained capacities, dispatches, logical bytes, allocated bytes, packed output size, and terminal failure state.
- API `0x000B0000` carries the coordinated timing and owner-tagged event contract across native/C/managed; descriptor-header and checkpoint versions are separate.
- Physical elapsed time and its represented float duration after substep division must be finite and positive. Reject invalid timing before command,
  force, kinematic-target or completed-output mutation. Zero-time stepping is not a maintenance operation.

Articulation creation is explicit:

```cpp
ArticulationBuilder builder;

auto root = builder.AddFloatingRoot(root_desc, root_to_world, root_velocity);
auto upper = builder.AddLink(root, ArticulationJointDesc::Revolute(v4::ZAxis()), upper_link_desc);
auto lower = builder.AddLink(upper, ArticulationJointDesc::Revolute(v4::ZAxis()), lower_link_desc);

auto articulation = builder.Build();
```

`Articulation` owns immutable tree topology, generalized state, stable link handles, link kinematics, per-link gravity/external wrenches, and whole-tree sleeping
state. Hidden link proxies are transient engine resources; the articulation does not own unrelated ordinary rigid bodies.

Native C++ spherical joints use `ArticulationJointDesc::Spherical(parent_frame, child_frame)`. Their configuration is a unit quaternion
`(x,y,z,w)` mapping the child joint frame into the parent joint frame, while the three generalized velocities and forces are angular velocity and torque
in the child joint frame. Input quaternions must be normalized; opposite signs describe the same physical state. The physical child-link transform is
`parent_frame * Rotation(q) * Inverse(child_frame)`. These joints have a constant three-column child-frame motion subspace, not three ordered Euler rotations.
Their quaternion update right-multiplies by the exponential of the child-frame angular displacement and removes normalization roundoff.
Midpoint integration and detached position correction use the same configuration convention, without interpreting quaternion components as scalar speeds.
Position and velocity offsets are independently packed across mixed spherical, fixed, revolute and prismatic joints.
CPU detached correction stages every joint and the floating-root transform before publishing or waking the tree. Rejected drift leaves accepted
coordinates, physical generalized velocities, sleep state and cached transforms unchanged.

`JointPosition` returns four entries for a spherical joint and `JointVelocity` returns three; `PositionCount()` and `m_dof_count` specify the respective
dimensions. Scalar joints retain their ordered screw representation. This native spherical foundation does not by itself establish angular-limit,
multi-contact convergence, scene-quality or performance acceptance. Spherical creation is available through native C++ and sandbox JSON; the C ABI below
still exposes scalar-axis joint creation. The sandbox README specifies the explicit quaternion joint and separate spherical-angular constraint schema.

D6 angular rows measure components of `Log(Ra^T Rb)` in frame A, not independent finite Euler rotations. Their velocity covectors include the
inverse rotation-log differential; using the frame axes alone is correct only at zero relative orientation (or along the rotation axis).
The principal coordinate chart is differentiable for relative angles strictly below pi. Coordinate springs, damping, velocity targets and force
bounds are conjugate to these coordinates. In particular, three equal coordinate damping coefficients are not an isotropic physical angular damper
away from identity; a physical viscous torque proportional to relative angular velocity has a separate contract.

The shared spherical coordinate geometry uses `q = swing * twist`, with Y twist and the X/Z components of the shortest swing rotation vector.
The first nonzero quaternion component in `(w,y,x,z)` is chosen positive. A no-twist half-turn swing has diagnostic twist zero but no regular
swing/twist differential. Gradients use parent-joint-frame relative angular velocity and do not differentiate across the signed twist branch cut.
Swing range endpoints specify quadrant-wise ellipse half-axes, not independent box bounds. The geometry helper measures signed Euclidean distance
to the nearest boundary across all four quadrants, including zero-width side segments. Smooth arcs retain their analytic outward normal; axis features
use their normal cone, rather than deriving a smooth Jacobian from a tiny difference between rounded points.
Break torque is measured from the summed physical angular wrench, not the norm of nonorthogonal coordinate impulses.

The spherical angular block has six fixed slots: 0-1 for up to two swing faces, 2 for twist, and 3-5 for physical damping along an orthonormal parent
joint basis. A smooth ellipse uses one swing slot. Zero-width sides use independent unilateral coordinate faces; a completely collapsed swing axis
leaves a segment, requiring its bilateral lock and the remaining coordinate interval. A side/arc corner has one coordinate face and one curved face.
Two side faces meet at the origin, strictly inside the outer arc, so no third swing face is required. Feature identities reset incompatible retained
impulses when a slot changes between a coordinate face and an ellipse row.

All physical damper rows have zero target speed, zero stiffness, and implicit coefficient `gamma = 1/(dt*c)`. Thus their committed angular impulse is
`-dt*c*omega_relative`, independent of the swing/twist chart. Full `[-pi,pi]` ranges are unrestricted; they create no artificial cut-boundary rows.
Limited split coordinates require a regular chart. Preparation reports failure for an undefined split or nonfinite gradients, rather than clamping
derivatives; conditioning near the antipode and behavior through limited coordinate cuts are not covered by the bounded fixtures.

Single-joint native fixtures reconstruct committed rigid and coupled GPU impulses and final physical velocities for damping-only, simultaneous
noncircular ellipse/twist limits plus damping, two zero-width side faces plus damping, and all six locked/viscous obligations. Independent coordinate
finite differences and the physical inertia equation check the hard and viscous laws; the CPU solver reaches the same velocities. This is local
velocity-law evidence, not multi-joint convergence, position/contact quality, scene-energy acceptance or rendered-performance clearance.

Additional fixtures use two moving, unequal-inertia endpoints and check equal-and-opposite world impulse, the implicit relative-speed equation, and
a two-rigid-body reference. Maximum-row coverage includes two supported swing faces, a geometrically active but separating twist, and three nonzero
damper impulses; both GPU preparation masks are checked. Exact-antipode limited preparation must reject without committing any damper, followed by
ordinary limited recovery and unrestricted damping at that same antipode.

Coupled fresh-solve feature replay checks a same-state upper-side-to-upper-ellipse change with the owner unchanged and a nonzero retained damper control.
Warm and cold solves start from the same free input. Changed-pose continuation uses the response ownership described below, not reinterpretation of retained
coefficients through a new Jacobian or a preparation flag.

`constraint_transaction.hlsli` provides the resource-independent staging core used by both Engine owners. It separates the last
accepted epoch/timestep from attempted preparation, stages `free = live - owned_response` only within that accepted epoch, and proposes publication
as `live + (replacement_response - owned_response)`. Its recorded response uses the represented primary-state change. A new substep logically resets
response ownership without subtracting accepted physical history; a rejected new-substep attempt retains the old cache's timestep. Coefficient
compatibility still requires matching row state and feature. Zero-row acceptance clears coefficients and accepted loads even for null-response support.

Rigid staging uses world-COM momentum; generalized scalar staging uses the articulation's existing velocity-component frames. Wake state follows only
a represented accepted physical change, not a nonzero provisional seed. The accepted-load reduction retains the detector's linear-row impulse norm
and pure-angular-row resultant; moments from linear impulses in the world-COM response ledger are not break torque. The caller must validate and commit
every member of the owning transaction together; these helpers do not write GPU resources or provide that owner-level publication gate.

The ordinary rigid stage also forms a provisional seed from compatible coefficients at the new geometry, solves one block step, and stages the resulting
total response. It consumes a successfully prepared active-row inverse and uses `ProjectConstraintVelocityStep`, the same numerical algebra called by the
production rigid shader. Physical-admission rejection is distinct from input/projection failure. Two-moving-endpoint CPU and D3D12 fixtures check actual
proposed momenta against independent viscous equations after changed basis/mobility and interleaved impulses. Exact-dyadic fixtures check no-effect wake,
fresh-substep damping, rejection/retry and zero-row retirement. Those resource-independent numerical-stage checks do not themselves establish GPU publication.

`GpuConstraintSolver::PrepareVelocityTransactions` and `CommitVelocityTransactions` provide the ordinary owner's actual staged GPU pipeline. Each stable
ordinary slot owns a 128-byte accepted record (16-byte epoch/dt stamp, 48-byte coefficient/load cache, and 64 bytes of endpoint world-COM momentum response)
plus a 4-byte attempt result. Prepared rows supply geometry, bounds and an inverse, not accepted coefficients. Existing dependency levels visit enabled
zero-current-row slots as well as active rows and give each body one block writer per dispatch. Both prospective bodies, their sleep state and the record
are published only after complete-block admission; contacts and other owners cannot interleave inside that dispatch. Distinct endpoint identity remains
the validated caller contract, matching `ConstraintSet`'s rejection of self-connections. Different valid blocks can share a body across transactions.
Each upload assigns a per-slot invalidation revision for semantic input changes or an explicit reset request. The CPU retains a four-byte clock per
high-water slot for the owner's lifetime, independently of source changes and shadow, upload-buffer, or accepted-record allocation. Allocation alone
does not create a semantic invalidation, and no new source or regrown slot restarts a clock that an accepted record could still reference. An accepted coefficient cache acknowledges
that revision; preparation flags are not cache authority. Repeated preparation without Upload therefore continues accepted guesses, while rejection
followed by an unchanged Upload cannot erase a pending invalidation. Existing padding holds the 30-bit revision and two endpoint-body reset acknowledgements;
revision exhaustion throws rather than wrapping a live receipt. A newly raised body reset invalidates once in the accepted epoch; a historical cache still
honours a raised reset, and lowering the flag does not restart a current solve. Slot generation remains separate.
The body flag is shared step-scoped input produced by `RigidBody::Motion` and cleared by `UnpackDynamics` after the step, not by a constraint block.
Each accepted block acknowledges its own A/B observations; accepting one block cannot consume the event for another block sharing either endpoint.

Transaction storage advertises nonzero capacity only after both resources, their initialization, and preparation have been recorded. Replacement first
retains displaced resources in the job's existing fence-bound KeepAlive. A failed replacement keeps returned resources owned but advertises zero capacity,
so retry must recreate the complete set rather than reuse partial buffers. Allocation failures propagate; they are not physical admission rejections.
The Upload/Prepare dependencies follow the same rule. The shared upload-buffer set advertises zero capacity from the start of replacement until its
complete initialization and CPU shadows have been recorded. Pseudo-velocity and optional break-state helpers publish their new capacity only after
successful resource creation; both retain displaced resources, including a break buffer removed by an upload. Successful one-to-two-body growth is
covered with two preparations retained in one unsubmitted job and with zero position iterations. Allocation-failure behavior remains source-traced,
not fault-injection evidence.

Direct native diagnostics execute those owner phases with GPU readback. They cover two moving unequal-inertia endpoints with independent physical-impulse
checks after pose/mobility changes, preservation of interleaved input, unchanged replay, accepted-only wake, zero iterations, and inactive cache/load
retirement including nonzero coefficients with zero represented response. Rejection preserves complete body and record bytes. Accepted timestep provenance
survives rejected `h/2` and `h/4` attempts; an under-relaxed retry discriminates correct scaling from the last accepted `h`. A finite momentum seed whose
current-mobility twist is nonfinite produces non-terminal admission rejection before projection. An invalid change to an already accepted epoch's timestep
still uses the fatal numerical transport. Existing compiler/factor/projection failures are not reclassified.

The shared guess-scale helper reports context validity separately from the derived scale's representability. A finite warm factor and an in-window
timestep ratio can produce an infinite provisional scale; both owners reject that attempt as `AdmissionRejected` before consuming coefficients, while
retaining all live and accepted state and leaving the fatal-slot stream clear. The helper does not cap the factor or substitute a cold guess for this
overflow. Invalid context, including changed dt within an accepted epoch, still produces `NumericalFailure`. The existing out-of-window warm policy
is unchanged. Direct-owner controls use accepted `h = 1/60`, fresh `2h`, and finite factor `2e38` in both lanes and contrast reused-epoch `2h`.

The Engine uses these explicit phases; there is no separately published persistent-constraint warm path. The staged sequence physically publishes no
persistent provisional guesses when there is no accepted solve, including zero persistent velocity iterations. This is not a guarantee about contact warm caches.

`GpuCoupledConstraintVelocity` exposes the same explicit staged preparation and publication to Engine and bounded native diagnostics.
Each coupled island owns an accepted epoch/dt, each constituent slot owns accepted coefficients/load/reset receipts, and each target
owns actual response components: world-COM momentum for ordinary rigid bodies and native generalized velocity increments for complete articulations.
The ordinary and coupled lanes partition slots, not physical bodies. A validated graph `A -- C0 -- R0 -- R -- R1 -- C1 -- B` therefore has independent
C0, R, and C1 accounting even though their consecutive commits share R0/R1.

One coupled attempt copies live bodies and the ABA working cache. Recovery writes every participating complete-tree generalized component, so a separate
whole-forest primary-velocity copy is unnecessary; the omitted copy is exactly `4 * N_velocity` bytes per attempt, not an allocation saving or measured speedup.
It recovers private `free = live - old_owned` only for
this epoch's accepted response, derives compatible provisional guesses from accepted cache provenance, and runs the existing numerical/backtracking
sweep on those private buffers. With live input, membership, configuration factors, and outside writers unchanged during the attempt, the identical
free state can be reconstructed again: `new_response = actual_private_result - free`. Publication stages
`proposed = live + (new_response - old_owned)` and `proposed_owned = old_owned + (proposed - live)`. This avoids both a second free-state snapshot and
reinterpretation of old impulses through new mobility. Contact and ordinary commits may interleave between attempts, never inside one attempt.
One complete-island admission gate publishes bodies, generalized speeds, reconstructed link velocities, responses, cache, and stamp; a final UAV barrier
is the visibility boundary. Other physical ABA fields remain unchanged. Prepared rows remain provisional geometry/numerical scratch, not cache authority.

Physical overflow of a new free/seed/final candidate is non-terminal admission rejection. Existing numerical rejection retains its original island
flags and a distinct result. Invalid accepted-epoch context or invalid participating-link factor state uses the fatal slot-tagged transport. Every link,
not just the root, is checked before response-only ABA failures may be classified as new physical representability failures. Input revisions and body
reset levels are acknowledged per accepted block, without clearing another block's shared body event. Wake follows only the accepted physical difference.
Enabled limits with zero current rows still retire cached coefficients/load, including cancelling support with a zero response record. Removing every
law from a descriptor can remove it from lowered topology; that is not a same-membership zero-row visit.

The shared result name `SolveRejected` identifies retained numerical rejection separately from physical admission and fatal context failures.
Three valid world-to-floating-link X drives with unit mass, target one, zero softness, and no backtracking demonstrate this outcome without injected
faults. Relaxation one quarter accepts nominal coefficients `(0.25,0.25,0.25)`, speed `0.75`, and merit change `-0.46875`. A same-epoch full-relaxation
attempt proposes speed `1.5` with merit change `+0.09375`; it rejects with the existing Merit flag while preserving all live/accepted state and leaving
the fatal-slot stream clear. Assertions allow small preconditioner-regularization differences rather than requiring bit-exact nominal impulses.

Native diagnostics cover the original non-coaxially rotated fixed-root spherical damper, unchanged-pose and cold controls, and fresh-substep/zero-iteration
behavior with a nonzero cache. A two-revolute-link fixture changes actual articulated mobility and independently applies link impulses before rebasing.
Its analytic mass matrix is `M = [[3.8 + 2*cos(q2), 1.4 + cos(q2)], [1.4 + cos(q2), 1.4]]`, and two isotropic link dampers give
`C = 6*[[2,1],[1,1]]`; the independent equation is `(M_new + dt*C)*v_new = M_new*(1,2)^T + external_impulse`.
The mixed-owner fixture checks complete rejected-owner bytes, independent island acceptance, an intervening ordinary commit, retry, and an exact sleeping
replay with nonzero support. A diagnostic child-validity fault distinguishes invalid factor context from provisional overflow. These are direct-owner
checks, not public Engine selective-refresh, scene, C ABI, default-budget convergence, or rendered-performance acceptance.

Coupled storage replacement keeps a partial set unusable and retains displaced resources through recording retirement. Topology allocation groups
publish capacity only after every member exists; uploaded streams and trial initialization retain their existing owning boundaries. CPU membership
keys are allocated before their joint publication. These exception paths are source-traced, not allocation-failure injection evidence. Response payload
is O(generalized components + bodies); per-island provenance, per-slot cache, and the private body/ABA working state are additional, explicitly reported costs.

### Engine phase, epoch and publication contract

Each physical internal substep consumes a fresh monotonic epoch. Its main and selective visits share that epoch and dt; rejected or abandoned attempts
never make an epoch reusable. Coupled packed-response membership changes require fresh ownership, while harmless ordinary body repacking preserves stable
identity. Disabling, removing, repairing or reusing a slot invalidates future guesses without undoing already-published physical history.

The rigid-only branch retains Position then Velocity: `CompileRows(q0)`, one shared pseudo clear, position colouring/inverse/sweeps/application, explicit
`CompileRows(q1)`, then `PrepareVelocitySolve(q1)` and staged physical sweeps. The additional compile is visible in Resolve, not hidden in a compound
preparation call. Compilation does not clear pseudo state or invalidate accepted records. Physical inverse and rows describe the same current pose.
With a coupled constraint/contact lane, Velocity remains before Position. Each selective visit recompiles geometry and rebuilds mobility at its current
pose before further same-epoch work. Configured outer, position, selective and backtracking budgets are unchanged; no final extra velocity solve is added.

One completed-substep detector samples only that epoch's final accepted load, after all main/selective/position work. Frame peaks cover those samples.
An earlier acceptance remains eligible after a later rejected replacement even if current geometry makes its rows inactive. A later accepted zero-row
replacement retires the load. Historical-only coefficients and provisional guesses do not count. Force and pure-angular torque thresholds remain strict.
The uploaded graph remains an inactive scheduling superset after disposition until the next frame; no solve follows disposition in the same epoch.
Unrepresentable accepted-load diagnostics reject frame publication through the existing fatal slot channel rather than clamping or hiding the sample.

One sparse owner-tagged rejection bank replaces the coupled-only stream. Ordinary entries identify stable slots and generations; coupled entries identify
frame-local islands, without fake constraint handles. Velocity results are captured before position scratch reuse. Admission rejection has no numerical
reason or details; numerical rejection retains its reason, phase and actual per-island attempt count. Fixed later no-op retry dispatches do not count.
A recorded joint-drift failure takes count and relaxation from the same island attempt and reports merit zero because that drift has no merit reduction.
The first record survives subsequent acceptance. Native, C and managed consumers share Event size 232 and the exact version check; short event buffers
receive no partial copy and reading does not drain completed events.

Terminal solver/capacity diagnostics remain separate from non-terminal reports. `HasTerminalFailure()` records a failure, not the inverse claim that
absence proves CPU publication. Allocating internal sleep-ID conversion completes before the caller-owned publication marker. A rejected gathered output
invalidates both persistent owners and restores rejected sleep input; later callback exceptions do not undo already-published state.
Actual terminal abandonment waits without publishing caller dynamics or events and invalidates both owners. Reuse after `GpuJob::Abandon` remains blocked
by that independent terminal lifecycle, which does not reset recording; this migration does not repair it or claim recovery from device removal.

Focused controls distinguish post-position rows and inverse with independent anisotropic physical equations and separate omissions; accepted load versus
current prepared masks; retained rejection versus later acceptance; timing validation versus command mutation; ordinary/coupled failed-frame invalidation;
historical epochs, exact break thresholds and repair; and actual C/managed numerical details. These are bounded native/public-boundary controls, not scene,
default-budget convergence or rendered-performance acceptance. The separate contact skipped-work ownership, passive small-scale merit tolerance, and
whole-island position-coordinate publication issues remain open physical-quality work; their numerical laws are not changed in this migration.
In particular, prospective speed validation does not prove that final joint integration succeeds, or that every final drift rejection is reported before
another constituent publishes its coordinates.

### Complete physical trial work and its local evaluation allowance

The physical coupled owner evaluates one current-row objective,
`F(x,lambda) = K_q(x) - sum_i s_i*lambda_i + .5*sum_i gamma_i*lambda_i^2`,
where `s = target - bias - prescribed endpoint row speed`. Coefficients do not reconstruct physical ownership: a nonzero accepted coefficient can have
zero represented response. `B` is the actual owner-free primary state, with zero coefficients. The other reference slot first captures the actual
same-epoch incumbent `C`, then holds the admitted numerical initial state `I`. A compatible, finite, feasible current-row incumbent is retained only
when its current objective is justified against B. The provisional warm initialization must satisfy both B's cap and descent from that trusted reference;
otherwise the same solver initializes from that reference before constructing a direction.

Every numerical trial restores I, constructs the ordinary projected direction, applies it privately, stages its actual publication candidate, refreshes
complete-tree link velocities, and compares that final candidate against both B and I. Candidate/projection/gather formulas, relaxation and retry budgets
are unchanged. Only Pending islands execute this loop; accepted staged output is preserved across later attempts. There is one final publication, not an
additive commit after the work check. Zero-row retirement remains structural rather than requiring passive numerical descent. Invalid required reference
arithmetic is admission rejection; an evaluated unavailable trial allowance follows bounded solve retry/rejection. Attempt counts count selected directions,
not initialization comparisons. Prepared row coefficients are private scratch; accepted rows and actual response ledgers remain authoritative on rejection.

For a rigid body, K uses stored world-COM momentum and the frozen symmetric inverse metric:
`.5*inverse_mass*(p_lin^2 + (r*p_ang)^T*unit_inverse_inertia*(r*p_ang))`.
For a tree, primary values are native generalized speeds. Each link's motion is reconstructed from its parent and retained motion screws, then contributes
`.5*mass*(omega^T*unit_inertia*omega + |v_origin + omega cross COM|^2)`.
Fixed roots have zero base motion; their movable descendants still contribute. Quaternion/frame transport and metric multiplication are inside the local
evaluation bound. Independently rounded momentum/twist pairs are not treated as exactly constitutive. Moving kinematic rigid endpoints supply prescribed
source work, not dynamic K; non-kinematic zero-inverse-mass endpoints have exact zero twist before any inertia product.

`coupled_constraint_work.hlsli` evaluates midpoint/difference forms of these fixed symmetric metrics and the current source/softness terms. Midpoints use
`.5*a + .5*b`, not `.5*(a+b)`. Exactly equal finite primary/coefficient pairs bypass polarization, including signed zeros and equal subnormals; distinct
subnormals decode exactly before arithmetic. Rigid targets, complete trees, and active rows reduce in stable order. Each vector dot has individually rounded products
and left-to-right addition. Local `precise` scalar operations forbid contraction/reassociation; C++ replay uses stored volatile scalar results.

The shader carries an independently scaled value/error pair for each scalar: `(q +/- e)*2^k`, with finite binary32 mantissas, nonnegative e, and a signed
64-bit exponent encoded in two unsigned 32-bit words. A nonzero enclosure has `.5 <= max(abs(q),e) < 1`; exact zero is canonical. No shader 64-bit
arithmetic feature is required. The shader does not propagate uncertainty through the preceding
gather/ABA pipeline or prove that its stored geometry and inertia inputs represent an exact mechanical system. For an addition, the propagated error is
`e_a + e_b + rho`. For multiplication it is `abs(a_hat)*e_b + abs(b_hat)*e_a + e_a*e_b + rho`. Here `rho` is one output ULP, a conservative rounding
bound, with exact zero/identity cases omitted. Signed value arithmetic can cancel; these nonnegative contributions cannot. This operation-by-operation
recurrence replaces a global `gamma_(N+10)` estimate: reduction error follows the actual active operation count, with no `1-N*u` denominator or
capacity-based estimate. It is deliberately conservative rather than a sharp error estimate.

Stored float bits, including subnormals, decode into exact normal mantissas plus exponents. Addition first aligns both enclosures to the greater exponent;
an exponent gap of at least 26 discards the smaller point and encloses its entire magnitude with mantissa error `2^-24`. Multiplication adds exponents.
Every operation renormalizes. Bound operations round upward by positive-float successor; nonzero mantissa FTZ loss is enclosed by `eta=2^-126` in those
mantissa units, not physical work units. Error rescaling is bounded before point rescaling loss is added. Exact empty work has zero error.

The whole mechanical/source/softness reduction returns `abs(Q*2^-k-q) <= e`, with one positive common factor for the complete comparison. Individual term
mantissas are never summed without their exponents. A large unchanged motion component therefore cannot erase a small changing component. The fresh
allowance is derived for this arithmetic, not obtained by scaling the old physical-unit FTZ floor.

The fixed linear parent transport/quadratic energy graph uses fewer than `512*(M+V+R+6*S+1)` elementary enclosure operations. Its exponent magnitude is
bounded by `2*152` times that count; for nonnegative signed-32-bit buffer counts this is at most `3008263812349952 < 2^53`, within the two-word exponent.
Transport is linear in parent motion; accumulated kinetic energy never feeds descendant motion. Checked two-word arithmetic marks signed overflow
unavailable rather than wrapping. This argument is specific to the fixed evaluation graph, not a new limit on caller speed or inertia.

A nonfinite work value or nonfinite allowance never passes the finite acceptance check. An unrepresentable allowance follows the existing bounded
retry/rejection path; it is not converted into permission, a clamped finite claim, or a new invalid-Engine-input restriction. Consumed invalid enclosures
propagate before zero multiplication or identity shortcuts; an unused raw input skipped before decoding is a different case. Residual/row
nonfinite failures retain their existing early-rejection path. The policy remains provisionally `Q_hat <= E_eval`, **not** certified strict descent:
even a valid evaluation bound can admit exact positive work up to `2*E_eval`. It is not a whole-Engine or arbitrary-duration no-growth theorem.
Position pseudo-merit and final-coordinate publication remain separate gates.

All validity and work comparisons precede conversion of `merit_change` into physical-unit float diagnostics. That field retains best-effort zero for
unavailable or unrepresentable work, including underflow; it is not the normalized mantissa. Diagnostic conversion never changes a valid Merit rejection
into NonFinite, never changes admission, and cannot establish that actual work is zero.

The retained pre-range native capsule **fails at FTZ range**. With unit mass, three equality rows, initial speed `2^-100`, full warm reuse and
relaxation `.45`, returned-state energy grows from `3.111507638930571e-61` to `1.0459465474558259e-42` J with no rejection or terminal error.
The second energy is already `2.8899995368` times the first. The six cold/warm cases at speeds 1, .001 and .000001 pass their finite-window material
criterion, but finite FTZ allowances still admit real positive work. This is not physical acceptance, a bounded-speed input contract, or a reason to
add damping. The retained native failure is immutable and remains separate from validation of the scaled evaluator. Range handling is not a strict
passivity proof: even identity-metric `(1,0,0) -> (0,1,2^-100)` has positive exact work `2^-201` that can lie inside cancellation uncertainty and pass
the provisional policy. No damping, policy tightening, physical staging or budget change accompanies this arithmetic cut.

The existing C ABI exposes axis-joint articulations and D6 constraints through the entries below. Quaternion spherical configuration and typed spherical
angular descriptors are available through native C++ and sandbox JSON, but not these C ABI entries:

- `Physics_ArticulationCreate`, state get/set, link-state copy, link force, and link gravity calls.
- `Physics_ConstraintCreateD6`, get/update, enable/disable, repair, and destroy calls.
- `Physics_BeginStepEx`/`Physics_StepEx` for one submitted frame with one to 64 internal substeps.
- `Physics_EventsCopy` and `Physics_DiagnosticsGet` for bounded events and exact resource/transaction diagnostics.

## Complexity

Let:

- $B$ be active constraint blocks.
- $R$ be active scalar rows.
- $N$ be ordinary rigid solver nodes.
- $L$ be articulation links.
- $D$ be articulation velocity DOFs.
- $I$ be the fixed outer iteration count.
- $C$ be the bounded rigid colour count.

Setup work is:

$$
O(B + R + N + L + D)
$$

Solve work is:

$$
O\left(I(CR_{\mathrm{rigid}} + R_{\mathrm{coupled}} + N + L + D)\right)
$$

Because $I$ and $C$ are configured constants, fixed-iteration frame work is linear. Required iterations for a requested error tolerance are not guaranteed to
remain constant. Long chains, loops, redundancy, mass ratios, and large timesteps can all increase the required iteration count.

GPU adjacency is built once per substep and reused by every coupled iteration. Endpoint wrench gathering therefore scans each adjacency entry once instead of
sorting contributions on every iteration.

ABA performs linear work per tree, but one deep tree has a depth-dependent GPU critical path. The implementation performs best with many
small-to-medium articulations.

## Resource Costs

### Constraint resources

The production rigid-constraint layouts are enforced by `static_assert`:

| Resource | Cost |
|---|---:|
| Stable endpoint | 32 B/slot |
| Persistent typed descriptor | 288 B/slot |
| Compiled block header | 32 B/slot |
| Prepared scalar row, including numerical scratch | 96 B/row |
| Six-row block storage | 576 B/slot |
| Inverse, dependency task and colour masks | 184 B/slot |
| Complete prepared slot | **1112 B/slot** |
| Accepted ordinary record plus attempt result | 132 B/allocated owner slot |
| Shared first-rejection record | 40 B/stable slot or coupled island |
| Rigid split-correction pseudo velocity | 32 B/packed body |
| Break latch, when any slot is breakable | 32 B/slot |
| Colour-overflow and frame-sticky numerical-failure state, when slots exist | 16 B total |

For $S$ stable slots, $T$ ordinary record slots when that storage is present, $R$ rejection records, $N$ packed rigid pseudo entries,
and $K$ breakable constraints, the ordinary/shared owner reports logical storage as:

$$
1112S + 132T + 40R + 16[S>0] + 32N + 32S[K>0]
$$

Retained allocation substitutes the reported slot, body, and break capacities for the logical counts. Capacities grow geometrically and can therefore exceed
current logical use after a workload shrinks. An empty lane performs no constraint dispatches; retained resources still contribute to allocated-byte counts.

Articulation-coupled persistent constraints allocate additional matrix-free transaction storage only when a constraint touches a link:

| Coupled stream | Exact logical cost |
|---|---:|
| Endpoint and block-local preconditioner | 128 B/slot |
| Velocity block topology, transaction scratch, and two link contributions | 144 B/slot |
| Target plus target impulse | 64 B/target |
| Endpoint adjacency | 4 B/entry |
| Shared velocity/position island descriptor, numerical state and failure word | 40 B/island |
| Island block index | 4 B/index |
| Participating articulation range | 12 B/range |
| Exact self-link mobility | 96 B/participating link plus 16 B/range |
| Detached impulse ABA | 64 B/participating link plus 4 B/generalized velocity |
| Position transaction | 32 B/participating link plus 4 B/generalized velocity |

Persistent ordinary ownership adds 128 B/slot plus a 4 B attempt result. The shared first-rejection bank adds 40 B per ordinary stable slot and coupled
island. Coupled ownership separately retains a 16 B acceptance stamp and 4 B result per island, 48 B accepted rows per stable slot, 32 B world-COM response
per packed body, 4 B generalized response per component, and private bodies/generalized/ABA working buffers. Those buffers are not an additional solver path.

Complete-work evaluation adds `64*R + 8*V + 24*S + 4*(R+A) + 20*N + 192*M` bytes: R participating rigid targets, V participating native speed
components, S stable slots, A participating trees, N islands and M participating links. The 64-byte rigid and 8-byte speed records hold B and C/I;
24 bytes per slot hold I's six coefficients. Compact grouped indices, 16-byte spans and 4-byte choices belong to the same velocity owner.
The evaluator-only 192-byte link record contains 96-byte Delta and Mid records, each with six independently scaled 16-byte scalar enclosures. Each comparison traverses the packed parent-before-child
range once, reading its parent once and writing its link once; the record is reused for B/C, warm/reference and both trial comparisons. No accepted-epoch
error state or second ABA forest is retained.

Capture writes C before recovery, B after its refresh, and I after initialization admission/restoration. Trial restoration reads I before each direction;
the work evaluator is the last reader after staging and refresh. Compact layout resources change with packing/membership; references and motion scratch
are transient contents reused between visits. The work signature is one root CBV plus 17 SRVs and 14 UAVs, exactly 64 DWORDs; its last UAV is the
linear motion scratch. Existing stage/refresh resources and allocation-retirement ownership are reused.

Contacts and joints alias the same detached position state rather than maintaining independent corrections. Rigid pseudo storage belongs to the rigid
constraint solver when present. Articulated contacts own canonical complete-forest link/generalized storage; coupled joint rows map their compact response
ranges into that storage. Without articulated contacts, coupled joints own compact position storage. Shared allocations are counted once, while any retained
compact scratch remains included in allocated bytes. One owner applies coordinates at the end of each fixed-manifold position phase.

The coupled-contact lane similarly uses 192 B/contact, 32 B/target, 4 B/participant, 8 B/participating tree, and a 16-byte transaction state before optional
position-correction, mobility, impulse-ABA, and deterministic-sort storage. All terms and retained capacities are exposed through
`EngineFeatureStats::m_coupled`.

Contact-indexed dispatches and endpoint sorting use the configured capacity and explicitly initialize the inactive suffix before each fixed-size sort. A
GPU-counted contact dispatch path was measured during review, but its setup shader, indirect-command transitions, and ExecuteIndirect records cost more than
the skipped empty threads in sparse articulated scenes. The reusable GPU-counted radix-sort primitive remains available for callers whose active GPU-resident
prefix is large enough to amortize that setup cost; the coupled-contact lane keeps the faster fixed-capacity path.

### Articulation resources

The pure-tree lane accounts for the exact packed arrays rather than a planning estimate. Let:

- $A$ be articulation count.
- $L$ be link count.
- $D$ be packed joint-DOF record count.
- $P$, $V$, $F$, and $X$ be generalized position, velocity, force, and acceleration scalar counts.
- $C$ and $K$ be child-index and level-link counts.
- $M=\sum_j d_j^2$ be small joint-factor scalar count.

Force ABA topology, state, and scratch use:

$$
80A + 192L + 16D + 4(P+V+F+X) + 64L + 4C + 4K + 352L + 64D + 4M
$$

The terms respectively represent articulation headers, link topology/inertia, DOF records, generalized arrays, local and world link wrenches, traversal schedules, link ABA
scratch, DOF scratch, and joint factors. Bounded midpoint integration adds:

$$
48A + 4P + 8V
$$

Hidden link transforms add 32 B/link; world force samples are consumed directly without a separate gathered-local-force buffer. Coupled lanes add the exact mobility and impulse terms listed above only for participating trees. As with
constraints, retained buffers use geometric high-water capacities and the current logical/allocated totals are queryable through
`EngineFeatureStats::m_articulations`.

### Final readback

The packed frame output has the following exact 16-byte-aligned layout:

$$
\begin{aligned}
o_b &= 64 \\
o_e &= o_b + 256N_r \\
o_a &= \operatorname{align}_{16}(o_e + 240E_c) \\
o_p &= o_a + 64A \\
o_v &= \operatorname{align}_{16}(o_p + 4P) \\
o_{\dot v} &= \operatorname{align}_{16}(o_v + 4V) \\
o_k &= \operatorname{align}_{16}(o_{\dot v} + 4V) \\
o_r &= \operatorname{align}_{16}(o_k + 32S) \\
\text{readback bytes} &= o_r + 40R
\end{aligned}
$$

where:

- $N_r$ is the number of ordinary rigid bodies.
- $E_c$ is collision-event capacity; it is zero unless the `Collisions` event has a subscriber.
- $A$, $P$, and $V$ are articulation, generalized-position, and generalized-velocity counts; acceleration contributes the second $4V$ term.
- $S$ is the persistent constraint slot count when break latches are present.
- $R$ is the shared rejection-bank size: persistent stable slots plus coupled islands when that output is present.

The 64-byte header carries final and peak collision counters, bounded event state, substep provenance, and the rigid constraint numerical-failure slot. Each gathered GPU collision record is 240 bytes and retains
the event-time relative transform, post-solve relative velocity, and collision timing needed to publish a coherent `RbContact`; warm-start and solver-response
state remain excluded. This GPU record is distinct from the C ABI's 232-byte `Event`. Articulation output is 64 bytes/tree plus generalized position, velocity, and acceleration scalars; link transforms are reconstructed
from accepted generalized state. Whole-tree sleeping reuses articulation-output padding. Break records are 32 bytes; owner-tagged rejection records are 40 bytes.

The packed UAV retains at least one 240-byte event element as a safe zero-capacity binding, and a 16-byte substep reservation exists only when collision counters
must be accumulated. The readback itself excludes these sentinels. A multi-substep frame additionally retains 32 B/packed body of immutable frame force, but
single-substep frames never create that resource or its pipeline state.

At 10,000 ordinary bodies with no event subscriber or optional output, body state remains 2.56 MB plus the 64-byte header and alignment. Constraint rows,
warm-start impulses, ABA topology/scratch, and collision solver state remain GPU-resident.

### Memory traffic

A coupled iteration moves approximately 160 to 240 bytes per active row plus articulation traversal data. At 100,000 rows:

- Approximately 16 to 24 MB per coupled iteration.
- Approximately 128 to 192 MB for eight iterations.

This is bandwidth-friendly on the target RTX 3080 Ti, but dispatch barriers, adjacency imbalance, low-clock behavior for tiny headless workloads, and
articulation depth can dominate. Production measurements therefore report both row scaling and complete-scene frame times.

## Stability and Energy Policy

A universal promise that arbitrary contradictory constraints cannot explode at an arbitrary timestep is impossible. The production contract is instead:

- Supported passive scenes have no secular energy growth within documented tolerances.
- Invalid or non-convergent scenes fail boundedly and diagnostically.
- Energy accounting reveals errors rather than an island-wide energy clamp concealing them.

The implementation uses:

- No real-velocity Baumgarte term for hard passive rows.
- Scalar normal complementarity and friction-disc-projected tangent updates.
- Restitution computed once from pre-solve velocity.
- Explicit motor force, torque, impulse, and work limits.
- Implicit compliance and damping for soft joints.
- Monotone merit checks for coupled projected iterations.
- Warm-start scaling when $h$ changes.
- Warm-start invalidation after material feature, normal, topology, or endpoint changes.
- Maximum-endpoint-degree partitioning of simultaneous physical contact impulses, preserving passivity for redundant shared-tree contacts.
- Capped position correction separated from physical velocity, with penetration-limited initial acceleration and degree-damped projected refinements inside each fixed manifold.
- Bounded rotation-log errors for angular correction.
- Finite-state validation in debug kernels.
- Explicit regularization and diagnostics for singular systems.
- Diagonal equilibration of joint and constraint response factors, so a physically small inertia is not mistaken for a singular one solely because of its units.
- Retention of the last accepted finite state when a coupled update fails.

Sleep is managed over complete constrained components. Sleeping trees with collision shapes remain discoverable through their proxies; wholly shapeless
sleepers need no GPU participant. Cached contact impulses cannot wake a sleeping tree. An accepted nonzero contact impulse wakes the whole tree on the GPU,
allowing later internal substeps to integrate it before the single readback publishes the wake to the CPU. Motors, unresolved residuals, and external link
wrenches can also wake the complete articulation.

## Coriolis, Gyroscopic, and Dzhanibekov Preservation

The constraint architecture does not replace unconstrained dynamics. It applies constraint impulses after the force-driven prediction.

For ordinary free rigid bodies, the existing momentum formulation retains world angular momentum and obtains angular velocity from current world inertia:

$$
\omega = I_{\mathrm{world}}^{-1}L_{\mathrm{world}}
$$

As orientation changes, $\omega$ changes even with constant $L_{\mathrm{world}}$. This is equivalent to the gyroscopic term in body-frame Euler equations and
produces the intermediate-axis instability known as the Dzhanibekov effect.

For articulations, the complete ABA kinematic and force cross-product terms preserve Coriolis, centrifugal, and gyroscopic behavior. Constraint impulses enter
as additional spatial wrenches or generalized impulses; they do not remove the bias terms.

If the simulation itself uses a rotating reference frame, its fictitious Coriolis, centrifugal, and Euler forces are a separate external-force module. They are
not implied by ordinary inertial-frame multibody dynamics.

## Limitations and Pathological Cases

| Case | Expected behavior and mitigation |
|---|---|
| Long rigid constraint chains | PGS propagates information one sweep at a time; use substeps, more iterations, or an explicit articulation |
| Closed loops | Supported as external rows, but convergence depends on loop conditioning |
| Redundant constraints | $A$ becomes semidefinite; regularization selects a bounded solution and reports poor conditioning |
| Contradictory constraints | Residual remains nonzero instead of allowing unbounded impulses |
| Extreme mass ratios | Poor conditioning; stress tests cover at least $10^6:1$ |
| High-degree body hub | Exhausts bounded colours; route conflicting excess blocks through the coherent fallback, preserving correctness with less parallelism |
| Many constraints on one articulation | ABA remains linear per iteration; degree-partitioned physical impulses preserve passivity but can converge slowly, so use internal substeps or more iterations |
| Deep single articulation | Linear work but depth-dependent critical path and low GPU occupancy |
| Many small articulations | Good GPU occupancy across independent trees |
| Stiff servo | Deliberately adds energy; require finite force/torque and work limits |
| Large timestep | Jacobians and contact topology become stale; use internal substeps |
| Fast collision | Constraint solving does not prevent tunnelling; CCD remains a collision-system responsibility |
| Coulomb friction | Nonsmooth and potentially non-unique at zero slip; small solution variation is expected |
| Near-180-degree angular error | Rotation-log conditioning degrades; clamp correction and substep |
| Far from origin | Float precision degrades; use local anchors and origin shifting |
| Link self-collision | Treat as external contact rows with configurable adjacent-link filtering |
| Mid-frame topology edit | Unsupported under internal one-readback substeps |
| Per-substep CPU callback | Unsupported without abandoning the one-readback contract |
| Constraint break notification | Delivered after final readback, although GPU disabling is immediate |

## Verification Strategy

### Three independent levels

1. Analytic and dense double-precision oracles:
   - Closed-form one-body and two-body cases.
   - Tiny dense KKT solves for bilateral constraints.
   - Tiny active-set or cone-reference solves for inequalities and friction.
2. Deterministic CPU production reference:
   - The same row compiler and projections as production.
   - Stable ordering and deterministic reductions.
   - No atomics or parallel-order dependence.
3. HLSL and real-GPU validation:
   - Reuse the existing HLSL-as-C++ interop pattern.
   - Compare individual kernels before complete scenes.
   - Compare residuals, bounds, momentum, and energy envelopes rather than long-trajectory bit identity.

### Metrics

Each relevant test records:

- Position residual $\|C\|$.
- Velocity residual $\|Jv-v_{\mathrm{target}}\|$.
- Impulse-bound and friction-cone violations.
- Linear and angular momentum error.
- Kinetic plus potential energy.
- External, motor, damping, restitution, and constraint work.
- Maximum finite state magnitude.
- Iteration convergence rate.
- Colour count and fallback count.
- Actual allocated bytes and readback bytes.
- Per-stage GPU timings.

Long-duration energy tests fit a trend to:

$$
E(t)-E(0)-W_{\mathrm{external}}-W_{\mathrm{motor}}
$$

A bounded oscillating error is acceptable. A statistically significant positive slope is not.

### Dynamics acceptance gates

These are permanent regression tests retained from the dynamics milestone.

#### Zero-feature identity

With no constraints or articulations:

- The ordinary rigid-body dispatch sequence remains unchanged after the milestone-zero colour correctness fix.
- Step-by-step body output is identical to the existing integrator for the same input.
- No general-constraint or articulation resource is allocated.
- No general-constraint or articulation dispatch executes.

This is the strongest guarantee that adding the feature does not suppress existing free-body gyroscopic dynamics.

#### Dzhanibekov effect

Use a torque-free asymmetric rigid body with distinct principal inertias $I_1 < I_2 < I_3$. Initialise angular velocity mostly along the intermediate principal
axis with a small perturbation.

The test must show:

- Repeated intermediate-axis flips over a long run.
- Constant world angular momentum within the existing integrator's established numerical envelope.
- Bounded kinetic-energy error with no statistically significant positive secular slope.
- Convergence of flip timing and invariant error as the timestep is reduced.
- Equivalent qualitative behavior and invariant envelopes on deterministic CPU reference, HLSL interop, and real GPU paths.

The test should judge flips through body-axis orientation or angular-velocity zero crossings rather than require long-trajectory bit identity.

#### Coriolis and gyroscopic ABA oracle

Generate random fixed-base and floating-base articulations with asymmetric link inertias, non-zero $q$, non-zero $\dot q$, and varied one-to-six-DOF joints.

Construct an independent dense double-precision reference:

1. Compute the generalized mass matrix $H(q)$ through CRBA or inverse-dynamics column probes.
2. Compute the bias vector $h(q,\dot q)$ through inverse dynamics with $\ddot q=0$.
3. Solve:

$$
\ddot q_{\mathrm{reference}} = H^{-1}(\tau-h)
$$

4. Compare production ABA generalized and link accelerations against this reference.

The suite must include $\tau=0$ cases where non-zero acceleration comes only from Coriolis, centrifugal, and gyroscopic bias terms. An implementation that omits
those terms must fail visibly.

#### Free-floating articulation conservation

For an unconstrained floating articulation with no external wrench:

- Total world linear momentum remains constant.
- Total world angular momentum about a fixed world origin remains constant.
- Energy error remains bounded with no positive secular trend.
- CPU and GPU results agree within precision-specific envelopes.

#### Constraint overlay

For both the asymmetric rigid body and free-floating articulation:

- Adding an empty `ConstraintSet` does not change the trajectory.
- Adding then disabling a constraint restores the unconstrained path after cached impulses are invalidated.
- A physical constraint changes momentum only through its reported impulse.
- Pseudo position correction does not directly alter physical momentum.

### Mandatory constraint regressions

- Dynamic hubs with degree 33 and 100.
- Constraint remapping after body reorder, insertion, and removal.
- Stale and destroyed generational handles.
- A rigid stack connected to an articulation.
- A loop between two links of one articulation.
- A constraint between two articulations.
- Buoyancy on ordinary and articulated versions of the same object.
- Force reset across 1, 2, 4, and 8 internal substeps.
- Capacity overflow during an intermediate substep.
- Restitution unaffected by passive-work safeguards.
- Redundant welds and impossible geometry.
- Mass ratios from $1:1$ through at least $10^6:1$.
- Random tiny systems compared with dense solutions.
- World translation and rotation metamorphic tests.
- Constraint insertion-order permutations.
- Warm-start on/off convergence comparisons.
- Nine and 100 coherent support contacts on one floating tree, including one, two, and four detached position sweeps.
- CPU determinism hashes.
- Long-duration non-bitwise GPU soak tests.
- Zero-feature allocation, dispatch, and readback identity.

### Performance gates

- No-feature path: zero new feature allocations and dispatches.
- Exactly one GPU submission, one final GPU-to-CPU `CopyBufferRegion`, and one fence wait per rendered frame.
- No CPU queue wait between internal substeps.
- Measured allocation within 5 percent of the published capacity formula.
- Once occupancy is saturated, doubling rows or links must not exceed approximately 2.2 times fixed-iteration solve time.
- The 100,000-row scene runs without capacity growth, CPU synchronization, or superlinear scheduling.
- Pure-tree ABA demonstrates the same near-linear doubling behavior independently of the constraint solver.

### Completed acceptance results

| Gate | Result |
|---|---|
| Native regression suite | All 1,188 Quick, 270 Extended, and 26 Stress tests passed |
| Internal GPU substeps | All 9 tests passed, covering force replay, one-boundary scheduling, event ordering/overflow, and warm starts |
| Coupled engine integration | All 19 tests passed, including rigid/tree transfer, private proxy contacts, failure transactions, split correction, late-activating contacts, and coherent degree-nine/degree-100 support without positive support gaps |
| Collision pair and compounds | All 10 collision-pair and 11 compound-shape tests passed |
| Articulation midpoint | All 21 tests passed, including fixed/floating roots, force ABA, midpoint convergence, rollback, and resource accounting |
| Saturated row scaling | 50,004 rows: 21.683 ms; 100,002 rows: 43.290 ms; ratio 2.00, below the 2.2 limit |
| Resource accounting | Logical and retained formulas match exact typed buffer sizes, including shrink/high-water and break-state cases |
| Zero-feature identity | Empty overlay preserves output, transaction counts, and ordinary rigid trajectory with zero constraint/articulation/coupled allocation |
| Coriolis and gyroscopic ABA | Dense force-ABA oracle and zero-applied-force bias cases pass for fixed and floating trees |
| Conservation and Dzhanibekov | Rigid and floating-root torque-free regressions preserve qualitative flips, momentum envelopes, and bounded energy |
| Passive constraints and restitution | Passive work guard, soft rows, friction, motors, split correction, and elastic restitution regressions pass without artificial energy clamps |
| Pathological systems | Redundancy, contradiction, colour exhaustion, $10^6:1$ mass ratio, loops, capacity, stale handles, and bounded failure tests pass |
| Submission/readback reports | The measured 1-, 2-, and 4-substep demo frames reported one final submission, one wait, and one readback copy; that submission field did not account for prefix flushes |

The legacy benchmark uses warmed medians on the same RTX 3080 Ti. Nineteen of 22 scenes are within 10% of baseline or faster. The remaining three are not
equivalent workloads after correctness changes increased their generated contacts:

| Scene | Contact change | Physics-time change | Contact-throughput change |
|---|---:|---:|---:|
| `generated_stress.json` | +31.81% | +11.99% | +17.70% |
| `simultaneous_impact_1000.json` | +33.33% | +18.13% | +12.87% |
| `stress_test_1000.json` | +31.75% | +16.99% | +12.62% |

The 2,000-brick legacy stress scene remains within the 10% gate at +8.95%. Instrumented profiling showed it is GPU dominated; compact events and copy-engine
body gathering reduced the final measured frame to approximately 8.33 ms.

## Incremental Roadmap

| Milestone | Status | Deliverable | Exit proof |
|---|---|---|---|
| **0. Baseline and colour correctness** | Complete | Instrument stages/resources and fix colour saturation | Degree-above-32 race test and saved baseline |
| **1. Mathematical oracle** | Complete | Dense double-precision bilateral, inequality, and friction-cone reference | Analytic and random tiny systems agree |
| **2. Constraint identities and row compiler** | Complete | `BodyId`, handles, `ConstraintSet`, D6 compiler, and endpoint remapping | Reorder/add/remove and stale-handle tests |
| **3. Deterministic CPU rigid solver** | Complete | Block PGS, warm start, limits, motors, compliance, friction, and split correction | Residual, momentum, bounds, energy, and determinism tests |
| **4. GPU rigid constraints** | Complete | GPU blocks, rows, colouring, adjacency, and coherent fallback | HLSL/GPU parity and 100,002-row scaling |
| **5. Internal GPU substeps** | Complete | One submission/readback, force lifecycle, and bounded event queues | 1/2/4/8-substep and overflow tests |
| **6. CPU Featherstone ABA** | Complete | Fixed/floating roots, 0-6 DOF joints, kinematics, force ABA, and impulse ABA | Dense dynamics, Coriolis, conservation, and Dzhanibekov gates |
| **7. Effective-mass feasibility gate** | Complete | Exact and approximate articulation block preconditioners | Exact self-link mobility selected; general exact cross-link setup rejected |
| **8. GPU pure-tree articulations** | Complete | GPU ABA, midpoint integration, link proxies, sleeping, collision, and force gathering | CPU/GPU parity, buoyancy, and scaling gates |
| **9. Coupled hybrid solver** | Complete | Link contacts/loops, projected impulse ABA, outer coupling, and generalized correction | Mixed stack, four-bar, two-tree, and self-loop tests |
| **10. Robustness and energy hardening** | Complete | Work accounting, backtracking, regularization, breakage, and diagnostics | Bounded pathology behavior and passive energy gates |
| **11. API and productionisation** | Complete | ABI 2.0, diagnostics, documentation, demos, capacities, and performance hardening | Zero-feature, 100,000-row, API, demo, and full regression gates |

## Articulation Preconditioner Go/No-Go Gate

**Decision: go.** Use exact self-link spatial mobilities, assembled into a block-local approximation, with $\omega=0.9$ and bounded monotone
backtracking. Do not precompute exact cross-link Delassus blocks for the general case.

For small systems, explicitly form:

$$
A=JH^{-1}J^T
$$

using independent double-precision mass-matrix and Jacobian probes, then compare it with production impulse ABA and candidate preconditioners.

### Exact setup result

Factoring one articulation is $O(L+D)$, but a general exact block containing two links of the same articulation needs their cross mobility. Repeated row probes
cost:

$$
O\left(L+D+\sum_b r_b(L+D)\right)
$$

in the worst case, or $O(RL)$ for bounded-DOF deep trees. Hoisting the configuration-dependent ABA factorization removes repeated factor work but does not remove
the traversal required by every arbitrary cross-link probe. No $O(L+R)$ construction for all arbitrary cross-link blocks has been established, so exact
per-block probing is rejected as the production default. The production matrix-free operation remains linear: all accepted endpoint impulses are gathered and
one impulse ABA applies their combined response in $O(L+D+R)$ per coupled sweep.

### Selected linear-time approximation

The ABA factorization does permit every exact **self-link** mobility $K_i$ to be computed in one outward pass. For link $i$, define:

$$
P_i=I-S_iD_i^{-1}U_i^T
$$

For a floating root, $K_0=(I_0^A)^{-1}$; for a fixed root, $K_0=0$. Each child self mobility is:

$$
K_i=P_iX_iK_{\operatorname{parent}(i)}X_i^TP_i^T+S_iD_i^{-1}S_i^T
$$

All matrices have fixed maximum dimension six, so this pass is $O(L+D)$ time and $O(L)$ optional storage. A coupled block uses:

$$
\widehat A_b=\Gamma_b+\sum_{e\in b}J_{be}K_eJ_{be}^T+A_b^{\text{rigid}}
$$

where $\Gamma_b$ is compliance/regularization and the rigid endpoint contribution is exact. This block is exact for one articulation endpoint, for endpoints on
different articulations, and for articulation-to-rigid blocks. A block joining two links of the same articulation omits their cross term. Blocks also omit
cross-block response through a shared articulation; the subsequent impulse ABA still applies that response exactly.

The approximation is symmetric positive definite after the same rank-aware proximal regularization used by the solver, but it is not a global majorizer.
Canonical fixed, floating, and loop systems all produced negative minimum eigenvalues for $\widehat A-A$. The solver therefore uses simultaneous block updates
with $\omega=0.9$ and the bounded island-merit backtracking already required by the coupled lane. A repeated-block hub can use a conservative fallback formed
from free-link physical inverse inertias scaled by the articulation's active coupled-block degree; this restores a majorizing step at the cost of slower
convergence.

Joint-coordinate limit and drive rows use the corresponding cached $D_i^{-1}$ block directly rather than passing through the link-space approximation.

### Measured gate

The Debug double-precision gate forms $A$, measures the generalized spectrum of $\widehat A^{-1}A$, and runs the same simultaneous block update shape intended
for the GPU. The required relative $A$-norm error is below 0.1 after 16 sweeps and below 0.01 after 64 sweeps.

| Canonical system | $\kappa(A)$ | $\kappa(\widehat A^{-1}A)$ | $\lambda_{\max}(\widehat A^{-1}A)$ | Error after 16 | Error after 64 |
|---|---:|---:|---:|---:|---:|
| Floating seven-link tree, three blocks | 53.40 | 10.00 | 1.436 | 0.0844 | $1.10\times10^{-4}$ |
| Fixed eight-link tree, three blocks | 10.74 | 1.99 | 1.307 | $5.83\times10^{-7}$ | Numerical floor |
| Same-articulation loop plus another block | 16.54 | 13.08 | 1.955 | 0.0611 | $5.94\times10^{-5}$ |
| Articulation-to-rigid, $10^4:1$ mass ratio | 3.23 | 1.00 | 1.000 | Numerical floor | Numerical floor |
| Two articulations joined by one block | 3.33 | 1.00 | 1.000 | Numerical floor | Numerical floor |

No canonical case required a backtrack at $\omega=0.9$. Separate gates prove exact dense/ABA response parity, fixed- and floating-root recurrence parity, and the
repeated-hub majorizing fallback. Counters inside the actual recurrence and block assembly approximately double when links and rows are doubled together,
confirming the selected setup's linear implementation. The rejected exact cross-link probe cost is the worst-case traversal analysis above, not a timing claim.

GPU storage is optional and only allocated for coupled articulations. A symmetric $6\times6$ self-link mobility occupies 96 bytes per participating link.
A symmetric block inverse has the same 96-byte upper bound for a six-row block. Setup is $O(L+R)$ time and memory; each fixed
coupled iteration remains $O(L+R)$.

This gate covers tiny bilateral systems and the preconditioner's numerical role. Separate completed acceptance suites cover projected bounds, Coulomb cones,
large mixed islands, transaction backtracking, and GPU float parity.

Pure-tree ABA is not conditional on this gate.

## Demonstration Catalogue

Each command is available through `physics-sandbox.exe -demo <command>`. Interactive FPS is the minimum of five observed post-warm-up samples; headless physics
FPS is the measured interval after a 300-step warm-up in the final production Release acceptance run.

| Command | Features proved | Interactive min FPS | Headless physics FPS |
|---|---|---:|---:|
| `pendulum` | Passive physical pendulum and analytic small-angle period | 60 | 63.45 |
| `rigid-joints` | Ball, hinge, slider, weld, motor, breakable D6, limits, and compliance | 60 | 60.33 |
| `rigid-chain` | Long graph-coloured rigid chain and iteration propagation | 60 | 51.86 |
| `four-bar` | Closed-loop mechanism that cannot be represented by one tree | 60 | 49.65 |
| `fixed-articulations` | Fixed-root Featherstone chains of several sizes | 60 | 150.83 |
| `ragdolls` | Many branched floating articulations and sustained ground contacts | 60 | 40.44 |
| `robot-motors` | Reduced-coordinate links controlled by driven and limited rows | 41 | 45.20 |
| `robot-gripper` | Motors, prismatic fingers, manipulation contact, and tree feedback | 44 | 50.06 |
| `vehicle-suspension` | Driven compliant suspension, travel limits, and wheel contacts | 60 | 62.60 |
| `suspension-bridge` | Cyclic rigid graph with pinned corners and distributed load | 60 | 56.55 |
| `mixed-coupling` | Rigid-to-tree and direct tree-to-tree persistent constraints | 44 | 50.92 |
| `articulation-push` | Driven articulation transferring momentum to a rigid stack | 45 | 48.74 |
| `two-robot-load` | Two independent trees supporting one constrained payload | 47 | 53.43 |
| `mixed-contacts` | Rigid/tree, tree/tree, and non-adjacent same-tree contacts | 60 | 137.22 |
| `buoyant-articulation` | GPU buoyancy applied independently to floating-tree links | 60 | 279.16 |
| `floating-conservation` | Coriolis effects, momentum conservation, and bounded energy | 60 | 77.55 |
| `dzhanibekov` | Rigid and floating-root intermediate-axis instability | 60 | 72.52 |
| `constraint-pathologies` | Redundancy, extreme mass ratios, and near-singular loops | 60 | 60.99 |
| `constraint-stress` | Cyclic two-dimensional graph and fixed-iteration scaling | 60 | 92.23 |
| `constraints-full-showcase` | Rigid joints, loops, breakage, fixed/floating articulations, motors, limits, mixed coupling, and self-contact | 60 | 43.68 |

The final ragdoll scene uses three internal substeps, 10 physical contact sweeps, and one detached position sweep with one final wait and
readback per frame. Its historical submission field counted only the final submission, not prefix flushes. Three 10-second runs settled to
35.4-50.7 mm maximum ground penetration with a 54.0 mm worst impact transient. Its warmed three-run median
was 49.65 physics FPS; the conservative final all-demo acceptance sample was 40.44 FPS. Visual checks at construction and after settling confirmed six
separated bodies, outward-facing arms, uncrossed legs, and no deep ground clipping.

## Final Design Position

The architecture preserves Rylogic's existing impulse philosophy:

- Ordinary rigid bodies retain their current momentum-based gyroscopic integration.
- Featherstone ABA provides the high-performance reduced-coordinate tree operator.
- Local projected impulses provide contacts, loops, limits, motors, and arbitrary graph topology.
- Constraint and force systems communicate only through spatial wrenches, impulses, and stable solver-node identities.
- No production path constructs a global constraint matrix.
- Users pay only for the solver features they instantiate.

All roadmap milestones and the effective-mass feasibility gate are complete. The resulting architecture retains the pure-tree ABA fast path while adding
arbitrary graph topology without introducing a global matrix, per-substep host synchronization, or mandatory feature overhead.
