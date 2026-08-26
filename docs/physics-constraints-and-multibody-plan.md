# GPU Constraint and Multibody Physics Architecture

**Status:** Implemented and accepted
**Original design date:** 2026-08-25
**Last verified:** 2026-08-26

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
and 19 physics-sandbox demonstrations.

| Design requirement | Verified outcome |
|---|---|
| Fixed-iteration scaling | 50,004 rows solved in 21.683 ms and 100,002 rows in 43.290 ms on an RTX 3080 Ti: 2.00x time for 2.00x rows |
| One host boundary per frame | Every 120-frame demo soak reported one command-list submission, one fence wait, and one GPU-to-CPU readback copy per frame, including 2- and 4-substep demos |
| Optional feature cost | Empty constraint and articulation inputs allocate and dispatch no constraint, articulation, or coupled-solver resources; collision-event and multi-substep force storage are also demand-created |
| Featherstone fast path | Fixed- and floating-root trees run force ABA and bounded implicit-midpoint integration without entering the arbitrary-constraint lane |
| Dynamics preservation | Dense ABA oracles, zero-torque Coriolis cases, free-floating conservation, torque-free rigid motion, and Dzhanibekov regressions pass |
| Robust arbitrary constraints | D6 joints, loops, rigid/tree and tree/tree coupling, link contacts, split position correction, breakage, and bounded non-convergence diagnostics pass targeted suites |
| Real-time demonstrations | All 19 Release demos measured at least 41 FPS interactively and at least 43.60 physics FPS in warmed headless measurement |
| Legacy performance | 19 of 22 baseline scenes remained within the 10% tolerance or improved; the other three generated 31.8-33.3% more contacts while taking 13.8-19.5% more time, improving contact throughput |

The final native validation passed all 1,188 Quick tests and the complete targeted internal-substep, coupled-engine, collision-pair, compound-shape, and
articulation-midpoint suites. Independent reviews of the architecture, acceptance coverage, demonstrations, and final GPU frame-path optimization found no
high-confidence correctness or design issues.

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
\Pi_{\mathcal K_i}
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

$\Pi_{\mathcal K}$ projects onto the allowed impulse set:

- A bilateral lock is unbounded.
- A lower limit or contact normal uses $\lambda \geq 0$.
- A motor uses $-F_{\max}h \leq \lambda \leq F_{\max}h$.
- A Coulomb contact block uses:

$$
\lambda_n \geq 0,\qquad \|\lambda_t\| \leq \mu\lambda_n
$$

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
Warm-start all active blocks

repeat outer_iteration_count:
    Solve one coloured rigid-only PGS sweep
    UAV barrier
    Solve one articulation-coupled projected/ABA sweep
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
| `Resolve` | Compile blocks, warm-start, and run rigid PGS plus articulation-coupled ABA sweeps |
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
        CSGatherArticulationLinkForces
        CSIntegrateOrdinaryBodies

        CSIntegrateArticulationsImplicitMidpoint
            one invocation serially traverses one articulation
            solve start acceleration and bounded midpoint iterations
            publish final generalized state and link proxies transactionally

        CSSleepWake
        CSBroadPhase
        CSNarrowPhase

        CSCompileContactBlocks
        CSCompilePersistentConstraintBlocks
        CSBuildEndpointAdjacency
        CSAssignRigidColoursAndFallbacks
        CSWarmStartRigidBlocks
        CSWarmStartCoupledBlocksThroughABA

        repeat outer_iteration_count:
            repeat active_rigid_colours:
                CSRigidBlockPGS
                UAV barrier

            CSCoupledResidualAndProjection
            CSCoupledGatherEndpointWrenches
            CSApplyRigidWrenches
            CSImpulseABAInward
            CSImpulseABAOutward
            CSCoupledAcceptOrBacktrack
            UAV barrier

        repeat position_iteration_count:
            CSSolveRigidPseudoBlocks
            CSSolveCoupledGeneralizedCorrections
            CSForwardKinematics
            CSWriteLinkProxyTransformsAndBounds

        CSSelectiveContactRefresh
        CSConstraintCleanupAndBreakDetection
        CSSleepUpdate
        if collision subscribers exist:
            CSCompactCollisionEventsAndReserveRange
            CSAppendCompactCollisionEvents
        else:
            CSPrepareSubstepCounters

    GPU-to-GPU CopyBufferRegion final ordinary-body range into packed output
    GPU-to-GPU CopyBufferRegion optional break and coupled-failure ranges
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
5. Consume and clear the working wrench during ordinary rigid integration or articulation inward dynamics.

Articulation link proxies follow this lifecycle even though the ordinary-body integration dispatch skips them.

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

	struct D6ConstraintDesc
	{
		BodyFrame m_frame_a;
		BodyFrame m_frame_b;
		std::array<ConstraintAxisDesc, 3> m_linear;
		std::array<ConstraintAxisDesc, 3> m_angular;
		float m_break_force;
		float m_break_torque;
		bool m_collide_connected;
		bool m_enabled;
	};

	class ConstraintSet
	{
	public:
		ConstraintHandle Add(D6ConstraintDesc const& desc);
		ConstraintHandle Add(BallSocketConstraintDesc const& desc);
		ConstraintHandle Add(HingeConstraintDesc const& desc);
		ConstraintHandle Add(SliderConstraintDesc const& desc);
		ConstraintHandle Add(WeldConstraintDesc const& desc);

		void Update(ConstraintHandle handle, D6ConstraintDesc const& desc);
		void SetEnabled(ConstraintHandle handle, bool enabled);
		void Repair(ConstraintHandle handle);
		void Remove(ConstraintHandle handle);
		D6ConstraintDesc const& Get(ConstraintHandle handle) const;
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

`D6ConstraintDesc` is the general primitive. Named joint descriptors are convenience interfaces that compile to it.

Additional API rules:

- Anchors are body-local or link-local, never ambiguous world snapshots.
- Linear, angular, force, torque, compliance, and damping units are documented independently.
- Constraint and link handles are generational.
- Every rigid body has a stable `BodyId`; remapping uses it rather than current vector position.
- Missing or duplicate endpoint identities fail `BeginStep` before submission.
- Constraint and articulation topology cannot change while a step is pending.
- Parameter changes upload as merged dirty stable-slot ranges.
- Link self-collision and connected-body collision are configurable.
- Per-row impulses remain GPU-resident unless diagnostics explicitly request them.
- `ConstraintsBroken` and `CoupledConstraintFailures` deliver bounded post-readback events; GPU break disabling takes effect during the substep that detects it.
- `LastFeatureStats()` reports active counts, retained capacities, dispatches, logical bytes, allocated bytes, packed output size, and terminal failure state.
- The ABI 2.0 DLL surface mirrors the generational engine, shape, and body handles.

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

The versioned C ABI exposes the same model through:

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
| Persistent D6 descriptor | 256 B/slot |
| Compiled block header | 32 B/slot |
| Compiled scalar row, including accumulated impulse | 96 B/row |
| Six-row block storage | 576 B/slot |
| Complete persistent slot | **896 B/slot** |
| Rigid split-correction pseudo velocity | 32 B/participating body |
| Break latch, when any slot is breakable | 32 B/slot |
| Overflow word, when slots exist | 4 B total |

For $S$ stable slots, $N$ participating rigid bodies, and $K$ breakable constraints, exact logical storage is:

$$
896S + 4[S>0] + 32N + 32S[K>0]
$$

Retained allocation substitutes the reported slot, body, and break capacities for the logical counts. Capacities grow geometrically and can therefore exceed
current logical use after a workload shrinks. An empty `ConstraintSet` reports zero constraint allocation and dispatches.

Articulation-coupled persistent constraints allocate additional matrix-free transaction storage only when a constraint touches a link:

| Coupled stream | Exact logical cost |
|---|---:|
| Endpoint and block-local preconditioner | 128 B/slot |
| Velocity block topology, transaction scratch, and two link contributions | 144 B/slot |
| Target plus target impulse | 64 B/target |
| Endpoint adjacency | 4 B/entry |
| Island state, failure state, and selection | 68 B/island |
| Island block index | 4 B/index |
| Participating articulation range | 12 B/range |
| Exact self-link mobility | 96 B/participating link plus 16 B/range |
| Detached impulse ABA | 64 B/participating link plus 4 B/generalized velocity |
| Position transaction | 32 B/participating link plus 4 B/generalized velocity |

The coupled-contact lane similarly uses 192 B/contact, 32 B/target, 4 B/participant, 8 B/participating tree, and a 16-byte transaction state before optional
position-correction, mobility, impulse-ABA, and deterministic-sort storage. All terms and retained capacities are exposed through
`EngineFeatureStats::m_coupled`.

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
80A + 192L + 16D + 4(P+V+F+X) + 32L + 4C + 4K + 336L + 64D + 4M
$$

The terms respectively represent articulation headers, link topology/inertia, DOF records, generalized arrays, link wrenches, traversal schedules, link ABA
scratch, DOF scratch, and joint factors. Bounded midpoint integration adds:

$$
48A + 4P + 8V
$$

Hidden link force/transform proxies add 64 B/link. Coupled lanes add the exact mobility and impulse terms listed above only for participating trees. As with
constraints, retained buffers use geometric high-water capacities and the current logical/allocated totals are queryable through
`EngineFeatureStats::m_articulations`.

### Final readback

The packed frame output has the following exact 16-byte-aligned layout:

$$
\begin{aligned}
o_b &= 64 \\
o_e &= o_b + 256N_r \\
o_a &= \operatorname{align}_{16}(o_e + 144E_c) \\
o_p &= o_a + 64A \\
o_v &= \operatorname{align}_{16}(o_p + 4P) \\
o_{\dot v} &= \operatorname{align}_{16}(o_v + 4V) \\
o_k &= \operatorname{align}_{16}(o_{\dot v} + 4V) \\
o_f &= \operatorname{align}_{16}(o_k + 32S) \\
\text{readback bytes} &= o_f + 32I
\end{aligned}
$$

where:

- $N_r$ is the number of ordinary rigid bodies.
- $E_c$ is collision-event capacity; it is zero unless the `Collisions` event has a subscriber.
- $A$, $P$, and $V$ are articulation, generalized-position, and generalized-velocity counts; acceleration contributes the second $4V$ term.
- $S$ is the persistent constraint slot count when break latches are present.
- $I$ is the coupled-island count when failure records are present.

The 64-byte header carries final and peak collision counters, bounded event state, and substep provenance. Each public collision event is 144 bytes and excludes
solver-only transforms, timing, warm-start, and response state. Articulation output is 64 bytes/tree plus generalized position, velocity, and acceleration
scalars; link transforms are reconstructed from accepted generalized state. Break and coupled-failure records are 32 bytes each.

The packed UAV retains at least one 144-byte event element as a safe zero-capacity binding, and a 16-byte substep reservation exists only when collision counters
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
- Cone-projected dissipative friction.
- Restitution computed once from pre-solve velocity.
- Explicit motor force, torque, impulse, and work limits.
- Implicit compliance and damping for soft joints.
- Monotone merit checks for coupled projected iterations.
- Warm-start scaling when $h$ changes.
- Warm-start invalidation after material feature, normal, topology, or endpoint changes.
- Capped position correction separated from physical velocity.
- Bounded rotation-log errors for angular correction.
- Finite-state validation in debug kernels.
- Explicit regularization and diagnostics for singular systems.
- Retention of the last accepted finite state when a coupled update fails.

Sleep is managed over complete constrained components. A motor, unresolved residual, collision, or external link wrench wakes the complete articulation.

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
| Many constraints on one articulation | ABA remains linear per iteration, but the projected iteration may need stronger damping |
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
| Native regression suite | All 1,188 Quick tests passed |
| Internal GPU substeps | All 9 tests passed, covering force replay, one-boundary scheduling, event ordering/overflow, and warm starts |
| Coupled engine integration | All 17 tests passed, including rigid/tree transfer, private proxy contacts, failure transactions, and split correction |
| Collision pair and compounds | All 10 collision-pair and 11 compound-shape tests passed |
| Articulation midpoint | All 21 tests passed, including fixed/floating roots, force ABA, midpoint convergence, rollback, and resource accounting |
| Saturated row scaling | 50,004 rows: 21.683 ms; 100,002 rows: 43.290 ms; ratio 2.00, below the 2.2 limit |
| Resource accounting | Logical and retained formulas match exact typed buffer sizes, including shrink/high-water and break-state cases |
| Zero-feature identity | Empty overlay preserves output, transaction counts, and ordinary rigid trajectory with zero constraint/articulation/coupled allocation |
| Coriolis and gyroscopic ABA | Dense force-ABA oracle and zero-applied-force bias cases pass for fixed and floating trees |
| Conservation and Dzhanibekov | Rigid and floating-root torque-free regressions preserve qualitative flips, momentum envelopes, and bounded energy |
| Passive constraints and restitution | Passive work guard, soft rows, friction, motors, split correction, and elastic restitution regressions pass without artificial energy clamps |
| Pathological systems | Redundancy, contradiction, colour exhaustion, $10^6:1$ mass ratio, loops, capacity, stale handles, and bounded failure tests pass |
| Submission/readback contract | Every measured 1-, 2-, and 4-substep demo frame reports one submission, one wait, and one GPU-to-CPU readback copy |

The legacy benchmark uses warmed medians on the same RTX 3080 Ti. Nineteen of 22 scenes are within 10% of baseline or faster. The remaining three are not
equivalent workloads after correctness changes increased their generated contacts:

| Scene | Contact change | Physics-time change | Contact-throughput change |
|---|---:|---:|---:|
| `generated_stress.json` | +31.81% | +13.77% | +15.86% |
| `simultaneous_impact_1000.json` | +33.33% | +19.48% | +11.60% |
| `stress_test_1000.json` | +32.15% | +18.89% | +11.16% |

The 2,000-brick legacy stress scene remains within the 10% gate at +9.36%. Instrumented profiling showed it is GPU dominated; compact events and copy-engine
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
FPS is the warmed production Release median.

| Command | Features proved | Interactive min FPS | Headless physics FPS |
|---|---|---:|---:|
| `pendulum` | Passive physical pendulum and analytic small-angle period | 60 | 69.36 |
| `rigid-joints` | Ball, hinge, slider, weld, motor, breakable D6, limits, and compliance | 60 | 63.47 |
| `rigid-chain` | Long graph-coloured rigid chain and iteration propagation | 60 | 55.68 |
| `four-bar` | Closed-loop mechanism that cannot be represented by one tree | 60 | 57.58 |
| `fixed-articulations` | Fixed-root Featherstone chains of several sizes | 60 | 89.23 |
| `ragdolls` | Many branched floating articulations and contacts | 60 | 108.10 |
| `robot-motors` | Reduced-coordinate links controlled by driven and limited rows | 41 | 43.60 |
| `robot-gripper` | Motors, prismatic fingers, manipulation contact, and tree feedback | 44 | 45.69 |
| `vehicle-suspension` | Driven compliant suspension, travel limits, and wheel contacts | 60 | 57.33 |
| `suspension-bridge` | Cyclic rigid graph with pinned corners and distributed load | 60 | 53.63 |
| `mixed-coupling` | Rigid-to-tree and direct tree-to-tree persistent constraints | 44 | 47.39 |
| `articulation-push` | Driven articulation transferring momentum to a rigid stack | 45 | 45.74 |
| `two-robot-load` | Two independent trees supporting one constrained payload | 47 | 48.34 |
| `mixed-contacts` | Rigid/tree, tree/tree, and non-adjacent same-tree contacts | 60 | 126.74 |
| `buoyant-articulation` | GPU buoyancy applied independently to floating-tree links | 60 | 67.55 |
| `floating-conservation` | Coriolis effects, momentum conservation, and bounded energy | 60 | 81.70 |
| `dzhanibekov` | Rigid and floating-root intermediate-axis instability | 60 | 75.38 |
| `constraint-pathologies` | Redundancy, extreme mass ratios, and near-singular loops | 60 | 59.94 |
| `constraint-stress` | Cyclic two-dimensional graph and fixed-iteration scaling | 60 | 92.32 |

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
