# Ordinary rigid-contact friction

`src/compute/resolve.hlsl` owns the ordinary rigid-contact velocity solve.
The material's combined normalized ratio `f` becomes the Coulomb coefficient
`mu = min(f, 0.9999) / (1.000001 - min(f, 0.9999))`.

The centroid solve preserves the coupled restitution response for impacts.
Its accepted physical impulse and the accepted normal warm-start impulse contribute
to the contact's accumulated normal support. Positional Baumgarte correction is
linear-only and does not contribute to that physical support or its cache.

Each manifold point owns an accumulated tangential impulse for the current solve.
Centroid friction is shared equally among these accumulators. For a point, compute
the slip-cancelling correction, add it to the previous tangent impulse, project that
candidate onto the disc of radius `mu * accumulated_normal / point_count`, and apply
only the difference. Store only the delta accepted by the existing passive-energy guard.
This limits the sum of point budgets without discarding broad-face torsional resistance.

Friction does not depend on fresh normal closing or positive penetration bias:
warm starting may already have supplied the normal support. Extra solver sweeps
cannot spend the same normal budget repeatedly. Point tangent accumulators are
cleared when collision generation creates the contact; only the normal physical
impulse is retained by the cross-frame warm-start cache.

The four point accumulators add 64 bytes to the private contact record and require
no additional GPU allocation or dispatch. The C++ shader replay runner must copy
updated contact records back after each colour batch, just as GPU records persist
between dispatches; otherwise repeated sweeps incorrectly reset the tangent budget.

`FrictionSolverTests` covers slip with already-satisfied normal support, iteration-
independent sliding bounds, four-point torsion versus bias-only correction, and a
default-warm-start sphere held on a ten-degree slope using the exact gravity-balancing
torque. Keep impact/restitution and resting-contact tests alongside these regressions.

## Articulation-coupled contact owner

`src/compute/coupled_contact_newton.hlsl` solves each connected contact component
against its complete mechanical response `W = Jp M^-1 Jp^T`. This includes same-tree
cross terms, cross-tree contacts, and ordinary dynamic rigid participants. Endpoint
self-response sums supply positive normal/tangent unit scales `a,c`; they do not
replace `W`. A locked tangent plane uses the positive normal scale, not a tangent
inverse or a regularized mechanical response.

For unit normal `n`, accumulated impulse `lambda`, and actual relative velocity `v`
with its normal target already subtracted, the signed ordinary-law residual is

```
lambda_n = dot(n, lambda), lambda_t = lambda - lambda_n*n
v_n      = dot(n, v),      v_t      = v - v_n*n
F = n*min(a*lambda_n, v_n)
  + c*lambda_t - ProjectDisc(c*lambda_t - v_t, c*mu*max(lambda_n, 0))
```

`contact_coulomb_linearisation.hlsli` owns this residual and its selected piecewise
derivatives. The normal equation has no friction multiplier term. It is not the
optimality condition of a cone-energy objective.

The contact update starts from actual primary velocity after the preceding
constraint update. Its equation is `v_trial = v_actual + W*delta_lambda`. An
equivalent total-impulse formulation must first recover
`v_free = v_actual - W*lambda`; applying the accumulated impulse again is incorrect.
The frozen Newton Jacobian is `A*W+B`, generally nonsymmetric. CGLS solves this one
linear equation with its existing `3*row_count` cap; this cap and linear residual
are not nonlinear convergence certificates.

### Nonlinear candidate safeguard

One caller sweep searches projected points on a relaxed raw Newton direction,
using the same scaled-coordinate metric as its linear solve. Write
`S = diag(s_n,s_t,s_t)` in the contact basis, where `s_n=1/a` and
`s_t=1/max(a,c)`. For `lambda=S*z`, the feasible cone in `z` has slope
`mu*s_n/s_t`, not `mu`. The projection minimizes `|S^-1*(p-value)|^2`;
it may change normal support. It constructs a search point, not an associated
cone-energy contact law or a constrained Newton least-squares solution.

For physical normal/tangent input `(x_n,x_t)`, an outside-cone point's projected
normal support is
`max(0,(s_t^2*x_n + mu*s_n^2*|x_t|)/(s_t^2 + mu^2*s_n^2))`.
The tangent points along `x_t` with length `mu` times that support.
`ProjectCoulombMetricImpulse` preserves already-feasible points and splits product
mantissas/exponents so squared scales, cone slope and individual numerator terms
need not be representable. The HLSL split restores the sign discarded by `frexp`
to match the signed C++ `std::frexp` contract. Tangent components are exponent-normalized
before division, so a large divisor cannot introduce a flushed reciprocal.
The feasibility comparison and final per-component tangent construction also keep
exponents separate, avoiding destructive flush-to-zero intermediates.
Zero friction projects to the nonnegative normal axis; the apex needs no division.
Tangent construction uses the represented projected normal without an additional
support-preserving clipping pass. Unrepresentable results are arithmetic failures,
not silently clamped impulses or guessed input limits.

The candidate search shortens the raw direction **before** projecting:

```
raw          = lambda + alpha*eta*S*direction
lambda_trial = ProjectMetric(raw)
delta        = represented_lambda_trial - lambda
v_trial      = v_actual + W*delta
alpha        = 1, 1/2, 1/4, ...
```

`direction` is the frozen CGLS `step`. This is not interpolation toward a fixed
projected endpoint: a poor full endpoint, including a non-descending endpoint,
does not veto the curve's shorter points. Every changed projected trial needs
its own complete `W*delta`; scaling a previous response is incorrect.

If the Newton arc exhausts without acceptance, the same search mechanism uses
the original-law negative gradient. The owner restores **original** `-F` and
calls the existing true transpose, so `work.gradient` is **negative**
`-S*J^T*F`. The raw replacement point is `lambda + alpha*eta*S*work.gradient`,
with a **plus** sign. There is no second linear solve or separate fallback
globalization rule.

For positive gradient `g=S*J^T*F` and `p_z=P_K(z-eta*g)`, nearest projection gives
`g^T*(p_z-z) <= -|p_z-z|^2/eta`. A strict negative selected-Jacobian slope is also
sufficient at the feasible law's nonsmooth ties: at a normal tie its common
residual is nonnegative, so the actual directional derivative uses the smaller
active derivative. At a positive-radius disc tie, with `u=z_disc/|z_disc|`,
feasibility implies `F_t.u <= 0`; the outward radial derivative can only reduce
the merit derivative relative to the selected interior branch. A zero tangent
residual at the apex has zero first derivative of its squared norm. This is a
one-sided descent argument, not an assumption that the merit is globally smooth.

Each trial evaluates
the full ordinary law and compares its summed squared signed residual against the
**original nonlinear residual**, not the residual overwritten by CGLS. A strict
decrease, or a component residual within the existing velocity tolerance, accepts
the trial. At most one candidate is committed per caller sweep; caller iteration
count, relaxation, tolerance, and the linear-solve budget are unchanged.

Each row whose raw represented point equals its accepted impulse bypasses
projection, before any projection arithmetic. The whole search stops when the
represented trial impulses no longer differ from the accepted impulses. This
does not assume bitwise projector idempotence at zero fraction. Exhaustion restores
the accepted candidate and clears both retained delta and response.

Such a no-change is unresolved finite state, not convergence or infeasibility.
In particular, ordinary-law squared merit can have a positive constrained
stationary value even with SPD physical response and a feasible law witness:
the projected gradient can be zero while every point on the Newton arc worsens
merit. Neither more iterations nor the local descent argument above proves
escape from that state. No recovery or new failure policy is implied.
There is no guessed magnitude cutoff, extra accepted nonlinear iteration, or
alternative physical law. Nonfinite arithmetic retains the existing
transaction-failure semantics, distinct from a legitimate unchanged proposal.
Predicted point response has floating-point rounding error; final evidence must
still reconstruct point velocities from actual primary root/q/qdot and ordinary
momenta in the frozen generating contact frames.

Each changed projected trial costs one complete mechanical product, including
trials later rejected. The fallback additionally costs one original-law transpose
product. Halving repeats mechanical response, but never CGLS or physical
publication. This variable cost has no small worst-case bound from represented
no-change termination alone. Original-law recovery and projection add row algebra
with no added buffers or ABI fields.

Final Krylov scratch semantics are not linear-solve telemetry: `residual` contains
original `-F`; `gradient` remains the last linear gradient unless the fallback
was attempted, when it contains `-S*J^T*F`. `step`, `direction` and
`jacobian_product` remain frozen-linear work. `operand`/`response` describe the
accepted represented candidate minus the base impulse and its full `W*delta`.
An unchanged component has exactly zero delta and response. `inverse_weights.xy`
retain the positive scales; after selection, `.zw` record the component's
candidate-response and fallback-transpose counts on every row, excluding CGLS
and final publication. None of these fields alone identifies convergence or
linear cap termination. Actual accepted impulse and primary velocity remain the
authoritative final physical state.

Each connected component still belongs to one shader invocation: independent
components parallelize, but a large cross-tree pile is serial inside one lane.
Small-component correctness tests establish neither large-component throughput
nor rendered frame rate.

`ArticulatedContactSafeguardGpuTests` supplies a standalone ten-link, three-contact
weak-support fixture. Its source-constant free speeds remove both restitution
targets and seed response; the GPU applies the seed exactly once. A signed
point-wrench witness checks feasibility without assigning expected point speeds.
One observation checks first-update nonlinear descent, and another checks the
actual final ordinary law. Both execute the complete ten-sweep submission. This
frozen-owner test has no interleaved joint corrections and does not replace the
original six-frame integrated ragdoll gate.

`ArticulatedContactArcGpuTests` uses three independently supplied frozen
original-workload models as standalone ten-update source-constant fixtures.
Their free generalized speeds and feasible witnesses are supplied data, not a
second mechanics reconstruction or a trajectory replay. Signed point wrenches
establish seeded/witness primary states before their ordinary laws are measured.
Reference-vector checks use `128*float_epsilon*max(1,|u_free|_inf,|u_reference|_inf)`
to account for cancellation between free velocity and impulse response. They
report absolute and relative differences; this representation check does not
replace or change the separate `.01` physical-law gate.
The observer also retains GPU warm-state speeds and iteration-major candidate
work. `ContactProjectedArcTests` calls the production search/selector through a
private runtime shader entry point, covering actual gradient-arc shortening,
raw represented no-change, exhausted zero-gradient state, convergence, and
invalid arithmetic without a production override.

`ArticulatedContactDirectionGpuTests.SupportedPointNonDescent` covers a distinct
case: one floating link with positive mass and a full physical central
inertia, contacting a static body. Its feasible sliding witness passes through
the signed point-wrench response. Its support-preserving Newton endpoint is not a
descent direction; the regression requires the actual ten-sweep physical law.
The fixture has no joints or position entries; its observer preserves those empty
prefixes. `ContactMetricProjectionTests` checks the nearest point against an
independent double-precision boundary minimizer with flush-to-zero enabled, then
dispatches the same helper and represented inputs on the GPU. Its cases include
the exact unequal-scale discriminator, mirrored signs, zero friction, the apex,
and large/small finite values. CPU agreement alone does not establish correct
shader arithmetic. These focused cases do not establish whole-workload
convergence or rendered throughput.
