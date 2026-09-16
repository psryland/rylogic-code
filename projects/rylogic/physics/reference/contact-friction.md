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
