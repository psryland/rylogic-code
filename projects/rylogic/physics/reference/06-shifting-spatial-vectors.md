# Tutorial 06 — Shifting Spatial Vectors

## Shift vs. Transform

There are two ways to "move" a spatial vector to a different place:

| Operation | What changes | What stays | Rylogic |
|-----------|-------------|------------|---------|
| **Shift** | Reference point | Coordinate axes | `Shift(vec, offset)` |
| **Transform** | Both point and axes | — | `a2b * vec` |

Shifting is simpler. It answers: "I know the velocity (or force) measured at point $A$. What is it measured at point $B$, using the same coordinate axes?"

## Shifting Motion Vectors

For a motion vector, the angular velocity is the same everywhere, and only the linear part changes:

$$\text{Shift}(\hat{\mathbf{v}}, \mathbf{d}) = \begin{bmatrix} \boldsymbol{\omega} \\ \mathbf{v} + \boldsymbol{\omega} \times \mathbf{d} \end{bmatrix}$$

where $\mathbf{d}$ is the offset from the old reference point to the new one.

```cpp
// spatial.h — c.f. RBDA 2.21
Vec8f<Motion> Shift(Vec8f<Motion> const& motion, v4_cref ofs)
{
    return Vec8f<Motion>(motion.ang, motion.lin + Cross(motion.ang, ofs));
}
```

**Intuition**: If you're standing further from a spinning object, you see a higher tangential velocity. The extra velocity is $\boldsymbol{\omega} \times \mathbf{d}$.

## Shifting Force Vectors

For a force vector, the linear force is the same everywhere, and only the torque changes:

$$\text{Shift}(\hat{\mathbf{f}}, \mathbf{d}) = \begin{bmatrix} \boldsymbol{\tau} + \mathbf{f} \times \mathbf{d} \\ \mathbf{f} \end{bmatrix}$$

```cpp
// spatial.h — c.f. RBDA 2.22
Vec8f<Force> Shift(Vec8f<Force> const& force, v4_cref ofs)
{
    return Vec8f<Force>(force.ang + Cross(force.lin, ofs), force.lin);
}
```

**Intuition**: Moving the reference point changes the moment arm, so the torque changes. The force itself doesn't depend on reference point.

## The Duality Pattern

Notice the symmetry:

| | What's intrinsic | What shifts | Cross product |
|---|---|---|---|
| Motion | `.ang` (ω) | `.lin` += ω × d | angular × offset |
| Force | `.lin` (f) | `.ang` += f × d | linear × offset |

The intrinsic component stays, the other adjusts by the cross product of the intrinsic component with the offset.

## Example: Applying a Force at a Point

When `RigidBody::ApplyForceWS()` receives a force at some point, it shifts it to the CoM:

```cpp
void ApplyForceWS(v4_cref ws_force, v4_cref ws_torque, v4_cref ws_at = v4Zero)
{
    auto spatial_force = v8force{ws_torque, ws_force};
    spatial_force = Shift(spatial_force, CentreOfMassWS() - ws_at);
    ApplyForceWS(spatial_force);
}
```

The offset is `CoM - application_point`. This computes the torque that the force creates about the CoM.

For example, pushing a box at its top-right corner creates a torque that tends to rotate it. `Shift` handles this calculation.

## Shifting Accelerations

There's a special case for accelerations. Unlike velocity, spatial acceleration needs an extra centrifugal term when shifted:

$$\text{Shift}_{\text{accel}}(\hat{\mathbf{a}}, \boldsymbol{\omega}, \mathbf{d}) = \begin{bmatrix} \boldsymbol{\alpha} \\ \mathbf{a} + \boldsymbol{\alpha} \times \mathbf{d} + \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{d}) \end{bmatrix}$$

```cpp
Vec8f<Motion> ShiftAccelerationBy(Vec8f<Motion> const& acc, v4_cref avel, v4_cref ofs)
{
    return Vec8f<Motion>(acc.ang,
        acc.lin + Cross(acc.ang, ofs) + Cross(avel, Cross(avel, ofs)));
}
```

The extra $\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{d})$ term is the **centripetal acceleration**. This is why acceleration shifting needs the angular velocity as an additional parameter.

## Summary

| Function | Use Case |
|----------|----------|
| `Shift(v8motion, ofs)` | Move velocity measurement to new point |
| `Shift(v8force, ofs)` | Move force measurement to new point |
| `ShiftAccelerationBy(v8motion, ω, ofs)` | Move acceleration (includes centripetal term) |

**Key insight**: Shifting doesn't rotate anything. The coordinate axes stay the same. Only the reference point moves. For changing coordinate frames (rotation + translation), you need a spatial transform — covered next.

## What's Next

[Tutorial 07](07-spatial-transforms.md) covers spatial transforms — how to change both the reference point *and* the coordinate frame, and why motion and force transform differently.
