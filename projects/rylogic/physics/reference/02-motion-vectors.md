# Tutorial 02 — Motion Vectors

## What Is a Motion Vector?

A **motion vector** (or *twist*) is a 6D vector that describes the instantaneous motion of a rigid body:

$$\hat{\mathbf{v}} = \begin{bmatrix} \boldsymbol{\omega} \\ \mathbf{v}_O \end{bmatrix}$$

where:
- $\boldsymbol{\omega}$ is the angular velocity (same everywhere on the body)
- $\mathbf{v}_O$ is the linear velocity of the reference point $O$

In Rylogic, this is `v8motion`:
```cpp
v8motion velocity;
velocity.ang;  // ω — angular velocity (v4, xyz components, w=0)
velocity.lin;  // v — linear velocity at the reference point (v4, xyz components, w=0)
```

## The Velocity Field

A rigid body doesn't have *a* velocity — every point on it can have a different velocity. What it has is a **velocity field**: for any point $P$, the velocity at $P$ is:

$$\mathbf{v}_P = \mathbf{v}_O + \boldsymbol{\omega} \times \overrightarrow{OP}$$

This is the fundamental equation. The angular velocity $\boldsymbol{\omega}$ is the same everywhere, but the linear velocity depends on where you ask.

A `v8motion` encodes this entire field. The `.ang` component is the field's "curl" (constant everywhere), and `.lin` is the field's value at the reference point.

### Rylogic: `LinAt()`

The `Vec8<T>` type provides `LinAt()` to evaluate the field at a point:
```cpp
// Get the linear velocity at point 'pt' (world space position vector, w=1)
auto vel_at_pt = spatial_velocity.LinAt(pt);
// Equivalent to: spatial_velocity.lin + Cross(spatial_velocity.ang, pt)
```

This is used in the unit tests to verify that a velocity field gives the same physical velocity regardless of which coordinate frame it's expressed in.

## Example: A Spinning Top

Imagine a top spinning at 10 rad/s about the Z axis, sitting at position $(1, 0, 0)$:

```
ω = (0, 0, 10)     — spinning about Z
v_origin = (0, 0, 0) — the world origin isn't moving
```

The spatial velocity *measured at the origin* is:
$$\hat{\mathbf{v}} = \begin{bmatrix} 0 \\ 0 \\ 10 \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

But the velocity of the top's contact point at $(1, 0, 0)$ is:
$$\mathbf{v}_{(1,0,0)} = \mathbf{v}_O + \boldsymbol{\omega} \times (1,0,0) = (0,0,0) + (0, 0, 10) \times (1, 0, 0) = (0, 10, 0)$$

The same `v8motion` gives the velocity at any point. That's the power of spatial vectors — they're not tied to a single point, they describe the whole field.

## Accelerations Are Motion Vectors Too

Spatial accelerations have the same structure:

$$\hat{\mathbf{a}} = \begin{bmatrix} \boldsymbol{\alpha} \\ \mathbf{a}_O \end{bmatrix}$$

where $\boldsymbol{\alpha}$ is angular acceleration and $\mathbf{a}_O$ is the linear acceleration of the reference point. The type is still `v8motion`.

> **Caution**: Spatial acceleration is *not* the same as the time derivative of spatial velocity. There's a cross-product correction term. This is why `ShiftAccelerationBy()` exists separately from `Shift()` — shifting an acceleration to a new point picks up an extra centrifugal term $\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})$.

## What the Reference Point Means

A spatial velocity is always measured **at** some point. In the Rylogic physics engine, that point is the **model origin** (not the centre of mass). This is a deliberate choice:

- The model origin is where you place the object in the world (`m_o2w.pos`)
- The centre of mass may be offset from the model origin (`m_os_com`)
- Forces and momentum are also measured at the model origin, keeping everything consistent

When you call `rb.VelocityWS()`, you get a `v8motion` measured at the model origin, in world space. To find the velocity at the CoM:
```cpp
auto vel_at_com = Shift(rb.VelocityWS(), rb.CentreOfMassWS() - rb.O2W().pos);
```

## Summary

| Aspect | Detail |
|--------|--------|
| Type | `v8motion` = `Vec8f<Motion>` |
| Components | `.ang` = angular velocity $\boldsymbol{\omega}$, `.lin` = linear velocity at reference point |
| Describes | An entire velocity field, not a single velocity |
| Velocity at point $P$ | $\mathbf{v}_P = \text{lin} + \boldsymbol{\omega} \times P$ |
| Also used for | acceleration, infinitesimal displacement |
| Reference point | Model origin in Rylogic (not CoM) |

## What's Next

[Tutorial 03](03-force-vectors.md) covers the dual side — force vectors (wrenches), and why forces and velocities live in different vector spaces.
