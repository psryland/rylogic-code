# Tutorial 03 — Force Vectors

## What Is a Force Vector?

A **force vector** (or *wrench*) is the 6D dual of a motion vector:

$$\hat{\mathbf{f}} = \begin{bmatrix} \boldsymbol{\tau}_O \\ \mathbf{f} \end{bmatrix}$$

where:
- $\boldsymbol{\tau}_O$ is the torque measured about the reference point $O$
- $\mathbf{f}$ is the linear force (same everywhere — force doesn't change with reference point)

In Rylogic, this is `v8force`:
```cpp
v8force wrench;
wrench.ang;  // τ — torque about the reference point (v4)
wrench.lin;  // f — linear force (v4)
```

## Contrast with Motion Vectors

Motion and force vectors are **duals** of each other. They have the same structure (6D, angular + linear) but the roles of the components are swapped:

| | Motion (`v8motion`) | Force (`v8force`) |
|---|---|---|
| Angular component | Same everywhere (ω) | Depends on reference point (τ) |
| Linear component | Depends on reference point (v) | Same everywhere (f) |

This is the key duality:
- For motion: **angular** is intrinsic, **linear** varies with position
- For force: **linear** is intrinsic, **angular** varies with position

### Why?

Think about it physically:
- A spinning body has the same angular velocity no matter where you stand. But the linear velocity you see depends on how far you are from the axis.
- A force pushes the same way regardless of where you measure it. But the *torque* it creates depends on the moment arm — your distance from the line of action.

## The Torque "Field"

Just as a `v8motion` describes a velocity field, a `v8force` describes a torque field. The torque at any point $P$ is:

$$\boldsymbol{\tau}_P = \boldsymbol{\tau}_O + \overrightarrow{OP} \times \mathbf{f}$$

Note the sign: for forces, the cross product goes **point × force** (not force × point). This is because force vectors live in the dual space.

### Rylogic: `AngAt()`

```cpp
// Get the torque at point 'pt'
auto torque_at_pt = spatial_force.AngAt(pt);
// Equivalent to: spatial_force.ang + Cross(pt, spatial_force.lin)
```

## Momentum Is a Force Vector

This might be surprising: **momentum lives in the force space**, not the motion space.

$$\hat{\mathbf{h}} = \begin{bmatrix} \mathbf{L}_O \\ \mathbf{p} \end{bmatrix} = \hat{I}\hat{\mathbf{v}}$$

where $\mathbf{L}_O$ is angular momentum about the reference point, and $\mathbf{p} = m\mathbf{v}$ is linear momentum.

Why? Because momentum is the thing you dot with velocity to get energy:

$$T = \frac{1}{2}\hat{\mathbf{v}} \cdot \hat{\mathbf{h}} = \frac{1}{2}\hat{\mathbf{v}} \cdot \hat{I}\hat{\mathbf{v}}$$

And the dot product is only defined between dual spaces (motion · force). So momentum must be a force vector.

In Rylogic:
```cpp
v8force momentum = rb.MomentumWS();  // Spatial momentum at model origin
```

## Impulse Is Also a Force Vector

An impulse $\hat{\mathbf{j}} = \hat{\mathbf{f}} \cdot \Delta t$ has the same units as momentum and lives in $\mathcal{F}^6$.

## Applying Forces in Rylogic

The `RigidBody` accumulates forces in a `v8force` member:

```cpp
// Apply a force and torque at the model origin
rb.ApplyForceWS(force, torque);

// What this does internally:
//   spatial_force = v8force{torque, force};
//   spatial_force = Shift(spatial_force, CentreOfMassWS() - at);
//   m_ws_force += spatial_force;
```

The `Shift()` operation adjusts the torque to account for the moment arm between the application point and the centre of mass. The linear force is unchanged (because force is intrinsic — it doesn't depend on reference point).

### How `Shift()` Works for Forces

```cpp
// Shift a force vector to a new reference point offset by 'ofs'
Vec8f<Force> Shift(Vec8f<Force> const& force, v4_cref ofs)
{
    // Torque changes: τ_new = τ_old + f × ofs
    // Force stays:    f_new = f_old
    return Vec8f<Force>(force.ang + Cross(force.lin, ofs), force.lin);
}
```

Compare with shifting a motion vector:
```cpp
Vec8f<Motion> Shift(Vec8f<Motion> const& motion, v4_cref ofs)
{
    // Angular stays: ω_new = ω_old
    // Velocity changes: v_new = v_old + ω × ofs
    return Vec8f<Motion>(motion.ang, motion.lin + Cross(motion.ang, ofs));
}
```

The pattern is symmetric: what's intrinsic stays, what's extrinsic adjusts via a cross product.

## Summary

| Aspect | Detail |
|--------|--------|
| Type | `v8force` = `Vec8f<Force>` |
| Components | `.ang` = torque $\boldsymbol{\tau}$ at reference point, `.lin` = linear force $\mathbf{f}$ |
| Intrinsic component | Linear force (same everywhere) |
| Reference-dependent | Torque (depends on moment arm) |
| Also used for | momentum, impulse |
| Torque at point $P$ | $\boldsymbol{\tau}_P = \text{ang} + P \times \text{lin}$ |

## What's Next

[Tutorial 04](04-dot-product-and-work.md) explains the dot product between motion and force vectors, and why it computes physical work/power.
