# Tutorial 04 — The Dot Product and Work

## The Spatial Dot Product

The dot product is only defined **between** the two dual spaces — motion dotted with force:

$$\hat{\mathbf{v}} \cdot \hat{\mathbf{f}} = \boldsymbol{\omega} \cdot \boldsymbol{\tau} + \mathbf{v} \cdot \mathbf{f}$$

This gives a **scalar** with physical meaning:

| Motion | Force | Dot Product = |
|--------|-------|---------------|
| velocity | force | power (watts) |
| displacement | force | work (joules) |
| velocity | momentum | twice kinetic energy |

In Rylogic:
```cpp
// Only defined for (Motion, Force) pairs — the compiler prevents (Motion, Motion)
float power = Dot(velocity, force);     // v8motion × v8force → float
float power = Dot(force, velocity);     // v8force × v8motion → float (commutative)
```

The implementation is straightforward — it's just the sum of the 3D dot products of the angular and linear parts:
```cpp
float Dot(Vec8f<Motion> const& lhs, Vec8f<Force> const& rhs)
{
    return Dot3(lhs.ang, rhs.ang) + Dot3(lhs.lin, rhs.lin);
}
```

## Why Is Motion · Motion Undefined?

Dotting two velocities doesn't produce a physically meaningful quantity. The result wouldn't be invariant under coordinate changes. Spatial algebra prevents this at the type level — `Dot(v8motion, v8motion)` simply doesn't compile.

This is one of the great benefits of using typed spatial vectors: the compiler catches physically meaningless operations.

## Reference Point Independence

A remarkable property: the dot product gives the **same result** regardless of which reference point the vectors are measured at.

**Proof sketch**: Shift both vectors to a new point offset by $\mathbf{d}$:

$$\hat{\mathbf{v}}' = \begin{bmatrix} \boldsymbol{\omega} \\ \mathbf{v} + \boldsymbol{\omega} \times \mathbf{d} \end{bmatrix} \qquad \hat{\mathbf{f}}' = \begin{bmatrix} \boldsymbol{\tau} + \mathbf{f} \times \mathbf{d} \\ \mathbf{f} \end{bmatrix}$$

$$\hat{\mathbf{v}}' \cdot \hat{\mathbf{f}}' = \boldsymbol{\omega} \cdot (\boldsymbol{\tau} + \mathbf{f} \times \mathbf{d}) + (\mathbf{v} + \boldsymbol{\omega} \times \mathbf{d}) \cdot \mathbf{f}$$

$$= \boldsymbol{\omega} \cdot \boldsymbol{\tau} + \boldsymbol{\omega} \cdot (\mathbf{f} \times \mathbf{d}) + \mathbf{v} \cdot \mathbf{f} + (\boldsymbol{\omega} \times \mathbf{d}) \cdot \mathbf{f}$$

The middle terms cancel because $\boldsymbol{\omega} \cdot (\mathbf{f} \times \mathbf{d}) = -(\boldsymbol{\omega} \times \mathbf{d}) \cdot \mathbf{f}$ (scalar triple product identity), leaving:

$$= \boldsymbol{\omega} \cdot \boldsymbol{\tau} + \mathbf{v} \cdot \mathbf{f} = \hat{\mathbf{v}} \cdot \hat{\mathbf{f}}$$

This is why power, work, and kinetic energy are well-defined regardless of where you measure them.

## Kinetic Energy

In Rylogic, the `RigidBody` computes kinetic energy as:

```cpp
float KineticEnergy() const
{
    auto ke = 0.5f * Dot(VelocityWS(), MomentumWS());
    return ke;
}
```

Since momentum $\hat{\mathbf{h}} = \hat{I}\hat{\mathbf{v}}$:

$$T = \frac{1}{2}\hat{\mathbf{v}} \cdot \hat{I}\hat{\mathbf{v}}$$

This is the spatial analogue of $T = \frac{1}{2}mv^2 + \frac{1}{2}\boldsymbol{\omega} \cdot I\boldsymbol{\omega}$, but it handles the cross-coupling between linear and angular terms automatically.

## Summary

| Rule | Detail |
|------|--------|
| `Dot(v8motion, v8force) → float` | ✅ Defined — gives power, work, or energy |
| `Dot(v8motion, v8motion)` | ❌ Not defined — won't compile |
| `Dot(v8force, v8force)` | ❌ Not defined — won't compile |
| Reference independence | Dot product is the same at every reference point |
| Kinetic energy | $T = \frac{1}{2}\text{Dot}(\hat{\mathbf{v}}, \hat{I}\hat{\mathbf{v}})$ |

## What's Next

[Tutorial 05](05-spatial-cross-products.md) introduces the two spatial cross products — one for motion vectors, one for force vectors — and their matrix forms.
