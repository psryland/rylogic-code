# Tutorial 01 — Why Spatial Algebra?

## The Problem

In classical mechanics, a rigid body in 3D has six degrees of freedom: three translational and three rotational. The standard approach treats linear and angular quantities separately:

- **Linear**: position $\mathbf{r}$, velocity $\mathbf{v}$, force $\mathbf{f}$, momentum $\mathbf{p} = m\mathbf{v}$
- **Angular**: orientation $R$, angular velocity $\boldsymbol{\omega}$, torque $\boldsymbol{\tau}$, angular momentum $\mathbf{L} = I\boldsymbol{\omega}$

This works, but it gets messy fast:

1. **Torque depends on reference point.** If you apply force $\mathbf{f}$ at point $\mathbf{r}$, the torque about the origin is $\boldsymbol{\tau} = \mathbf{r} \times \mathbf{f}$. Change your reference point and the torque changes. You end up tracking reference points everywhere.

2. **The equations of motion couple linear and angular terms.** For a body whose centre of mass isn't at the origin, Newton-Euler becomes:

$$\begin{bmatrix} \boldsymbol{\tau} \\ \mathbf{f} \end{bmatrix} = \begin{bmatrix} I_c - m\mathbf{c}^{\times}\mathbf{c}^{\times} & m\mathbf{c}^{\times} \\ -m\mathbf{c}^{\times} & m\mathbf{1} \end{bmatrix} \begin{bmatrix} \boldsymbol{\alpha} \\ \mathbf{a} \end{bmatrix} + \text{velocity terms}$$

where $\mathbf{c}^{\times}$ is the cross-product matrix of the CoM offset. That coupling matrix **is** the spatial inertia — but without spatial algebra you're juggling it by hand.

3. **Coordinate transforms involve different rules** for forces vs. velocities. A velocity transforms one way, a force transforms the dual way. Keeping track of which rule to use is error-prone.

## The Solution: Spatial Vectors

Spatial algebra bundles the angular and linear parts into a single 6D vector:

$$\hat{\mathbf{v}} = \begin{bmatrix} \boldsymbol{\omega} \\ \mathbf{v} \end{bmatrix} \qquad \hat{\mathbf{f}} = \begin{bmatrix} \boldsymbol{\tau} \\ \mathbf{f} \end{bmatrix}$$

These are called **Plücker coordinates**. The upper three components are angular, the lower three are linear. With this notation:

- Newton-Euler becomes a single matrix equation: $\hat{\mathbf{f}} = \hat{I}\hat{\mathbf{a}} + \hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$
- Coordinate transforms are a single 6×6 matrix multiply
- The inertia is a 6×6 matrix that handles all the coupling naturally

## Plücker Coordinates

The term "Plücker coordinates" comes from line geometry. A line in 3D can be described by a direction vector $\boldsymbol{\omega}$ and a moment $\mathbf{v} = \mathbf{r} \times \boldsymbol{\omega}$, where $\mathbf{r}$ is any point on the line. Together, $(\boldsymbol{\omega}, \mathbf{v})$ are the Plücker coordinates of the line.

A spatial velocity is exactly this: the angular velocity $\boldsymbol{\omega}$ gives the axis and rate of rotation, and $\mathbf{v}$ gives the velocity of the reference point (the origin).

**Key insight**: the angular part is the same everywhere in space, but the linear part depends on where you measure it. This is why `Shift()` only modifies the linear component for motion vectors.

## Two Vector Spaces

Spatial algebra defines two **dual** vector spaces:

| Space | Name | Contains | Rylogic Type |
|-------|------|----------|-------------|
| $\mathcal{M}^6$ | Motion | velocity, acceleration, displacement | `v8motion` |
| $\mathcal{F}^6$ | Force | force, momentum, impulse | `v8force` |

These are **not** the same space. You cannot add a motion vector to a force vector. The dot product is only defined *between* them (motion · force = power), not within a single space.

The Rylogic codebase enforces this with C++ type tags:
```cpp
struct Motion {};  // Tag for motion space
struct Force {};   // Tag for force space

using v8motion = Vec8f<Motion>;  // 6D motion vector
using v8force  = Vec8f<Force>;   // 6D force vector
```

A `v8motion` and a `v8force` are both `Vec8<float, T>` with `.ang` and `.lin` members, but the type parameter prevents you from mixing them accidentally.

## In the Rylogic Code

| Concept | 3-vector approach | Spatial approach |
|---------|------------------|-----------------|
| Velocity | `v4 vel`, `v4 ang_vel` (2 variables) | `v8motion vel` (1 variable) |
| Force + torque | `v4 force`, `v4 torque` (2 variables) | `v8force force` (1 variable) |
| Inertia | `m3x4 I`, `float mass`, `v4 com` (3 variables) | `Inertia` → `Mat6x8<Motion,Force>` (1 matrix) |
| Transform | Different formulas for vel vs. force | `a2b * motion_vec` or `a2b * force_vec` (operator overloads handle the duality) |

## What's Next

[Tutorial 02](02-motion-vectors.md) dives into motion vectors — what the `.ang` and `.lin` components mean physically, and how a single `v8motion` describes a velocity *field* (not just the velocity at a point).
