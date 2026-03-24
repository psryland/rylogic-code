# Tutorial 07 — Spatial Transforms

## From 3D to 6D Transforms

A 3D affine transform ${}^bX_a$ takes a point from frame $a$ to frame $b$. It has a rotation $E$ and a translation $\mathbf{d}$:

$${}^bX_a = \begin{bmatrix} E & \mathbf{d} \\ 0 & 1 \end{bmatrix}$$

A spatial transform does the same for 6D spatial vectors, but motion and force vectors transform **differently**.

## Spatial Transform for Motion Vectors

$${}^b\hat{X}_a = \begin{bmatrix} E & 0 \\ -E\mathbf{d}^{\times} & E \end{bmatrix}$$

where $\mathbf{d}^{\times}$ is the 3×3 cross-product matrix of the translation.

In Rylogic, the `operator *` for `m4x4 × v8motion` implements this:
```cpp
// spatial.h — Transform a spatial motion vector by an affine transform
Vec8f<T> operator * (Mat4x4_cref<float,Motion,T> a2b, Vec8f<Motion> const& vec)
{
    // [ E    0] * [v.ang] = [E*v.ang             ]
    // [-E*rx E]   [v.lin]   [E*v.lin - E*rx*v.ang]
    auto ang_b = a2b.rot * vec.ang;
    auto lin_b = a2b.rot * vec.lin + Cross(a2b.pos, ang_b);
    return Vec8f<T>{ang_b, lin_b};
}
```

Reading the code: the angular part is simply rotated ($E\boldsymbol{\omega}$). The linear part is rotated AND shifted by the cross product with the translation ($E\mathbf{v} + \mathbf{d} \times E\boldsymbol{\omega}$).

## Spatial Transform for Force Vectors

$${}^b\hat{X}^{*}_a = \begin{bmatrix} E & -E\mathbf{d}^{\times} \\ 0 & E \end{bmatrix}$$

Note: the off-diagonal block has moved from lower-left to upper-right!

```cpp
// spatial.h — Transform a spatial force vector by an affine transform
Vec8f<T> operator * (Mat4x4_cref<float,Force,T> a2b, Vec8f<Force> const& vec)
{
    // [E -E*rx] * [v.ang] = [E*v.ang - E*rx*v.lin]
    // [0     E]   [v.lin]   [E*v.lin             ]
    auto lin_b = a2b.rot * vec.lin;
    auto ang_b = a2b.rot * vec.ang + Cross(a2b.pos, lin_b);
    return Vec8f<T>{ang_b, lin_b};
}
```

The linear force is simply rotated. The torque is rotated AND picks up a moment arm contribution.

## The Duality Relationship

The force transform and motion transform are related:

$${}^b\hat{X}^{*}_a = ({}^b\hat{X}_a)^{-T}$$

That is, the force transform is the **inverse-transpose** of the motion transform. This is noted in the `spatial.h` header:
```
// If X transforms M6 vectors, then X* = X^{-T} transforms F6 vectors.
```

You never need to compute this explicitly — the `operator *` overloads handle it automatically based on the type tag.

## Pure Rotation (No Translation)

When $\mathbf{d} = 0$ (rotation only), both transforms simplify to the same thing:

$$\begin{bmatrix} E & 0 \\ 0 & E \end{bmatrix}$$

Rotate both components independently. This is why `m3x4 * v8motion` and `m3x4 * v8force` have the same implementation — there's no translation term to handle differently.

## Explicit Spatial Transform Matrices

For algorithms that need the full 6×6 matrix (e.g., articulated-body), Rylogic provides `Transform<>()`:

```cpp
// Create a 6×6 spatial transform matrix for motion vectors
auto A2B_motion = Transform<Motion>(a2b);  // Mat6x8f<Motion,Motion>

// Create a 6×6 spatial transform matrix for force vectors
auto A2B_force = Transform<Force>(a2b);    // Mat6x8f<Force,Force>
```

The implementations build the block matrix explicitly:
```cpp
// Motion transform matrix
template <>
Mat6x8f<Motion,Motion> Transform<Motion>(m4_cref a2b)
{
    // [E    0   ]
    // [-rxE  E  ]  (note: rxE not Erx because right-to-left multiply convention)
    return Mat6x8f<Motion,Motion>{a2b.rot, m3x4Zero, CPM(a2b.pos) * a2b.rot, a2b.rot};
}

// Force transform matrix
template <>
Mat6x8f<Force,Force> Transform<Force>(m4_cref a2b)
{
    // [E   -rxE]
    // [0     E ]
    return Mat6x8f<Force,Force>{a2b.rot, CPM(a2b.pos) * a2b.rot, m3x4Zero, a2b.rot};
}
```

## Composing Transforms

Spatial transforms compose by matrix multiplication, just like 3D transforms:

$${}^c\hat{X}_a = {}^c\hat{X}_b \cdot {}^b\hat{X}_a$$

The unit tests verify this property:
```cpp
auto A2B = Transform<Motion>(a2b);
auto B2C = Transform<Motion>(b2c);
auto A2C = Transform<Motion>(a2c);  // a2c = b2c * a2b

auto R = B2C * A2B;
PR_EXPECT(FEql(A2C, R));  // Composition is consistent
```

## When to Use What

| Situation | Use |
|-----------|-----|
| Transform a single vector | `a2b * motion_vec` or `a2b * force_vec` (operator overloads) |
| Need the full 6×6 matrix | `Transform<Motion>(a2b)` or `Transform<Force>(a2b)` |
| Only changing reference point | `Shift(vec, offset)` (tutorial 06) |
| Only rotating | `rot * vec` (same for motion and force) |

## Summary

| Transform | Motion (velocity, accel) | Force (force, momentum) |
|-----------|------------------------|------------------------|
| Matrix form | $\begin{bmatrix} E & 0 \\ -E\mathbf{d}^{\times} & E \end{bmatrix}$ | $\begin{bmatrix} E & -E\mathbf{d}^{\times} \\ 0 & E \end{bmatrix}$ |
| Angular | Rotated | Rotated + moment arm |
| Linear | Rotated + shifted | Rotated |
| Relationship | $X$ | $X^{-T}$ |

## What's Next

[Tutorial 08](08-spatial-inertia.md) covers the spatial inertia matrix — the 6×6 mass matrix that maps velocity to momentum and naturally handles all the coupling between linear and angular motion.
