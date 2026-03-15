# Tutorial 05 — Spatial Cross Products

## Two Cross Products

In 3D, there's one cross product. In spatial algebra, there are **two**, one for each vector space:

| Name | Notation | Produces | Physical meaning |
|------|----------|----------|-----------------|
| Motion cross | $\hat{\mathbf{a}} \times \hat{\mathbf{b}}$ | `v8motion` | Rate of change of a motion vector |
| Force cross | $\hat{\mathbf{a}} \times^{*} \hat{\mathbf{b}}$ | `v8force` | Rate of change of a force vector |

The star ($\times^{*}$) denotes the force-space version.

## Motion Cross Product ($\times$)

Given two motion vectors $\hat{\mathbf{a}}$ and $\hat{\mathbf{b}}$:

$$\hat{\mathbf{a}} \times \hat{\mathbf{b}} = \begin{bmatrix} \boldsymbol{\omega}_a \times \boldsymbol{\omega}_b \\ \boldsymbol{\omega}_a \times \mathbf{v}_b + \mathbf{v}_a \times \boldsymbol{\omega}_b \end{bmatrix}$$

In Rylogic:
```cpp
template <typename T>
Vec8f<Motion> Cross(Vec8f<T> const& lhs, Vec8f<Motion> const& rhs)
{
    return Vec8f<Motion>(
        Cross3(lhs.ang, rhs.ang),                                // ω_a × ω_b
        Cross3(lhs.ang, rhs.lin) + Cross3(lhs.lin, rhs.ang));    // ω_a × v_b + v_a × ω_b
}
```

**Physical meaning**: If $\hat{\mathbf{v}}$ is a spatial velocity and $\hat{\mathbf{a}}$ is a spatial acceleration, then the relationship involves $\hat{\mathbf{v}} \times$ acting on motion vectors. This appears in the equation of motion as the velocity-dependent terms.

## Force Cross Product ($\times^{*}$)

Given a vector $\hat{\mathbf{a}}$ and a force vector $\hat{\mathbf{b}}$:

$$\hat{\mathbf{a}} \times^{*} \hat{\mathbf{b}} = \begin{bmatrix} \boldsymbol{\omega}_a \times \boldsymbol{\tau}_b + \mathbf{v}_a \times \mathbf{f}_b \\ \boldsymbol{\omega}_a \times \mathbf{f}_b \end{bmatrix}$$

In Rylogic:
```cpp
template <typename T>
Vec8f<Force> Cross(Vec8f<T> const& lhs, Vec8f<Force> const& rhs)
{
    return Vec8f<Force>(
        Cross3(lhs.ang, rhs.ang) + Cross3(lhs.lin, rhs.lin),    // ω_a × τ_b + v_a × f_b
        Cross3(lhs.ang, rhs.lin));                                // ω_a × f_b
}
```

**Physical meaning**: This appears in the equation of motion as $\hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$, the velocity-dependent force (Coriolis and centrifugal effects).

## The Duality Relationship

The two cross products are related by:

$$(\hat{\mathbf{a}} \times^{*})^T = -(\hat{\mathbf{a}} \times)$$

In other words, the force cross-product matrix is the **negative transpose** of the motion cross-product matrix. The unit tests verify this:
```cpp
{// Test: vx* == -Transpose(vx)
    auto v = v8(-2.3f, +1.3f, 0.9f, -2.2f, 0.0f, -1.0f);
    auto m0 = CPM(static_cast<v8motion>(v)); // vx
    auto m1 = CPM(static_cast<v8force>(v));  // vx*
    auto m2 = Transpose(m1);
    auto m3 = static_cast<m6x8m>(-m2);
    PR_EXPECT(FEql(m0, m3));
}
```

## Cross Product Matrices (`CPM`)

Just as the 3D cross product $\mathbf{a} \times \mathbf{b}$ can be written as a matrix multiply $[\mathbf{a}]_{\times}\mathbf{b}$, the spatial cross product can be written using a 6×6 matrix.

### Motion CPM

$$[\hat{\mathbf{a}}]_{\times} = \begin{bmatrix} [\boldsymbol{\omega}_a]_{\times} & 0 \\ [\mathbf{v}_a]_{\times} & [\boldsymbol{\omega}_a]_{\times} \end{bmatrix}$$

```cpp
Mat6x8f<Motion,Motion> CPM(Vec8f<Motion> const& a)
{
    auto cx_ang = CPM(a.ang);    // 3×3 cross-product matrix of angular part
    auto cx_lin = CPM(a.lin);    // 3×3 cross-product matrix of linear part
    return Mat6x8f<Motion, Motion>(cx_ang, m3x4Zero, cx_lin, cx_ang);
    //                             m00     m01       m10     m11
}
```

### Force CPM

$$[\hat{\mathbf{a}}]_{\times^{*}} = \begin{bmatrix} [\boldsymbol{\omega}_a]_{\times} & [\mathbf{v}_a]_{\times} \\ 0 & [\boldsymbol{\omega}_a]_{\times} \end{bmatrix}$$

```cpp
Mat6x8f<Force,Force> CPM(Vec8f<Force> const& a)
{
    auto cx_ang = CPM(a.ang);
    auto cx_lin = CPM(a.lin);
    return Mat6x8f<Force, Force>(cx_ang, cx_lin, m3x4Zero, cx_ang);
    //                           m00     m01     m10       m11
}
```

## When Do Cross Products Appear?

1. **Equation of motion**: $\hat{\mathbf{f}} = \hat{I}\hat{\mathbf{a}} + \hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$

   The $\hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$ term is the "velocity-dependent force" — it includes gyroscopic, Coriolis, and centrifugal effects.

2. **Time derivative of inertia**: When the body rotates, its world-space inertia changes. The rate of change involves the cross product.

3. **Articulated-body algorithms**: The recursive Newton-Euler algorithm uses spatial cross products extensively.

## Summary

| Operation | Result Type | Formula (angular, linear) |
|-----------|------------|---------------------------|
| $\hat{a} \times \hat{m}$ | `v8motion` | $(\omega_a \times \omega_m,\; \omega_a \times v_m + v_a \times \omega_m)$ |
| $\hat{a} \times^{*} \hat{f}$ | `v8force` | $(\omega_a \times \tau_f + v_a \times f_f,\; \omega_a \times f_f)$ |
| Relationship | | $\times^{*} = -(\times)^T$ |

## What's Next

[Tutorial 06](06-shifting-spatial-vectors.md) covers `Shift()` — how to express the same physical quantity at a different reference point without changing coordinate frames.
